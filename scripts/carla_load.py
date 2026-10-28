#!/usr/bin/env python3
"""CARLA Town01 ego vehicle with ROS 2 RGB camera and LiDAR topics.

Keeps camera and LiDAR outputs, ego spawning, Town01 loading, and optional
keyboard/autopilot driving. Omits /clock, odometry, status reporting, and
other unrelated republisher features.

Topics (default role name: hero):
  /carla/hero/rgb/image             sensor_msgs/msg/Image
  /carla/hero/rgb/image/compressed  sensor_msgs/msg/CompressedImage (if cv2/Pillow is installed)
  /carla/hero/rgb/camera_info       sensor_msgs/msg/CameraInfo
  /carla/hero/lidar                 sensor_msgs/msg/PointCloud2

TO USE MAPS: 
Convert an OpenStreetMap export (e.g. Collip Circle) to OpenDRIVE with CARLA's
built-in osm2odr, then load it into the running CARLA server as a standalone
OpenDRIVE world.

    python3 scripts/carla_load_osm.py maps/collip_circle.osm
    python3 scripts/carla_load_osm.py maps/collip_circle.osm --props-every 4   # add LiDAR features
    python3 scripts/carla_load_osm.py --xodr maps/collip_circle.xodr          # reload saved .xodr

Then start scripts/carla_ros2_republisher.py as usual (WITHOUT --town).
The world stays loaded until CARLA is restarted or another map is loaded.

Notes
  * The input must be an OpenStreetMap export (roads tagged highway=*), not the
    team's Lanelet2 .osm - osm2odr ignores lanelet relations.
  * A standalone OpenDRIVE world is just a road mesh: no buildings, curbs or trees.
    LIO-SAM / NDT need 3D structure, so use --wall-height and/or --props-every
    for mapping/localization runs.
"""
import argparse
import io
import math
import threading
import random
import os
import xml.etree.ElementTree as ET
from collections import Counter


import carla
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2, PointField

# JPEG encoding is optional; raw Image and CameraInfo work without it.
try:
    import cv2

    def encode_jpeg(bgra: np.ndarray, quality: int) -> bytes:
        bgr = np.ascontiguousarray(bgra[:, :, :3])
        ok, encoded = cv2.imencode('.jpg', bgr, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if not ok:
            raise RuntimeError('OpenCV JPEG encoding failed')
        return encoded.tobytes()
except ImportError:
    try:
        from PIL import Image as PILImage

        def encode_jpeg(bgra: np.ndarray, quality: int) -> bytes:
            rgb = np.ascontiguousarray(bgra[:, :, 2::-1])
            out = io.BytesIO()
            PILImage.fromarray(rgb).save(out, format='JPEG', quality=quality)
            return out.getvalue()
    except ImportError:
        encode_jpeg = None


def ros_time(seconds: float) -> Time:
    stamp = Time()
    stamp.sec = int(seconds)
    stamp.nanosec = int((seconds - stamp.sec) * 1_000_000_000)
    return stamp

# CARLA's documented default list plus `service`: campus and parking-lot roads
# (likely including Collip Circle) are usually tagged highway=service.
DEFAULT_WAY_TYPES = [
    'motorway', 'motorway_link', 'trunk', 'trunk_link', 'primary', 'primary_link',
    'secondary', 'secondary_link', 'tertiary', 'tertiary_link', 'unclassified',
    'residential', 'service', 'living_street',
]

# Static props that give the LiDAR vertical structure; first ones found are used.
PROP_PREFERENCE = [
    'static.prop.streetbarrier', 'static.prop.constructioncone', 'static.prop.trafficcone01',
    'static.prop.busstop', 'static.prop.container', 'static.prop.box03', 'static.prop.bin',
]

def lanelet_to_carla_osm(osm_data):
    """Create a simple road-centerline OSM from a Lanelet2 XML export."""
    root = ET.fromstring(osm_data)

    nodes = {n.get("id"): n for n in root.findall("node")}
    ways = {w.get("id"): w for w in root.findall("way")}

    def local_xy(node_id):
        node = nodes[node_id]
        tags = {
            t.get("k"): t.get("v")
            for t in node.findall("tag")
        }
        return float(tags["local_x"]), float(tags["local_y"])

    next_node_id = 9000000
    next_way_id = 9000000

    out = ET.Element("osm", version="0.6", generator="lanelet2_to_carla")

    for relation in root.findall("relation"):
        tags = {
            t.get("k"): t.get("v")
            for t in relation.findall("tag")
        }
        if tags.get("type") != "lanelet" or tags.get("subtype") != "road":
            continue

        members = {
            m.get("role"): ways.get(m.get("ref"))
            for m in relation.findall("member")
            if m.get("type") == "way"
        }

        left = members.get("left")
        right = members.get("right")
        if left is None or right is None:
            continue

        left_ids = [nd.get("ref") for nd in left.findall("nd")]
        right_ids = [nd.get("ref") for nd in right.findall("nd")]
        if len(left_ids) < 2 or len(right_ids) < 2:
            continue

        left_xy = [local_xy(i) for i in left_ids]
        right_xy = [local_xy(i) for i in right_ids]

        # Pair points along each boundary by normalized distance.
        def resample(points, count):
            lengths = [0.0]
            for a, b in zip(points, points[1:]):
                lengths.append(lengths[-1] + ((b[0]-a[0])**2 + (b[1]-a[1])**2)**0.5)
            total = lengths[-1]
            if total == 0:
                return [points[0]] * count
            result = []
            for j in range(count):
                target = total * j / (count - 1)
                k = next((k for k in range(len(lengths)-1)
                          if lengths[k] <= target <= lengths[k+1]),
                         len(lengths)-2)
                span = lengths[k+1] - lengths[k]
                t = 0 if span == 0 else (target-lengths[k])/span
                result.append((
                    points[k][0] + t*(points[k+1][0]-points[k][0]),
                    points[k][1] + t*(points[k+1][1]-points[k][1])
                ))
            return result

        count = max(len(left_xy), len(right_xy), 2)
        l = resample(left_xy, count)
        r = resample(right_xy, count)
        center = [((a[0]+b[0])/2, (a[1]+b[1])/2) for a, b in zip(l, r)]

        refs = []
        for x, y in center:
            node_id = str(next_node_id)
            next_node_id += 1
            # Synthetic coordinates: 1 degree latitude/longitude is not
            # a metre. Use a local scale around the equator for conversion.
            node = ET.SubElement(out, "node", id=node_id,
                                 lat=f"{y / 111320:.9f}",
                                 lon=f"{x / 111320:.9f}")
            refs.append(node_id)

        way = ET.SubElement(out, "way", id=str(next_way_id))
        next_way_id += 1
        for ref in refs:
            ET.SubElement(way, "nd", ref=ref)
        ET.SubElement(way, "tag", k="highway", v="residential")
        ET.SubElement(way, "tag", k="lanes", v="1")

    return ET.tostring(out, encoding="unicode")

def convert(osm_path, args):
    """ THIS IS TO CONVERT FROM OSM TO XODR """
    with open(osm_path, encoding='utf-8') as f:
        osm_data = f.read()

    s = carla.Osm2OdrSettings()
    s.set_osm_way_types(args.way_types)
    s.default_lane_width = args.lane_width
    s.generate_traffic_lights = args.traffic_lights
    s.all_junctions_with_traffic_lights = False
    if hasattr(s, 'center_map'):
        s.center_map = True  # map centre -> CARLA origin (geo-reference kept in the .xodr header)

    # DIAGNOSTIC
    print("OSM data type:", type(osm_data))
    print("OSM data length:", len(osm_data))
    print("OSM data preview:", repr(osm_data[:500]))

    root = ET.fromstring(osm_data)
    nodes = root.findall("node")
    ways = root.findall("way")

    print("Root:", root.tag)
    print("Nodes:", len(nodes))
    print("Ways:", len(ways))

    for node in nodes[:5]:
        print(
            "Node:", node.get("id"),
            "lat:", node.get("lat"),
            "lon:", node.get("lon")
        )

    #diagnostic
    print("Way types:")
    for key, count in Counter(
        tag.get("v")
        for way in root.findall("way")
        for tag in way.findall("tag")
        if tag.get("k") == "highway"
    ).most_common():
        print(f"  {key}: {count}")

    print("Configured way types:", args.way_types)
    print("First way tags:")

    for way in root.findall("way")[:3]:
        print(way.get("id"), {
            tag.get("k"): tag.get("v")
            for tag in way.findall("tag")
        })
    # MORE DIAGNOSTIC

    print("Way tag types:")
    print(Counter(
        tag.get("v")
        for way in root.findall("way")
        for tag in way.findall("tag")
        if tag.get("k") == "type"
    ))

    print("\nRelation tag types:")
    print(Counter(
        tag.get("v")
        for relation in root.findall("relation")
        for tag in relation.findall("tag")
        if tag.get("k") == "type"
    ))

    print("\nRelation subtypes:")
    print(Counter(
        tag.get("v")
        for relation in root.findall("relation")
        for tag in relation.findall("tag")
        if tag.get("k") == "subtype"
    ))

    print("\nFirst lanelet relation:")
    for relation in root.findall("relation"):
        tags = {
            tag.get("k"): tag.get("v")
            for tag in relation.findall("tag")
        }
        if tags.get("type") == "lanelet":
            print("Tags:", tags)
            print("Members:", [
                (m.get("type"), m.get("ref"), m.get("role"))
                for m in relation.findall("member")
            ])
            break

    nodes = {
        n.get("id"): n
        for n in root.findall("node")
    }

    for way in root.findall("way"):
        tags = {
            t.get("k"): t.get("v")
            for t in way.findall("tag")
        }

        if tags.get("type") in ("virtual", "road_border"):
            refs = [nd.get("ref") for nd in way.findall("nd")]
            coords = []

            for ref in refs:
                node = nodes.get(ref)
                if node is not None:
                    coords.append((
                        float(node.find("tag[@k='local_x']").get("v")),
                        float(node.find("tag[@k='local_y']").get("v"))
                    ))

            if len(coords) >= 2:
                print(
                    "Way:", way.get("id"),
                    "type:", tags.get("type"),
                    "points:", len(coords),
                    "first:", coords[0],
                    "last:", coords[-1]
                )
                break


    # Check how many ways reference existing nodes
    node_ids = {node.get("id") for node in root.findall("node")}
    ways = root.findall("way")

    valid_refs = 0
    total_refs = 0

    for way in ways:
        for nd in way.findall("nd"):
            total_refs += 1
            if nd.get("ref") in node_ids:
                valid_refs += 1

    print("Total node references:", total_refs)
    print("References to existing nodes:", valid_refs)
    print("OSM bounds:", root.find("bounds").attrib if root.find("bounds") is not None else "No bounds element")
    #

    #added this later for the helper function
    carla_osm_data = lanelet_to_carla_osm(osm_data)
    print("Generated CARLA OSM length:", len(carla_osm_data))
    print("Generated CARLA road ways:",
        carla_osm_data.count("<way "))
    #

    xodr = carla.Osm2Odr.convert(osm_data, s)
    if '<road ' not in xodr:
        raise SystemExit('osm2odr produced no roads - check the OSM file has highway=* ways '
                         f'of these types: {args.way_types}')
    out = args.xodr_out or os.path.splitext(osm_path)[0] + '.xodr'
    with open(out, 'w', encoding='utf-8') as f:
        f.write(xodr)
    print(f'Wrote {out} ({xodr.count("<road ")} roads)')
    return xodr


def add_props(client, world, every_m, offset_m, seed):
    """ use to add props to the world """
    lib = world.get_blueprint_library()
    names = {bp.id for bp in lib.filter('static.prop.*')}
    chosen = [n for n in PROP_PREFERENCE if n in names] or sorted(names)[:5]
    rng = random.Random(seed)
    batch = []
    for wp in world.get_map().generate_waypoints(every_m):
        right = wp.transform.get_right_vector()
        half = wp.lane_width / 2.0 + offset_m
        for side in (1.0, -1.0):
            if side < 0 and wp.get_left_lane() is not None and \
                    wp.get_left_lane().lane_type == carla.LaneType.Driving:
                continue  # don't put props on the other carriageway
            loc = wp.transform.location + carla.Location(x=right.x * half * side,
                                                         y=right.y * half * side, z=0.1)
            tf = carla.Transform(loc, carla.Rotation(yaw=wp.transform.rotation.yaw))
            batch.append(carla.command.SpawnActor(lib.find(rng.choice(chosen)), tf))
    res = client.apply_batch_sync(batch)
    ok = sum(1 for r in res if not r.error)
    print(f'Spawned {ok}/{len(batch)} props ({", ".join(chosen)})')


class CarlaEgoSensors(Node):
    def __init__(self, args):
        super().__init__('carla_ego_sensors')
        self.args = args
        ns = f'/carla/{args.role_name}'


        if args.xodr:
            with open(args.xodr, encoding='utf-8') as f:
                xodr = f.read()
        elif args.osm:
            xodr = convert(args.osm, args)
        else:
            print("Parser Error: give an .osm file or --xodr\n")
        if args.convert_only:
            return

        self.client = carla.Client(args.host, args.port)
        self.client.set_timeout(120.0)
        # Loading Town01 changes the map for all clients connected to this server.
        self.world = self.client.load_world('Town01')
        self.bp_lib = self.world.get_blueprint_library()

        params = carla.OpendriveGenerationParameters(
            vertex_distance=2.0,
            max_road_length=500.0,
            wall_height=args.wall_height,
            additional_width=args.extra_width,
            smooth_junctions=True,
            enable_mesh_visibility=True,
            enable_pedestrian_navigation=True)
        world = self.client.generate_opendrive_world(xodr, params)
        cmap = world.get_map()
        wps = cmap.generate_waypoints(2.0)
        print(f'Loaded OpenDRIVE world: {len(wps)} waypoints @2 m, '
                f'{len(cmap.get_spawn_points())} spawn points')

        # Geo-reference: needed later to line CARLA's frame up with the Lanelet2 map (lat/lon).
        geo = cmap.transform_to_geolocation(carla.Location(0, 0, 0))
        print(f'CARLA origin = lat {geo.latitude:.7f}, lon {geo.longitude:.7f}')
    
        if args.props_every > 0:
            add_props(self.client, world, args.props_every, args.props_offset, args.seed)
    
        # Put the spectator above the map so you can see it in the CARLA window.
        xs = [w.transform.location.x for w in wps] or [0.0]
        ys = [w.transform.location.y for w in wps] or [0.0]
        cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
        h = max(max(xs) - min(xs), max(ys) - min(ys), 50.0)
        world.get_spectator().set_transform(
            carla.Transform(carla.Location(cx, cy, h), carla.Rotation(pitch=-90)))
        print('Spectator moved above the map. Now run carla_ros2_republisher.py (no --town).')

        self.ego = None
        self.camera = None
        self.lidar = None
        self._lock = threading.Lock()
        self._latest_image = None
        self._latest_lidar = None

        self.pub_image = self.create_publisher(Image, f'{ns}/rgb/image', args.queue_size)
        self.pub_camera_info = self.create_publisher(CameraInfo, f'{ns}/rgb/camera_info', args.queue_size)
        self.pub_compressed = None
        if encode_jpeg is not None and not args.no_compressed:
            self.pub_compressed = self.create_publisher(
                CompressedImage, f'{ns}/rgb/image/compressed', args.queue_size)
        elif not args.no_compressed:
            self.get_logger().warning(
                'Compressed image disabled: install python3-opencv or python3-pil for JPEG encoding.')
        self.pub_lidar = self.create_publisher(PointCloud2, f'{ns}/lidar', args.queue_size)

        self._pygame = None
        if args.keyboard:
            try:
                import pygame
            except ImportError as exc:
                raise RuntimeError(
                    'Keyboard driving requires pygame. Install it with: python3 -m pip install pygame'
                ) from exc
            pygame.init()
            self._pygame = pygame
            pygame.display.set_caption('CARLA Keyboard Control')
            pygame.display.set_mode((480, 120))
            pygame.font.init()
            self._font = pygame.font.Font(None, 25)
            self._keys = set()

        self._spawn_ego()
        self._spawn_camera()
        self._spawn_lidar()

        if args.autopilot:
            self.ego.set_autopilot(True, args.tm_port)
            self.get_logger().info('Autopilot enabled.')
        elif args.keyboard:
            self.get_logger().info(
                'Keyboard driving: focus the control window. W/S throttle/brake, A/D steer, '
                'Space handbrake, Esc quit.')
            self.create_timer(0.02, self._keyboard_control)
        else:
            self.get_logger().info('Autopilot is off. Use --keyboard or --autopilot to drive.')

        self.create_timer(1.0 / args.publish_rate, self._publish_latest)
        self.get_logger().info(f'Loaded {self.world.get_map().name}; spawned ego id={self.ego.id}')
        self.get_logger().info(f'Image: {ns}/rgb/image; camera info: {ns}/rgb/camera_info')
        if self.pub_compressed is not None:
            self.get_logger().info(f'Compressed image: {ns}/rgb/image/compressed')
        self.get_logger().info(f'LiDAR: {ns}/lidar ({args.lidar_channels} channels)')

    def _spawn_ego(self):
        bp = self.bp_lib.find(self.args.vehicle)
        if bp.has_attribute('role_name'):
            bp.set_attribute('role_name', self.args.role_name)
        spawn_points = self.world.get_map().get_spawn_points()
        if not spawn_points:
            raise RuntimeError('Town01 has no vehicle spawn points.')
        for transform in spawn_points:
            self.ego = self.world.try_spawn_actor(bp, transform)
            if self.ego is not None:
                break
        if self.ego is None:
            raise RuntimeError('Could not spawn ego: all Town01 spawn points may be occupied.')

    def _spawn_camera(self):
        bp = self.bp_lib.find('sensor.camera.rgb')
        bp.set_attribute('image_size_x', str(self.args.width))
        bp.set_attribute('image_size_y', str(self.args.height))
        bp.set_attribute('fov', str(self.args.fov))
        bp.set_attribute('sensor_tick', str(1.0 / self.args.camera_hz))
        transform = carla.Transform(carla.Location(x=1.5, z=2.0))
        self.camera = self.world.spawn_actor(bp, transform, attach_to=self.ego)
        self.camera.listen(self._on_image)

        focal = self.args.width / (2.0 * math.tan(math.radians(self.args.fov) / 2.0))
        cx, cy = self.args.width / 2.0, self.args.height / 2.0
        info = CameraInfo()
        info.width, info.height = self.args.width, self.args.height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [focal, 0.0, cx, 0.0, 0.0, focal, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self._camera_info = info

    def _spawn_lidar(self):
        bp = self.bp_lib.find('sensor.lidar.ray_cast')
        bp.set_attribute('channels', str(self.args.lidar_channels))
        bp.set_attribute('range', str(self.args.lidar_range))
        bp.set_attribute('points_per_second', str(self.args.lidar_points_per_second))
        bp.set_attribute('rotation_frequency', str(self.args.lidar_rotation_frequency))
        bp.set_attribute('upper_fov', str(self.args.lidar_upper_fov))
        bp.set_attribute('lower_fov', str(self.args.lidar_lower_fov))
        transform = carla.Transform(carla.Location(
            x=self.args.lidar_x, y=self.args.lidar_y, z=self.args.lidar_z))
        self.lidar = self.world.spawn_actor(bp, transform, attach_to=self.ego)
        self.lidar.listen(self._on_lidar)

    # CARLA callback threads only copy the newest measurement; ROS publishing is timer-driven.
    def _on_image(self, image):
        data = (image.timestamp, image.width, image.height, bytes(image.raw_data))
        with self._lock:
            self._latest_image = data

    def _on_lidar(self, measurement):
        data = (measurement.timestamp, bytes(measurement.raw_data))
        with self._lock:
            self._latest_lidar = data

    def _publish_latest(self):
        with self._lock:
            image, self._latest_image = self._latest_image, None
            lidar, self._latest_lidar = self._latest_lidar, None
        if image is not None:
            self._publish_image(*image)
        if lidar is not None:
            self._publish_lidar(*lidar)

    def _publish_image(self, timestamp, width, height, raw):
        stamp = ros_time(timestamp)
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = 'rgb_camera'
        msg.height, msg.width = height, width
        msg.encoding = 'bgra8'
        msg.is_bigendian = 0
        msg.step = width * 4
        msg.data = raw
        self.pub_image.publish(msg)

        self._camera_info.header.stamp = stamp
        self._camera_info.header.frame_id = 'rgb_camera'
        self.pub_camera_info.publish(self._camera_info)

        if self.pub_compressed is not None:
            pixels = np.frombuffer(raw, dtype=np.uint8).reshape(height, width, 4)
            compressed = CompressedImage()
            compressed.header.stamp = stamp
            compressed.header.frame_id = 'rgb_camera'
            compressed.format = 'jpeg'
            compressed.data = encode_jpeg(pixels, self.args.jpeg_quality)
            self.pub_compressed.publish(compressed)

    def _publish_lidar(self, timestamp, raw):
        points = np.frombuffer(raw, dtype=np.float32).reshape(-1, 4).copy()
        points[:, 1] *= -1.0  # CARLA left-handed Y -> ROS right-handed Y.
        msg = PointCloud2()
        msg.header.stamp = ros_time(timestamp)
        msg.header.frame_id = self.args.lidar_frame
        msg.height, msg.width = 1, points.shape[0]
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * points.shape[0]
        msg.is_dense = True
        msg.data = points.tobytes()
        self.pub_lidar.publish(msg)

    def _keyboard_control(self):
        if self._pygame is None or self.ego is None:
            return
        pygame = self._pygame
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                rclpy.shutdown()
                return
            if event.type == pygame.KEYDOWN:
                self._keys.add(event.key)
                if event.key == pygame.K_ESCAPE:
                    rclpy.shutdown()
                    return
            elif event.type == pygame.KEYUP:
                self._keys.discard(event.key)
        keys = self._keys
        control = carla.VehicleControl()
        control.throttle = 0.6 if pygame.K_w in keys or pygame.K_UP in keys else 0.0
        control.brake = 0.7 if pygame.K_s in keys or pygame.K_DOWN in keys else 0.0
        if pygame.K_a in keys or pygame.K_LEFT in keys:
            control.steer -= 0.5
        if pygame.K_d in keys or pygame.K_RIGHT in keys:
            control.steer += 0.5
        control.hand_brake = pygame.K_SPACE in keys
        self.ego.apply_control(control)
        surface = pygame.display.get_surface()
        if surface is not None:
            surface.fill((25, 25, 25))
            lines = ['CARLA manual driving', 'W/S or Up/Down: throttle/brake',
                     'A/D or Left/Right: steer | Space: handbrake', 'Esc: exit']
            for i, line in enumerate(lines):
                surface.blit(self._font.render(line, True, (240, 240, 240)), (10, 5 + i * 28))
            pygame.display.flip()

    def cleanup(self):
        actors = [a for a in (self.camera, self.lidar, self.ego) if a is not None]
        for sensor in (self.camera, self.lidar):
            if sensor is not None:
                try:
                    sensor.stop()
                except Exception:
                    pass
        try:
            if actors:
                self.client.set_timeout(3.0)
                self.client.apply_batch([carla.command.DestroyActor(a.id) for a in actors])
        except Exception as exc:
            self.get_logger().warning(f'Actor cleanup failed: {exc}')
        if self._pygame is not None:
            self._pygame.quit()
        self.destroy_node()


def main():
    parser = argparse.ArgumentParser(description='CARLA Town01 ego with ROS 2 RGB camera and LiDAR.')
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--role-name', default='hero')
    parser.add_argument('--vehicle', default='vehicle.tesla.model3')
    parser.add_argument('--autopilot', action='store_true')
    parser.add_argument('--keyboard', action='store_true')
    parser.add_argument('--tm-port', type=int, default=8000)
    parser.add_argument('--width', type=int, default=800)
    parser.add_argument('--height', type=int, default=600)
    parser.add_argument('--fov', type=float, default=90.0)
    parser.add_argument('--camera-hz', type=float, default=10.0)
    parser.add_argument('--no-compressed', action='store_true', help='disable JPEG compressed image topic')
    parser.add_argument('--jpeg-quality', type=int, default=80)
    parser.add_argument('--lidar-frame', default='lidar')
    parser.add_argument('--queue-size', type=int, default=10)
    parser.add_argument('--publish-rate', type=float, default=50.0)
    parser.add_argument('--lidar-channels', type=int, default=96)
    parser.add_argument('--lidar-range', type=float, default=100.0)
    parser.add_argument('--lidar-points-per-second', type=int, default=1200000)
    parser.add_argument('--lidar-rotation-frequency', type=float, default=10.0)
    parser.add_argument('--lidar-upper-fov', type=float, default=15.0)
    parser.add_argument('--lidar-lower-fov', type=float, default=-25.0)
    parser.add_argument('--lidar-x', type=float, default=0.0)
    parser.add_argument('--lidar-y', type=float, default=0.0)
    parser.add_argument('--lidar-z', type=float, default=2.4)

    # from carla load map script
    parser.add_argument('osm', nargs='?', help='OpenStreetMap export (.osm)')
    parser.add_argument('--xodr', help='load an existing .xodr instead of converting')
    parser.add_argument('--xodr-out', help='where to save the converted .xodr (default: next to the .osm)')
    parser.add_argument('--convert-only', action='store_true')
    parser.add_argument('--way-types', nargs='+', default=DEFAULT_WAY_TYPES)
    parser.add_argument('--lane-width', type=float, default=3.5, help='used where OSM has no width tag')
    parser.add_argument('--traffic-lights', action='store_true')
    parser.add_argument('--wall-height', type=float, default=1.0,
                    help='walls along road edges (m); 0 = none. Gives LiDAR something to see')
    parser.add_argument('--extra-width', type=float, default=0.6, help='extra road mesh width (m)')
    parser.add_argument('--props-every', type=float, default=0.0,
                    help='spawn a roadside prop every N metres (0 = off)')
    parser.add_argument('--props-offset', type=float, default=1.5, help='metres beyond lane edge')
    parser.add_argument('--seed', type=int, default=0)

    args = parser.parse_args()

    #diagnostic
    print("Way types passed to CARLA:", repr(args.way_types))

    if args.publish_rate <= 0 or args.camera_hz <= 0:
        parser.error('--publish-rate and --camera-hz must be greater than zero')
    if args.width <= 0 or args.height <= 0:
        parser.error('--width and --height must be greater than zero')
    if not 1 <= args.jpeg_quality <= 100:
        parser.error('--jpeg-quality must be between 1 and 100')
    if args.keyboard and args.autopilot:
        parser.error('--keyboard and --autopilot cannot be used together')

    rclpy.init()
    node = None
    try:
        node = CarlaEgoSensors(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.cleanup()
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
