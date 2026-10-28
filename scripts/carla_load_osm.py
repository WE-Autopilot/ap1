#!/usr/bin/env python3
"""
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
import os
import random

import carla

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


def convert(osm_path, args):
    with open(osm_path, encoding='utf-8') as f:
        osm_data = f.read()
    s = carla.Osm2OdrSettings()
    s.set_osm_way_types(args.way_types)
    s.default_lane_width = args.lane_width
    s.generate_traffic_lights = args.traffic_lights
    s.all_junctions_with_traffic_lights = False
    if hasattr(s, 'center_map'):
        s.center_map = True  # map centre -> CARLA origin (geo-reference kept in the .xodr header)
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


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('osm', nargs='?', help='OpenStreetMap export (.osm)')
    p.add_argument('--xodr', help='load an existing .xodr instead of converting')
    p.add_argument('--xodr-out', help='where to save the converted .xodr (default: next to the .osm)')
    p.add_argument('--convert-only', action='store_true')
    p.add_argument('--host', default='localhost')
    p.add_argument('--port', type=int, default=2000)
    p.add_argument('--way-types', nargs='+', default=DEFAULT_WAY_TYPES)
    p.add_argument('--lane-width', type=float, default=3.5, help='used where OSM has no width tag')
    p.add_argument('--traffic-lights', action='store_true')
    p.add_argument('--wall-height', type=float, default=1.0,
                   help='walls along road edges (m); 0 = none. Gives LiDAR something to see')
    p.add_argument('--extra-width', type=float, default=0.6, help='extra road mesh width (m)')
    p.add_argument('--props-every', type=float, default=0.0,
                   help='spawn a roadside prop every N metres (0 = off)')
    p.add_argument('--props-offset', type=float, default=1.5, help='metres beyond lane edge')
    p.add_argument('--seed', type=int, default=0)
    args = p.parse_args()

    if args.xodr:
        with open(args.xodr, encoding='utf-8') as f:
            xodr = f.read()
    elif args.osm:
        xodr = convert(args.osm, args)
    else:
        p.error('give an .osm file or --xodr')
    if args.convert_only:
        return

    client = carla.Client(args.host, args.port)
    client.set_timeout(120.0)  # mesh generation can take a while
    params = carla.OpendriveGenerationParameters(
        vertex_distance=2.0,
        max_road_length=500.0,
        wall_height=args.wall_height,
        additional_width=args.extra_width,
        smooth_junctions=True,
        enable_mesh_visibility=True,
        enable_pedestrian_navigation=True)
    world = client.generate_opendrive_world(xodr, params)
    cmap = world.get_map()
    wps = cmap.generate_waypoints(2.0)
    print(f'Loaded OpenDRIVE world: {len(wps)} waypoints @2 m, '
          f'{len(cmap.get_spawn_points())} spawn points')

    # Geo-reference: needed later to line CARLA's frame up with the Lanelet2 map (lat/lon).
    geo = cmap.transform_to_geolocation(carla.Location(0, 0, 0))
    print(f'CARLA origin = lat {geo.latitude:.7f}, lon {geo.longitude:.7f}')

    if args.props_every > 0:
        add_props(client, world, args.props_every, args.props_offset, args.seed)

    # Put the spectator above the map so you can see it in the CARLA window.
    xs = [w.transform.location.x for w in wps] or [0.0]
    ys = [w.transform.location.y for w in wps] or [0.0]
    cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
    h = max(max(xs) - min(xs), max(ys) - min(ys), 50.0)
    world.get_spectator().set_transform(
        carla.Transform(carla.Location(cx, cy, h), carla.Rotation(pitch=-90)))
    print('Spectator moved above the map. Now run carla_ros2_republisher.py (no --town).')


if __name__ == '__main__':
    main()