"""Writes one CARLA tick at a time into a raw rosbag2 bag, stamped with sim time."""
import os
from datetime import date

import numpy as np

from carla_frames import (MAP_FRAME, ODOM_TOPIC, VEHICLE_FRAME, body_rate, ns_to_stamp,
                          quat_from_rpy, ros_pose, rot_matrix, topic_metadata)


def bag_base(map_name, lidar_path, lidar_cfg, imu_path, imu_cfg, short=False):
    """Bag name without the _raw or _filtered ending; see "Quick start" in the README."""
    def tag(path, cfg):
        stem = os.path.splitext(os.path.basename(path))[0]
        return f'{stem.replace("_", "")}-v{cfg.get("version", 0)}'
    return (f'{date.today():%Y-%m-%d}_{map_name.lower()}_'
            f'{tag(lidar_path, lidar_cfg)}_{tag(imu_path, imu_cfg)}' + '_short' * short)


def latched_metadata(name, type_name):
    """Topic metadata with transient_local QoS, so /tf_static latches on playback."""
    import rosbag2_py
    qos_cls = getattr(rosbag2_py, 'QoS', None) or rosbag2_py._storage.QoS  # Jazzy has it only here
    qos = qos_cls(1).reliable().transient_local()
    return rosbag2_py.TopicMetadata(id=0, name=name, type=type_name, serialization_format='cdr',
                                    offered_qos_profiles=[qos])


class BagWriter:
    """Builds the ROS messages for one tick and writes them, stamped with sim time."""

    def __init__(self, uri, lidar_cfg, imu_cfg):
        import rosbag2_py
        self.lidar_topic, self.lidar_frame = lidar_cfg['topics']['input'], lidar_cfg['frame_id']
        self.imu_topic, self.imu_frame = imu_cfg['topics']['input'], imu_cfg['frame_id']
        types = {
            '/clock': 'rosgraph_msgs/msg/Clock',
            self.lidar_topic: 'sensor_msgs/msg/PointCloud2',
            self.imu_topic: 'sensor_msgs/msg/Imu',
            ODOM_TOPIC: 'nav_msgs/msg/Odometry',
            '/tf': 'tf2_msgs/msg/TFMessage',
            '/tf_static': 'tf2_msgs/msg/TFMessage',
        }
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(rosbag2_py.StorageOptions(uri=uri, storage_id='mcap'),
                         rosbag2_py.ConverterOptions('cdr', 'cdr'))
        for name, type_name in types.items():
            meta = latched_metadata if name == '/tf_static' else topic_metadata
            self.writer.create_topic(meta(name, type_name))
        self.first_ns = None
        self.prev_q = None

    def _write(self, topic, msg, t_ns):
        from rclpy.serialization import serialize_message
        self.writer.write(topic, serialize_message(msg), t_ns)

    @staticmethod
    def _transform(stamp, parent, child, xyz, q):
        from geometry_msgs.msg import TransformStamped
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, parent, child
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, xyz)
        r = tf.transform.rotation
        r.x, r.y, r.z, r.w = map(float, q)
        return tf

    def write_static(self, t_ns, mounts):
        from tf2_msgs.msg import TFMessage
        stamp = ns_to_stamp(t_ns)
        tfs = [self._transform(stamp, VEHICLE_FRAME, frame, mounts[frame]['xyz'],
                               quat_from_rpy(*np.radians(mounts[frame]['rpy'])))
               for frame in (self.lidar_frame, self.imu_frame)]
        self._write('/tf_static', TFMessage(transforms=tfs), t_ns)

    def write_tick(self, t_ns, ego, lidar, imu, dt):
        """ego: carla.ActorSnapshot; lidar/imu: this tick's CARLA measurements."""
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Imu, PointCloud2, PointField
        from tf2_msgs.msg import TFMessage
        stamp = ns_to_stamp(t_ns)
        self.first_ns = t_ns if self.first_ns is None else self.first_ns
        self._write('/clock', Clock(clock=stamp), t_ns)

        # Ground truth: map -> vehicle, twist in the vehicle frame.
        xyz, q = ros_pose(ego.get_transform())
        self._write('/tf', TFMessage(transforms=[
            self._transform(stamp, MAP_FRAME, VEHICLE_FRAME, xyz, q)]), t_ns)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, MAP_FRAME, VEHICLE_FRAME
        p, o = odom.pose.pose.position, odom.pose.pose.orientation
        p.x, p.y, p.z = map(float, xyz)
        o.x, o.y, o.z, o.w = map(float, q)
        vel = ego.get_velocity()
        lin = rot_matrix(q).T @ np.array([vel.x, -vel.y, vel.z])
        ang = body_rate(self.prev_q, q, dt) if self.prev_q is not None else np.zeros(3)
        self.prev_q = q
        tw = odom.twist.twist
        tw.linear.x, tw.linear.y, tw.linear.z = map(float, lin)
        tw.angular.x, tw.angular.y, tw.angular.z = map(float, ang)
        self._write(ODOM_TOPIC, odom, t_ns)

        # Every lidar slice, in the sensor frame, y flipped to ROS.
        pts = np.frombuffer(lidar.raw_data, dtype=np.float32).reshape(-1, 4).copy()
        pts[:, 1] *= -1.0
        cloud = PointCloud2()
        cloud.header.stamp, cloud.header.frame_id = stamp, self.lidar_frame
        cloud.height, cloud.width = 1, len(pts)
        cloud.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                        for i, n in enumerate(('x', 'y', 'z', 'intensity'))]
        cloud.is_bigendian, cloud.point_step, cloud.row_step = False, 16, 16 * len(pts)
        cloud.is_dense = True
        cloud.data = pts.tobytes()
        self._write(self.lidar_topic, cloud, t_ns)

        # IMU in the sensor frame; orientation is the sensor's true world pose.
        msg = Imu()
        msg.header.stamp, msg.header.frame_id = stamp, self.imu_frame
        a, g = imu.accelerometer, imu.gyroscope
        _, q_imu = ros_pose(imu.transform)
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = a.x, -a.y, a.z
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = -g.x, g.y, -g.z
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = map(float, q_imu)
        # covariances stay 0: exact, and orientation_covariance[0] != -1 marks it present
        self._write(self.imu_topic, msg, t_ns)

    def close(self):
        self.writer = None  # rosbag2 finalizes the bag when the writer is destroyed
