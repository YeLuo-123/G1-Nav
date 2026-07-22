#!/usr/bin/env python3
"""Adapt G1 odometry and Mid-360 PointCloud2 to G1Pilot interfaces."""

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class G1SensorBridge(Node):
    def __init__(self):
        super().__init__('g1_sensor_bridge')
        self.declare_parameter('odom_topic', '/dog_odom')
        self.declare_parameter(
            'cloud_topic', '/utlidar/cloud_livox_mid360')
        self.declare_parameter('min_height', -0.45)
        self.declare_parameter('max_height', 0.80)
        self.declare_parameter('range_min', 0.20)
        self.declare_parameter('range_max', 8.0)
        self.declare_parameter('scan_rate', 10.0)
        self.declare_parameter('laser_height', 0.65)
        self.min_height = float(self.get_parameter('min_height').value)
        self.max_height = float(self.get_parameter('max_height').value)
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.scan_period = 1.0 / float(
            self.get_parameter('scan_rate').value)
        self.last_scan_ns = 0

        qos = QoSProfile(depth=5)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self.on_odom, qos)
        self.create_subscription(
            PointCloud2, self.get_parameter('cloud_topic').value,
            self.on_cloud, qos)
        self.odom_pub = self.create_publisher(
            Odometry, '/lidar_odometry/pose_fixed', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.publish_laser_tf()

    def publish_laser_tf(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_footprint'
        transform.child_frame_id = 'base_scan'
        transform.transform.translation.z = float(
            self.get_parameter('laser_height').value)
        transform.transform.rotation.w = 1.0
        self.static_tf.sendTransform(transform)

    def on_odom(self, msg):
        out = Odometry()
        out.header = msg.header
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_footprint'
        out.pose = msg.pose
        out.pose.pose.position.z = 0.0
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        out.pose.pose.orientation.x = 0.0
        out.pose.pose.orientation.y = 0.0
        out.pose.pose.orientation.z = math.sin(yaw / 2.0)
        out.pose.pose.orientation.w = math.cos(yaw / 2.0)
        out.twist = msg.twist
        self.odom_pub.publish(out)

        transform = TransformStamped()
        transform.header = out.header
        transform.child_frame_id = out.child_frame_id
        transform.transform.translation.x = out.pose.pose.position.x
        transform.transform.translation.y = out.pose.pose.position.y
        transform.transform.rotation = out.pose.pose.orientation
        self.tf.sendTransform(transform)

    def on_cloud(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_scan_ns < int(self.scan_period * 1e9):
            return
        self.last_scan_ns = now_ns
        try:
            points = point_cloud2.read_points_numpy(
                msg, field_names=['x', 'y', 'z'], skip_nans=True)
            points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        except Exception as exc:
            self.get_logger().error(f'Point cloud conversion failed: {exc}')
            return
        if not len(points):
            return
        height = points[:, 2]
        distance = np.hypot(points[:, 0], points[:, 1])
        valid = (
            (height >= self.min_height) & (height <= self.max_height) &
            (distance >= self.range_min) & (distance <= self.range_max))
        points, distance = points[valid], distance[valid]

        count = 720
        ranges = np.full(count, np.inf, dtype=np.float32)
        if len(points):
            angles = np.arctan2(points[:, 1], points[:, 0])
            indices = np.floor((angles + math.pi) * count /
                               (2.0 * math.pi)).astype(np.int32)
            indices = np.clip(indices, 0, count - 1)
            np.minimum.at(ranges, indices, distance)

        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = 'base_scan'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2.0 * math.pi / count
        scan.scan_time = self.scan_period
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges.tolist()
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = G1SensorBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
