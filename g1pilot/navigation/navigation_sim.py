#!/usr/bin/env python3
"""Small planar simulator for exercising the navigation stack without a robot."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


class NavigationSimulator(Node):
    def __init__(self):
        super().__init__('navigation_sim')
        self.declare_parameter('rate', 50.0)
        self.declare_parameter('vx_limit', 0.6)
        self.declare_parameter('vy_limit', 0.6)
        self.declare_parameter('wz_limit', 0.5)
        self.declare_parameter('odom_topic', '/lidar_odometry/pose_fixed')
        self.declare_parameter('joy_topic', '/g1pilot/auto_joy')
        self.declare_parameter('auto_enable_topic', '/g1pilot/auto_enable')

        self.rate = float(self.get_parameter('rate').value)
        self.vx_limit = float(self.get_parameter('vx_limit').value)
        self.vy_limit = float(self.get_parameter('vy_limit').value)
        self.wz_limit = float(self.get_parameter('wz_limit').value)
        self.x = self.y = self.yaw = 0.0
        self.vx = self.vy = self.wz = 0.0

        self.create_subscription(
            Joy, self.get_parameter('joy_topic').value, self.joy_callback, 10
        )
        self.odom_pub = self.create_publisher(
            Odometry, self.get_parameter('odom_topic').value, 10
        )
        self.enable_pub = self.create_publisher(
            Bool, self.get_parameter('auto_enable_topic').value, 10
        )
        self.tf = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / self.rate, self.step)
        self.get_logger().info('Navigation demo simulator started at (0, 0, 0).')

    def joy_callback(self, msg):
        axes = list(msg.axes) + [0.0] * max(0, 3 - len(msg.axes))
        self.vx = -float(axes[1]) * self.vx_limit
        self.vy = -float(axes[0]) * self.vy_limit
        self.wz = -float(axes[2]) * self.wz_limit

    def step(self):
        dt = 1.0 / self.rate
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (c * self.vx - s * self.vy) * dt
        self.y += (s * self.vx + c * self.vy) * dt
        self.yaw += self.wz * dt

        stamp = self.get_clock().now().to_msg()
        qz, qw = math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.wz
        self.odom_pub.publish(odom)
        self.enable_pub.publish(Bool(data=True))

        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = odom.child_frame_id
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
