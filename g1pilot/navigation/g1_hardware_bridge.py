#!/usr/bin/env python3
"""Fail-safe bridge from G1Pilot Joy commands to the G1 high-level API."""

import importlib
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class G1HardwareBridge(Node):
    def __init__(self):
        super().__init__('g1_hardware_bridge')
        self.declare_parameter('connect_sdk', False)
        self.declare_parameter('network_interface', 'eth10')
        self.declare_parameter(
            'sdk_path', '/home/dev/unitree_sdk2_python')
        self.declare_parameter('command_timeout', 0.25)
        self.declare_parameter('max_vx', 0.20)
        self.declare_parameter('max_vy', 0.12)
        self.declare_parameter('max_wz', 0.25)
        self.declare_parameter('input_vx_scale', 0.60)
        self.declare_parameter('input_vy_scale', 0.60)
        self.declare_parameter('input_wz_scale', 0.50)

        self.connect_sdk = bool(self.get_parameter('connect_sdk').value)
        self.timeout = float(self.get_parameter('command_timeout').value)
        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_wz = float(self.get_parameter('max_wz').value)
        self.input_scales = (
            float(self.get_parameter('input_vx_scale').value),
            float(self.get_parameter('input_vy_scale').value),
            float(self.get_parameter('input_wz_scale').value),
        )
        self.robot = None
        self.armed = False
        self.last_command = None
        self.command = (0.0, 0.0, 0.0)
        self.stop_sent = False

        if self.connect_sdk:
            sdk_path = str(self.get_parameter('sdk_path').value)
            if sdk_path not in sys.path:
                sys.path.insert(0, sdk_path)
            channel = importlib.import_module(
                'unitree_sdk2py.core.channel')
            loco = importlib.import_module(
                'unitree_sdk2py.g1.loco.g1_loco_client')
            channel.ChannelFactoryInitialize(
                0, str(self.get_parameter('network_interface').value))
            self.robot = loco.LocoClient()
            self.robot.SetTimeout(1.0)
            self.robot.Init()
            self.get_logger().info(
                'G1 SDK connected; motion remains DISARMED.')
        else:
            self.get_logger().warn(
                'Dry-run mode: SDK is not connected and motion is impossible.')

        self.create_subscription(
            Joy, '/g1pilot/auto_joy', self.on_command, 10)
        self.create_subscription(
            Bool, '/g1pilot/hardware_enable', self.on_enable, 10)
        self.create_subscription(
            Bool, '/g1pilot/emergency_stop', self.on_emergency, 10)
        self.enable_pub = self.create_publisher(
            Bool, '/g1pilot/auto_enable', 10)
        self.timer = self.create_timer(0.05, self.control_tick)

    @staticmethod
    def clamp(value, limit):
        return max(-limit, min(limit, value))

    def on_command(self, msg):
        axes = list(msg.axes) + [0.0] * max(0, 3 - len(msg.axes))
        vx = -axes[1] * self.input_scales[0]
        vy = -axes[0] * self.input_scales[1]
        wz = -axes[2] * self.input_scales[2]
        self.command = (
            self.clamp(vx, self.max_vx),
            self.clamp(vy, self.max_vy),
            self.clamp(wz, self.max_wz),
        )
        self.last_command = time.monotonic()

    def on_enable(self, msg):
        requested = bool(msg.data)
        self.armed = requested and self.robot is not None
        if requested and self.robot is None:
            self.get_logger().error(
                'Arm request rejected: SDK is in dry-run mode.')
        if not self.armed:
            self.safe_stop()
        self.get_logger().warn(
            'Hardware motion ARMED.' if self.armed
            else 'Hardware motion DISARMED.')

    def on_emergency(self, msg):
        if msg.data:
            self.armed = False
            self.safe_stop()
            self.get_logger().error('Emergency stop received; motion disarmed.')

    def safe_stop(self):
        if self.robot is not None and not self.stop_sent:
            try:
                self.robot.StopMove()
            except Exception as exc:
                self.get_logger().error(f'StopMove failed: {exc}')
        self.stop_sent = True
        self.command = (0.0, 0.0, 0.0)

    def control_tick(self):
        fresh = (
            self.last_command is not None and
            time.monotonic() - self.last_command <= self.timeout)
        enabled = self.armed and fresh and self.robot is not None
        self.enable_pub.publish(Bool(data=enabled))
        if not enabled:
            self.safe_stop()
            return
        try:
            vx, vy, wz = self.command
            self.robot.SetVelocity(vx, vy, wz, duration=0.15)
            self.stop_sent = False
        except Exception as exc:
            self.get_logger().error(f'Velocity command failed: {exc}')
            self.armed = False
            self.safe_stop()

    def destroy_node(self):
        self.armed = False
        self.safe_stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = G1HardwareBridge()
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
