#!/usr/bin/env python3
"""Unitree G1 RL locomotion in MuJoCo, driven by G1Pilot navigation."""

from pathlib import Path
import json
import math
import os
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
RL_ROOT = ROOT / '.deps' / 'unitree_rl_gym'
sys.path.insert(0, str(ROOT / '.deps' / 'python'))
sys.path.insert(0, str(RL_ROOT))

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
import torch
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState, LaserScan
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


def gravity_orientation(quaternion):
    qw, qx, qy, qz = quaternion
    return np.array([
        2 * (-qz * qx + qw * qy),
        -2 * (qz * qy + qw * qx),
        1 - 2 * (qw * qw + qz * qz),
    ], dtype=np.float32)


class G1RLMujoco(Node):
    def __init__(self):
        super().__init__('g1_rl_mujoco')
        config_path = RL_ROOT / 'deploy/deploy_mujoco/configs/g1.yaml'
        with config_path.open() as stream:
            cfg = yaml.safe_load(stream)

        resolve = lambda value: value.replace('{LEGGED_GYM_ROOT_DIR}', str(RL_ROOT))
        office_root = ROOT / 'simulation' / 'office_reconstruction'
        use_office = (
            os.environ.get('G1PILOT_SCENE', 'office').lower() == 'office'
            and (ROOT / 'simulation/office_scene.xml').exists()
        )
        scene_path = (
            ROOT / 'simulation/office_scene.xml' if use_office
            else ROOT / 'simulation' / 'aws_small_house_scene.xml'
        )
        self.model = mujoco.MjModel.from_xml_path(str(scene_path))
        self.data = mujoco.MjData(self.model)
        geometry_path = (
            office_root / 'office_geometry.json' if use_office
            else ROOT / 'simulation/aws_small_house_geometry.json'
        )
        geometry = json.loads(geometry_path.read_text())
        self.data.qpos[0:2] = geometry['spawn']
        self.dt = float(cfg['simulation_dt'])
        self.decimation = int(cfg['control_decimation'])
        self.model.opt.timestep = self.dt

        self.kp = np.asarray(cfg['kps'], dtype=np.float32)
        self.kd = np.asarray(cfg['kds'], dtype=np.float32)
        self.default_q = np.asarray(cfg['default_angles'], dtype=np.float32)
        self.action_scale = float(cfg['action_scale'])
        self.cmd_scale = np.asarray(cfg['cmd_scale'], dtype=np.float32)
        self.ang_vel_scale = float(cfg['ang_vel_scale'])
        self.q_scale = float(cfg['dof_pos_scale'])
        self.dq_scale = float(cfg['dof_vel_scale'])
        self.action = np.zeros(int(cfg['num_actions']), dtype=np.float32)
        self.target_q = self.default_q.copy()
        self.obs = np.zeros(int(cfg['num_obs']), dtype=np.float32)
        self.cmd = np.zeros(3, dtype=np.float32)
        self.policy = torch.jit.load(resolve(cfg['policy_path']), map_location='cpu')
        self.policy.eval()
        self.counter = 0

        self.create_subscription(Joy, '/g1pilot/auto_joy', self.on_joy, 10)
        self.odom_pub = self.create_publisher(
            Odometry, '/lidar_odometry/pose_fixed', 10
        )
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.enable_pub = self.create_publisher(Bool, '/g1pilot/auto_enable', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.static_scan_pub = self.create_publisher(
            LaserScan, '/scan_static', 10)
        self.tf = TransformBroadcaster(self)
        self.joint_names = [
            self.model.joint(i).name for i in range(1, self.model.njnt)
        ]
        self.scan_period = 0.04
        self.scan_decimation = max(
            1,int(round(self.scan_period/(self.dt*self.decimation))))
        # (center_x, center_y, half_x, half_y); matches g1_complex_scene.xml.
        self.rectangles = [tuple(r) for r in geometry['rectangles']]
        self.dynamic_obstacles = {
            'dynamic_box': (.40, .32),
            'dynamic_pedestrian': (.26, .26),
        }
        self.get_logger().info(
            f"Unitree G1 RL policy ready in {scene_path.name}: "
            "500 Hz physics, 50 Hz policy."
        )

    def on_joy(self, msg):
        axes = list(msg.axes) + [0.0] * max(0, 3 - len(msg.axes))
        # Reverse Nav2Point's normalized Joy encoding.
        self.cmd[:] = [-axes[1] * 0.6, -axes[0] * 0.6, -axes[2] * 0.5]

    def control_step(self):
        self.update_dynamic_obstacles()
        q = self.data.qpos[7:]
        dq = self.data.qvel[6:]
        tau = (self.target_q - q) * self.kp - dq * self.kd
        self.data.ctrl[:] = tau
        mujoco.mj_step(self.model, self.data)
        self.counter += 1

        if self.counter % self.decimation == 0:
            phase = (self.counter * self.dt % 0.8) / 0.8
            self.obs[:3] = self.data.qvel[3:6] * self.ang_vel_scale
            self.obs[3:6] = gravity_orientation(self.data.qpos[3:7])
            self.obs[6:9] = self.cmd * self.cmd_scale
            self.obs[9:21] = (q - self.default_q) * self.q_scale
            self.obs[21:33] = dq * self.dq_scale
            self.obs[33:45] = self.action
            self.obs[45:47] = [math.sin(2 * math.pi * phase),
                               math.cos(2 * math.pi * phase)]
            with torch.inference_mode():
                tensor = torch.from_numpy(self.obs).unsqueeze(0)
                self.action = self.policy(tensor).numpy().squeeze()
            self.target_q = self.action * self.action_scale + self.default_q
            self.publish_state()
            if (self.counter // self.decimation) % self.scan_decimation == 0:
                self.publish_scan()

    def update_dynamic_obstacles(self):
        t=self.counter*self.dt
        poses={
            'dynamic_box': (2.2*math.sin(0.45*t), -2.2, .45),
            'dynamic_pedestrian': (-3.2+6.4*((0.12*t) % 1.0), .8, .75),
        }
        for name,(x,y,z) in poses.items():
            body_id=mujoco.mj_name2id(
                self.model,mujoco.mjtObj.mjOBJ_BODY,name)
            mocap_id=self.model.body_mocapid[body_id]
            self.data.mocap_pos[mocap_id]=[x,y,z]

    @staticmethod
    def ray_rectangle(ox, oy, dx, dy, rect):
        cx, cy, hx, hy = rect
        tmin, tmax = 0.0, 8.0
        for origin, direction, low, high in (
            (ox, dx, cx - hx, cx + hx), (oy, dy, cy - hy, cy + hy)
        ):
            if abs(direction) < 1e-9:
                if origin < low or origin > high:
                    return math.inf
                continue
            a, b = (low - origin) / direction, (high - origin) / direction
            if a > b:
                a, b = b, a
            tmin, tmax = max(tmin, a), min(tmax, b)
            if tmin > tmax:
                return math.inf
        return tmin if tmin >= .10 else tmax

    def publish_scan(self):
        q = self.data.qpos
        yaw = math.atan2(
            2 * (q[3] * q[6] + q[4] * q[5]),
            1 - 2 * (q[5] * q[5] + q[6] * q[6]),
        )
        rectangles=list(self.rectangles)
        for name,(hx,hy) in self.dynamic_obstacles.items():
            body_id=mujoco.mj_name2id(
                self.model,mujoco.mjtObj.mjOBJ_BODY,name)
            mocap_id=self.model.body_mocapid[body_id]
            x,y=self.data.mocap_pos[mocap_id,:2]
            rectangles.append((float(x),float(y),hx,hy))

        stamp=self.get_clock().now().to_msg()
        self.static_scan_pub.publish(
            self.make_scan(q,yaw,self.rectangles,stamp))
        self.scan_pub.publish(self.make_scan(q,yaw,rectangles,stamp))

    def make_scan(self,q,yaw,rectangles,stamp):
        count=360
        scan=LaserScan()
        scan.header.stamp=stamp
        scan.header.frame_id='base_scan'
        scan.angle_min=-math.pi
        scan.angle_max=math.pi
        scan.angle_increment=2*math.pi/count
        scan.time_increment=0.0
        scan.scan_time=self.scan_period
        scan.range_min=0.10
        scan.range_max=8.0
        angles=yaw+scan.angle_min+np.arange(count)*scan.angle_increment
        dx=np.cos(angles)[:,None]
        dy=np.sin(angles)[:,None]
        rect=np.asarray(rectangles,dtype=np.float64)
        low_x=(rect[:,0]-rect[:,2])[None,:]
        high_x=(rect[:,0]+rect[:,2])[None,:]
        low_y=(rect[:,1]-rect[:,3])[None,:]
        high_y=(rect[:,1]+rect[:,3])[None,:]
        with np.errstate(divide='ignore',invalid='ignore'):
            tx1=(low_x-float(q[0]))/dx
            tx2=(high_x-float(q[0]))/dx
            ty1=(low_y-float(q[1]))/dy
            ty2=(high_y-float(q[1]))/dy
        entry=np.maximum.reduce([
            np.minimum(tx1,tx2),np.minimum(ty1,ty2),
            np.zeros_like(tx1)])
        exit_=np.minimum.reduce([
            np.maximum(tx1,tx2),np.maximum(ty1,ty2),
            np.full_like(tx1,scan.range_max)])
        hits=(entry<=exit_)&(exit_>=scan.range_min)
        distance=np.where(hits,np.where(entry>=scan.range_min,entry,exit_),np.inf)
        ranges=np.min(distance,axis=1)
        ranges=np.where(ranges<=scan.range_max,ranges,np.inf)
        scan.ranges=ranges.astype(np.float32).tolist()
        return scan

    def publish_state(self):
        stamp = self.get_clock().now().to_msg()
        q = self.data.qpos
        yaw = math.atan2(
            2 * (q[3] * q[6] + q[4] * q[5]),
            1 - 2 * (q[5] * q[5] + q[6] * q[6]),
        )
        sy, cy = math.sin(yaw / 2), math.cos(yaw / 2)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = float(q[0])
        odom.pose.pose.position.y = float(q[1])
        odom.pose.pose.orientation.w = cy
        odom.pose.pose.orientation.z = sy
        odom.twist.twist.linear.x = float(self.data.qvel[0])
        odom.twist.twist.linear.y = float(self.data.qvel[1])
        odom.twist.twist.angular.z = float(self.data.qvel[5])
        self.odom_pub.publish(odom)
        self.enable_pub.publish(Bool(data=True))

        joint = JointState()
        joint.header.stamp = stamp
        joint.name = self.joint_names
        joint.position = self.data.qpos[7:].tolist()
        joint.velocity = self.data.qvel[6:].tolist()
        joint.effort = self.data.ctrl[:].tolist()
        self.joint_pub.publish(joint)

        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = odom.child_frame_id
        transform.transform.translation.x = float(q[0])
        transform.transform.translation.y = float(q[1])
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(transform)

        # Preserve the physical body's height/roll/pitch below the planar frame.
        # q_relative = inverse(q_yaw) * q_body, in MuJoCo wxyz ordering.
        rw = cy * q[3] + sy * q[6]
        rx = cy * q[4] + sy * q[5]
        ry = cy * q[5] - sy * q[4]
        rz = cy * q[6] - sy * q[3]
        body_tf = TransformStamped()
        body_tf.header.stamp = stamp
        body_tf.header.frame_id = 'base_footprint'
        body_tf.child_frame_id = 'base_link'
        body_tf.transform.translation.z = float(q[2])
        body_tf.transform.rotation.w = float(rw)
        body_tf.transform.rotation.x = float(rx)
        body_tf.transform.rotation.y = float(ry)
        body_tf.transform.rotation.z = float(rz)
        self.tf.sendTransform(body_tf)

        pelvis_tf = TransformStamped()
        pelvis_tf.header.stamp = stamp
        pelvis_tf.header.frame_id = 'base_link'
        pelvis_tf.child_frame_id = 'pelvis'
        pelvis_tf.transform.rotation.w = 1.0
        self.tf.sendTransform(pelvis_tf)

        laser_tf = TransformStamped()
        laser_tf.header.stamp = stamp
        laser_tf.header.frame_id = 'base_footprint'
        laser_tf.child_frame_id = 'base_scan'
        laser_tf.transform.translation.z = 0.45
        laser_tf.transform.rotation.w = 1.0
        self.tf.sendTransform(laser_tf)


def main():
    rclpy.init()
    bridge = G1RLMujoco()
    try:
        with mujoco.viewer.launch_passive(bridge.model, bridge.data) as viewer:
            while viewer.is_running() and rclpy.ok():
                start = time.monotonic()
                rclpy.spin_once(bridge, timeout_sec=0.0)
                bridge.control_step()
                viewer.sync()
                time.sleep(max(0.0, bridge.dt - (time.monotonic() - start)))
    finally:
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
