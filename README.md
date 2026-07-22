# G1 Nav：Unitree G1 MuJoCo + ROS 2 导航仿真

本仓库只保留 Unitree G1 的 MuJoCo 动力学仿真与 ROS 2 导航功能。机器人使用
Unitree RL 行走策略驱动腿部关节，通过 MuJoCo 接触动力学产生实际位移，不直接
修改浮动基座，因此不会出现“漂移式导航”。

## 功能

- Unitree G1 12-DoF 强化学习行走策略；
- MuJoCo 500 Hz 物理仿真与 50 Hz 策略推理；
- 360° 静态/动态二维激光模拟；
- `slam_toolbox` 在线建图；
- 基于膨胀地图的 Dijkstra 全局规划；
- 路径平滑后的二次碰撞校验；
- 实时局部动态避障；
- RViz 地图、膨胀层、激光、路径与 G1 模型可视化；
- 办公室场景与 AWS RoboMaker Small House 测试场景。

## 系统结构

```text
RViz 导航目标
      ↓
Dijkstra 全局规划（/map_inflated）
      ↓
nav2point 路径跟踪与局部避障（/scan）
      ↓
Unitree G1 RL 行走策略
      ↓
MuJoCo 关节力矩与接触动力学
      ↓
里程计、关节状态、静态/动态激光
      ↓
slam_toolbox 在线地图与重新规划
```

## 仓库内容

```text
g1pilot/navigation/               ROS 2 规划与路径跟踪节点
tools/g1_rl_mujoco_navigation.py  MuJoCo、RL 行走、激光和里程计
launch/navigation_mapping.launch.py
                                  SLAM、规划、跟踪、G1 模型和 RViz
config/slam_toolbox.yaml          SLAM 参数
config/navigation_mapping.rviz    RViz 配置
simulation/                       MuJoCo 场景与导航几何
description_files/                RViz 使用的 G1 URDF 和网格
docs/REPRODUCE_ZH.md              详细复现说明
```

机械臂控制、遥操作、真机状态、Livox/MOLA、重建原始数据、构建产物和本地 Python
依赖不属于该精简仓库。

## 环境要求

- Ubuntu 22.04；
- ROS 2 Humble；
- Python 3.10；
- 支持 OpenGL 的桌面环境；
- 推荐 NVIDIA GPU，但当前策略可在 CPU 上推理。

安装 ROS 2 依赖：

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2 \
  ros-humble-slam-toolbox
```

## 克隆与依赖

必须同时获取 Git submodule：

```bash
git clone --recurse-submodules git@github.com:YeLuo-123/G1-Nav.git
cd G1-Nav
```

如果已经普通克隆：

```bash
git submodule update --init --recursive
```

将仿真 Python 依赖安装到项目本地目录：

```bash
mkdir -p .deps/python
/usr/bin/python3 -m pip install --target .deps/python \
  mujoco numpy pyyaml torch
```

第三方依赖以 submodule 形式固定版本：

- `unitreerobotics/unitree_rl_gym`：G1 模型、部署配置和行走策略；
- `aws-robotics/aws-robomaker-small-house-world`：复杂测试场景来源；
- `google-deepmind/mujoco_menagerie`：官方 MuJoCo 模型资源。

## 构建

```bash
cd /path/to/G1-Nav
source /opt/ros/humble/setup.bash
colcon build --packages-select g1pilot --symlink-install
source install/setup.bash
```

## 启动完整导航仿真

### 终端 1：MuJoCo 与 G1 行走

```bash
cd /path/to/G1-Nav
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/.deps/python:${PYTHONPATH:-}"
export G1PILOT_SCENE=office
/usr/bin/python3 tools/g1_rl_mujoco_navigation.py
```

办公室场景不存在或希望使用 AWS 场景时：

```bash
export G1PILOT_SCENE=aws
```

### 终端 2：SLAM、规划、避障与 RViz

```bash
cd /path/to/G1-Nav
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch g1pilot navigation_mapping.launch.py
```

在 RViz 中使用 **2D Goal Pose**，并在已经建图的白色自由区域设置目标。目标位于
未知区域、障碍物、0.40 m 膨胀层或地图外时，规划器会拒绝执行。

也可以通过命令行发布目标：

```bash
ros2 topic pub --once /g1pilot/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 3.122, y: -4.034}, orientation: {w: 1.0}}}"
```

## 主要话题

| 话题 | 作用 |
|---|---|
| `/scan_static` | 只含静态障碍物，供 SLAM 使用 |
| `/scan` | 包含动态障碍物，供局部避障和 RViz 使用 |
| `/lidar_odometry/pose_fixed` | MuJoCo 实际机器人里程计 |
| `/joint_states` | G1 关节状态 |
| `/map` | 在线 SLAM 地图 |
| `/map_inflated` | 0.40 m 安全膨胀地图 |
| `/g1pilot/goal` | 导航目标 |
| `/g1pilot/path` | 全局安全路径 |
| `/g1pilot/auto_joy` | 发送给 G1 RL 策略的速度指令 |

## 快速检查

```bash
ros2 topic hz /scan
ros2 topic echo /lidar_odometry/pose_fixed --once
ros2 topic echo /map --once --field info
ros2 topic echo /map_inflated --once --field info
ros2 topic echo /g1pilot/path --once
```

## 安全规划与动态避障

全局规划器默认禁止未知区域，并在 0.40 m 膨胀地图上搜索。路径快捷化和平滑后会
重新逐段检查碰撞；若平滑结果不安全，则退回原始安全路径。规划器不会使用穿过
障碍物或膨胀层的直线作为降级方案。

局部控制器使用 `/scan` 在 1.20 m 范围内计算障碍物斥力，前方距离小于 0.45 m
时停止前进，并向空间更大的一侧侧移和转向。动态障碍物不会写入长期 SLAM 地图，
避免产生幽灵障碍。

## 无 MuJoCo 的导航逻辑测试

如只需测试规划与路径跟踪逻辑：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch g1pilot navigation_demo.launch.py
```

该模式会直接积分平面机器人位置，仅用于算法单元测试，不用于运动真实性或
Sim2Real 验证。

## 文档

- [详细复现与启动说明](docs/REPRODUCE_ZH.md)
- [导航系统技术说明](docs/OFFICE_RECONSTRUCTION_TECHNICAL_REPORT_ZH.md)

## 第三方资源与许可证

本项目主体采用 [BSD-3-Clause](LICENSE)。AWS Small House 场景遵循仓库内
`simulation/AWS_SMALL_HOUSE_LICENSE`。各 submodule 遵循其上游项目许可证。

## 部署到真机

项目中包含辅助脚本，用于将 Git 已跟踪的导航源码同步到 G1 并构建。实机地址
已通过 Wi-Fi 确认为 `dev@10.11.32.162`：

- 脚本路径： [scripts/deploy_to_fangqi.sh](scripts/deploy_to_fangqi.sh)

用法示例：

```bash
# 只部署和构建，不启动运动控制
./scripts/deploy_to_fangqi.sh dev@10.11.32.162 \
  --remote-ws ~/g1nav_ws --ros2 humble

# 实机上进行无运动输出验证（默认 connect_sdk:=false）
ssh dev@10.11.32.162
source /opt/ros/humble/setup.bash
source ~/g1nav_ws/install/setup.bash
export ROS_DOMAIN_ID=50
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DISABLE_DAEMON=1
ros2 launch g1pilot navigation_hardware.launch.py \
  connect_sdk:=false use_rviz:=false
```

注意事项：

- 请先确保可以通过 SSH 访问真机（使用密钥或密码）。
- 目标主机需已安装相应的 ROS 2 发行版（例如 humble）与 `colcon` 构建工具。
- 脚本只同步 Git 跟踪文件，并在远端运行 `colcon build`；不会删除或覆盖实机其他工作区。
- `connect_sdk` 默认为 `false`，此时不连接运动 SDK，不可能输出运动指令。
- 即使连接 SDK，机器人也默认未解锁，必须显式发布 `/g1pilot/hardware_enable`。
- 实机首次运动前必须准备物理急停、安全员和清空的测试区域。

实机运动控制使用 `eth10` 上的 Unitree 高层接口；ROS 2 和远程管理使用
`wlan0`。不要将运动 DDS 接口改成 Wi-Fi 网卡。

实机导航复用 G1 已有的 Mid-360/Faster-LIO 感知链：

- `/localization/odom`：地图坐标系定位；
- `/cloud_registered_body`：机器人坐标系局部点云；
- `/g1_lidar_slam/map`：二维占据地图。

启动导航前先启动不包含运动控制的雷达建图链：

```bash
sudo bash /home/dev/g1_lidar_mapping_docker.sh start
```
