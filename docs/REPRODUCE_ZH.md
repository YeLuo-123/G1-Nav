# G1Pilot 导航复现

## 无真机闭环验证

本模式只验证导航数据链路，不会连接或控制 Unitree G1。

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select g1pilot --symlink-install
source install/setup.bash
ros2 launch g1pilot navigation_demo.launch.py
```

另开终端，加载环境并发送目标点：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic pub --once /g1pilot/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}"
```

检查规划路径和机器人位置：

```bash
ros2 topic echo --once /g1pilot/path
ros2 topic echo --once /lidar_odometry/pose_fixed
```

### MuJoCo 可视化

保持导航仿真运行，另开终端：

```bash
cd /home/fq/g1pilot-main
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/python3 tools/mujoco_navigation_viewer.py
```

窗口加载的是 Google DeepMind 官方维护的 MuJoCo Menagerie
`unitree_g1/scene.xml`（29-DoF，BSD-3-Clause）。G1 使用官方 `stand`
关键帧保持站姿，浮动基座跟随 `/lidar_odometry/pose_fixed` 的平面位姿。

官方模型资源位于：

```text
.deps/mujoco_menagerie/unitree_g1/
```

### 动力学行走（用于 Sim2Real）

不要使用 `navigation_demo.launch.py` 中直接积分基座位姿的演示节点。行走模式使用
宇树官方 `unitree_rl_gym` 的 G1 预训练策略：

```bash
# 终端1：MuJoCo 接触动力学和 G1 行走策略
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/python3 tools/g1_rl_mujoco_navigation.py

# 终端2：地图、规划和路径跟踪（不启动 navigation_sim）
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch g1pilot navigation_walk.launch.py
```

此模式不直接修改浮动基座。策略输出12个腿部关节目标，PD控制器产生关节力矩，
MuJoCo 接触动力学决定机器人实际位姿，再发布里程计供导航闭环使用。

### 复杂场景、SLAM与RViz

默认复杂场景来自AWS RoboMaker Small House World的官方占据地图（MIT-0），
包含住宅多房间、走廊、门洞和家具轮廓。导入脚本将地图占据像素合并为MuJoCo
碰撞体，并生成激光共用的几何清单：

```bash
/usr/bin/python3 tools/import_aws_small_house.py
```

生成文件为 `simulation/aws_small_house_scene.xml`、
`simulation/aws_small_house_geometry.json` 和地图纹理
`simulation/aws_small_house_map.png`。MuJoCo节点发布360线二维激光，
`slam_toolbox` 使用 `/scan`、`odom→base_footprint` 在线生成 `/map`。

```bash
# 终端1：复杂场景、G1动力学、行走策略和模拟激光
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/python3 tools/g1_rl_mujoco_navigation.py

# 终端2：SLAM、规划、路径跟踪、RobotModel与RViz
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch g1pilot navigation_mapping.launch.py
```

在RViz中选择 `2D Goal Pose` 并点击地图即可发布 `/g1pilot/goal`。显示项包括：

- `Online SLAM Map`：在线占据栅格；
- `Simulated LiDAR`：红色激光点；
- `Planned Path`：绿色Dijkstra路径；
- `Unitree G1`：由 `/joint_states` 和TF驱动的机器人模型；
- `Goal`、`Current Waypoint`：目标和当前跟踪点。

局部避障订阅 `/scan`，在1.2 m范围内叠加斥力，前方距离不足时减速，
小于0.45 m时停止前进并向空间更大的一侧侧移和转向。场景中的移动箱体与
移动圆柱由MuJoCo mocap刚体驱动，参与激光检测和物理碰撞。

`/scan_static` 仅包含静态场景并供SLAM使用；`/scan` 同时包含动态物体并供
局部避障与RViz使用。该分层避免移动物体在长期地图中形成幽灵障碍。

规划器将0.4 m膨胀半径处理后的地图发布到 `/map_inflated`。RViz中的
`Inflated Costmap` 图层用于显示机器人安全半径；SLAM地图更新后，规划器以
1 Hz对当前目标重新规划。

全局规划默认禁止未知区域，并禁止斜向穿越相邻障碍物的墙角。路径快捷化和
Catmull-Rom平滑后会再次逐段检查膨胀地图；若平滑路径不安全则退回未平滑
安全路径。目标位于障碍物、未知区、地图外或膨胀层时会发布空路径并拒绝执行，
不会再退化成穿越障碍物的直线。

### 实拍办公室 COLMAP 场景

导入或更新 `data/` 后先重建：

```bash
cd /home/fq/g1pilot-main
PYTHONPATH="$PWD/.deps/python" /usr/bin/python3 tools/reconstruct_office_colmap.py
```

工具会生成 `simulation/office_scene.xml`、彩色三维点云
`simulation/office_reconstruction/office_sparse_colored.ply`、占据图和碰撞几何。
当前 `depth_pro_results` 是由 RGB 灰度合成的伪深度，因此工具会拒绝使用它，
只使用 COLMAP 的真实位姿与三维点。绝对尺度暂按 1.55 m 拍摄相机高度标定。

启动命令与上一节相同；办公室场景存在时会默认加载。也可在终端1显式设置：

```bash
export G1PILOT_SCENE=office
/usr/bin/python3 tools/g1_rl_mujoco_navigation.py
```

当前验证目标点可用以下命令发送（也可直接使用 RViz 的 `2D Goal Pose`）：

```bash
ros2 topic pub --once /g1pilot/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 3.122, y: -4.034}, orientation: {w: 1.0}}}"
```

## 真机复现前置条件

- 使用仓库 Dockerfile 对应的 ROS 2 Jazzy 环境；
- 安装 `unitree_sdk2py`、`astroviz_interfaces`、Livox ROS Driver 2 和 MOLA；
- 工作站通过以太网连接 G1，并确认网卡名；
- 首次测试应架起机器人并准备物理急停。

进入容器、构建并设置网卡后：

```bash
./cbuild g1pilot
source install/setup.bash
source setup_uri.sh <网卡名>
ros2 launch g1pilot navigation_launcher.launch.py
```

真机导航依赖 `/lidar_odometry/pose_fixed` 里程计，并通过 `/g1pilot/auto_joy`
输出运动指令。首次运行前应确认坐标系、速度上限和急停链路。
