# my_robot 自动导航功能（模块化实现说明）

在原有"机械臂 + MoveIt2"基础上，新增**差速底盘自动导航**能力（M1~M5 模块）。
采用模块化设计：导航与机械臂完全解耦，可独立启停；后续如需"导航+抓取"联动，
只需新增编排节点订阅 `nav_status` 再发 `pose_cmd` / `gripper_cmd`，无需改动本模块。

## 模块划分

| 模块 | 位置 | 职责 |
|---|---|---|
| M1 感知层 | `my_robot_description/urdf/lidar.xacro` | 新增 2D 激光雷达（gz gpu_lidar，360° / 10m / 10Hz，装在 base_link 上方 z=0.25） |
| M2 桥接层 | `my_robot_bringup/config/gazebo_bridge.yaml` | 新增 `/scan`(LaserScan)、`/odom`(Odometry) 桥接（里程计来自 GZ DiffDrive 插件） |
| M3 导航核心 | `my_robot_nav2`（新包） | Nav2 参数 + launch：SLAM 建图、导航、一键启动 |
| M4 接口层 | `my_robot_interfaces` | 新增 `NavGoal.msg`（导航目标）、`NavStatus.msg`（导航状态） |
| M5 命令节点 | `my_robot_nav_commander`（新包） | `nav_commander` 节点：订阅 `nav_cmd` → 调 Nav2 action → 发布 `nav_status` |

## 数据流

```
用户/上层 ──nav_cmd──> nav_commander ──navigate_to_pose(action)──> Nav2(planner/controller)
                                                                    │ /cmd_vel
                                                                    ▼
                                              GZ DiffDrive 插件 → Gazebo 底盘运动
                                                                    │ odom TF + /odom
                                                                    ▼
                                              slam_toolbox(SLAM) ──map->odom TF + /map
                                                                    │
nav_status(IDLE/RUNNING/ARRIVED/FAILED + 当前位姿) <── nav_commander
```

## 使用步骤

### 1. 启动 Gazebo 仿真（含新激光雷达）
```bash
cd ~/ros2_ws && colcon build --symlink-install
source install/setup.bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
```
验证感知与里程计：
```bash
ros2 topic echo /scan --once      # 有 360 个激光点
ros2 topic echo /odom --once      # 有里程计数据
```

### 2. 启动 SLAM 建图 + Nav2 导航（一键）
```bash
ros2 launch my_robot_nav2 nav_bringup.launch.py
```
也可分开启动（模块化）：
```bash
ros2 launch my_robot_nav2 slam.launch.py          # 仅 SLAM
ros2 launch my_robot_nav2 navigation.launch.py    # 仅 Nav2 导航核心
```

### 3. 启动导航命令节点
```bash
ros2 run my_robot_nav_commander nav_commander
```

### 4. 发送导航命令
```bash
# 导航到 map 系下 (2.0, 1.0)，朝向 yaw=0
ros2 topic pub --once /nav_cmd my_robot_interfaces/msg/NavGoal \
  "{frame_id: 'map', x: 2.0, y: 1.0, yaw: 0.0}"

# 取消当前导航
ros2 topic pub --once /nav_cancel std_msgs/msg/Bool "{data: true}"

# 查看状态（0=IDLE 1=RUNNING 2=ARRIVED 3=FAILED）
ros2 topic echo /nav_status
```

### 5. AMCL 固定地图定位模式（替代 SLAM，加载保存的地图）
```bash
ros2 launch my_robot_nav2 amcl_bringup.launch.py
```
- 启动 map_server（加载 `config/my_world.yaml/pgm`）+ amcl 定位 + Nav2 导航 + RViz。
- **启动后必须在 RViz 里用 2D Pose Estimate 点出机器人在地图上的初始位置**（或用 `ros2 topic pub /initialpose`），AMCL 粒子收敛后（`map -> odom` 稳定）才能导航。
- 也可单独启动定位：`ros2 launch my_robot_nav2 localization.launch.py`。
- 地图文件在 `my_robot_nav2/config/my_world.{yaml,pgm}`，可用 `map:=/path/to/xxx.yaml` 参数切换；`image` 相对路径会相对 yaml 所在目录解析。
- 保存地图命令（SLAM 模式下）：
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

## 关键话题 / 动作接口汇总

| 名称 | 类型 | 方向 |
|---|---|---|
| `nav_cmd` | `my_robot_interfaces/NavGoal` | in |
| `nav_cancel` | `std_msgs/Bool` | in |
| `nav_status` | `my_robot_interfaces/NavStatus` | out |
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | action client → Nav2 |
| `/scan` `/odom` `/cmd_vel` | sensor/odom/twist | Gazebo bridge |

## 常见问题

- **`/scan` frame 为 `my_robot/base_footlink/gpu_lidar`，slam/costmap 报 transform 不可用、map 不存在**：GZ gpu_lidar 传感器生成的 frame 带模型前缀（`<model>/<link>/<sensor>`），不在 ROS TF 树中，导致 slam_toolbox / costmap 无法把 scan 变换到 odom，进而 map 也发布不出来。已内置修复：`scan_frame_tf.launch.py` 发布静态 TF `lidar_link -> <scan_frame>`，`slam` / `navigation` / `nav_bringup` 三个 launch 都会自动带上。若你的 frame 名不同，用 `scan_frame` 参数覆盖：
  ```bash
  ros2 launch my_robot_nav2 nav_bringup.launch.py scan_frame:=my_robot/base_footlink/gpu_lidar
  ```
- **`/scan` 无数据**：确认 `my_robot_gazebo.launch.xml` 已重启（模型需重新 spawn），或 `ros2 topic list` 中无 `/scan`。
- **导航不启动/action 超时**：确认 SLAM 已发布 `map->odom` TF（`ros2 run tf2_ros tf2_echo map odom`），且 Nav2 lifecycle 全部 active（`ros2 lifecycle get /bt_navigator` 等）。
- **机器人原地打转/撞障碍**：`robot_radius: 0.3` 与底盘 0.6×0.4 匹配；若导航环境改变，调整 `nav2_params.yaml` 中 costmap 的 `robot_radius` 与速度限制。
- **Gazebo 模式下机械臂不能动**：当前 `my_robot_gazebo.launch.xml` 未启动 ros2_control/move_group（底盘由 GZ 插件驱动）。需要"导航到位后抓取"时，将 `my_robot.launch.xml` 中的 `ros2_control_node`、spawner、`move_group.launch.py` 合并进 Gazebo 启动，并增加编排节点（M6，待扩展）。
