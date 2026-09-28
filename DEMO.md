# my_robot 一键 Demo 使用说明（M10）

把原来手敲 6 条命令的流程压缩成 1 条。

> 抓取/放置相关内容（夹爪修复、机械臂可达性数据、`/arm_task` action）
> 见 [`PICK_PLACE.md`](PICK_PLACE.md)。

## 一条命令跑完整流程

```bash
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash

ros2 launch my_robot_bringup demo_all.launch.py \
  obs_x:=2.2 obs_y:=3.0 obs_yaw:=-0.13 \
  task_object_id:=red_cube place_x:=0.5 place_y:=0.5 place_yaw:=0.0
```

这一条命令等价于原来的：

| 原步骤 | 现在由谁负责 |
|---|---|
| 1. `my_robot_gazebo.launch.xml` | `t=0` 直接 include |
| 2. `amcl_bringup.launch.py` | `t=nav_delay`（默认 10s）include |
| 3. `nav_commander` | `t=nav_delay` 起节点 |
| 4. `aruco_detector` | `t=nav_delay` 起节点 |
| 5. `orchestrator.launch.py` | `t=task_delay`（默认 18s）起节点 |
| 6. `ros2 topic pub /task_cmd` | `t=send_delay`（默认 20s）由 `task_sender` 自动发 |
| **（原来漏了）** `my_robot_commander` | `t=arm_delay`（默认 10s）include `commander.launch.py` |

> 默认会打开 DetachableJoint 吸附（`use_detachable`+`attach_enabled`，
> 仿真抓取可靠性用）。若只想验证纯物理抓取或吸附插件出问题，把这两个参数
> 一起设成 `false`。

> 世界文件里的两张台子（取件台 + 放置台）台面高度都是 **0.40 m**，这是被机械臂
> "可规划工作空间下限"倒逼出来的（自由规划最低约 z=0.45）。原因与实测数据见
> [`PICK_PLACE.md`](PICK_PLACE.md) 第六节 —— 改场景时不要随意调低台面。

> 第 7 项是原来流程里缺失的一环：`my_robot_commander` 是 `/pose_cmd`、
> `/joint_cmd`、`/gripper_cmd` 以及 `/arm_task`(action) 的唯一服务端，
> 也是后续抓取放置的执行入口。不起它，机械臂和夹爪指令没有任何节点响应。
>
> 注意它**必须用 launch 启动**（`my_robot_commander_cpp/launch/commander.launch.py`），
> 因为 `MoveGroupInterface` 需要 `robot_description` 等一整套 MoveIt 参数；
> 裸 `ros2 run my_robot_commander_cpp my_robot_commander` 会
> `Unable to construct robot model` 然后 abort。原因见
> [`PICK_PLACE.md`](PICK_PLACE.md)。

## 启动时序

```
t=0                    Gazebo + robot_state_publisher + ros_gz_bridge
                       + 3 个 controller spawner + move_group
t=nav_delay    (10s)   AMCL(map_server+amcl) + Nav2 8 节点 + RViz
                       + nav_commander + aruco_detector
t=arm_delay    (10s)   my_robot_commander
t=task_delay   (18s)   task_orchestrator
t=send_delay   (20s)   task_sender 自动下发 /task_cmd
```

延时的取值理由：Gazebo 起来后要等 `/clock`、控制器 spawner、`move_group`
就绪（约 8~10s）；AMCL 的 `set_initial_pose` 自身还有 `initial_delay`（默认 5s），
所以 orchestrator 放到 18s 才起，留给 AMCL 粒子收敛的余量。

**所有延时都是参数，不许改代码**：

```bash
ros2 launch my_robot_bringup demo_all.launch.py \
  nav_delay:=15.0 task_delay:=25.0 send_delay:=27.0
```

## 常用变体

```bash
# A. 只起底盘 + 导航 + 感知，任务自己反复手发（调目标点时最常用）
ros2 launch my_robot_bringup demo_all.launch.py auto_task:=false
ros2 topic pub --once /task_cmd my_robot_interfaces/msg/TaskCommand \
  "{object_id: 'red_cube', place_frame_id: 'map', place_x: 0.5, place_y: 0.5, place_yaw: 0.0}"
ros2 topic echo /task_status

# B. 复用已经在跑的 Gazebo，只重启上层（省去每次重开仿真）
ros2 launch my_robot_bringup demo_all.launch.py use_gazebo:=false nav_delay:=1.0 \
  task_delay:=10.0 send_delay:=12.0

# C. 不要 RViz（省显存 / 服务器上跑）
ros2 launch my_robot_bringup demo_all.launch.py use_rviz:=false

# D. 回到 SLAM 在线建图模式（不加载固定地图）
ros2 launch my_robot_bringup demo_all.launch.py localization_mode:=slam

# E. 本次不涉及机械臂
ros2 launch my_robot_bringup demo_all.launch.py use_arm:=false

# F. 单独只发一次任务（不起其它任何东西）
ros2 run my_robot_task_orchestrator task_sender --ros-args \
  -p object_id:=red_cube -p place_x:=0.5 -p place_y:=0.5 -p delay:=0.0
```

## 全部启动参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `use_gazebo` | `true` | 是否启动 Gazebo（false 则复用已运行的仿真） |
| `use_rviz` | `true` | 是否启动 RViz |
| `use_arm` | `true` | 是否启动 `my_robot_commander` |
| `auto_task` | `true` | 启动后是否自动下发一次 `/task_cmd` |
| `localization_mode` | `amcl` | `amcl`（固定地图）或 `slam`（在线建图） |
| `nav_delay` | `10.0` | 定位+Nav2 启动延时（s） |
| `arm_delay` | `10.0` | `my_robot_commander` 启动延时（s） |
| `task_delay` | `18.0` | `task_orchestrator` 启动延时（s） |
| `send_delay` | `20.0` | 自动下发任务的延时（s） |
| `use_sim_time` | `true` | 仿真时钟，Gazebo 模式必须 true |
| `map` | `my_robot_nav2/config/my_world.yaml` | 固定地图（仅 amcl 模式） |
| `use_initial_pose` | `true` | amcl 模式自动发 `/initialpose` |
| `initial_x/y/yaw` | `0.0/0.0/0.0` | 初始位姿（map 系） |
| `initial_delay` | `5.0` | amcl 启动后多久发初始位姿（s） |
| `obs_x` / `obs_y` | `2.2` / `3.0` | 物体观测点（map 系） |
| `obs_yaw` | `-0.13` | 观测点朝向（rad） |
| `obs_frame` | `map` | 观测点坐标系 |
| `enable_pick` | `true` | 是否执行真实抓取（false = 退回旧占位行为，只导航） |
| `grasp_forward` | `0.175` | 观测点→抓取点沿朝向前进距离（m） |
| `tool_x` / `tool_y` | `0.42` / `0.078` | 抓取时物体在 base_footlink 系的位置 |
| `pregrasp_z` | `0.60` | 预抓取工具高度（必须 ≥ 0.45，见 `PICK_PLACE.md`） |
| `grasp_z` | `0.50` | 抓取工具高度（由指爪长度定；指爪底端 = grasp_z − 0.10） |
| `lift_z` | `0.70` | 抬起工具高度 |
| `place_standoff` | `0.42` | 机器人在放置点前方多远停车 |
| `place_approach_z` | `0.65` | 预放置工具高度 |
| `place_down_z` | `0.50` | 下放工具高度（放置台面 0.40，指爪刚好贴面） |
| `attach_enabled` | `true` | 是否用 DetachableJoint 吸附 |
| `use_detachable` | `true` | 是否给夹爪挂 DetachableJoint 插件（需与 `attach_enabled` 一致） |
| `task_object_id` | `red_cube` | 要抓的物体类别 |
| `place_frame_id` | `map` | 放置点坐标系 |
| `place_x` / `place_y` | `0.5` / `0.5` | 放置点坐标 |
| `place_yaw` | `0.0` | 放置点朝向（rad） |

`task_sender` 自身的参数（`repeat`、`period`、`wait_for_subscriber`、
`wait_timeout`、`delay`）见 `my_robot_task_orchestrator/my_robot_task_orchestrator/task_sender.py`
的模块 docstring。

## 启动自检清单

一条命令起来后，按顺序确认（`ROS_LOG_DIR` 若不可写会影响 launch，见下方"已知问题"）：

```bash
# 1. 时钟与仿真
ros2 topic echo /clock --once
ros2 topic hz /scan
# 2. 定位就绪（amcl 模式）
ros2 lifecycle get /amcl                 # 期望 active
ros2 run tf2_ros tf2_echo map odom       # 期望有持续输出
# 3. Nav2 就绪
ros2 lifecycle get /bt_navigator         # 期望 active
ros2 lifecycle get /controller_server    # 期望 active
# 4. 各命令/编排节点在线
ros2 node list | grep -E "nav_commander|aruco_detector|my_robot_commander|task_orchestrator"
# 5. 任务状态
ros2 topic echo /task_status
```

## 分步调试入口（仍然保留）

一键启动只是组合器，定位问题时照样可以拆开：

```bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml      # 只有仿真
ros2 launch my_robot_nav2 amcl_bringup.launch.py             # 只有定位+Nav2
ros2 launch my_robot_nav2 navigation.launch.py               # 只有 Nav2 核心
ros2 launch my_robot_nav2 slam.launch.py                     # 只有 SLAM
ros2 launch my_robot_task_orchestrator orchestrator.launch.py obs_x:=2.2 obs_y:=3.0
ros2 run my_robot_nav_commander nav_commander
ros2 run my_robot_perception aruco_detector
ros2 launch my_robot_commander_cpp commander.launch.py       # 手臂/夹爪（必须用 launch）
```

## 已知问题

- **`ROS_LOG_DIR` 不可写会导致 launch 直接报错**：`launch` 会往 `~/.ros/log`
  写日志，在只读 home 或受限沙箱里会抛
  `OSError: [Errno 30] Read-only file system: '/home/<user>/.ros/log/...'`。
  规避：
  ```bash
  export ROS_LOG_DIR=/tmp/roslog && mkdir -p "$ROS_LOG_DIR"
  ```
- **固定延时不等于就绪**：`TimerAction` 只是"等够时间"，不检查 Nav2 lifecycle
  是否 active。如果机器慢、Gazebo 首次加载卡顿，可能 18s 时 Nav2 还没起来，
  表现为 orchestrator 发了 `/nav_cmd` 但没反应。先把 `task_delay`、`send_delay`
  调大；若反复出现，再上就绪门方案（等 `map→odom` 可用后再拉起 orchestrator）。
- **不要同时起 amcl 和 slam**：两者都发布 `map→odom`，会互相打架。
  `demo_all.launch.py` 用 `localization_mode` 保证只起一个。
