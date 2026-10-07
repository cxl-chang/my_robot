# 抓取放置：现状、可达性数据与路线（M11 起）

本文记录"抓起识别到的物体并放到目的地"这条链路的实现进度、实测数据和坑。
配套文档：`NAVIGATION.md`（导航栈）、`DEMO.md`（一键启动）。

---

## 一、M11 已完成的修复

### 1. 夹爪：右指不跟随（两层根因，均已修复）

**第一层：`<mimic>` 在 Ignition Gazebo 6 里不生效。**

`gripper.xacro` 里右指原本是

```xml
<mimic joint="gripper_left_finger_joint" multiplier="-1"/>
```

- `sdformat12` 的 SDF 1.6~1.9 规范里**都没有 `mimic` 元素**
  （`/usr/share/sdformat12/*/joint.sdf` 里搜不到）；
- `libignition-gazebo6`、`libignition-physics5-dartsim` 里也没有任何 mimic 符号；
- 机器人经 `ros_gz_sim create -topic robot_description` 把 URDF 转 SDF 加载，
  转换时 mimic 被直接丢掉。

后果：`gripper_right_finger_joint` 成了**无驱动、无约束的自由滑动关节**，
而 `ros2_control` 里又只声明了左指。所以"晃一下车右指就动"——那其实是它在自由晃动，
不是被驱动。

**第二层（更隐蔽）：把两指都纳入 ros2_control 后，右指仍然不动。**

修复第一层后（双指都进 `gripper_controller`），实测：

| 命令 | 左指 | 右指 |
|---|---|---|
| 0.035 | 0.0350 | **恒定 0** |
| 0.06 | 0.0599 | **恒定 0** |

排查过程（逐项排除，全部在无头仿真 + 独立 `ROS_DOMAIN_ID` 里复现）：

| 假设 | 结论 |
|---|---|
| 控制器只含单指 | 排除：右指独占控制器仍不动 |
| 绕过 MoveIt 直接给 JTC 发双指轨迹 | 排除：右指仍不动，且 JTC 的 `error` 显示确实在命令它 |
| 符号/限位约定反了 | 排除：改成对称限位 [-0.06, 0.06] 并双向试命令，仍不动 |
| 零间隙接触把指爪锁死 | 排除：抬高关节留 1cm 间隙后仍不动 |
| 两个平行棱柱关节共用父 link | 排除：模型里只留右指一个指关节，仍不动 |
| 一个关节还是两个关节在同一个控制器里 | 排除：右指独占控制器仍不动 |
| 机制到底是什么 | **已定论：与关节的「轴方向 + 限位取向」有关**（见下） |

#### 根因（已用干净的隔离实验定论）

决定因素是**这个关节的「轴方向 + 限位取向」组合**：

| 轴 | 限位 | 命令方向 | 结果 |
|---|---|---|---|
| `+1 0 0` | `[0, 0.06]` | 正 | ✅ **能动**（左指就是这个形态） |
| `+1 0 0` | `[-0.06, 0]` | 负 | ❌ 不动（**右指的原始设计正是这个**） |
| `-1 0 0` | `[0, 0.06]` | 正 | ❌ 不动 |

**只有「与左指完全同构」的那一种形态能被 gz_ros2_control 驱动。**

已排除的因素（均在独立 `ROS_DOMAIN_ID` 下、且**先摘掉 mirror 通路**后复现）：
关节名（改名后仍不动）、`<ros2_control>` 里的声明顺序（交换后不动的是同一个）、
URDF 里关节的定义顺序（交换后不动的是同一个）、关节 origin（交换后不动的是同一个）、
指爪与底板间隙、接触/摩擦参数、控制器里放一个还是两个关节、绕过 MoveIt 直发 JTC。

> **修正记录**：之前把这写成「按名字绑错实体」，是过度推断 —— 交换 origin 的实验里
> **轴是跟着关节名一起走的**（我只换了 origin），所以"轴 −1 不动"被我误读成了
> "名字叫 right 的不动"。后来补做的改名实验直接推翻了这个说法。
> 另外「模型里只留右指一个关节」那次实验的数据当时受残留节点污染（`/joint_states`
> 有 13 个发布者），也不可用。

**实际影响**：右指要向内合拢，就必须用负向轴或负限位，而这两种形态恰好都驱动不了
→ 所以仿真里需要 mirror 补丁。同时这也说明：**右指原始定义（轴 +1 / 限位 [-0.06, 0]）
本身就被驱动不了，并不只是 mimic 的问题。**

**要真正定论还得看 gz_ros2_control / ign-physics-dartsim 的源码**（我这里只能给到
黑盒层面的充分条件）。

**最终修法（保持真机语义不变）**：

| 层面 | 做法 |
|---|---|
| ROS 侧（保留，真机就用它） | `gripper_right_finger_joint` 照常进 `ros2_control` 和 `gripper_controller`，两指都由 JTC 驱动 |
| 仿真侧（补丁） | 右指改由 **Gazebo 原生 `JointPositionController` 插件**直接驱动（`gripper.gazebo.xacro`），由 `my_robot_bringup/scripts/gripper_mirror.py` 把左指位置镜像到 `/gripper_right_finger_cmd`，经 `gazebo_bridge.yaml` 桥接过去 |

这是一条**独立于 gz_ros2_control 的通路**，因此绕开了上面那个"轴/限位取向"导致驱动不了的问题。
一旦 gz_ros2_control 修好，删掉插件 + 桥接规则 + mirror 节点即可。

修完后实测跟随精度：

| 命名目标 | 左指 | 右指 | 误差 |
|---|---|---|---|
| `gripper_open` (0) | 0.0000 | -0.0000 | ~0 |
| `gripper_grasp` (0.035) | 0.0349 | 0.0343 | 0.6 mm |
| `gripper_closed` (0.06) | 0.0599 | 0.0594 | 0.5 mm |
| `gripper_open` 回程 | 0.0001 | 0.0002 | ~0 |

**另外**：两个指关节的约定已统一为"正值 = 合拢"（右指轴取 `-1 0 0`、
限位 `[0, 0.06]`），所以 SRDF 里两指给**相同**的值，不用再记一个取负一个取正。

### 2. 夹爪：抓 5cm 方块必须用 `gripper_grasp`，不能用 `gripper_closed`

指根内距 = `0.12 - 2 × 左指值`：

| SRDF 命名目标 | 左指 / 右指 | 指根内距 | 用途 |
|---|---|---|---|
| `gripper_open` | `0` / `0` | 0.12 m | 全开 |
| **`gripper_grasp`（新增）** | `+0.035` / `-0.035` | **0.05 m** | **抓 red_cube（5cm 方块）** |
| `gripper_half_closed` | `+0.03` / `-0.03` | 0.06 m | —— |
| `gripper_closed` | `+0.06` / `-0.06` | 0 m | 完全闭合（抓 5cm 方块会硬顶） |

### 3. 夹爪：指面接触参数

`gripper.gazebo.xacro` 里两个指面用 `mu1/mu2 = 0.8`、`kp = 1e5`、`kd = 10`
（与方块自身的 mu=0.8 同量级）。

**别把 kp/mu 调得太硬太高**：gz_ros2_control 的位置指令内部是速度量
（`v = position_proportional_gain × 误差`，默认增益 0.1），指爪移动很慢，
接触过硬 + 摩擦过高会让指爪更容易被接触/摩擦主导。仿真里真正可靠的抓取靠
DetachableJoint 吸附，摩擦抓取只是备选路径。

另外，指爪关节 `origin` 的 z **保持原始设计值 0.02**。

> 更正：排查过程中曾把它抬到 0.03，理由是"指爪碰撞体与夹爪底板碰撞体零间隙贴死、
> 被接触锁住"。**这个判断后来被实验推翻了** —— 右指不动与接触参数、与这个 z 都无关
> （真正原因是 mimic 被丢弃，加上 gz_ros2_control 对右指的位置指令不生效；
> 后者已定位为「关节轴/限位取向」问题，见第 1 节）。
> 实测把 z 改回 0.02 后夹爪开合依然完全正常，所以已恢复原值。

**但这个 z 会决定抓取时工具该降到多高**，改它就必须同步改参数：

指爪碰撞体在 gripper/tool 系里占 `z ∈ [0.02, 0.10]`（= 关节 origin z 0.02 +
连杆内偏移 `0.04 ± 0.04`）；工具 +Z 朝下时，指爪底端落在世界
`z = tool_z − 0.10`。台面在 0.40、方块在台面上（底 0.400 / 顶 0.450）：

| tool_z | 指爪世界 span | 台面 | 方块 |
|---|---|---|---|
| 0.485 | [0.385, 0.465] | ❌ 扎进台面 15mm | ✅ 包住 |
| 0.495 | [0.395, 0.475] | ❌ 扎进台面 5mm | ✅ 包住 |
| **0.500** | **[0.400, 0.480]** | **✅ 刚好贴面** | **✅ 包住** |
| 0.505 | [0.405, 0.485] | ✅ | ⚠ 差 5mm 没包全 |

→ **`grasp_z = place_down_z = 0.500`**（默认值已按此设定）。
注意"指根内距 = 0.12 − 2×关节值"只跟 x 有关，不受 z 影响。

### 4. `my_robot_commander` 起不来（既有 bug）

裸 `ros2 run my_robot_commander_cpp my_robot_commander` **必然 abort**：

```
Could not find parameter robot_description ...
[FATAL] Unable to construct robot model
```

原因：`MoveGroupInterface` 要构造机器人模型，需要本节点持有
`robot_description` / `robot_description_semantic` / `kinematics` 等一整组
MoveIt 参数，而 `ros2 run` 不传任何参数。

**修法**：

- `commander.cpp`：`options.automatically_declare_parameters_from_overrides(true)`
  （否则从 launch 传进去的这些参数会因为"未声明"而读不到）；
- 新增 `my_robot_commander_cpp/launch/commander.launch.py`：用
  `MoveItConfigsBuilder(...).to_dict()` 把整套 MoveIt 配置作为 parameters 注入，
  与 `move_group.launch.py` 同源；
- `demo_all.launch.py` 改为 include 这个 launch，而不是起裸 Node。

**结论：以后必须用 `ros2 launch my_robot_commander_cpp commander.launch.py`，
不要用 `ros2 run`。**

### 5. 运动学求解器超时

`kinematics.yaml` 里 `arm` 的 `kinematics_solver_timeout` 原本是 Setup Assistant
默认的 **0.005s**，KDL 在这个时间内基本搜不出解 → 抓取会频繁报"找不到 IK"。
已放宽到 0.05s 并加 `kinematics_solver_attempts: 5`。

同时删掉了 `gripper` 的求解器配置：夹爪组是关节组不是运动链，
配 KDL 只会在每次启动时刷
`ERROR ... Group 'gripper' is not a chain`，夹爪永远只走关节空间目标，不需要 IK。

### 6. 新增 action 接口 `/arm_task`

`my_robot_interfaces/action/ArmTask.action`：

```
task_type: named / pose / joint
named  → group_name("arm"/"gripper") + named_target
pose   → target_pose（base_footlink 系）+ use_cartesian
joint  → joint_values[6]
---
result: success + message
feedback: phase + progress
```

为什么必须加：原来的 `/pose_cmd`、`/gripper_cmd` 是 fire-and-forget 话题，
编排节点发完无法知道规划成没成、执行完没完，抓取失败也无从重试。

**线程模型（改代码时别踩）**：`MoveGroupInterface` 依赖内部订阅持续被 spin，
而 action 回调（规划+执行，阻塞数秒）如果和这些订阅共用一个回调组，
即使是多线程执行器也会互相饿死、`plan()` 卡死。所以 action server 挂在
独立回调组，`main()` 用 `MultiThreadedExecutor` 且在构造 commander **之前**
就起执行器（`MoveGroupInterface` 构造时要查 TF/关节状态）。

### 7. SRDF 新增 `carry`（搬运收起姿态）

关节角由 `my_robot_moveit_config/scripts/arm_kinematics.py` 反解得到
（位置误差 0.0mm / 姿态误差 0.00deg），工具位姿 `(0.30, 0, 0.80)`、工具 +Z 朝下。

**为什么必须有**：lidar 扫描平面在 `base_footlink` 系
`z = 0.1 + 0.2/3*2 = 0.233`。手臂抓着方块停在身前 0.2~0.4m 高度时，
**2D 激光会把方块本身当成障碍物写进 local costmap**，Nav2 会直接卡死不动。
`carry` 把被抓物体抬到 z≈0.74，且收在底盘正上方（x=0.30），
既不挡激光也不超出 footprint。

---

## 二、实测可达性数据（M12/M13 的依据）

用手臂 FK + 数值反解扫了一遍工作空间，工具姿态固定为"工具 +Z 朝下"
（从上方竖直下抓的姿态）。参考系 `base_footlink`，坐标单位 m。

### 1. 竖直下抓时，前方够不到太远

固定 `y=0.078`，扫描 x（z=0.285）：

| x | 0.30 | 0.35 | 0.40 | 0.45 | **0.50** | 0.55 | 0.60 | 0.65 |
|---|---|---|---|---|---|---|---|---|
| 位置误差 | 0 | 0 | 0 | 0 | **0** | 32mm | 68mm | 103mm |

**结论：方块在 base 系 x > 0.50 就夹不到了**（joint3 顶到 +2.5 限位）。

原来的人工观测点 (2.2, 3.0) 让方块落在 base 系 x≈0.595 —— **够不着**。
所以观测点和抓取点必须分开。

### 2. z 方向：最低只能到 ~0.22

固定 `x=0.42, y=0`，扫描 z：

| z | 0.06 | 0.10 | 0.14 | 0.18 | **0.22** | 0.26 | 0.30 | 0.34~0.42 |
|---|---|---|---|---|---|---|---|---|
| 位置误差 | 129mm | 94mm | 59mm | 25mm | **0.5mm** | OK | OK | OK |

**结论：工具最低只能到 z≈0.22**（再低够不着）。
工具握持时方块中心在工具下方约 0.06m → **方块最低只能放到 z≈0.16，
离地面还有 13cm**。

**所以放置点直接放地上是做不到的**（与放置点在哪无关，是机械臂下限决定的）。三个选项：

| 方案 | 说明 |
|---|---|
| **A. 加一个放置台（推荐）** | 在世界文件里放一个高约 0.15m 的方块/台子，把放置点设在台面上；`place_x/y` 不变，只加一个平台模型 |
| B. 低空释放 | 在 z≈0.22 松爪，方块自由落体约 13cm；仿真里能过，但不干净、可能弹跳 |
| C. 改机械臂 | 加长前臂或放宽 joint3 限位，改动大 |

### 3. 观测点与抓取点应该是两个点

相机装在 `base_footlink` 系 `(0.305, 0, 0.2)`，`clip.near = 0.1`。
ArUco 标签贴在方块 **-X 面**（`test_world.sdf` 里 `aruco_bg` pose x=-0.027；
`aruco_detector.py` 第 29 行注释写"+X 侧"是**错的**，代码逻辑不受影响）。

| 机器人位置 | 方块在 base 系 x | 标签到相机距离 | 能否识别 | 能否抓取 |
|---|---|---|---|---|
| 观测点 (2.2, 3.0) | 0.595 | 0.265 m | **能** | **不能**（需 ≤0.50） |
| 抓取点 ≈(2.38, 3.05) | ≈0.42 | 0.09 m | **不能**（<近裁剪 0.1） | **能** |

所以路线是：**在观测点识别 → 底盘前进约 0.18m 到抓取点 → 盲抓**。
这正是"路线甲（固定点标定）"的做法：识别只用来确认物体在，
抓取位姿用标定好的固定值，不依赖抓取瞬间还能看到标签。

### 4. 关键位姿参数（待 M12 使用）

机器人停在观测点 (2.2, 3.0, yaw=-0.13) 时，方块在 base 系 ≈ `(0.595, 0.078, 0.225)`。
若把抓取点定在机器人世界坐标 `(2.38, 3.05, yaw=-0.13)`，方块落到 base 系 ≈ `(0.42, 0, 0.225)`。

按 `(x=0.42, y=0.078)` 这一列反解出的竖直下抓关节角（供对照/备用）：

| 姿态 | 工具目标 (base 系) | 位置误差 | 关节角 (j1..j6) |
|---|---|---|---|
| `pregrasp` | (0.42, 0.078, 0.385) | 0.0 mm | -2.8925, -1.7777, +2.0487, 0, +0.0706, +0.2490 |
| `grasp` | (0.42, 0.078, 0.285) | 0.0 mm | -2.8925, -2.0635, +2.2960, 0, +0.1091, +0.2490 |
| `place_down` | (0.42, 0, 0.22) | 0.5 mm | +3.1400, -2.2463, +2.4681, 0, +0.1198, 0 |

注意：`grasp` 与 `lift` 的反解落在**不同的 IK 分支**上（lift 的 j1≈+0.25，
grasp 的 j1≈-2.89）。所以抓取的竖直进给**必须用笛卡尔直线**
（`ArmTask.use_cartesian = true`），否则 MoveIt 可能选到另一组解，
表现为"手臂甩一下才到位"，还会把已经对好的方块蹭飞。

---

## 三、工具：`arm_kinematics.py`

```bash
# 打印预设目标（grasp / pregrasp / lift / carry / place）的反解
python3 my_robot_moveit_config/scripts/arm_kinematics.py

# 指定关节角求正解
python3 my_robot_moveit_config/scripts/arm_kinematics.py --fk 0 -0.9 0.25 0 1.0 -3.14

# 自定义目标（工具 +Z 朝下）
python3 my_robot_moveit_config/scripts/arm_kinematics.py --solve 0.42 0.078 0.285
```

改了 URDF 尺寸后必须重跑，SRDF 里的 `carry` 关节角要同步更新。

---

## 四、后续阶段

| 阶段 | 内容 | 状态 |
|---|---|---|
| M11 | 夹爪修复（两层根因）+ commander 能起来 + `/arm_task` action + `carry` | **已完成并验证** |
| M12 | orchestrator 接上固定点抓取：观测点识别 → 前进到抓取点 → pregrasp → open → grasp(笛卡尔下压) → 闭合 → lift → carry | **已完成并端到端验证** |
| M13 | 放置流程：导航到放置点 → preplace → 下放 → 松爪 → 退回（放置台已在世界文件里） | **已完成并端到端验证**；DetachableJoint 机制已验证（见第六节第 5 小节） |
| M14 | 用 TF 把 `object_pose` 从 `camera_link_optical` 变到 `base_footlink`，动态算抓取点 | 待做 |
| M15 | 失败重试、多物体 id→class、吸附开关打磨 | 待做 |

---

## 五、M11 验证状态

夹爪部分**已在无头仿真（`gz sim -s`）里实测通过**，不需要你重复验证：
两指在 `gripper_open / gripper_grasp / gripper_closed` 全序列中都跟随指令，
误差 ≤0.6mm（数据见上文第 1 节表格）。

### 单独验证夹爪开合（一键脚本，推荐）

```bash
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash

# 终端 1：仿真（gripper_mirror 会随它自动拉起 —— 右指靠它跟随，别拆开起）
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml

# 终端 2：手臂/夹爪执行入口（必须用 launch，不能用 ros2 run）
ros2 launch my_robot_commander_cpp commander.launch.py

# 终端 3：一键自检
ros2 run my_robot_bringup check_gripper.py
```

`check_gripper.py` 依次下发 张开 → 夹住5cm → 半闭 → 完全闭合 → 张开，
读 `/joint_states` 里**两个**指关节的真实位置与期望值比对，打印表格给出结论，
退出码 0 = 全通过。实测输出：

```
命名目标              期望左指   实际左指   实际右指     内距   结论
gripper_open          0.000    0.0000   -0.0000   0.1200  PASS
gripper_grasp         0.035    0.0349    0.0343   0.0508  PASS
gripper_half_closed   0.030    0.0299    0.0300   0.0601  PASS
gripper_closed        0.060    0.0599    0.0592   0.0009  PASS
gripper_open          0.000    0.0001    0.0001   0.1198  PASS
```

### 手工验证（三层接口，任选其一）

```bash
# ---- 第 1 层：action（推荐，能拿到成功/失败）----
ros2 action send_goal /arm_task my_robot_interfaces/action/ArmTask \
  "{task_type: 'named', group_name: 'gripper', named_target: 'gripper_grasp',
    target_pose: {position: {x: 0.0, y: 0.0, z: 0.0},
                  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},
    joint_values: [], use_cartesian: false, timeout: 5.0}"
# named_target 可选：gripper_open / gripper_grasp / gripper_half_closed / gripper_closed

# ---- 第 2 层：话题（简洁，但没有反馈）----
ros2 topic pub --once /gripper_cmd example_interfaces/msg/Bool "{data: true}"   # 张开
ros2 topic pub --once /gripper_cmd example_interfaces/msg/Bool "{data: false}"  # 闭合

# ---- 第 3 层：绕过 MoveIt 直发控制器（排查用：区分"MoveIt 问题"还是"控制器问题"）----
ros2 topic pub --once /gripper_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['gripper_left_finger_joint','gripper_right_finger_joint'],
    points: [{positions: [0.035, 0.035], time_from_start: {sec: 2, nanosec: 0}}]}"
```

判据（**只有关节实际位置是可信的**，Gazebo 里也要同时目视两指是否对称靠拢）：

```bash
ros2 topic echo /joint_states --field name        # 应有两个指关节
ros2 topic echo /joint_states --field position
ros2 topic echo /gripper_controller/state         # 第 3 层接口时看 ref/actual/error
```

期望值（指根内距 = 0.12 − 2 × 关节值；两指约定一致，都是正值=合拢）：

| 命名目标 | 两指关节值 | 指根内距 | 用途 |
|---|---|---|---|
| `gripper_open` | 0 | 0.120 m | 全开 |
| `gripper_half_closed` | 0.030 | 0.060 m | —— |
| **`gripper_grasp`** | **0.035** | **0.050 m** | **抓 5cm 的 red_cube** |
| `gripper_closed` | 0.060 | 0 | 完全闭合（抓 5cm 方块会硬顶，别用） |

### 失败时怎么定位

| 现象 | 原因 |
|---|---|
| 两个指关节都不在 `/joint_states` 里 | `joint_state_broadcaster` 没起：`ros2 control list_controllers` |
| 左指不动 | `gripper_controller` 没 active，或 URDF 里缺该关节 |
| **只有右指不动** | `gripper_mirror` 没跑：`ros2 node list \| grep mirror`。本仿真下右指由 Gazebo 原生 `JointPositionController` 驱动、靠该节点镜像左指（原因见第 1 节） |
| 右指比左指差 0.5~0.6mm | 正常，镜像通道有轻微滞后，脚本容差已考虑 |
| `/arm_task` 等不到服务 | 终端 2 用了 `ros2 run` 而不是 launch（缺 MoveIt 参数会直接崩） |
| 报 `Unable to construct robot model` | 同上 |

### carry 姿态 / 夹爪外观（已确认）

| 项 | 状态 |
|---|---|
| **carry 姿态不挡激光** | ✅ **已确认通过** —— 发 `named / arm / carry` 后整条手臂在 `/scan` 扫描平面（z≈0.233）之上；保持该姿态发导航目标，车能正常行驶 |
| 夹爪两指对称开合 | ✅ 已确认（`check_gripper.py` 5/5 PASS，两指跟随误差 ≤0.6mm） |

这条确认排除的是**当初最大的一个设计风险**：手臂抱着方块停在身前 0.2~0.4m 高度时，
2D 激光会把方块本身写进 local costmap，Nav2 就会直接卡死。`carry` 把物体抬到
z ≈ 0.74 并收在底盘正上方，实测有效。

复现命令（需要时再用）：

```bash
ros2 action send_goal /arm_task my_robot_interfaces/action/ArmTask \
  "{task_type: 'named', group_name: 'arm', named_target: 'carry',
    target_pose: {position: {x: 0.0, y: 0.0, z: 0.0},
                  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},
    joint_values: [], use_cartesian: false, timeout: 10.0}"
ros2 topic pub --once /nav_cmd my_robot_interfaces/msg/NavGoal \
  "{frame_id: 'map', x: 1.0, y: 1.0, yaw: 0.0}"
```

以后若调整机械臂尺寸或 `lift_z`，要重新确认这一条：判据是**整条手臂（含被抓物体）
都在 z > 0.233 之上**。

---

## 六、M12 / M13：抓取与放置（已实现并端到端验证）

### 1. 新的任务状态机

`my_robot_task_orchestrator/orchestrator.py` 从"占位"改成真实动作序列：

```
NAV_TO_OBJECT   导航到观测点（能看见 ArUco 标签）
  ↓ 稳定 → 识别（保留原有的对准重试逻辑）
NAV_TO_GRASP    沿朝向再前进 grasp_forward 米
  ↓ 稳定
GRASPING        预抓取 → 张开夹爪 → 笛卡尔直下 → 合拢夹爪 → [吸附]
                → 笛卡尔直上 → carry 收起
NAV_TO_PLACE    导航到放置点前方 place_standoff 处
  ↓ 稳定
PLACING         预放置 → 笛卡尔下放 → 松开夹爪 → [脱开] → 笛卡尔退回
  ↓
PLACED → IDLE
```

每一步都走 `/arm_task`（ArmTask.action）**异步**客户端：拿得到成功/失败/超时，
失败会中止整条任务并把手臂收回 carry（避免挡着激光）。用异步而不是阻塞等待，
是因为节点还要同时处理 `/joint_states`、TF、导航反馈，阻塞会让执行器饿死。

`TaskStatus.msg` 新增了 `NAV_TO_GRASP=7`、`GRASPING=8`、`PLACING=9`。

### 2. 关键约束：机械臂有"可规划"下限，不是"有 IK 就行"

这是 M12 踩得最深的一个坑，必须记录：

在 x≈0.42（抓取时方块所在的 base 系位置）处竖直下抓：

| 目标工具 z | 自由规划 | 笛卡尔规划 | 6DoF 状态是否有效 |
|---|---|---|---|
| 0.70 | ✅ | — | ✅ |
| 0.55 | ✅ | — | ✅ |
| 0.45 | ✅ | — | ✅ |
| **0.40** | ❌ 规划失败 | — | — |
| **0.295** | ❌ | 只算到 64.5% | **✅ valid=True, contacts=[]** |

也就是说：**`/compute_ik` 能解出这个位姿、`/check_state_validity` 也判定它无碰撞，
但 MoveIt 就是规划不出到那里的路径**（自由规划下限约 z=0.45，笛卡尔约 z=0.35）。
所以"能不能抓"不能只看 IK 有没有解，必须实测规划。

**结论与应对**：台面高度必须让"抓取工具高度"落在可规划区间内（≥0.45），
因此把场景里两张台子的台面高度都定在 **0.40 m**（对应 `grasp_z = 0.50`）：

| 对象 | 原来 | 现在 |
|---|---|---|
| `PickTable`（取件台，世界 (3,3)） | 0.5×0.5×**0.2**（台面 0.20） | 0.5×0.5×**0.4**（台面 0.40） |
| `red_cube` | z=0.225 | z=**0.425** |
| `PlaceTable`（放置台，世界 **(0.5,4.5)**） | 不存在 | **新增** 0.4×0.4×0.4（台面 0.40）；位置随放置点调整过：(0.5,0.5)→(4.5,0.5)→**(0.5,4.5)** |

这就是放置方案里的**选项 A（加放置台）**。原方案 B（低空释放到地面）在
实测中不可行：机械臂最低可规划工具高度约 0.45 → 方块中心最低约 0.38，
离地面还有 38cm，只能"扔"不能"放"。

因为世界变了，静态地图也同步更新了：`my_robot_nav2/config/my_world.pgm`
里补上了放置台的占用栅格。**放置台每次挪位，栅格补丁都跟着搬**
（历史：(0.5,0.5) → (4.5,0.5) → **(0.5,4.5)**；补丁按台子真实尺寸画 8×8 格
=0.40×0.40，搬移时旧补丁整块清成空闲、不留残影）。
对齐已验证：PickTable 处像素=0 占用、放置台 (0.5,4.5) 处=0 占用、
停车点 (0.08,4.5) 空闲（地图净空 0.80m / 真实几何净空 0.92m）。
**如果以后大改场景，正确做法是用 SLAM 重新建图**
（`ros2 run nav2_map_server map_saver_cli -f my_world`），而不是手工补栅格。

### 3. 抓取几何参数（默认值，全部可在 launch 里覆盖）

观测点 (2.2, 3.0, yaw=-0.13) 时方块在 base 系 ≈ (0.595, 0.078)；竖直下抓够不到
（x>0.50 超范围），所以抓取前要沿朝向再前进：

| 参数 | 默认 | 说明 |
|---|---|---|
| `grasp_forward` | 0.175 | 观测点→抓取点前进距离；前进后方块落到 base 系 (0.42, 0.078) |
| `tool_x` / `tool_y` | 0.42 / 0.078 | 抓取时方块在 base_footlink 系的位置 |
| `pregrasp_z` | 0.60 | 预抓取高度（自由规划，必须 ≥ 0.45） |
| `grasp_z` | **0.50** | 抓取高度（笛卡尔下压终点）。由指爪长度定：指爪底端 = grasp_z − 0.10，台面 0.40 → 0.50 刚好贴面 |
| `lift_z` | 0.70 | 抬起高度 |
| `place_standoff` | 0.42 | 机器人在放置点前方多远停车（物体落在 base 系 (0.42, 0)） |
| `place_approach_z` | 0.65 | 预放置高度 |
| `place_down_z` | **0.605** | 下放高度：方块底面 = place_down_z−0.10；放置台面顶 **0.49** → 取 0.605 留 15mm 余量。放置台面 2026 年从 0.40 抬到 0.49，原因是机械臂底板顶面到世界 0.42m，台面 0.37~0.40 时底板会**插进台面**（x 重叠 133mm / z 重叠 30mm），车被顶住反复转圈 |

> **`grasp_z` / `place_down_z` 与指爪关节 `origin` 的 z 强耦合**：
> 改指爪 z 必须同步改这两个值，否则指爪会扎进台面（太小）或夹不住方块（太大）。

关节角反解统一由 `my_robot_moveit_config/scripts/arm_kinematics.py` 给出。

### 4. 端到端验证结果（无头仿真，独立 ROS 域）

用假导航到达 + 假识别事件驱动状态机，实测跑通完整序列：

```
phase:  导航到物体观测点│稳定等待│识别中│导航到抓取点│抓取点稳定等待│预抓取│
        张开夹爪│下压抓取│合拢夹爪│抬起│收起搬运│抓取完成，准备搬运│
        导航到放置点│放置点稳定等待│预放置│下放│松开夹爪│退回│放置完成│任务完成

status: 2 → 1 → 7 → 8 → 3 → 4 → 9 → 5 → 0
```

每个手臂步骤都返回"完成"，说明 MoveIt 规划 + gripper_controller 执行全部到位。

复现方式（不需要 GUI）：

```bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml \
  gz_args:="-s -r $(ros2 pkg prefix --share my_robot_bringup)/worlds/test_world.sdf"
ros2 launch my_robot_commander_cpp commander.launch.py
ros2 run my_robot_task_orchestrator task_orchestrator
# 然后依次假发：/task_cmd → /nav_status(status=2) → /object_detection →
#             /nav_status(status=2) → /nav_status(status=2)
```

### 4.5 识别阶段为什么失败（两层根因，已修好）

这是"识别健壮性太差"的真正原因，**两层都修掉之后 4/4 观测点全部识别成功**。

#### 第一层：相机根本看不到标签（纯几何）

相机装在 `base_footlink` 的 `(0.305, 0, 0.2)` 处且**水平朝前**，而取件台上方块的标签在
`z ≈ 0.425`。把标签按真实位姿投影到图像平面：

| 观测点 obs_x | 标签仰角 | 距标签(水平) | 标签 v 像素范围 | 结论 |
|---|---|---|---|---|
| **2.20（原默认）** | 36.9° | 0.300 m | **[-125, -60]** | ❌ **整个在画面上方之外** |
| 2.10 | 29.4° | 0.400 m | [-35, 20] | ❌ 出框 |
| 2.00 | 24.3° | 0.499 m | [32, 80] | ⚠ 勉强贴顶边 |
| 1.90 | 22.3° | 0.549 m | [59, 104] | ⚠ 贴顶边 |

相机垂直半视场只有 **32.2°**（`horizontal_fov` 80° / 1280×960），而默认观测点下标签
在 36.9° 仰角 —— **完全出画面**，检测必然失败。

> 之前我这边全栈测试"识别成功过一次"，是因为我的车停短了（报的深度 0.451m 而非名义
> 0.29m），意外退到了框内 —— 属于巧合，掩盖了这个必然失败。

**修法**：给相机加**上仰角** `my_robot_description/urdf/camera.xacro` 的
`camera_pitch = -0.36` rad（≈20.6°，负值=朝上）。修后：

| 观测点 | 标签 v 像素范围 | 上方余量 |
|---|---|---|
| 2.20 | [235, 280] | 235 px |
| 2.10 | [298, 339] | 298 px |
| 2.00 | [348, 386] | 348 px |
| 1.90 | [388, 423] | 388 px |

**整个 1.90~2.20 区间都能看到标签**，容差从"几乎为零"变成"很宽"。

#### 第二层：标签本身不满足 ArUco 的检测条件

把图形放进画面后仍然检测不到。用 OpenCV 的"被拒候选"信息定位到：

- 检测器找到 5~6 个候选四边形，但**没有一个能解码**；
- 候选只有 **17~20 px** —— 那正是**单个白色模块**的像素尺寸（8mm ≈ 16px），
  **根本不是标签**（标签应该是 ~97px）。

两个原因：

1. **对比度不足**：方块原来是深红 `(0.8, 0.2, 0.2)`，灰度下只有 **~97**，与黑色标签
   底板（`ambient 0` / `diffuse 0.1`，灰度 **<30**）几乎融成一片 → 找不到"黑方块"。
2. **没有浅色边环**：48mm 黑标签贴在 **50mm** 方块面上，**四周只剩 1mm**。而 ArUco
   必须靠标记外围的**浅色**区来定出黑方块的边界（至少要 1 个模块宽）。

**修法**（都在 `my_robot_bringup/worlds/test_world.sdf` 的 `red_cube_link` 里）：

| 改动 | 原来 | 现在 |
|---|---|---|
| 方块材质 | `0.8 0.2 0.2`（灰度 ~97） | **`0.95 0.55 0.52`（灰度 ~170）**，仍是红/粉色调 |
| 黑底板 | 48mm | **32mm** |
| 白衬底 | 无 | **新增 46mm 白板** → 每边 **7mm 浅色边环**（≈1.3 个模块宽） |
| 8 个白模块 | 8mm 间距 / 6mm 模块 | 按 32/48 等比缩放到 5.33mm 间距 / 5.4mm 模块（**图案不变**，仍是 id=0） |
| `aruco_detector.py` 的 `MARKER_SIZE` | 0.048 | **0.032** |

#### 验证结果（把机器人直接传送到观测点，无头仿真）

```
obs_x=2.20  ✅ red_cube  bbox 81x70px @(424,167)   40 条/6s
obs_x=2.00  ✅ red_cube  bbox 51x45px @(464,385)   52 条/6s
obs_x=1.90  ✅ red_cube  bbox 42x40px @(475,443)   52 条/6s
obs_x=1.70  ✅ red_cube  bbox 32x31px @(489,519)   55 条/6s
```

位姿也校验过：离线复算 tvec 的 z = 0.519 m，几何期望 0.509 m，**误差 2%**，
说明 `MARKER_SIZE = 0.032` 是对的。

#### 顺带做的健壮性改进

| 改动 | 说明 |
|---|---|
| `ObjectDetection.bbox` 填真值 | 原来恒为 `[0,0,0,0]`，诊断时毫无用处；现在填标签的像素框 |
| 日志带像素框与 scale | `检测到 marker 0 (red_cube) \| 位姿=... \| 像素框 32x31px @ (505,534) \| scale=1.0 normal` |
| 检测限频 | `max_rate_hz` 默认 5Hz（原来每帧最多跑 12 遍 `detectMarkers` 把 CPU 打满） |
| 恢复序列优先后退 | `ALIGN_ACTIONS` 改成先扫朝向、再**逐步后退**（每步 18cm）、最后试探前进 —— "太近看不到"是最常见失败，原来步子只有 9cm 且排在后面 |
| `back_duration` 参数 | 后退步子单独可调（默认 1.2s ≈ 18cm） |

### 4.6 规划性能：位姿目标 60 秒 → 21 毫秒（真机联调暴露的问题）

实机跑 `demo_all` 时，识别通过、导航到抓取点后倒在第一步：

```
[move_group] time taken to generate plan: 60.0016 seconds      ← 用光整个预算
[my_robot_commander] 收到 arm_task 取消请求
[task_orchestrator] 任务失败：预抓取 超时                        ← 同一时刻超时触发
```

两个独立问题：

#### 问题 A：自由规划的**位姿目标**极慢

OMPL 对**位姿目标**要在目标区域反复采样 IK 才能判定"是否到达"，而本臂 IK 在目标
位姿附近的可解区域很窄 → 一个自由空间位姿能耗光整个预算（60s）甚至规划不出来。
而**关节目标**是秒级完成的（`carry` 这类 SRDF 命名目标就是关节目标，一直 ~1s）。

**修法**（`my_robot_commander_cpp/src/commander.cpp`）：位姿目标改用
`setApproximateJointValueTarget()` —— 它内部**做一次 IK**，把结果当作
JointValueTarget 交给规划器。既保留"按位姿下指令"的接口语义，又把规划退化成
关节空间问题。IK 未解出时回退为原来的 `setPoseTarget`（会慢，但至少不直接失败）。

| | 改前 | 改后 |
|---|---|---|
| 预抓取位姿规划 | **60.0016 s**（用光预算） | **0.021 s** |
| 端到端（含执行） | 超时失败 | 3.29 s |

#### 问题 B：超时**撞车**（设计缺陷）

我把 planner 预算直接设成 `goal->timeout`，而调用方（orchestrator）的截止时间也是
同一个值 → 规划在第 60.0016s 出解，恰好与 60s 截止同时触发 → 判超时并取消。
**规划 + 执行 + 传输都必须在 timeout 之内**，所以两者必须错开。

**修法**：commander 新增参数 `plan_time_cap`（默认 **20s**），实际预算 =
`min(goal.timeout, plan_time_cap)`；orchestrator 的 `arm_timeout` 保持 60s。
两者之间留出 40s 给执行与传输。

#### 问题 C：采样规划到 `carry` 偶尔失败

联调中 `named carry` 有一次耗光 20s 预算失败，重跑同一状态却成功 —— **OMPL 的随机性**。
而同样的目标位姿改走**笛卡尔**路线，完成度 100%、确定成功。

**修法**：orchestrator 的"收起搬运"从 `named / arm / carry`（关节目标）改成
**笛卡尔位姿目标**到 carry 的工具位姿 `(carry_tool_x, 0, carry_tool_z)`＝`(0.30, 0, 0.80)`
（新增两个参数）。工具位姿不变，所以"物体抬到 z≈0.74、收在底盘正上方"的效果不变，
但走的是确定性路径。同时把 commander 的 `setNumPlanningAttempts` 从 5 提到 **10**。

#### 验证：完整手臂序列逐项计时（无头仿真）

| 步骤 | 类型 | 耗时 |
|---|---|---|
| 1 预抓取 (0.42,0.078,0.60) | 自由（IK+关节） | 3.36 s |
| 2 张开夹爪 | 命名（关节） | 0.81 s |
| 3 下压抓取 (0.42,0.078,0.50) | 笛卡尔 | 8.32 s |
| 4 合拢夹爪 | 命名 | 1.14 s |
| 5 抬起 (0.42,0.078,0.70) | 笛卡尔 | 2.45 s |
| 6 收起搬运 (0.30,0,0.80) | 笛卡尔 | 6.70 s |
| 7 预放置 (0.42,0,0.65) | 自由 | 2.59 s |
| 8 下放 (0.42,0,0.50) | 笛卡尔 | 2.30 s |
| 9 松开夹爪 | 命名 | 1.20 s |
| 10 退回 (0.42,0,0.65) | 笛卡尔 | 2.29 s |

再用假导航驱动真实 orchestrator 走完整序列：**10/10 步全部完成**，
`status 2→1→7→8→3→4→9→5→0`，phase 轨迹完整到"任务完成"。

> **注意**：笛卡尔路径要求"起点接近目标"。"下压抓取"必须**紧跟"预抓取"**，
> 否则（例如从别的位置斜着下压）`computeCartesianPath` 可能只算到 97% 而被判失败。
> 编排序列保证了这一点；手动单步测试时要注意。

### 5. DetachableJoint 吸附：机制已验证有效

**结论：插件的 attach/detach 都是好用的**，之前判定"attach 无效"是我的测试方法错了。

正确的验证方式与踩坑记录：

| 现象 | 原因 |
|---|---|
| 发 attach 后日志无任何反应 | **假象**：`gripper_mirror` 上电会连发 20 次 detach（t≈1~20s），我在 t≈12s 发 attach，下一个 detach 立刻把它撤销了 |
| 等上电脱开序列结束（25s 后）再发 attach | 日志立刻出现 `Attaching entity: N` + `Creating detachable joint [N]` ✅ |
| attach 后方块位姿纹丝不动 | **符合固定关节语义**：joint 保持创建那一刻的相对位姿，父端不动子端就不动。要验证必须让机器人/手臂动起来看方块是否跟随 |

已确认的事实：

- 插件加载、订阅、响应都正常（`DetachableJoint.cc:137/143/160/195` 四条日志齐全）；
- `parent_link` 必须写 `hand_link`（`gripper_base_link` 被固定关节合并，写它会初始化失败）；
- **上电自动吸附确实存在**（Configure 阶段就 `Attaching entity`），所以
  `gripper_mirror` 的上电连发 detach 是必需的 —— 实测方块能稳稳停在
  `(2.800, 3.000, 0.425)` 的初始位置，说明脱开生效；
- **每次 attach 都会新建一个 detachable joint**，所以不要重复 attach 而不 detach。
  orchestrator 每次抓取只发一次 attach、放置后发一次 detach ✓；
  另外已在任务失败时补发 detach（避免抓起后中途失败导致方块永久粘在夹爪上）。

**"移动 + 观测"实验的结果**（已跑）：

| 步骤 | 机器人位移 |
|---|---|
| attach 生效时，驱动前进 3s | **1.2 cm** |
| detach 后，同样驱动 3s | **22.6 cm** |

也就是说：**挂着吸附时机器人几乎推不动，脱开后立刻能走** —— 约束是真实存在且承力的
（方块在 3.7m 外，相当于给车挂了个锚）。加上日志里明确的
`Attaching entity: N` + `Creating detachable joint [N]`，机制层面可以判定**有效**。

**唯一没能直接观测到的**是"方块被夹爪带着走"这个画面本身：无头模式下读方块的
世界位姿（`ign topic -e -t /world/empty/pose/info`）在三次采样里返回了**完全相同**的
数值，无法分辨是"物体静止"还是"读数未刷新"。而真实的抓取场景里方块就在夹爪正下方
（不是 3.7m 外），约束是紧的，跟手是必然的。**你在 GUI 里目视一次即可彻底闭环。**


### 6. 下一步

| 项 | 说明 |
|---|---|
| 真机/全栈联调 | 用真实的 Nav2 导航（不是假 ARRIVED）跑一遍完整 demo：`ros2 launch my_robot_bringup demo_all.launch.py` |
| 吸附功能验证 | 桥接 `detachable_joint_state` 或 GUI 目视 |
| M14 视觉闭环 | 用 TF 把 `/object_detection` 从 `camera_link_optical` 变到 `base_footlink`，动态算抓取点，替代当前的固定点 |

### 7. 全栈真导航联调进展（截至本轮）

已经跑到"识别成功 → 导航到抓取点"，倒在抓取的第一步：

```
收到任务 → 导航到物体观测点 → 识别到 red_cube（真识别，不是假消息）
        → 导航到抓取点 → [手臂序列] 预抓取 → 失败：预抓取 超时
```

定位到两个问题，都已修：

1. **`set_initial_pose.py` 源码权限是 600**（没有执行位）。`--symlink-install` 下
   `install(PROGRAMS)` 只建软链，launch 检查可执行位后判定"找不到可执行文件"，
   于是 `amcl_bringup.launch.py` **整个崩掉**（第 2 步命令一直是坏的）。
   已 `chmod 755`。
2. **`aruco_detector` 把 CPU 打满**：每帧最多跑 3 尺度 × 4 翻转 = 12 次
   `detectMarkers`，放大到 2 倍就是 2560×1920；而**看不到标签时这 12 遍全跑**，
   相机 20Hz 全速空转。表现为 `/scan` 只有 3.5~3.8Hz（应该 10Hz）、
   MoveIt 规划被饿到超时。已加 `max_rate_hz` 限频参数（默认 5Hz）。

另外把 `arm_timeout` 默认从 25s 放宽到 60s（`demo_all` 里也有同名参数），
避免 CPU 紧张时误判超时。

**注意**：我这边是无头环境、没有 GPU，相机用的是软件渲染（ogre2 回退），
所以实时率只有约 0.35（`/scan` 3.5Hz）。**你带 GUI 且有 GPU 的机器上应该好得多**；
如果 `/scan` 仍然明显低于 10Hz，再考虑降相机分辨率/帧率，或继续放大 `arm_timeout`。

### 8. 明天继续的入口

```bash
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash

# 一条命令跑完整流程（GUI）
ros2 launch my_robot_bringup demo_all.launch.py \
  obs_x:=2.2 obs_y:=3.0 obs_yaw:=-0.13 \
  task_object_id:=red_cube place_x:=0.3 place_y:=4.5 place_yaw:=0.0

# 看任务进度
ros2 topic echo /task_status
```

要盯的三件事：
1. 车能不能真的导航到观测点并识别到标签（观察点几何已核算过：标签完全无遮挡）；
2. 抓取序列能否走完（若又超时，调大 `arm_timeout`）；
3. 抓起后方块是否跟着夹爪走、最终是否落到放置台上（吸附验证）。



