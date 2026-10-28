#!/usr/bin/env python3
"""M9 任务编排节点 task_orchestrator（M12/M13 起：真实抓取 + 放置）。

订阅 /task_cmd（TaskCommand: object_id + 放置点）后执行状态机：

  1. NAV_TO_OBJECT   导航到"物体观测点"（能看见 ArUco 标签的位置）
  2. 识别确认         等 /object_detection 认到目标；超时则原地小幅转动/前后试探，
                      最多 max_retry 次（ALIGN_ACTIONS 序列）
  3. NAV_TO_GRASP    从观测点沿朝向再前进 grasp_forward 米 —— 必须走这一步，
                      因为竖直下抓时方块在 base 系 x>0.50 就超出机械臂可达范围，
                      而观测点上方块在 x≈0.60
  4. GRASPING        预抓取 → 张开夹爪 → 笛卡尔直下 → 合拢 → [吸附] → 笛卡尔直上 → carry
  5. NAV_TO_PLACE    导航到放置点前方 place_standoff 处
  6. PLACING         预放置 → 笛卡尔下放 → 松爪 → [脱开] → 笛卡尔退回
  7. PLACED → IDLE

全程发布 /task_status（数值状态 + 中文 phase）。

为什么抓取用 action 而不是话题
------------------------------
原来的 /pose_cmd、/gripper_cmd 是 fire-and-forget，发完不知道成没成。
抓取必须能判断成功/超时/失败，否则只能 sleep 硬等、失败也无法重试，
所以 M11 起 my_robot_commander 提供了 /arm_task（ArmTask.action），
本节点用**异步** action client 驱动它：不阻塞执行器（节点还要处理
/joint_states、TF、导航反馈），也不会和 MoveIt 内部回调互相饿死。

依赖（需同时运行）：
  - my_robot_nav_commander nav_commander（/nav_cmd → /nav_status）
  - my_robot_perception aruco_detector（/object_detection）
  - my_robot_commander_cpp commander.launch.py（/arm_task action server）
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (QoSProfile, DurabilityPolicy)
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (PlanningScene, PlanningSceneWorld, CollisionObject,
                             AttachedCollisionObject)
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from tf2_msgs.msg import TFMessage
from my_robot_interfaces.msg import (
    TaskCommand, TaskStatus, NavGoal, NavStatus, ObjectDetection)
from my_robot_interfaces.action import ArmTask


def quat_from_rpy(roll, pitch, yaw):
    """欧拉角 → 四元数 (x, y, z, w)。"""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


class TaskOrchestrator(Node):
    # 对准动作序列（按 retry 顺序执行；识别超时一次取一个动作）：
    #   rot+ 左转 / rot- 右转 / back 后退 / fwd 前进
    # 序列设计：覆盖【左偏→回正→右偏→回正】两侧朝向 + 前后距离试探
    # 序列设计：先扫朝向（左右各偏一点），再**逐步后退**——"离得太近、标签出了
    # 相机画面"是最常见的失败模式（相机视场是有限的，越近仰角越大），所以后退
    # 排在前面且步长更大；最后才试探前进。
    ALIGN_ACTIONS = ['rot+', 'rot-', 'rot-', 'rot+',
                     'back', 'back', 'back',
                     'fwd', 'rot+', 'rot-']

    def __init__(self):
        super().__init__('task_orchestrator')

        # ================= 参数 =================
        # --- 观测点 ---
        self.declare_parameter('obs_x', 2.2)
        self.declare_parameter('obs_y', 3.0)
        self.declare_parameter('obs_yaw', -0.13)
        self.declare_parameter('obs_frame', 'map')
        # --- 识别与对准 ---
        self.declare_parameter('detect_timeout', 5.0)
        # 导航栈坐标系与机械臂坐标系之间差多少（详见 _to_nav_frame/_to_base_link）：
        # 驱动轮在 base_link 后方 base_length/4=0.15m，差速转心=轮轴中点，
        # URDF 把根坐标系 base_footlink（= Nav2 的 robot_base_frame）放在转心上。
        # 本节点里所有目标点/位姿换算都以 **base_link（机械臂那一侧）** 为准，
        # 发给导航前减去这个偏移、从 /amcl_pose 读回后加回来。
        # 若把 URDF 的 base_joint 改回 x=0，这里也要设成 0.0。
        self.declare_parameter('base_axle_offset', 0.15)
        self.declare_parameter('max_retry', 10)
        self.declare_parameter('align_vel', 0.5)
        self.declare_parameter('rot_angle', 0.12)
        self.declare_parameter('move_vel', 0.15)
        self.declare_parameter('move_duration', 0.6)    # 前进时长 s（≈9cm）
        self.declare_parameter('back_duration', 1.2)    # 后退时长 s（≈18cm，步子更大）
        self.declare_parameter('settle_time', 1.5)
        # 物体识别使能开关话题（详见 aruco_detector.py）
        self.declare_parameter('detect_enable_topic', '/detection_enable')
        # --- M12 抓取 ---
        self.declare_parameter('enable_pick', True)
        self.declare_parameter('grasp_forward', 0.175)   # 观测点→抓取点前进距离
        self.declare_parameter('tool_x', 0.42)           # 抓取时物体在 base 系 x
        self.declare_parameter('tool_y', 0.078)          # 抓取时物体在 base 系 y
        # 取件台面 0.40 → 0.48（方块中心 0.505），抓取高度整体 +0.08：
        # 原来的 0.50 比"竖直下抓自碰地板 0.52"还低 —— 手腕要擦过肩部，目标位姿
        # 的所有 IK 解都自碰，笛卡尔直降只能算到 75~78%（停在 z≈0.522，指爪只
        # 套进方块顶部 3cm，看起来像"碰到方块上方、够不到另一面"）。
        # 抬到 0.58 后与放置高度 0.605 基本一致，抓/放共用一个已验证姿态区；
        # 指爪与方块的相对关系不变（工具始终在方块中心上方 0.075）。
        self.declare_parameter('pregrasp_z', 0.68)
        self.declare_parameter('grasp_z', 0.58)
        self.declare_parameter('lift_z', 0.70)
        self.declare_parameter('carry_tool_x', 0.30)
        # 0.70 而不是 0.80：SRDF 里 carry 命名姿态的工具位姿就是 (0.30,0,0.70)，是标定过的
        # 可行构型；(0.30,0,0.80) 求出的解 joint6 全落在 ±π 边界上，MoveIt 判越界 →
        # 报"位姿 IK 无解"，收起搬运直接失败（实测）。
        self.declare_parameter('carry_tool_z', 0.70)
        # 抓取可达性校验：固定点抓取默认"方块就在 (tool_x,tool_y)"，这个假设只在
        # 观测点/前进距离与方块实际位置匹配时成立。改过观测点却没重标定时，机械臂
        # 会夹空，而 /red_cube/attach 是无条件发的 → 方块被从原地焊到夹爪上，
        # 表现为"手臂离方块很远却显示已抓取"。所以动手前先算一遍：
        self.declare_parameter('object_x', 2.8)   # 方块世界坐标（map 系）
        self.declare_parameter('object_y', 3.0)
        self.declare_parameter('grasp_tol', 0.03)          # 偏差≤此值用名义目标
        # 偏差在 (grasp_tol, grasp_compensate_xy] 内时**直接改用实测位置**当 IK 目标。
        # 取 0.30 而不是 0.08：到位姿态漂移十几度就能让横向坐标差 0.15~0.20 m，
        # 那属于"正常误差、机械臂仍够得到"，不该判失败；真正的坏情况（观测点改错、
        # 定位跳变）偏差都在 0.35 m 以上，仍会被拦下。
        self.declare_parameter('grasp_compensate_xy', 0.30)
        # 自动标定前进距离：观测点离方块多远就往前开多远，使得到抓取点时方块正好
        # 落在 base 系 tool_x 处。这样换观测点（obs_x/obs_y）后不必重新手标
        # grasp_forward —— 手标模式下 obs=(1.84,2.82) 时前进了 0.175 还剩 0.795m，
        # 远超机械臂 0.42m 的固定抓取半径，只能靠"加长机械臂"硬解决（不可取：
        # 会连带 carry 姿态/SRDF 命名目标/抓取放置 z 全部重标）。
        self.declare_parameter('grasp_auto', True)
        # 固定点竖直下抓的可靠区间（工具目标必须落在这里面才允许抓）
        # x 上限 0.45 而不是 0.50：`PICK_PLACE.md` 那张"x≤0.50 都行"的表是在
        # z=0.285 扫的（相对臂基座 3D 半径仅 0.40），而抓取在 z=0.50（半径
        # 0.436），此时 joint3 的 +2.5 限位把半径卡在 ~0.45-0.46。实测方块落到
        # base 系 x=0.465（到位残差 4cm 造成）时，下压直降的笛卡尔路径只算到
        # 75% 就失败。留 1cm 余量取 0.45。
        # 【窗口为什么要双向卡】竖直下抓时：
        #   太近（x<0.42）：手臂要折起来才够得到 → 撞自己肩膀。实测 bx=0.384 时
        #     FCL 报 shoulder_link <-> gripper_base_link 穿透 28.9mm，全部 IK 解无效。
        #   太远（x>0.45）：joint3 顶限位，笛卡尔直降算不完（实测 75~78%）。
        # 而导航到位残差本身就有 ±0.05m，和安全窗口宽度（0.42~0.45=3cm）同量级，
        # 所以必须**到位后按实测闭环微调**，不能只靠一次导航。
        self.declare_parameter('grasp_x_min', 0.42)
        self.declare_parameter('grasp_x_mid', 0.435)   # 微调瞄准的中间值
        # 导航总在目标前 ~0.045m 就判到位（xy_goal_tolerance=0.05），前后都一样，
        # 所以"目标步长 = 期望位移 + 这个量"，否则微调会走不够。
        self.declare_parameter('grasp_park_shortfall', 0.045)
        self.declare_parameter('grasp_reach_x_max', 0.45)
        self.declare_parameter('grasp_reach_y', 0.25)
        # grasp_auto=true 时抓取点由"方块 map 位置 + 目标朝向"反解；反解出来
        # 离观测点超过这个距离就认为 object_x/object_y 参数不对，退回固定前进。
        self.declare_parameter('grasp_auto_max_fwd', 1.2)
        # 方块超出可靠区间但只差一点点时，自动补走一小段再抓（视觉伺服式）
        self.declare_parameter('grasp_correct_max', 0.30)    # 单次最大补进量 m
        self.declare_parameter('grasp_correct_steps', 2)     # 最多补进几次
        # 【必须】补进的最小步长：要补的距离若小于 Nav2 的 xy_goal_tolerance
        # (0.05)，导航会认为"已经在目标点上"而一步不动（踩过）。而且导航总在
        # 目标点前 ~0.05m 就判到位，所以"目标步长 ≈ 期望位移 + 0.05"：
        # 取 0.12 ⇒ 实际大约走 0.07。宁可多走一点（落点偏近，机械臂低限 0.30
        # 还远），也不要"发了目标却没动"。
        # 微调的**目标步长下限**：小于导航的 xy_goal_tolerance(0.05) 时导航会判
        # "已到位"一步不动；实际位移 ≈ 目标步长 − 0.045（到位短差）。取 0.06
        # 而不是 0.12，避免过冲把方块推到窗口另一侧（踩过）。
        self.declare_parameter('grasp_correct_min', 0.06)
        # 微调**不用导航**，直接发 /cmd_vel 蠕动（/cmd_vel 经 gazebo_bridge 直连
        # 底盘插件）。原因：导航走一次 BT+DWB 要好几秒，而这里往往只需要挪 2~5cm；
        # 更要命的是导航总在目标前 ~0.045m 判到位，得靠"补偿"发更远的目标，
        # 结果 2.5cm 的修正会变成一个 7cm 的导航任务（用户实测抱怨点）。
        # 蠕动是开环按里程计推的，短距离精度足够，且位移精确等于需要值。
        self.declare_parameter('creep_vel', 0.08)        # 蠕动线速度 m/s
        self.declare_parameter('creep_settle', 0.8)      # 停稳等待 s
        self.declare_parameter('creep_deadband', 0.005)  # 小于它就不动了 m
        # 收起搬运的目标**工具位姿**（不是关节目标）：工具抬到 0.80、收到底盘
        # 正上方 x=0.30，物体 z≈0.74 高于激光面。用笛卡尔直线走过去而不是
        # 用 SRDF 的 carry 关节目标 —— 实测采样规划到 carry 偶尔会耗光预算
        # 失败（OMPL 随机性），笛卡尔路径是确定性的且完成度 100%。
        self.declare_parameter('tool_roll', math.pi)     # 工具 +Z 朝下
        self.declare_parameter('tool_pitch', 0.0)
        self.declare_parameter('tool_yaw', 0.0)
        # --- M13 放置 ---
        self.declare_parameter('place_standoff', 0.42)   # 机器人在放置点前方多远
        self.declare_parameter('place_approach_z', 0.72)
        # 下放高度：方块底面 = place_down_z − 0.10。放置台面顶已抬到 0.49
        # （见 test_world.sdf：台面原来 0.37~0.40，会被机械臂底板顶面 0.42 插进去），
        # 所以取 0.605 = 0.49 + 0.10 + 15mm 余量。
        self.declare_parameter('place_down_z', 0.605)
        # 【先退后进】放置前先在停车点后方 place_via_dist 米处把朝向转好，再直行
        # 进入停车点 —— 避免在贴着放置台处原地旋转（车体旋转外接半径 0.361m，
        # 而停车点到台子支柱只有 0.36m，原地转必然扫到台子）。
        self.declare_parameter('place_via', True)
        self.declare_parameter('place_via_dist', 0.5)
        # --- 通用 ---
        self.declare_parameter('arm_timeout', 60.0)
        self.declare_parameter('arm_server_timeout', 40.0)
        self.declare_parameter('nav_timeout', 150.0)
        # "已到达"的可信距离：消息里报的当前位姿离刚发出的目标超过这个值，
        # 就判为上一目标遗留的陈旧状态并忽略（见 nav_status_cb）
        self.declare_parameter('nav_arrive_slack', 0.30)
        # 里程计话题：用于把"小步移动"叠加到最近一次 AMCL 位姿上（见 _map_pose）
        self.declare_parameter('odom_topic', '/odom')
        # --- MoveIt 规划场景（碰撞体）---
        # 为什么要有这些开关：MoveIt 的碰撞检查只看 planning scene，而这里一直是
        # 空的 ⇒ 手臂真撞到方块/桌子，规划照样报 100% 成功，只能靠肉眼发现
        # "偶发蹭到方块顶部"。加进去之后这类接触会变成明确的规划失败。
        # 分步开关，便于逐级验证（对应落地方案的第①②④⑤步）：
        #   scene_tables   ①任务开始时加取件台（验证 diff 通路）
        #   scene_cube     ②识别后按方块位姿加方块
        #   scene_attach   ④抓取成功后摘出世界、挂到夹爪（touch_links 放行）
        #   scene_place_table ⑤放置台与放置侧，默认先关
        self.declare_parameter('scene_enable', True)
        self.declare_parameter('scene_frame', 'map')
        self.declare_parameter('scene_apply_service', '/apply_planning_scene')
        self.declare_parameter('scene_tables', True)
        self.declare_parameter('scene_place_table', False)
        self.declare_parameter('scene_cube', True)
        self.declare_parameter('scene_attach', True)
        self.declare_parameter('scene_attach_link', 'tool_link')
        # 方块放进碰撞世界时向外膨胀的量：吃掉参数/感知误差，避免"擦边"规划
        self.declare_parameter('scene_cube_padding', 0.005)
        # 台面顶面压低多少：竖直下抓时指尖本来就要贴到台面(0.48)，用真实台面高
        # 会让每次下压都被判碰撞。压 5mm 既不误报，也不失去防护意义。
        self.declare_parameter('scene_table_shrink', 0.005)
        # 方块位姿来源：truth=/gz_pose_info（Gazebo 真值），param=object_x/object_y
        self.declare_parameter('scene_cube_source', 'truth')
        self.declare_parameter('truth_topic', '/gz_pose_info')
        self.declare_parameter('truth_link', 'red_cube')
        # 真值与参数推算差超过这个值就认为坐标系不一致，退回参数
        self.declare_parameter('truth_max_dev', 0.50)
        # 方块中心离地高度（台面 0.48 + 半高 0.025）
        self.declare_parameter('object_z', 0.505)
        # 取件台几何（与 test_world.sdf 的 PickTable 一致）
        self.declare_parameter('scene_pick_table_xyz', [3.04, 3.0, 0.24])
        self.declare_parameter('scene_pick_table_size', [0.5, 0.5, 0.48])
        # 放置台几何（台面 + 支柱，与 test_world.sdf 的 PlaceTable 一致）
        self.declare_parameter('scene_place_top_xyz', [0.5, 4.5, 0.475])
        self.declare_parameter('scene_place_top_size', [0.6, 0.6, 0.03])
        self.declare_parameter('scene_place_pillar_xyz', [0.6, 4.5, 0.23])
        self.declare_parameter('scene_place_pillar_size', [0.12, 0.12, 0.46])

        # --- 吸附（DetachableJoint，仿真专用）---
        self.declare_parameter('attach_enabled', False)
        self.declare_parameter('attach_topic', '/red_cube/attach')
        self.declare_parameter('detach_topic', '/red_cube/detach')
        self.declare_parameter('detach_on_start', True)
        self.declare_parameter('attach_settle', 0.5)

        g = self.get_parameter
        self.obs_x = g('obs_x').value
        self.obs_y = g('obs_y').value
        self.obs_yaw = g('obs_yaw').value
        self.obs_frame = g('obs_frame').value
        self.detect_timeout = g('detect_timeout').value
        self.base_axle_offset = g('base_axle_offset').value
        self.max_retry = g('max_retry').value
        self.align_vel = g('align_vel').value
        self.rot_angle = g('rot_angle').value
        self.move_vel = g('move_vel').value
        self.move_duration = g('move_duration').value
        self.back_duration = g('back_duration').value
        self.settle_time = g('settle_time').value
        self.enable_pick = bool(g('enable_pick').value)
        self.grasp_forward = g('grasp_forward').value
        self.tool_x = g('tool_x').value
        self.tool_y = g('tool_y').value
        self.pregrasp_z = g('pregrasp_z').value
        self.grasp_z = g('grasp_z').value
        self.lift_z = g('lift_z').value
        # 护栏：抓取高度必须高于竖直下抓的自碰地板。注意这一句必须在上面两个
        # 参数都赋值之后再写（第一版写在 grasp_z 赋值之前，直接把节点崩了）。
        if self.grasp_z < 0.55:
            self.get_logger().error(
                f'grasp_z={self.grasp_z:.3f} 低于自碰地板 0.52 —— 手腕要擦过肩部，'
                f'目标位姿的所有 IK 解都会自碰。取件台面 0.48/方块中心 0.505 ⇒ 应设 0.58。'
                f'注意 demo_all.launch.py 的 grasp_z 会覆盖本节点默认值。')
        self.carry_tool_x = g('carry_tool_x').value
        self.carry_tool_z = g('carry_tool_z').value
        self.tool_rpy = (g('tool_roll').value, g('tool_pitch').value,
                         g('tool_yaw').value)
        self.place_standoff = g('place_standoff').value
        self.place_approach_z = g('place_approach_z').value
        self.place_down_z = g('place_down_z').value
        self.place_via = bool(g('place_via').value)
        self.place_via_dist = g('place_via_dist').value
        self.place_park = None                  # 放置停车点 (x, y, yaw)
        self.arm_timeout = g('arm_timeout').value
        self.arm_server_timeout = g('arm_server_timeout').value
        self.nav_timeout = g('nav_timeout').value
        self.nav_arrive_slack = g('nav_arrive_slack').value
        self.odom_topic = g('odom_topic').value
        self.attach_enabled = bool(g('attach_enabled').value)
        self.attach_topic = g('attach_topic').value
        self.detach_topic = g('detach_topic').value
        self.detach_on_start = bool(g('detach_on_start').value)
        self.attach_settle = g('attach_settle').value
        self.scene_enable = bool(g('scene_enable').value)
        self.scene_frame = g('scene_frame').value
        self.scene_tables = bool(g('scene_tables').value)
        self.scene_place_table = bool(g('scene_place_table').value)
        self.scene_cube_on = bool(g('scene_cube').value)
        self.scene_attach_on = bool(g('scene_attach').value)
        self.scene_attach_link = g('scene_attach_link').value
        self.scene_cube_padding = g('scene_cube_padding').value
        self.scene_table_shrink = g('scene_table_shrink').value
        self.scene_cube_source = g('scene_cube_source').value
        self.truth_topic = g('truth_topic').value
        self.truth_link = g('truth_link').value
        self.truth_max_dev = g('truth_max_dev').value
        self.object_z = g('object_z').value
        self.scene_pick_xyz = list(g('scene_pick_table_xyz').value)
        self.scene_pick_size = list(g('scene_pick_table_size').value)
        self.scene_place_top_xyz = list(g('scene_place_top_xyz').value)
        self.scene_place_top_size = list(g('scene_place_top_size').value)
        self.scene_place_pillar_xyz = list(g('scene_place_pillar_xyz').value)
        self.scene_place_pillar_size = list(g('scene_place_pillar_size').value)
        # --- 抓取可达性校验（防"空抓+假吸附"，详见 _check_grasp_reachable）---
        self.object_x = g('object_x').value     # 方块世界坐标（map 系）
        self.object_y = g('object_y').value
        self.grasp_tol = g('grasp_tol').value   # 允许偏差（米），超过就判失败
        self.grasp_compensate_xy = g('grasp_compensate_xy').value  # 可选补偿窗口
        self.grasp_auto = bool(g('grasp_auto').value)
        self.grasp_x_min = g('grasp_x_min').value
        self.grasp_x_mid = g('grasp_x_mid').value
        self.grasp_park_shortfall = g('grasp_park_shortfall').value
        self.grasp_reach_x_max = g('grasp_reach_x_max').value
        self.grasp_reach_y = g('grasp_reach_y').value
        self.grasp_auto_max_fwd = g('grasp_auto_max_fwd').value
        self.grasp_correct_max = g('grasp_correct_max').value
        self.grasp_correct_steps = int(g('grasp_correct_steps').value)
        self.grasp_correct_min = g('grasp_correct_min').value
        self.creep_vel = g('creep_vel').value
        self.creep_settle = g('creep_settle').value
        self.creep_deadband = g('creep_deadband').value
        self.grasp_correct_count = 0
        self.creep_start = None       # 微调蠕动开始时刻（None=不在蠕动）
        self.creep_dist = 0.0         # 本次要走的距离（带符号）
        self.creep_settle_until = None

        # 本次抓取实际用的工具目标（默认等于标定值，必要时被校验逻辑补偿）
        self.pick_tool_x = self.tool_x
        self.pick_tool_y = self.tool_y
        # "名义"抓取目标：自动标定会更新它（前进距离按实际观测距离算）
        self.grasp_target_x = self.tool_x
        self.grasp_target_y = self.tool_y
        self.robot_pose = None                  # (x, y, yaw)，来自 /amcl_pose
        self.odom_pose = None                   # (x, y, yaw)，来自 odom
        # AMCL 位姿 + 同一时刻的里程计位姿快照：(ax,ay,ath,ox,oy,oth)
        self.amcl_snapshot = None
        self.nav_goal = None                    # 最近发出的导航目标 (x, y)

        # ================= 接口 =================
        self.task_sub = self.create_subscription(
            TaskCommand, '/task_cmd', self.task_cmd_cb, 10)
        self.nav_status_sub = self.create_subscription(
            NavStatus, '/nav_status', self.nav_status_cb, 10)
        self.det_sub = self.create_subscription(
            ObjectDetection, '/object_detection', self.det_cb, 10)
        # 机器人实际位姿（AMCL）：抓取前用它把"方块世界坐标"换算到 base 系，
        # 判断机械臂到底够不够得到（详见 _check_grasp_reachable）
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.odom_cb, 10)
        self.nav_pub = self.create_publisher(NavGoal, '/nav_cmd', 10)
        self.status_pub = self.create_publisher(TaskStatus, '/task_status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # 识别使能：只在识别窗口打开，其余阶段（导航/抓取/搬运/放置）关掉，
        # 省掉每帧最多 12 遍 detectMarkers 的 CPU 开销。
        # 用 transient_local（锁存）保证 detector 晚启动也能收到最后一次状态。
        self.detect_enable_pub = self.create_publisher(
            Bool, self.get_parameter('detect_enable_topic').value,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.attach_pub = self.create_publisher(Empty, self.attach_topic, 10)
        self.detach_pub = self.create_publisher(Empty, self.detach_topic, 10)
        self.arm_client = ActionClient(self, ArmTask, '/arm_task')
        # 规划场景服务（加/删碰撞体、attach/detach 都走它，diff 增量）
        self.scene_cli = self.create_client(
            ApplyPlanningScene, g('scene_apply_service').value)
        # 方块真值（Gazebo /world/<world>/pose/info 桥出的 TFMessage）
        self.truth_cube = None
        self._truth_logged = False
        if self.scene_enable and self.scene_cube_source == 'truth':
            self.truth_sub = self.create_subscription(
                TFMessage, self.truth_topic, self.truth_cb, 10)

        # ================= 状态 =================
        self.state = TaskStatus.IDLE
        self.phase = '空闲'
        self.stage = 'IDLE'          # 内部状态机
        self.task = None
        self.target_class = ''
        self.retry = 0
        self.current_action = ''
        self.align_target_angle = 0.0
        self.det_deadline = None
        self.align_start = None
        self.settle_start = None
        self.settle_next = None
        self.stage_deadline = None
        self.object_pose = None
        # 手臂动作序列驱动状态
        self.ops = None
        self.op_idx = 0
        self.op_kind = None          # None / 'GOAL' / 'WAIT' / 'SERVER_WAIT'
        self.op_deadline = None
        self.op_wait_until = None
        self.op_goal_handle = None
        self.ops_done = None         # (state, phase)
        self.ops_then = None

        self.create_timer(0.1, self.timer_cb)

        # 上电先脱开吸附：DetachableJoint 插件在 Configure 时就会把方块
        # 和夹爪连起来，如果不管它，方块从第一帧起就被夹爪拖着走。
        if self.attach_enabled and self.detach_on_start:
            self.startup_detach_left = 3
            self.create_timer(1.0, self.startup_detach_cb)

        self._pub_status()
        self.get_logger().info(
            f'task_orchestrator 启动：观测点=({self.obs_x},{self.obs_y}) '
            f'yaw={self.obs_yaw} | 抓取前进 {self.grasp_forward}m | '
            f'抓取工具位 ({self.tool_x},{self.tool_y}) z={self.grasp_z} | '
            f'转心偏移 {self.base_axle_offset:.2f}m（base_link↔导航坐标系）| '
            f'吸附={"开" if self.attach_enabled else "关"}')

    # ================= 状态发布 =================
    def _pub_status(self, state=None, phase=None):
        if state is not None:
            self.state = state
        if phase is not None:
            self.phase = phase
        msg = TaskStatus()
        msg.status = self.state
        msg.phase = self.phase
        self.status_pub.publish(msg)

    def _set_detect(self, on, why=''):
        """开关物体识别（发给 aruco_detector；锁存话题，重复发无害）"""
        self.detect_enable_pub.publish(Bool(data=bool(on)))
        self.get_logger().info(
            f'物体识别{"开启" if on else "暂停"}' + (f'（{why}）' if why else ''))

    def _now(self):
        return self.get_clock().now()

    def _pub_empty(self, publisher, what):
        publisher.publish(Empty())
        self.get_logger().info(f'已发布 {what}')

    # ================= 任务入口 =================
    def task_cmd_cb(self, msg: TaskCommand):
        if self.stage != 'IDLE':
            self.get_logger().warn(f'任务进行中（stage={self.stage}），忽略新命令')
            return
        self.task = msg
        self.target_class = msg.object_id
        self._set_detect(False, '导航阶段不需要识别')
        self.retry = 0
        self.object_pose = None
        self.get_logger().info(
            f'收到任务：抓取 {msg.object_id} → 放置 '
            f'({msg.place_x:.2f},{msg.place_y:.2f}) '
            f'yaw={msg.place_yaw:.2f} frame={msg.place_frame_id}')
        self._scene_add_tables()
        self._scene_set_cube(None)      # 清掉上一轮可能残留的方块
        self._nav_to_object()

    # ================= 导航 =================
    def _to_nav_frame(self, x, y, yaw):
        """base_link 目标 → 导航/AMCL 坐标系（轮轴中点）的目标。

        本车驱动轮在 base_link 的 x=-base_length/4=-0.15，差速原地转的转心就是
        轮轴中点，URDF 把根坐标系 base_footlink（= Nav2 的 robot_base_frame）
        放在转心上（见 mobile_base.xacro 的 base_joint），所以：

            p_base_link = p_base_footlink + R(yaw)·(offset, 0)

        本节点里所有目标点（观测点、抓取点、放置接近点/停车点）说的都是
        "机械臂那一侧"的位置，发给导航前要换算到 base_footlink，否则每个点
        都会偏 0.15m（yaw=0 时偏在 x 上）。这样已标定的 obs_x/place_x 数值
        一个都不用动。
        """
        if not self.base_axle_offset:
            return float(x), float(y)
        return (x - self.base_axle_offset * math.cos(yaw),
                y - self.base_axle_offset * math.sin(yaw))

    def _to_base_link(self, x, y, yaw):
        """导航/AMCL 坐标系（轮轴中点）的位姿 → base_link 位姿（_to_nav_frame 逆）。

        /amcl_pose 和 /odom 报的都是 base_footlink，而 _map_pose() 的语义是
        "base_link 在 map 系的位姿"（下游 _object_in_base / 抓取校验都按 base_link
        算机械臂够不够得到），所以这里统一加回偏移，下游不用改。
        """
        if not self.base_axle_offset:
            return float(x), float(y), float(yaw)
        return (x + self.base_axle_offset * math.cos(yaw),
                y + self.base_axle_offset * math.sin(yaw), float(yaw))

    def _send_nav(self, x, y, yaw, frame, state, phase, stage):
        gx, gy = self._to_nav_frame(x, y, yaw)
        g = NavGoal()
        g.frame_id = frame
        g.x = float(gx)
        g.y = float(gy)
        g.yaw = float(yaw)
        self.nav_goal = (float(gx), float(gy))  # 用于校验"已到达"是否可信
        self._pub_status(state, phase)
        self.stage = stage
        self.stage_deadline = self._now() + rclpy.duration.Duration(
            seconds=self.nav_timeout)
        if abs(gx - x) > 1e-9 or abs(gy - y) > 1e-9:
            self.get_logger().info(
                f'{phase}：base_link ({x:.2f},{y:.2f}) → 导航目标 '
                f'({gx:.2f},{gy:.2f}) yaw={g.yaw:.2f} frame={frame}'
                f'（差 {self.base_axle_offset:.2f}m 转心偏移）')
        else:
            self.get_logger().info(
                f'{phase}：({gx:.2f},{gy:.2f}) yaw={g.yaw:.2f} frame={frame}')
        self.nav_pub.publish(g)

    def _nav_to_object(self):
        self._send_nav(self.obs_x, self.obs_y, self.obs_yaw, self.obs_frame,
                       TaskStatus.NAV_TO_OBJECT, '导航到物体观测点', 'NAV_OBS')

    def _nav_to_grasp(self):
        """去抓取点：目标由**方块在 map 系的位置**直接反解，而不是在观测点上平移。

        为什么不能用"观测点 + 前进 fwd"（旧写法）：`_object_in_base()` 给的是
        方块在**当前朝向**下的 base 系坐标，它随车头朝向可变。观测点离方块 ~1m、
        抓取点 ~0.4m，车头若两次差 20°（yaw_goal_tolerance 允许 ±11°），同一个
        方块就会从 base 系 y=0.004 变成 y=0.292 —— 实测就是这样报"横向偏 288mm"
        而失败的（0.96m × (sin10.6°+sin11.2°) ≈ 0.28m，与日志完全吻合）。

        正确做法：要让方块落在 base 系 (tool_x, tool_y)，机器人 base_link 应停在
            p_robot = p_object − R(obs_yaw)·(tool_x, tool_y)
        这个式子只跟方块的世界位置和**目标朝向**有关，与到位时的实际朝向无关，
        所以抓取点不会因为车头歪几度就飘掉；抓取校验里的 base 系偏差也就回到
        "真实的到位位置误差"这一个来源。
        """
        fwd = self.grasp_forward
        if self.grasp_auto:
            c, s = math.cos(self.obs_yaw), math.sin(self.obs_yaw)
            gx = self.object_x - (self.tool_x * c - self.tool_y * s)
            gy = self.object_y - (self.tool_x * s + self.tool_y * c)
            self.grasp_target_x = self.tool_x
            self.grasp_target_y = self.tool_y
            fwd = math.hypot(gx - self.obs_x, gy - self.obs_y)
            if fwd > self.grasp_auto_max_fwd:
                self.get_logger().warn(
                    f'由方块位置反解出的抓取点离观测点 {fwd:.2f}m'
                    f'（上限 {self.grasp_auto_max_fwd:.2f}m）→ 方块参数 '
                    f'object_x/object_y=({self.object_x},{self.object_y}) 可能不对，'
                    f'退回固定前进 {self.grasp_forward:.3f}m')
                fwd = self.grasp_forward
                gx = self.obs_x + fwd * math.cos(self.obs_yaw)
                gy = self.obs_y + fwd * math.sin(self.obs_yaw)
            else:
                self.get_logger().info(
                    f'抓取点由方块 ({self.object_x},{self.object_y}) 反解：'
                    f'base_link 停在 ({gx:.2f},{gy:.2f})（离观测点 {fwd:.2f}m），'
                    f'届时方块应落在 base 系 '
                    f'({self.grasp_target_x:.3f},{self.grasp_target_y:.3f})')
        else:
            gx = self.obs_x + fwd * math.cos(self.obs_yaw)
            gy = self.obs_y + fwd * math.sin(self.obs_yaw)
        self._send_nav(gx, gy, self.obs_yaw, self.obs_frame,
                       TaskStatus.NAV_TO_GRASP, '导航到抓取点', 'NAV_GRASP')

    def _object_in_base(self):
        """方块在当前机器人 base 系下的位置 (x, y, 距离)；拿不到位姿返回 None。"""
        pose = self._map_pose()
        if pose is None:
            return None
        rx, ry, ryaw = pose
        dx, dy = self.object_x - rx, self.object_y - ry
        c, s = math.cos(-ryaw), math.sin(-ryaw)
        bx = c * dx - s * dy
        by = s * dx + c * dy
        return bx, by, math.hypot(bx, by)

    def _nav_to_place(self):
        # 放置点前方 standoff 处停车，物体落在 base 系 (standoff, 0)
        px = self.task.place_x - self.place_standoff * math.cos(self.task.place_yaw)
        py = self.task.place_y - self.place_standoff * math.sin(self.task.place_yaw)
        self.place_park = (px, py, self.task.place_yaw)
        frame = self.task.place_frame_id or 'map'
        if self.place_via and self.place_via_dist > 0.0:
            # 【先退后进】先到停车点**后方** place_via_dist 米处（同一朝向），
            # 再直行进入停车点。
            # 为什么必须这样做：从抓取点过来的航向与放置朝向差约 150°，Nav2 会在
            # **到点之后**做原地对齐（RotateToGoal）。而停车点离放置台支柱只有
            # 0.36 m，车体旋转外接半径是 0.361 m —— 原地转就等于拿车角去扫台子
            # （实测正好差 1 mm 扫上，日志/RViz 显示"被放置台卡住"）。
            # 先在远处把朝向转好、再直着开进去，贴台子处就不需要旋转了。
            vx = px - self.place_via_dist * math.cos(self.task.place_yaw)
            vy = py - self.place_via_dist * math.sin(self.task.place_yaw)
            self._send_nav(vx, vy, self.task.place_yaw, frame,
                           TaskStatus.NAV_TO_PLACE,
                           f'导航到放置点接近点（后退 {self.place_via_dist:.2f}m 处先转好朝向）',
                           'NAV_PLACE_VIA')
            return
        self._send_nav(px, py, self.task.place_yaw, frame,
                       TaskStatus.NAV_TO_PLACE, '导航到放置点', 'NAV_PLACE')

    def nav_status_cb(self, msg: NavStatus):
        if msg.status == NavStatus.FAILED:
            self.get_logger().error(f'导航失败（stage={self.stage}）')
            self._fail('导航失败')
            return
        if msg.status != NavStatus.ARRIVED:
            return
        # 【到位可信度校验】"已到达"必须来自**刚发出的那个目标**。
        # 为什么需要：nav_commander 的状态是锁存的，若它把上一个目标的 ARRIVED
        # 重播出来（旧版本每 200ms 重播一次终态），编排器刚发出新目标、stage 刚切到
        # 导航态就会立刻收到这条陈旧消息 → 误判到位、跳过等待。实测抓取点目标发出
        # 2ms 后就"已到位"，机器人只走了 6cm。这里用消息里带的当前位姿反查：
        # 离目标还远就判定为陈旧状态，忽略。
        if self.nav_goal is not None and msg.current_pose.header.frame_id:
            d = math.hypot(msg.current_pose.pose.position.x - self.nav_goal[0],
                           msg.current_pose.pose.position.y - self.nav_goal[1])
            if d > self.nav_arrive_slack:
                self.get_logger().warn(
                    f'收到"已到达"但当前位姿离目标还有 {d:.2f} m'
                    f'（>{self.nav_arrive_slack:.2f} m）→ 判为上一目标遗留的陈旧状态，忽略',
                    throttle_duration_sec=2.0)
                return
        if self.stage == 'NAV_OBS':
            self._enter_settle(TaskStatus.RUNNING, '稳定等待', 'DETECT')
        elif self.stage == 'NAV_GRASP':
            self._enter_settle(TaskStatus.GRASPING, '抓取点稳定等待', 'PICK')
        elif self.stage == 'NAV_PLACE_VIA':
            # 接近点到位（朝向已在远处转好）→ 直行进入停车点，贴台子处不再旋转
            px, py, pyaw = self.place_park
            self._send_nav(px, py, pyaw, self.task.place_frame_id or 'map',
                           TaskStatus.NAV_TO_PLACE, '导航到放置点（直行进入）',
                           'NAV_PLACE')
        elif self.stage == 'NAV_PLACE':
            self._enter_settle(TaskStatus.PLACING, '放置点稳定等待', 'PLACE')

    def _enter_settle(self, state, phase, next_stage):
        self.get_logger().info(f'已到位，等待稳定 {self.settle_time}s 后进入 {next_stage}')
        self._pub_status(state, phase)
        self.settle_start = self._now()
        self.stage = 'SETTLE'
        self.settle_next = next_stage

    # ================= 定位 =================
    def amcl_cb(self, msg: PoseWithCovarianceStamped):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.robot_pose = (msg.pose.pose.position.x,
                           msg.pose.pose.position.y, yaw)
        # 同时刻的里程计位姿一起记下，之后用增量推到"现在"
        o = self.odom_pose or (0.0, 0.0, 0.0)
        self.amcl_snapshot = self.robot_pose + o

    def odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom_pose = (msg.pose.pose.position.x,
                          msg.pose.pose.position.y, yaw)

    def _map_pose(self):
        """当前 map 系位姿估计 = 最近一次 /amcl_pose ⊕ 之后的 /odom 增量。

        为什么不能直接用 /amcl_pose：AMCL 只在移动超过 update_min_d/_a
        （原配置 0.25m / 0.2rad）时才更新并发布位姿。抓取阶段全是 5~20cm 的小步
        移动，触发不了更新 → /amcl_pose 长时间冻结（实测比 Nav2 的实际位置落后
        23cm），用它折算方块位置会系统性偏大，抓取连续失败。
        叠加轮式里程计增量后，小步移动也算得准（0.2m 内里程计几乎无漂移）。
        """
        if self.robot_pose is None:
            return None
        if self.amcl_snapshot is None or self.odom_pose is None:
            return self._to_base_link(*self.robot_pose)
        ax, ay, ath, ox, oy, oth = self.amcl_snapshot
        cx, cy, cth = self.odom_pose
        dx, dy, dth = cx - ox, cy - oy, cth - oth
        c, s = math.cos(ath), math.sin(ath)
        # 增量是"位移"，与参考点无关，直接叠加；最后统一换算成 base_link 位姿
        return self._to_base_link(ax + c * dx - s * dy, ay + s * dx + c * dy,
                                  ath + dth)

    # ================= 识别 =================
    def det_cb(self, msg: ObjectDetection):
        self.get_logger().info(
            f'[det_cb] 收到 class={msg.class_id} | stage={self.stage} '
            f'phase={self.phase} | target={self.target_class}',
            throttle_duration_sec=2.0)
        if (self.stage == 'DETECT' and self.phase == '识别中'
                and msg.class_id == self.target_class):
            self.object_pose = msg.pose
            self.get_logger().info(
                f'识别到 {msg.class_id}，位姿(camera 系)='
                f'({msg.pose.pose.position.x:.3f},{msg.pose.pose.position.y:.3f},'
                f'{msg.pose.pose.position.z:.3f})')
            self._set_detect(False, '已识别到目标，抓取及之后不再需要')
            self._scene_cube_on_detect()
            self._begin_pick()

    def _begin_pick(self):
        self.grasp_correct_count = 0        # 本轮抓取的"补进"计数清零
        if not self.enable_pick:
            # 调试模式：跳过机械臂，直接当"已抓起"（等价于 M9 的占位行为）
            self.get_logger().warn('enable_pick=false，跳过抓取，直接去放置点')
            self._pub_status(TaskStatus.PICKED, '抓取完成（已跳过）')
            self._nav_to_place()
            return
        self._nav_to_grasp()

    # ================= 手臂动作序列 =================
    def _goal_named(self, group, target):
        goal = ArmTask.Goal()
        goal.task_type = ArmTask.Goal.NAMED
        goal.group_name = group
        goal.named_target = target
        goal.timeout = float(self.arm_timeout)
        return goal

    def _goal_pose(self, x, y, z, cartesian):
        goal = ArmTask.Goal()
        goal.task_type = ArmTask.Goal.POSE
        goal.target_pose.position.x = float(x)
        goal.target_pose.position.y = float(y)
        goal.target_pose.position.z = float(z)
        qx, qy, qz, qw = quat_from_rpy(*self.tool_rpy)
        goal.target_pose.orientation.x = qx
        goal.target_pose.orientation.y = qy
        goal.target_pose.orientation.z = qz
        goal.target_pose.orientation.w = qw
        goal.use_cartesian = bool(cartesian)
        goal.timeout = float(self.arm_timeout)
        return goal

    @staticmethod
    def _op_goal(label, goal):
        return {'kind': 'goal', 'label': label, 'goal': goal}

    @staticmethod
    def _op_wait(label, seconds):
        return {'kind': 'wait', 'label': label, 'seconds': seconds}

    @staticmethod
    def _op_fn(label, fn):
        return {'kind': 'fn', 'label': label, 'fn': fn}

    def _check_grasp_reachable(self):
        """抓取前校验：方块到底在不在机械臂够得到的地方。

        为什么必须有这一步（实测事故）：抓取是**固定点抓取**——观测点 → 沿朝向
        前进 grasp_forward → 伸手到 base 系 (tool_x, tool_y)。它隐含假设"方块
        就在 (tool_x, tool_y)"，这只在观测点/前进距离与方块实际位置匹配时成立。
        一旦观测点被改成别的值（例如 obs_x=1.84 / obs_y=2.82），到抓取点时方块
        落在 base 系 x≈0.79，而手臂只伸到 0.42 —— 夹爪夹了个空。
        更糟的是 /red_cube/attach 是**无条件发布**的：DetachableJoint 会把方块
        从它当时所在的位置直接焊到 hand_link 上，于是日志/界面都显示"抓取成功"，
        实际机械臂离方块还有 0.37m（"空抓"）。所以这里在动手之前就算清楚：
        够不到就直接失败，并把该改哪个参数说清楚。

        返回 True = 可以继续抓取（必要时已把工具目标改成补偿值）。
        """
        self.pick_tool_x, self.pick_tool_y = self.grasp_target_x, self.grasp_target_y
        if not self.enable_pick:
            return 'ok'
        pose = self._map_pose()
        if pose is None:
            self.get_logger().warn(
                '还没收到 /amcl_pose，无法做抓取可达性校验（跳过；'
                '若观测点被改过，可能出现"空抓+假吸附"）')
            return 'ok'

        rx, ry, ryaw = pose
        dx, dy = self.object_x - rx, self.object_y - ry
        c, s = math.cos(-ryaw), math.sin(-ryaw)
        bx = c * dx - s * dy          # 方块在 base 系的位置
        by = s * dx + c * dy
        ex = bx - self.grasp_target_x
        ey = by - self.grasp_target_y
        err = math.hypot(ex, ey)
        self.get_logger().info(
            f'抓取校验：方块在 base 系 ({bx:.3f},{by:.3f})，抓取目标 '
            f'({self.grasp_target_x:.3f},{self.grasp_target_y:.3f})，偏差 '
            f'{err * 1000:.0f} mm（机器人实际位姿 '
            f'({rx:.2f},{ry:.2f},{math.degrees(ryaw):.1f}°)）')

        # 判据分三档（核心思想：**只要方块实测位置在机械臂够得到的区间里，
        # 就直接按实测位置抓**，不要去比对"名义标定值"——名义值只是用于决定
        # 前进距离，它扛不住到位姿态的整体漂移）。
        #   ① 偏差 ≤ grasp_tol         → 用名义值（几何完全吻合）
        #   ② 偏差 ≤ grasp_compensate  → **改用实测位置抓**（IK 目标 = 实测）
        #   ③ 超过                    → 失败（几何整体不对，例如观测点改错/定位跳变）
        # 实测教训：观测点→抓取点只前进不横移，但机器人到位时朝向偏了 16°，
        # 方块横向坐标就从 -0.022 变成 +0.138（偏 171 mm）；而 (0.481,0.138)
        # 完全在可靠区间内，机械臂够得到 —— 却被原来只有 80 mm 的②档挡住了。
        if err <= self.grasp_tol:
            pass
        else:
            tol = self.grasp_compensate_xy
            if abs(ex) <= tol and abs(ey) <= tol:
                lvl = 'warn' if err > 0.10 else 'info'
                msg = (f'抓取点偏 {err * 1000:.0f} mm（到位姿态漂移）→ '
                       f'本次抓取直接改用实测位置 ({bx:.3f},{by:.3f}) 作为工具目标')
                (self.get_logger().warn if lvl == 'warn'
                 else self.get_logger().info)(msg)
                self.pick_tool_x, self.pick_tool_y = bx, by
            else:
                # 顺手算出"要让方块落在抓取点上，参数应该怎么改"，让报错可操作
                psi = self.obs_yaw
                sug_fwd = max(0.0, (bx + self.grasp_forward) - self.tool_x) \
                    if not self.grasp_auto else None
                self._fail(
                    f'方块不在抓取点：base 系应为 '
                    f'({self.grasp_target_x:.3f},{self.grasp_target_y:.3f})，'
                    f'实际 ({bx:.3f},{by:.3f})，偏 {err * 1000:.0f} mm，'
                    f'超出机械臂可达/补偿范围（固定点竖直下抓的可靠区间约 '
                    f'x∈[0.30,{self.grasp_reach_x_max:.2f}]、'
                    f'|y|≤{self.grasp_reach_y:.2f}）。'
                    + (f'自动标定已开启，请检查 /amcl_pose 与 object_x/object_y；'
                       f'或改参数 obs_x/obs_y 使观测点正对方块。'
                       if self.grasp_auto else
                       f'请重标 grasp_forward/tool_x/tool_y（当前 '
                       f'{self.grasp_forward:.3f}/{self.tool_x:.3f}/'
                       f'{self.tool_y:.3f}，obs_yaw={psi:.3f}）'))
                return 'fail'

        # 最后一道：最终工具目标必须落在"竖直下抓安全窗口"内（见参数处注释）。
        # 越界就前后微调一小段再来 —— 上界/下界是两个不同的物理原因
        # （够不着 / 撞自己），窗口只有 3cm，必须双向闭环。
        if not (self.grasp_x_min <= self.pick_tool_x <= self.grasp_reach_x_max
                and abs(self.pick_tool_y) <= self.grasp_reach_y):
            if abs(by) > self.grasp_reach_y:
                self._fail(
                    f'方块横向偏到 base 系 y={by:.3f}，超出机械臂可靠侧向范围 '
                    f'±{self.grasp_reach_y:.2f} m（机器人不能横移，只能靠手臂侧够）'
                    f'——观测点没正对方块，请调整 obs_x/obs_y 或 obs_yaw')
                return 'fail'
            rx, ry, ryaw = pose
            need = bx - self.grasp_x_mid          # >0 方块太远要往前，<0 太近要往后退
            step = need + math.copysign(self.grasp_park_shortfall, need)
            if abs(step) < self.grasp_correct_min:
                step = math.copysign(self.grasp_correct_min, need)
            if (abs(step) <= self.grasp_correct_max
                    and self.grasp_correct_count < self.grasp_correct_steps):
                self.grasp_correct_count += 1
                self.creep_dist = need          # 精确走这么多，不再加"到位短差补偿"
                if abs(need) < self.creep_deadband:
                    self.get_logger().info(
                        f'方块在 base 系 x={bx:.3f}，与窗口中点 {self.grasp_x_mid:.3f} '
                        f'只差 {need * 1000:+.1f} mm（< {self.creep_deadband * 1000:.0f} mm）'
                        f'→ 不再微调，直接按实测位置抓')
                    self.pick_tool_x, self.pick_tool_y = bx, by
                    self.stage = 'PICK'
                    self._run_ops(self._pick_ops(),
                                  TaskStatus.PICKED, '抓取完成，准备搬运',
                                  then=self._nav_to_place)
                    return 'done'
                self.creep_start = self._now()
                self.creep_settle_until = None
                self.stage = 'PICK_CREEP'
                self.get_logger().warn(
                    f'方块在 base 系 x={bx:.3f}，安全窗口 '
                    f'[{self.grasp_x_min:.2f},{self.grasp_reach_x_max:.2f}] → '
                    f'直接蠕动{"前进" if need > 0 else "后退"} '
                    f'{abs(need) * 1000:.0f} mm（{self.creep_vel} m/s，约 '
                    f'{abs(need) / self.creep_vel:.1f}s，不走导航）'
                    f'（第 {self.grasp_correct_count}/{self.grasp_correct_steps} 次）')
                return 'retry'
            self._fail(
                f'方块在 base 系 x={bx:.3f}，安全窗口 '
                f'[{self.grasp_x_min:.2f},{self.grasp_reach_x_max:.2f}]，'
                f'需要{"前进" if need > 0 else "后退"} {abs(step) * 1000:.0f} mm，'
                f'超出单次微调上限 {self.grasp_correct_max * 1000:.0f} mm'
                f'（或已用完 {self.grasp_correct_steps} 次）')
            return 'fail'

        return 'ok'

    def _pick_ops(self):
        ops = [
            self._op_goal('预抓取',
                          self._goal_pose(self.pick_tool_x, self.pick_tool_y,
                                          self.pregrasp_z, False)),
            self._op_goal('张开夹爪', self._goal_named('gripper', 'gripper_open')),
            self._op_goal('下压抓取',
                          self._goal_pose(self.pick_tool_x, self.pick_tool_y,
                                          self.grasp_z, True)),
            self._op_goal('合拢夹爪',
                          self._goal_named('gripper', 'gripper_grasp')),
        ]
        if self.attach_enabled:
            ops.append(self._op_fn('吸附物体', self._do_attach))
            ops.append(self._op_wait('吸附稳定', self.attach_settle))
        ops += [
            self._op_goal('抬起',
                          self._goal_pose(self.pick_tool_x, self.pick_tool_y,
                                          self.lift_z, True)),
            # 收起搬运：走**笛卡尔**到 carry 的工具位姿（确定性强于采样规划到 carry 关节目标）
            # 【必须用命名关节目标，不能发位姿】这个"收回身前"的位姿，其 IK 解
            # 全部落在 joint6 = ±π 的边界上（SRDF carry 自己的 joint6=-3.1416 就
            # 比限位 -π 还小 1e-5）⇒ MoveIt 的 IK 判越界 ⇒ 笛卡尔插补一步都算不
            # 出来（实测 followed 0.000%、只有 1 个点）。而命名关节目标**不走 IK**，
            # 规划器 20ms 就能到（日志里的 "arm -> carry 规划成功" 就是它）。
            # 注意 carry_tool_z 保持与 SRDF carry 的工具高度一致（0.70），
            # 否则位姿与命名目标就不是同一处了。
            self._op_goal('收起搬运', self._goal_named('arm', 'carry')),
        ]
        return ops

    def _place_ops(self):
        ops = [
            # 预放置走**标定关节目标**（SRDF: place_approach）——不经过 IK。
            # 演进过程：①关节空间+孤立 IK → IK 只给出压限位/自碰的解；
            # ②笛卡尔 → 插补在 joint6=±π 边界处断（88.9%）；③现在是标定关节目标：
            # 离线用全局 IK 求出限位余量 0.578rad 的解写进 SRDF，运行时规划器直接
            # 规划到该构型（跟 carry 一样 20ms 搞定），彻底没有 IK 收敛问题。
            # 真正的根因是 KDL 的 setFromIK 会返回**未收敛的错解**（实测偏 180mm
            # 仍报成功），commander 现已加 FK 自校验拦截这类解。
            self._op_goal('预放置', self._goal_named('arm', 'place_approach')),
            # 下放同样走**标定关节目标**（SRDF: place_down）。实测笛卡尔只能算到
            # 96.2%（最后 1cm 的 IK 收敛不了，与 pre-place 同一类问题）。
            # 这里用关节插值的代价：离线验过工具轨迹最多偏离直线 5.4mm（只在 x 向、
            # y 恒为 0），对"把方块放到台面上"完全够用。
            self._op_goal('下放', self._goal_named('arm', 'place_down')),
            self._op_goal('松开夹爪', self._goal_named('gripper', 'gripper_open')),
        ]
        if self.attach_enabled:
            ops.append(self._op_fn('脱开物体', self._do_detach))
            ops.append(self._op_wait('脱开稳定', self.attach_settle))
        ops += [
            # 退回 = 回到标定的预放置构型（已知良好、余量 0.578rad），同样不走 IK
            self._op_goal('退回', self._goal_named('arm', 'place_approach')),
        ]
        return ops

    def _run_ops(self, ops, done_state, done_phase, then=None):
        self.ops = list(ops)
        self.op_idx = 0
        self.op_kind = None
        self.ops_done = (done_state, done_phase)
        self.ops_then = then
        self.op_server_deadline = self._now() + rclpy.duration.Duration(
            seconds=self.arm_server_timeout)
        self._op_start()

    def _op_start(self):
        if self.ops is None:
            return
        while True:
            if self.op_idx >= len(self.ops):
                st, ph = self.ops_done
                self.ops = None
                self.op_kind = None
                self._pub_status(st, ph)
                then, self.ops_then = self.ops_then, None
                if then:
                    then()
                return
            op = self.ops[self.op_idx]
            self._pub_status(None, op['label'])

            if op['kind'] == 'fn':
                self.get_logger().info(f'[手臂序列] {op["label"]}')
                try:
                    op['fn']()
                except Exception as e:  # noqa: BLE001
                    self._fail(f'{op["label"]} 异常: {e}')
                    return
                self.op_idx += 1
                continue

            if op['kind'] == 'wait':
                self.get_logger().info(
                    f'[手臂序列] {op["label"]}（等 {op["seconds"]}s）')
                self.op_wait_until = self._now() + rclpy.duration.Duration(
                    seconds=float(op['seconds']))
                self.op_kind = 'WAIT'
                return

            # goal
            if not self.arm_client.server_is_ready():
                self.get_logger().warn(
                    f'/arm_task 服务还没就绪，等待中（{op["label"]}）',
                    throttle_duration_sec=2.0)
                self.op_kind = 'SERVER_WAIT'
                return
            self.get_logger().info(f'[手臂序列] {op["label"]} → 发送 /arm_task')
            self.op_kind = 'GOAL'
            self.op_deadline = self._now() + rclpy.duration.Duration(
                seconds=self.arm_timeout)
            fut = self.arm_client.send_goal_async(op['goal'])
            fut.add_done_callback(self._on_goal_response)
            return

    def _on_goal_response(self, future):
        if self.ops is None:
            return
        try:
            handle = future.result()
        except Exception as e:  # noqa: BLE001
            self._fail(f'发送手臂目标异常: {e}')
            return
        if not handle.accepted:
            self._fail(f'{self.ops[self.op_idx]["label"]} 被拒绝（可能上一动作未结束）')
            return
        self.op_goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        if self.ops is None:
            return
        label = self.ops[self.op_idx]['label']
        try:
            wrapped = future.result()
            ok = bool(wrapped.result.success)
            msg = wrapped.result.message
        except Exception as e:  # noqa: BLE001
            self._fail(f'{label} 结果异常: {e}')
            return
        self.op_kind = None
        self.op_goal_handle = None
        if not ok:
            self._fail(f'{label} 失败: {msg}')
            return
        self.get_logger().info(f'[手臂序列] {label} 完成')
        self.op_idx += 1
        self._op_start()

    def _cancel_arm(self):
        if self.op_goal_handle is not None:
            try:
                self.op_goal_handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
        self.op_goal_handle = None

    # ================= MoveIt 规划场景（碰撞体） =================
    # 为什么要把方块/台子放进来：MoveIt 的碰撞检查只看 planning scene，而这里
    # 之前一直是空的 ⇒ 手臂真撞到方块/桌子，规划照样报 100% 成功，只能靠肉眼
    # 发现"偶发蹭到方块顶部"。加进去之后这类接触会变成明确的规划失败。

    def truth_cb(self, msg):
        """收 /gz_pose_info（Gazebo 真值），取方块的世界位姿。"""
        for tr in msg.transforms:
            if tr.child_frame_id.strip('/') == self.truth_link:
                t = tr.transform.translation
                self.truth_cube = (t.x, t.y, t.z)
                if not self._truth_logged:
                    self._truth_logged = True
                    self.get_logger().info(
                        f'已收到方块真值 /gz_pose_info：{self.truth_link} @ '
                        f'({t.x:.3f},{t.y:.3f},{t.z:.3f})')
                return

    def _make_box(self, obj_id, xyz, size, operation=None):
        """构造一个 box 碰撞体；REMOVE 时只需要 id。"""
        if operation is None:
            operation = CollisionObject.ADD
        co = CollisionObject()
        co.header.frame_id = self.scene_frame
        co.header.stamp = self.get_clock().now().to_msg()
        co.id = obj_id
        if operation != CollisionObject.REMOVE:
            co.primitives = [SolidPrimitive(
                type=SolidPrimitive.BOX,
                dimensions=[float(v) for v in size])]
            p = Pose()
            p.position.x, p.position.y, p.position.z = [float(v) for v in xyz]
            p.orientation.w = 1.0
            co.primitive_poses = [p]
        co.operation = operation
        return co

    def _apply_scene(self, world=None, attached=None, why=''):
        """把一份**增量**场景发给 /apply_planning_scene。"""
        if not self.scene_enable:
            return
        if not self.scene_cli.service_is_ready():
            self.get_logger().warn(f'规划场景服务未就绪，跳过场景更新（{why}）')
            return
        diff = PlanningScene()
        diff.is_diff = True       # 必须增量，否则会把别人加的场景整份清空
        if world is not None:
            diff.world = world
        if attached is not None:
            diff.robot_state.attached_collision_objects = attached
        fut = self.scene_cli.call_async(ApplyPlanningScene.Request(scene=diff))
        fut.add_done_callback(lambda f: self._scene_done(f, why))

    def _scene_done(self, fut, why):
        try:
            ok = fut.result().success
        except Exception as e:      # noqa: BLE001
            self.get_logger().warn(f'规划场景更新失败（{why}）: {e}')
            return
        if ok:
            self.get_logger().info(f'规划场景已更新（{why}）')
        else:
            self.get_logger().warn(f'规划场景更新被拒绝（{why}）')

    def _scene_add_tables(self):
        """step①：任务开始时把静态台子放进碰撞世界。"""
        if not (self.scene_enable and self.scene_tables):
            return
        objs = []
        x, y, z = self.scene_pick_xyz
        sx, sy, sz = self.scene_pick_size
        shrink = self.scene_table_shrink
        sz2 = max(0.05, sz - shrink)        # 顶面压低，指尖贴台面不误报
        objs.append(self._make_box(
            'pick_table', (x, y, z - (sz - sz2) / 2.0), (sx, sy, sz2)))
        top_z = z + sz / 2.0 - shrink
        if self.scene_place_table:
            objs.append(self._make_box('place_table_top',
                                       self.scene_place_top_xyz,
                                       self.scene_place_top_size))
            objs.append(self._make_box('place_pedestal',
                                       self.scene_place_pillar_xyz,
                                       self.scene_place_pillar_size))
        self._apply_scene(world=PlanningSceneWorld(collision_objects=objs),
                          why='加入台子')
        self.get_logger().info(
            f'规划场景：台子 {[o.id for o in objs]}，取件台顶面 {top_z:.3f} m'
            f'（已压低 {shrink * 1000:.0f}mm，防指尖贴台面误报）')

    def _cube_world_xyz(self):
        """方块在 map 系的位姿：优先真值，与参数推算差太多则退回参数。"""
        bx, by, bz = self.object_x, self.object_y, self.object_z
        if self.truth_cube is None:
            return bx, by, bz, 'param'
        tx, ty, tz = self.truth_cube
        dev = math.hypot(tx - bx, ty - by)
        if dev > self.truth_max_dev:
            self.get_logger().warn(
                f'方块真值({tx:.3f},{ty:.3f}) 与参数({bx:.3f},{by:.3f}) 差 '
                f'{dev * 1000:.0f}mm > {self.truth_max_dev:.2f}m，'
                f'可能不是同一坐标系，退回参数')
            return bx, by, bz, 'param'
        return tx, ty, tz, 'truth'

    def _scene_set_cube(self, xyz=None):
        """step②：按位姿放/删方块（同一份 diff 里先删后加，避免残留）。"""
        if not (self.scene_enable and self.scene_cube_on):
            return
        objs = [self._make_box('red_cube', (0, 0, 0), (0, 0, 0),
                               CollisionObject.REMOVE)]
        if xyz is not None:
            s = 0.05 + 2.0 * self.scene_cube_padding
            objs.append(self._make_box('red_cube', xyz, (s, s, s)))
        self._apply_scene(world=PlanningSceneWorld(collision_objects=objs),
                          why='加入方块' if xyz else '移除方块')

    def _scene_cube_on_detect(self):
        """step②：识别成功后加方块，并把"真值 / 参数"的偏差打出来。"""
        if not (self.scene_enable and self.scene_cube_on):
            return
        x, y, z, src = self._cube_world_xyz()
        dx = (x - self.object_x) * 1000.0
        dy = (y - self.object_y) * 1000.0
        self.get_logger().info(
            f'方块位姿[{src}] map=({x:.3f},{y:.3f},{z:.3f}) | '
            f'参数=({self.object_x:.3f},{self.object_y:.3f},{self.object_z:.3f}) | '
            f'偏差=({dx:+.0f},{dy:+.0f})mm')
        self._scene_set_cube((x, y, z))

    def _scene_attach_cube(self):
        """step④：抓取成功后把方块从世界摘掉、挂到 tool_link（放行夹爪接触）。"""
        if not (self.scene_enable and self.scene_attach_on):
            return
        dz = self.grasp_z - self.object_z     # 方块中心在工具系 +z 的距离
        aco = AttachedCollisionObject()
        aco.link_name = self.scene_attach_link
        aco.object = self._make_box('red_cube', (0.0, 0.0, dz),
                                    (0.05, 0.05, 0.05))
        aco.touch_links = [self.scene_attach_link, 'gripper_base_link',
                           'gripper_left_finger_link',
                           'gripper_right_finger_link', 'hand_link']
        self._apply_scene(
            world=PlanningSceneWorld(collision_objects=[
                self._make_box('red_cube', (0, 0, 0), (0, 0, 0),
                               CollisionObject.REMOVE)]),
            attached=[aco], why='方块挂到夹爪')

    def _scene_detach_cube(self):
        """放置脱开后：MoveIt 侧也摘掉（世界位姿等下轮识别重新加）。"""
        if not (self.scene_enable and self.scene_attach_on):
            return
        aco = AttachedCollisionObject()
        aco.link_name = self.scene_attach_link
        aco.object = self._make_box('red_cube', (0, 0, 0), (0, 0, 0),
                                    CollisionObject.REMOVE)
        self._apply_scene(attached=[aco], why='方块脱离夹爪')
        self._scene_set_cube(None)

    def _scene_cleanup(self):
        """任务结束/失败：把方块从场景里清掉，避免影响下一次任务。"""
        if not self.scene_enable:
            return
        self._scene_detach_cube()

    # ================= 吸附（DetachableJoint） =================
    def _do_attach(self):
        self._pub_empty(self.attach_pub, f'吸附 attach → {self.attach_topic}')
        # MoveIt 侧同步：世界里的方块摘掉、挂到夹爪，否则搬运规划会绕着它走
        self._scene_attach_cube()

    def _do_detach(self):
        self._pub_empty(self.detach_pub, f'脱开 detach → {self.detach_topic}')
        self._scene_detach_cube()

    def startup_detach_cb(self):
        """上电后补几次脱开，抵消 DetachableJoint 在 Configure 时的自动连接。"""
        if self.startup_detach_left <= 0:
            return
        self.startup_detach_left -= 1
        self._pub_empty(self.detach_pub, '上电脱开 detach')

    # ================= 轮询 =================
    def timer_cb(self):
        now = self._now()

        # --- 手臂序列驱动 ---
        if self.ops is not None:
            if self.op_kind == 'WAIT' and now >= self.op_wait_until:
                self.op_idx += 1
                self._op_start()
                return
            if self.op_kind == 'SERVER_WAIT':
                if now >= self.op_server_deadline:
                    self._fail('等待 /arm_task 服务超时')
                    return
                self._op_start()
                return
            if self.op_kind == 'GOAL' and now >= self.op_deadline:
                self._cancel_arm()
                self._fail(f'{self.ops[self.op_idx]["label"]} 超时')
                return
            return

        # --- 抓取点微调（直接蠕动，不走导航）---
        if self.stage == 'PICK_CREEP':
            if self.creep_start is not None:
                elapsed = (now - self.creep_start).nanoseconds / 1e9
                dur = abs(self.creep_dist) / max(self.creep_vel, 1e-3)
                if elapsed >= dur:
                    self.cmd_vel_pub.publish(Twist())       # 停
                    self.creep_start = None
                    self.creep_settle_until = now + rclpy.duration.Duration(
                        seconds=self.creep_settle)
                    self.get_logger().info(
                        f'微调蠕动完成（目标 {self.creep_dist * 1000:+.0f} mm，'
                        f'用时 {elapsed:.1f}s），稳定后重新校验')
                else:
                    t = Twist()
                    t.linear.x = math.copysign(self.creep_vel, self.creep_dist)
                    self.cmd_vel_pub.publish(t)
            elif self.creep_settle_until is not None and now >= self.creep_settle_until:
                self.creep_settle_until = None
                self.stage = 'SETTLE'        # 回到"稳定 → 重新校验"的循环
                self.settle_start = now
                self.settle_next = 'PICK'
            return

        # --- 稳定等待 ---
        if self.stage == 'SETTLE':
            if self.settle_start is not None and \
                    (now - self.settle_start).nanoseconds / 1e9 > self.settle_time:
                nxt, self.settle_next = self.settle_next, None
                if nxt == 'DETECT':
                    self.stage = 'DETECT'
                    self._set_detect(True, '进入识别窗口')
                    self._pub_status(None, '识别中')
                    self._reset_detect()
                    self.get_logger().info('已稳定，开始识别')
                elif nxt == 'PICK':
                    verdict = self._check_grasp_reachable()
                    if verdict == 'fail':
                        return          # 校验内部已 _fail，别再去空抓
                    if verdict == 'retry':
                        return          # 已进入微调（蠕动）或已发导航，稍后重新校验
                    if verdict == 'done':
                        return          # 偏差极小，已直接开始抓取序列
                    self.stage = 'PICK'
                    self._run_ops(self._pick_ops(),
                                  TaskStatus.PICKED, '抓取完成，准备搬运',
                                  then=self._nav_to_place)
                elif nxt == 'PLACE':
                    self.stage = 'PLACE'
                    self._run_ops(self._place_ops(),
                                  TaskStatus.PLACED, '放置完成',
                                  then=self._task_done)
            return

        # --- 识别超时 / 对准 ---
        if self.stage != 'DETECT':
            return
        if self.phase == '识别中' and self.det_deadline is not None and \
                now > self.det_deadline:
            if self.retry >= len(self.ALIGN_ACTIONS):
                self.get_logger().error(f'识别超时，{self.retry} 次对准仍失败')
                self._fail('识别失败')
                return
            self._start_align(self.ALIGN_ACTIONS[self.retry], now)
        elif self.phase == '对准转' and self.align_start is not None:
            dur = abs(self.align_target_angle) / self.align_vel
            if (now - self.align_start).nanoseconds / 1e9 > dur:
                self.cmd_vel_pub.publish(Twist())
                self.phase = '稳定等待'
                self.settle_start = now
                self.get_logger().info(
                    f'对准动作 {self.current_action} 结束（转 '
                    f'{self.align_target_angle * 57.3:.1f}°），稳定 '
                    f'{self.settle_time}s 后继续识别')
        elif self.phase == '对准平移' and self.align_start is not None and \
                (now - self.align_start).nanoseconds / 1e9 > self.align_duration:
            self.cmd_vel_pub.publish(Twist())
            self.phase = '稳定等待'
            self.settle_start = now
            self.get_logger().info(
                f'对准动作 {self.current_action} 结束（移动 '
                f'{self.move_vel * self.align_duration * 100:.0f}cm），稳定 '
                f'{self.settle_time}s 后继续识别')
        elif self.phase == '稳定等待' and self.settle_start is not None and \
                (now - self.settle_start).nanoseconds / 1e9 > self.settle_time:
            self._pub_status(None, '识别中')
            self._set_detect(True, '对准后继续识别')
            self._reset_detect()
            self.get_logger().info('已稳定，继续识别')

    def _start_align(self, action, now):
        t = Twist()
        if action == 'rot+':
            t.angular.z = self.align_vel
            self.align_target_angle = self.rot_angle
        elif action == 'rot-':
            t.angular.z = -self.align_vel
            self.align_target_angle = -self.rot_angle
        elif action == 'back':
            t.linear.x = -self.move_vel
        elif action == 'fwd':
            t.linear.x = self.move_vel
        self.retry += 1
        self.current_action = action
        self.cmd_vel_pub.publish(t)
        self.phase = '对准转' if 'rot' in action else '对准平移'
        # 后退步子比前进大（"太近看不到"要一次退够）
        self.align_duration = (self.back_duration if action == 'back'
                               else self.move_duration)
        self.align_start = now
        if 'rot' in action:
            self.get_logger().info(
                f'识别超时，对准动作 {action}（第 {self.retry} 次，'
                f'目标 {self.rot_angle * 57.3:.1f}°）')
        else:
            self.get_logger().info(
                f'识别超时，对准动作 {action}（第 {self.retry} 次，'
                f'移动 {self.move_vel * (self.back_duration if action == "back" else self.move_duration) * 100:.0f}cm）')

    def _reset_detect(self):
        self.det_deadline = self._now() + rclpy.duration.Duration(
            seconds=self.detect_timeout)

    # ================= 收尾 =================
    def _task_done(self):
        self.cmd_vel_pub.publish(Twist())      # 收尾同样清零底盘
        self._scene_cleanup()
        self.creep_start = None
        self.creep_settle_until = None
        self._set_detect(False, '任务结束')
        self._pub_status(TaskStatus.IDLE, '任务完成')
        self.stage = 'IDLE'
        self.task = None
        self.object_pose = None

    def _fail(self, reason):
        self._cancel_arm()
        self._scene_cleanup()
        # 【安全】必须先停底盘：蠕动微调是按 /cmd_vel 开环发的，而底盘插件会
        # 一直保持最后一条指令 —— 不在失败路径上清零，车会带着 0.08m/s 一直走。
        self.cmd_vel_pub.publish(Twist())
        self.creep_start = None
        self.creep_settle_until = None
        self.ops = None
        self.op_kind = None
        self.get_logger().error(f'任务失败：{reason}')
        self._set_detect(False, '任务失败')
        # 若失败发生在"已吸附"之后（例如抓起后导航到放置点失败），必须补一次
        # detach，否则方块会一直粘在夹爪上，场景再也回不到初始状态。
        if self.attach_enabled:
            self._pub_empty(self.detach_pub, '失败收尾 detach')
        self._pub_status(TaskStatus.FAILED, reason)
        self.state = TaskStatus.IDLE
        self.phase = '空闲'
        self.stage = 'IDLE'
        self.task = None
        self.object_pose = None
        # 失败后把手臂收起来，避免挡着激光影响后续导航
        if self.arm_client.server_is_ready():
            self.arm_client.send_goal_async(self._goal_named('arm', 'carry'))


def main(args=None):
    rclpy.init(args=args)
    node = TaskOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
