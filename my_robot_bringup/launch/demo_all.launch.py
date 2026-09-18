# my_robot_bringup/launch/demo_all.launch.py
# 一键启动完整流程：Gazebo → 定位+Nav2 → 导航命令节点 → ArUco 检测
#                    → 机械臂命令节点 → 任务编排 →（可选）自动下发任务
#
# 原来需要手敲 6 条命令，现在 1 条：
#   ros2 launch my_robot_bringup demo_all.launch.py \
#     obs_x:=2.2 obs_y:=3.0 obs_yaw:=-0.13 \
#     task_object_id:=red_cube place_x:=0.5 place_y:=0.5 place_yaw:=0.0
#
# 只起底盘+导航，任务手发（反复调试不同目标点时）：
#   ros2 launch my_robot_bringup demo_all.launch.py auto_task:=false
#   ros2 topic pub --once /task_cmd my_robot_interfaces/msg/TaskCommand \
#     "{object_id: 'red_cube', place_frame_id: 'map', place_x: 0.5, place_y: 0.5, place_yaw: 0.0}"
#
# 复用已在运行的 Gazebo（只重启上层，省去每次重开仿真）：
#   ros2 launch my_robot_bringup demo_all.launch.py use_gazebo:=false nav_delay:=1.0
#
# 时序（相对 t=0，全部可用参数覆盖，不用改代码）：
#   t=0                    Gazebo + robot_state_publisher + bridge + 控制器 + move_group
#   t=gazebo_delay  (8s)   my_robot_commander（MoveIt 手臂/夹爪唯一执行入口）
#   t=nav_delay     (10s)  AMCL/SLAM + Nav2 + RViz + nav_commander + aruco_detector
#   t=arm_delay     (14s)  my_robot_commander（与 arm 相关，默认与 nav 同批）
#   t=task_delay    (18s)  task_orchestrator
#   t=send_delay    (20s)  task_sender（仅 auto_task:=true）
#
# 注意：amcl 模式的 set_initial_pose 自身还有 initial_delay(默认 5s)，
#       所以 18s 才起 orchestrator 是留了余量的。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource, PythonLaunchDescriptionSource)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('my_robot_bringup')
    nav2_dir = get_package_share_directory('my_robot_nav2')

    # ---------------- 开关 ----------------
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_rviz = LaunchConfiguration('use_rviz')
    use_arm = LaunchConfiguration('use_arm')
    auto_task = LaunchConfiguration('auto_task')
    localization_mode = LaunchConfiguration('localization_mode')

    # ---------------- 时序 ----------------
    nav_delay = LaunchConfiguration('nav_delay')
    arm_delay = LaunchConfiguration('arm_delay')
    task_delay = LaunchConfiguration('task_delay')
    send_delay = LaunchConfiguration('send_delay')

    # ---------------- 定位 / 地图 ----------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    use_initial_pose = LaunchConfiguration('use_initial_pose')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_yaw = LaunchConfiguration('initial_yaw')
    initial_delay = LaunchConfiguration('initial_delay')

    # ---------------- 编排 / 任务 ----------------
    obs_x = LaunchConfiguration('obs_x')
    obs_y = LaunchConfiguration('obs_y')
    obs_yaw = LaunchConfiguration('obs_yaw')
    obs_frame = LaunchConfiguration('obs_frame')
    enable_pick = LaunchConfiguration('enable_pick')
    grasp_forward = LaunchConfiguration('grasp_forward')
    tool_x = LaunchConfiguration('tool_x')
    tool_y = LaunchConfiguration('tool_y')
    pregrasp_z = LaunchConfiguration('pregrasp_z')
    grasp_z = LaunchConfiguration('grasp_z')
    lift_z = LaunchConfiguration('lift_z')
    place_standoff = LaunchConfiguration('place_standoff')
    place_approach_z = LaunchConfiguration('place_approach_z')
    place_down_z = LaunchConfiguration('place_down_z')
    attach_enabled = LaunchConfiguration('attach_enabled')
    use_detachable = LaunchConfiguration('use_detachable')
    gz_args = LaunchConfiguration('gz_args')
    arm_timeout = LaunchConfiguration('arm_timeout')
    detect_rate = LaunchConfiguration('detect_rate')
    task_object_id = LaunchConfiguration('task_object_id')
    place_frame_id = LaunchConfiguration('place_frame_id')
    place_x = LaunchConfiguration('place_x')
    place_y = LaunchConfiguration('place_y')
    place_yaw = LaunchConfiguration('place_yaw')

    declared_args = [
        # --- 开关 ---
        DeclareLaunchArgument(
            'use_gazebo', default_value='true',
            description='是否启动 Gazebo 仿真（false 则复用已在运行的仿真）'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='是否启动 RViz'),
        DeclareLaunchArgument(
            'use_arm', default_value='true',
            description='是否启动 my_robot_commander（MoveIt 手臂/夹爪执行入口）'),
        DeclareLaunchArgument(
            'auto_task', default_value='true',
            description='启动后自动下发一次 /task_cmd'),
        DeclareLaunchArgument(
            'localization_mode', default_value='amcl',
            description="定位方式：'amcl'（固定地图）或 'slam'（在线建图）"),

        # --- 时序（秒）---
        DeclareLaunchArgument(
            'nav_delay', default_value='10.0',
            description='定位+Nav2 的启动延时（等 Gazebo/时钟/控制器就绪）'),
        DeclareLaunchArgument(
            'arm_delay', default_value='10.0',
            description='my_robot_commander 的启动延时'),
        DeclareLaunchArgument(
            'task_delay', default_value='18.0',
            description='task_orchestrator 的启动延时（等 Nav2 active + AMCL 初值）'),
        DeclareLaunchArgument(
            'send_delay', default_value='20.0',
            description='自动下发任务的延时'),

        # --- 定位 / 地图 ---
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='使用仿真时钟（Gazebo 模式必须 true）'),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(
                nav2_dir, 'config', 'my_world.yaml'),
            description='固定地图 yaml 路径（仅 amcl 模式使用）'),
        DeclareLaunchArgument(
            'use_initial_pose', default_value='true',
            description='amcl 模式自动发布初始位姿（省去 RViz 手动 2D Pose Estimate）'),
        DeclareLaunchArgument(
            'initial_x', default_value='0.0', description='初始位姿 x (map 系)'),
        DeclareLaunchArgument(
            'initial_y', default_value='0.0', description='初始位姿 y (map 系)'),
        DeclareLaunchArgument(
            'initial_yaw', default_value='0.0', description='初始朝向 yaw (rad)'),
        DeclareLaunchArgument(
            'initial_delay', default_value='5.0',
            description='amcl 启动后多少秒发布初始位姿'),

        # --- 编排参数 ---
        DeclareLaunchArgument(
            'obs_x', default_value='1.84',
            description='物体观测点 map x（人工确认能识别的位置）'),
        DeclareLaunchArgument(
            'obs_y', default_value='2.82', description='物体观测点 map y'),
        DeclareLaunchArgument(
            'obs_yaw', default_value='0.0',
            description='观测点朝向 rad（面朝 +X 约为 0）'),
        DeclareLaunchArgument(
            'obs_frame', default_value='map', description='观测点坐标系'),

        # --- M12/M13 抓取与放置 ---
        DeclareLaunchArgument(
            'enable_pick', default_value='true',
            description='是否执行真实抓取（false = 退回 M9 占位行为，只导航）'),
        DeclareLaunchArgument(
            'grasp_forward', default_value='0.175',
            description='观测点→抓取点沿朝向前进的距离 m（竖直下抓的可达范围限制）'),
        DeclareLaunchArgument(
            'tool_x', default_value='0.42',
            description='抓取时物体在 base_footlink 系的 x'),
        DeclareLaunchArgument(
            'tool_y', default_value='0.078',
            description='抓取时物体在 base_footlink 系的 y'),
        DeclareLaunchArgument(
            'pregrasp_z', default_value='0.60',
            description='预抓取工具高度（自由规划用，要高于可规划下限 ~0.45）'),
        DeclareLaunchArgument(
            'grasp_z', default_value='0.495',
            description='抓取工具高度（= 方块中心 z + 0.07；笛卡尔下压终点）'),
        DeclareLaunchArgument(
            'lift_z', default_value='0.70', description='抬起工具高度'),
        DeclareLaunchArgument(
            'place_standoff', default_value='0.42',
            description='机器人在放置点前方多远停车'),
        DeclareLaunchArgument(
            'place_approach_z', default_value='0.65', description='预放置工具高度'),
        DeclareLaunchArgument(
            'place_down_z', default_value='0.495',
            description='下放工具高度（= 放置台面 0.40 + 0.095）。'
                        '注意：机械臂"可规划"的最低工具高度约 0.45，'
                        '比它的 IK 可达范围更严，别往低调'),
        DeclareLaunchArgument(
            'arm_timeout', default_value='60.0',
            description='单个手臂动作的规划+执行超时（秒）。'
                        '与 Gazebo/Nav2/识别同机运行时 CPU 紧张、规划变慢，'
                        '实测 25s 会不够'),
        DeclareLaunchArgument(
            'detect_rate', default_value='5.0',
            description='ArUco 检测限频（Hz），0=不限频。'
                        '不限频时它每帧最多跑 12 遍 detectMarkers（放大到 2 倍'
                        '即 2560x1920），在看不到标签的空转期会把 CPU 打满，'
                        '拖慢 Nav2 与 MoveIt'),
        DeclareLaunchArgument(
            'attach_enabled', default_value='true',
            description='是否用 DetachableJoint 吸附（仿真抓取可靠性）'),
        DeclareLaunchArgument(
            'gz_args',
            default_value=os.path.join(
                bringup_dir, 'worlds', 'test_world.sdf') + ' -r',
            description='Gazebo 启动参数。默认就是带 GUI 加载 test_world.sdf，'
                        '与 my_robot_gazebo.launch.xml 保持一致。'
                        '无 GUI 跑法：gz_args:="-s -r <world.sdf 绝对路径>"'
                        '（注意：这里必须给非空默认值 —— 空串会覆盖掉 '
                        'my_robot_gazebo.launch.xml 里的默认 world，'
                        '导致 gz sim 起一个空世界、场景里没有台子和方块）'),
        DeclareLaunchArgument(
            'use_detachable', default_value='true',
            description='是否给夹爪挂 DetachableJoint 插件（需与 attach_enabled 一致）'),

        # --- 任务参数 ---
        DeclareLaunchArgument(
            'task_object_id', default_value='red_cube',
            description='要抓取的物体类别'),
        DeclareLaunchArgument(
            'place_frame_id', default_value='map', description='放置点坐标系'),
        DeclareLaunchArgument(
            'place_x', default_value='0.5', description='放置点 x'),
        DeclareLaunchArgument(
            'place_y', default_value='0.5', description='放置点 y'),
        DeclareLaunchArgument(
            'place_yaw', default_value='0.0', description='放置点朝向 rad'),
    ]

    # ================= 1) Gazebo（t=0） =================
    # 注意：gz_args 在这里必须给一个非空默认值。IncludeLaunchDescription 传空串
    # 会覆盖掉 my_robot_gazebo.launch.xml 自己的默认 world，gz sim 就会起空世界
    # （没有台子、没有方块）。所以这里的默认值复制了那边的默认世界路径。
    gazebo_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'my_robot_gazebo.launch.xml')),
        launch_arguments={
            'use_detachable': use_detachable,
            'gz_args': gz_args,
        }.items(),
        condition=IfCondition(use_gazebo))

    # ================= 2) 定位 + Nav2（t=nav_delay） =================
    # AMCL 固定地图模式
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'amcl_bringup.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': use_rviz,
            'map': map_yaml,
            'use_initial_pose': use_initial_pose,
            'initial_x': initial_x,
            'initial_y': initial_y,
            'initial_yaw': initial_yaw,
            'initial_delay': initial_delay,
        }.items(),
        condition=LaunchConfigurationEquals('localization_mode', 'amcl'))

    # SLAM 在线建图模式
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'nav_bringup.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': use_rviz,
        }.items(),
        condition=LaunchConfigurationEquals('localization_mode', 'slam'))

    # 导航命令节点：/nav_cmd → Nav2 navigate_to_pose，回 /nav_status
    nav_commander_node = Node(
        package='my_robot_nav_commander',
        executable='nav_commander',
        name='nav_commander',
        output='screen')

    # ArUco 物体检测：/camera/image_raw → /object_detection
    aruco_node = Node(
        package='my_robot_perception',
        executable='aruco_detector',
        name='aruco_detector',
        output='screen',
        parameters=[{'max_rate_hz': detect_rate}])

    nav_group = TimerAction(
        period=nav_delay,
        actions=[amcl_launch, slam_launch, nav_commander_node, aruco_node])

    # ================= 3) 机械臂执行入口（t=arm_delay） =================
    # 订阅 /pose_cmd、/joint_cmd、/gripper_cmd（话题）与 /arm_task（action），
    # 驱动 MoveIt 的 arm / gripper 组。这是抓取放置的唯一执行入口。
    #
    # 必须走它自己的 launch 而不是裸 Node：MoveGroupInterface 需要
    # robot_description 等一整组 MoveIt 参数，launch 里用 MoveItConfigsBuilder
    # 注入；直接 ros2 run 会 "Unable to construct robot model" 然后 abort。
    arm_commander_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('my_robot_commander_cpp'),
                'launch', 'commander.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(use_arm))

    arm_group = TimerAction(period=arm_delay, actions=[arm_commander_launch])

    # ================= 4) 任务编排（t=task_delay） =================
    orchestrator_node = Node(
        package='my_robot_task_orchestrator',
        executable='task_orchestrator',
        name='task_orchestrator',
        output='screen',
        parameters=[{
            'obs_x': obs_x,
            'obs_y': obs_y,
            'obs_yaw': obs_yaw,
            'obs_frame': obs_frame,
            'enable_pick': enable_pick,
            'grasp_forward': grasp_forward,
            'tool_x': tool_x,
            'tool_y': tool_y,
            'pregrasp_z': pregrasp_z,
            'grasp_z': grasp_z,
            'lift_z': lift_z,
            'place_standoff': place_standoff,
            'place_approach_z': place_approach_z,
            'place_down_z': place_down_z,
            'attach_enabled': attach_enabled,
            'arm_timeout': arm_timeout,
        }])

    task_group = TimerAction(period=task_delay, actions=[orchestrator_node])

    # ================= 5) 自动下发任务（t=send_delay） =================
    task_sender_node = Node(
        package='my_robot_task_orchestrator',
        executable='task_sender',
        name='task_sender',
        output='screen',
        condition=IfCondition(auto_task),
        parameters=[{
            'object_id': task_object_id,
            'place_frame_id': place_frame_id,
            'place_x': place_x,
            'place_y': place_y,
            'place_yaw': place_yaw,
            # 自身再等 2s，并等 /task_cmd 真的有订阅者（orchestrator 起来）再发
            'delay': 2.0,
            'wait_for_subscriber': True,
            'wait_timeout': 60.0,
        }])

    send_group = TimerAction(period=send_delay, actions=[task_sender_node])

    return LaunchDescription(
        declared_args +
        [gazebo_launch, nav_group, arm_group, task_group, send_group])
