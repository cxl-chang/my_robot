# my_robot_task_orchestrator/launch/orchestrator.launch.py
# 启动 M9 任务编排节点。
# 用法: ros2 launch my_robot_task_orchestrator orchestrator.launch.py
# 前置: Gazebo + Nav2 + nav_commander + aruco_detector 均在运行。

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    obs_x = LaunchConfiguration('obs_x')
    obs_y = LaunchConfiguration('obs_y')
    obs_yaw = LaunchConfiguration('obs_yaw')
    obs_frame = LaunchConfiguration('obs_frame')
    enable_pick = LaunchConfiguration('enable_pick')
    attach_enabled = LaunchConfiguration('attach_enabled')

    # 默认值与 orchestrator.py 里的节点默认值保持一致（2.2），
    # 否则"直接 ros2 run"和"ros2 launch"会走到两个不同的观测点。
    declare_obs_x = DeclareLaunchArgument(
        'obs_x', default_value='2.2', description='观测点 map x（人工确认能识别的位置）')
    declare_obs_y = DeclareLaunchArgument(
        'obs_y', default_value='3.0', description='观测点 map y')
    declare_obs_yaw = DeclareLaunchArgument(
        'obs_yaw', default_value='-0.13', description='观测点朝向 rad（面朝 +X 约为 0）')
    declare_obs_frame = DeclareLaunchArgument(
        'obs_frame', default_value='map', description='观测点坐标系')
    declare_enable_pick = DeclareLaunchArgument(
        'enable_pick', default_value='true',
        description='是否执行真实抓取（false = 退回 M9 占位行为，只导航）')
    declare_attach = DeclareLaunchArgument(
        'attach_enabled', default_value='false',
        description='是否用 DetachableJoint 吸附（需 Gazebo 侧 use_detachable:=true）')

    orchestrator_node = Node(
        package='my_robot_task_orchestrator',
        executable='task_orchestrator',
        output='screen',
        parameters=[
            {'obs_x': obs_x, 'obs_y': obs_y, 'obs_yaw': obs_yaw,
             'obs_frame': obs_frame, 'enable_pick': enable_pick,
             'attach_enabled': attach_enabled}
        ],
    )

    return LaunchDescription([
        declare_obs_x,
        declare_obs_y,
        declare_obs_yaw,
        declare_obs_frame,
        declare_enable_pick,
        declare_attach,
        orchestrator_node,
    ])
