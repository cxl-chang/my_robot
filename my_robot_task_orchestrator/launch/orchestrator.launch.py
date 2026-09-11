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

    declare_obs_x = DeclareLaunchArgument(
        'obs_x', default_value='2.1', description='观测点 map x（人工确认能识别的位置）')
    declare_obs_y = DeclareLaunchArgument(
        'obs_y', default_value='3.0', description='观测点 map y')
    declare_obs_yaw = DeclareLaunchArgument(
        'obs_yaw', default_value='0.0', description='观测点朝向 rad（面朝 +X=0）')

    orchestrator_node = Node(
        package='my_robot_task_orchestrator',
        executable='task_orchestrator',
        output='screen',
        parameters=[
            {'obs_x': obs_x, 'obs_y': obs_y, 'obs_yaw': obs_yaw}
        ],
    )

    return LaunchDescription([
        declare_obs_x,
        declare_obs_y,
        declare_obs_yaw,
        orchestrator_node,
    ])
