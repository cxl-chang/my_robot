# my_robot_nav2/launch/amcl_bringup.launch.py
# 一键启动：AMCL 固定地图定位 + Nav2 导航 + RViz（可单独开关）+ 自动发布初始位姿
# 用法: ros2 launch my_robot_nav2 amcl_bringup.launch.py
# 前置: 先启动 Gazebo 仿真（ros2 launch my_robot_bringup my_robot_gazebo.launch.xml）
# 说明: 默认会自动发布 /initialpose（use_initial_pose:=true，参数 initial_x/y/yaw/delay），
#       免去 RViz 手动 2D Pose Estimate；如需手动设置则 use_initial_pose:=false。
# 与 SLAM 模式（nav_bringup.launch.py）的区别：本模式加载固定地图 my_world.yaml，
#       不建图、地图不漂移；但需要给出初始位姿（默认已自动发布）。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_nav2')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    use_initial_pose = LaunchConfiguration('use_initial_pose')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_yaw = LaunchConfiguration('initial_yaw')
    initial_delay = LaunchConfiguration('initial_delay')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz with Nav2 default view if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file')

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_dir, 'config', 'my_world.yaml'),
        description='Full path to the map yaml file to load')

    # ---- 自动初始位姿（便利功能）----
    declare_use_initial_pose_cmd = DeclareLaunchArgument(
        'use_initial_pose', default_value='true',
        description='自动发布 /initialpose（省去 RViz 手动 2D Pose Estimate）')
    declare_initial_x_cmd = DeclareLaunchArgument(
        'initial_x', default_value='0.0', description='初始位姿 x (map 系)')
    declare_initial_y_cmd = DeclareLaunchArgument(
        'initial_y', default_value='0.0', description='初始位姿 y (map 系)')
    declare_initial_yaw_cmd = DeclareLaunchArgument(
        'initial_yaw', default_value='0.0', description='初始朝向 yaw (rad)')
    declare_initial_delay_cmd = DeclareLaunchArgument(
        'initial_delay', default_value='5.0',
        description='启动后多少秒发布初始位姿（等 AMCL/map_server 就绪）')

    initial_pose_node = Node(
        package='my_robot_nav2',
        executable='set_initial_pose.py',
        name='set_initial_pose',
        output='screen',
        condition=IfCondition(use_initial_pose),
        parameters=[{
            'x': initial_x,
            'y': initial_y,
            'yaw': initial_yaw,
            'delay': initial_delay,
        }])

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'localization.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'map': map_yaml,
        }.items())

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items())

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'rviz_launch.py')),
        condition=IfCondition(use_rviz))

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_use_initial_pose_cmd)
    ld.add_action(declare_initial_x_cmd)
    ld.add_action(declare_initial_y_cmd)
    ld.add_action(declare_initial_yaw_cmd)
    ld.add_action(declare_initial_delay_cmd)
    ld.add_action(localization_launch)
    ld.add_action(navigation_launch)
    ld.add_action(initial_pose_node)
    ld.add_action(rviz_launch)
    return ld
