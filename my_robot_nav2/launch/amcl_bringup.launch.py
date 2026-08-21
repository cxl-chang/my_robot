# my_robot_nav2/launch/amcl_bringup.launch.py
# 一键启动：AMCL 固定地图定位 + Nav2 导航 + RViz（可单独开关）
# 用法: ros2 launch my_robot_nav2 amcl_bringup.launch.py
# 前置: 先启动 Gazebo 仿真（ros2 launch my_robot_bringup my_robot_gazebo.launch.xml）
# 注意: 启动后需在 RViz 用 2D Pose Estimate 给出初始位姿（或发布 /initialpose），
#       AMCL 收敛后（map -> odom 稳定）才能导航。
# 与 SLAM 模式（nav_bringup.launch.py）的区别：本模式加载固定地图 my_world.yaml，
#       不建图、地图不漂移；但每次启动都要手动初始化位姿。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_nav2')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')

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
    ld.add_action(localization_launch)
    ld.add_action(navigation_launch)
    ld.add_action(rviz_launch)
    return ld
