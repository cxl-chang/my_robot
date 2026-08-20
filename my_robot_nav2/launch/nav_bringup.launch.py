# my_robot_nav2/launch/nav_bringup.launch.py
# 一键启动：SLAM 建图 + Nav2 导航 + RViz（可单独开关）
# 用法: ros2 launch my_robot_nav2 nav_bringup.launch.py
# 前置: 先启动 Gazebo 仿真（ros2 launch my_robot_bringup my_robot_gazebo.launch.xml）

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

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz with Nav2 default view if true')

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'slam.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'navigation.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'rviz_launch.py')),
        condition=IfCondition(use_rviz))

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(slam_launch)
    ld.add_action(navigation_launch)
    ld.add_action(rviz_launch)
    return ld
