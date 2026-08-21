# my_robot_nav2/launch/localization.launch.py
# 启动 AMCL 固定地图定位（map_server 加载地图 + amcl 定位 + lifecycle_manager）
# 用法: ros2 launch my_robot_nav2 localization.launch.py
# 前置:
#   - Gazebo 仿真运行中（/scan、/odom、/cmd_vel 可用）
#   - 地图文件位于 my_robot_nav2/config/my_world.{yaml,pgm}（可通过 map 参数覆盖）
# 注意: 启动后必须在 RViz 用 2D Pose Estimate 给出初始位姿（或发布 /initialpose），
#       AMCL 粒子才会收敛，此时 map -> odom 才有效。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_nav2')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file (contains amcl/map_server sections)')

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_dir, 'config', 'my_world.yaml'),
        description='Full path to the map yaml file to load')

    # 修复 /scan frame 断链（GZ sensor frame -> lidar_link 静态 TF），AMCL 同样需要
    scan_frame_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'scan_frame_tf.launch.py')))

    # 复用 nav2_bringup 官方 localization launch（map_server + amcl + lifecycle_manager_localization）
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'localization_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'use_composition': 'False',
        }.items())

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(scan_frame_tf_launch)
    ld.add_action(localization_launch)
    return ld
