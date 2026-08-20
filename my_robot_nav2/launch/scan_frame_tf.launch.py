# my_robot_nav2/launch/scan_frame_tf.launch.py
# 修复 GZ gpu_lidar 传感器 frame 与 TF 树断链的问题：
#   GZ 生成的 /scan 消息 frame 为 "<model>/<link>/<sensor>"，即 "my_robot/base_footlink/gpu_lidar"，
#   该 frame 在 ROS TF 树中不存在，导致 slam_toolbox / costmap 无法把 scan 变换到 odom。
#   这里发布静态 TF：lidar_link -> <scan_frame>（lidar_link 已由 robot_state_publisher 发布）。
# 用法: ros2 launch my_robot_nav2 scan_frame_tf.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scan_frame = LaunchConfiguration('scan_frame')

    declare_scan_frame_cmd = DeclareLaunchArgument(
        'scan_frame',
        default_value='my_robot/base_footlink/gpu_lidar',
        description='Frame id of the bridged /scan message (gz sensor frame)')

    # static_transform_publisher 参数顺序: x y z yaw pitch roll frame_id child_frame_id
    static_scan_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', scan_frame],
        output='screen')

    ld = LaunchDescription()
    ld.add_action(declare_scan_frame_cmd)
    ld.add_action(static_scan_tf)
    return ld
