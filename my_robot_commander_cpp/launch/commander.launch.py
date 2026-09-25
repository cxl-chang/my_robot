# my_robot_commander_cpp/launch/commander.launch.py
# 启动 MoveIt 手臂/夹爪执行入口 my_robot_commander。
#
# 为什么必须用 launch 而不是 `ros2 run`：
#   my_robot_commander 内部用 MoveGroupInterface 构造机器人模型，需要本节点持有
#   robot_description / robot_description_semantic / kinematics / joint_limits 等
#   一整组 MoveIt 参数。裸 `ros2 run my_robot_commander_cpp my_robot_commander`
#   不带任何参数，会直接报：
#     Could not find parameter robot_description ... Unable to construct robot model
#   然后 abort。所以这里用 MoveItConfigsBuilder 把整套配置作为 parameters 传进去
#   （与 my_robot_moveit_config/launch/move_group.launch.py 同源，保证一致）。
#
# 用法:
#   ros2 launch my_robot_commander_cpp commander.launch.py                 # 仿真
#   ros2 launch my_robot_commander_cpp commander.launch.py use_sim_time:=false

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        'my_robot', package_name='my_robot_moveit_config').to_moveit_configs()

    node = Node(
        package='my_robot_commander_cpp',
        executable='my_robot_commander',
        name='my_robot_commander',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='使用仿真时钟（Gazebo 模式必须 true）'),
        node,
    ])
