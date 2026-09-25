#!/usr/bin/env python3
"""仿真专用：把左指位置镜像到右指（gripper_mirror）。

背景
----
本仿真后端 Ignition Gazebo 6 (Fortress) 下，名叫 `gripper_right_finger_joint`
的关节经 gz_ros2_control 完全驱动不了：接口 available/claimed、控制器里也含它、
JTC 的 error 显示确实在命令它，但关节状态恒为 0、指爪物理上不动
（`gripper_left_finger_joint` 一切正常）。
已排除的因素：  关节限位与符号约定、
接触/摩擦参数、指爪与底板的间隙、控制器里放一个还是两个关节、绕过 MoveIt
直发 JTC。

**根因（已定论）**：决定因素是关节的「轴方向 + 限位取向」—— 只有与左指完全同构的
形态（轴 +1、限位 [0, +X]、正向命令）能被 gz_ros2_control 驱动；右指要向内合拢就
必须用负向轴或负限位，而这两种恰好都驱动不了。
已排除：关节名、声明顺序、URDF 定义顺序、关节 origin、摩擦参数、控制器关节数、MoveIt。
（早先写成"按名字绑错实体"是过度推断，已更正，详见 gripper.gazebo.xacro。）

因此右指改由 Gazebo 原生插件 `JointPositionController`（见 gripper.gazebo.xacro）
驱动，本节点负责把左指的实际位置镜像到该插件的指令话题。

用法
----
由 my_robot_gazebo.launch.xml 自动拉起，一般不用手动运行：
    ros2 run my_robot_bringup gripper_mirror.py

话题
----
订阅 /joint_states（取 source_joint 的位置）
发布 <target_topic>（std_msgs/Float64），经 ros_gz_bridge 转到 GZ 插件

参数
----
source_joint   镜像源关节名，默认 gripper_left_finger_joint
target_topic   目标指令话题，默认 /gripper_right_finger_cmd
publish_period 发布周期（秒），默认 0.05（20Hz）
deadband       变化小于该值不重复发布（米），默认 0.0005
enabled        设为 false 可直接关掉镜像（例如 gz_ros2_control 修好后）
detach_topic   上电脱开话题（DetachableJoint），默认 /red_cube/detach
detach_count   上电脱开发几次，默认 20；设 0 关闭
detach_period  上电脱开的间隔（秒），默认 1.0

为什么本节点还要负责"上电脱开"
------------------------------
`DetachableJoint` 插件在 Configure 阶段就会把 red_cube 和夹爪连起来
（Gazebo 日志里有 `Attaching entity: ...` / `Creating detachable joint`），
如果不管它，方块从仿真第一帧起就会被夹爪拽着走 —— 而 orchestrator 默认
要到 t=18s 才启动，太晚了。本节点随仿真最早启动（t≈0），所以由它在启动后
立刻按周期连发几次 detach 抵消；orchestrator 里还留了一份兜底
（detach_on_start）。桥接要几秒才建好，因此不能只发一次。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float64


class GripperMirror(Node):

    def __init__(self):
        super().__init__('gripper_mirror')

        self.declare_parameter('source_joint', 'gripper_left_finger_joint')
        self.declare_parameter('target_topic', '/gripper_right_finger_cmd')
        self.declare_parameter('publish_period', 0.05)
        self.declare_parameter('deadband', 0.0005)
        self.declare_parameter('enabled', True)
        self.declare_parameter('detach_topic', '/red_cube/detach')
        self.declare_parameter('detach_count', 20)
        self.declare_parameter('detach_period', 1.0)

        self.source_joint = self.get_parameter('source_joint').value
        self.target_topic = self.get_parameter('target_topic').value
        period = float(self.get_parameter('publish_period').value)
        self.deadband = float(self.get_parameter('deadband').value)
        self.enabled = bool(self.get_parameter('enabled').value)
        self.detach_topic = self.get_parameter('detach_topic').value
        self.detach_left = int(self.get_parameter('detach_count').value)
        self.detach_period = float(self.get_parameter('detach_period').value)

        if not self.enabled:
            self.get_logger().warn('gripper_mirror 已禁用（enabled=false）')

        self.last_source = None
        self.last_pub = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        self.pub = self.create_publisher(Float64, self.target_topic, 10)
        self.sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_cb, qos)
        self.timer = self.create_timer(period, self.timer_cb)

        if self.detach_left > 0:
            self.detach_pub = self.create_publisher(Empty, self.detach_topic, 10)
            self.detach_timer = self.create_timer(self.detach_period,
                                                  self.detach_cb)
        else:
            self.detach_pub = None
            self.detach_timer = None

        self.get_logger().info(
            f'gripper_mirror 启动：{self.source_joint} → {self.target_topic} '
            f'（{1.0 / period:.0f}Hz, deadband={self.deadband}）'
            + (f'；上电将向 {self.detach_topic} 连发 {self.detach_left} 次 detach'
               if self.detach_left > 0 else ''))

    def joint_state_cb(self, msg: JointState):
        if self.source_joint in msg.name:
            idx = msg.name.index(self.source_joint)
            if idx < len(msg.position):
                self.last_source = float(msg.position[idx])

    def detach_cb(self):
        if self.detach_left <= 0:
            return
        self.detach_left -= 1
        self.detach_pub.publish(Empty())
        if self.detach_left == 0:
            self.get_logger().info(
                f'上电脱开完成（{self.detach_topic}）；'
                f'若未启用 DetachableJoint 插件，这些消息会被忽略')

    def timer_cb(self):
        if not self.enabled or self.last_source is None:
            return
        if self.last_pub is not None and \
                abs(self.last_source - self.last_pub) < self.deadband:
            return
        self.pub.publish(Float64(data=self.last_source))
        self.last_pub = self.last_source


def main(args=None):
    rclpy.init(args=args)
    node = GripperMirror()
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
