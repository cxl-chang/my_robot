#!/usr/bin/env python3
"""启动后自动发布一次 /initialpose（AMCL 初始位姿），免去在 RViz 手动 2D Pose Estimate。

参数（可通过 launch 传）：
  x, y       初始位置（map 系，默认 0.0）
  yaw        初始朝向 rad（默认 0.0）
  delay      启动后延迟多少秒发布（等 AMCL/map_server 就绪，默认 5.0）
  frame_id   默认 'map'
  covariance 初始位姿协方差（默认 0.25，越小越信任初始值）

发布一次后节点退出。
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('set_initial_pose')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('delay', 5.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('covariance', 0.25)

        self.x = self.get_parameter('x').value
        self.y = self.get_parameter('y').value
        self.yaw = self.get_parameter('yaw').value
        self.delay = self.get_parameter('delay').value
        self.frame_id = self.get_parameter('frame_id').value
        self.cov = self.get_parameter('covariance').value

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.timer = self.create_timer(self.delay, self.publish_once)
        self.get_logger().info(
            f'将在 {self.delay}s 后自动发布初始位姿 '
            f'({self.x}, {self.y}, yaw={self.yaw})')

    def publish_once(self):
        self.timer.cancel()
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(self.x)
        msg.pose.pose.position.y = float(self.y)
        msg.pose.pose.position.z = 0.0
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        # 协方差：x, y, yaw 三个方向
        cov = [0.0] * 36
        cov[0] = self.cov    # x
        cov[7] = self.cov    # y
        cov[35] = self.cov   # yaw
        msg.pose.covariance = cov

        self.pub.publish(msg)
        self.get_logger().info(
            f'已发布初始位姿 ({self.x}, {self.y}, yaw={self.yaw}) 到 /initialpose')


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    # 等一次发布完成（timer 到期 + 发布），随后退出
    while rclpy.ok() and not node.timer.is_canceled():
        rclpy.spin_once(node, timeout_sec=0.1)
    # 再 spin 一小会确保消息发出
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
