#!/usr/bin/env python3
"""一次性任务下发节点 task_sender。

用于一键启动（demo_all.launch.py）时自动把 TaskCommand 发到 /task_cmd，
省掉手敲 ros2 topic pub。它不是业务节点，发完就正常退出。

参数
  object_id            要抓取的物体类别，默认 "red_cube"
  place_frame_id       放置点坐标系，默认 "map"
  place_x / place_y    放置点平面坐标，默认 0.5 / 0.5
  place_yaw            放置点朝向 rad，默认 0.0
  delay                节点启动后再等多少秒才发，默认 2.0
  wait_for_subscriber  是否等 /task_cmd 出现订阅者（orchestrator 起来）再发，默认 True
  wait_timeout         等待订阅者的超时秒数，默认 60.0（超时则告警并照样发）
  repeat               发送次数，默认 1
  period               多次发送之间的间隔秒，默认 0.0

退出码：0 = 已发送（含超时后强发）；1 = 参数非法
"""

import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import TaskCommand


class TaskSender(Node):

    def __init__(self):
        super().__init__('task_sender')

        self.declare_parameter('object_id', 'red_cube')
        self.declare_parameter('place_frame_id', 'map')
        self.declare_parameter('place_x', 0.5)
        self.declare_parameter('place_y', 0.5)
        self.declare_parameter('place_yaw', 0.0)
        self.declare_parameter('delay', 2.0)
        self.declare_parameter('wait_for_subscriber', True)
        self.declare_parameter('wait_timeout', 60.0)
        self.declare_parameter('repeat', 1)
        self.declare_parameter('period', 0.0)

        self.object_id = self.get_parameter('object_id').value
        self.place_frame_id = self.get_parameter('place_frame_id').value
        self.place_x = self.get_parameter('place_x').value
        self.place_y = self.get_parameter('place_y').value
        self.place_yaw = self.get_parameter('place_yaw').value
        self.delay = float(self.get_parameter('delay').value)
        self.wait_sub = bool(self.get_parameter('wait_for_subscriber').value)
        self.wait_timeout = float(self.get_parameter('wait_timeout').value)
        self.repeat = int(self.get_parameter('repeat').value)
        self.period = float(self.get_parameter('period').value)

        if not self.object_id:
            self.get_logger().error('object_id 为空，无法下发任务')
            self.done = True
            self.exit_code = 1
            return
        if self.repeat < 1:
            self.get_logger().error(f'repeat={self.repeat} 非法，至少为 1')
            self.done = True
            self.exit_code = 1
            return

        self.pub = self.create_publisher(TaskCommand, '/task_cmd', 10)

        self.done = False
        self.exit_code = 0
        self.sent = 0
        self.start_time = self.get_clock().now()
        self.last_send_time = None
        self.waited_warned = False

        self.get_logger().info(
            f'task_sender 就绪：{self.object_id} → '
            f'({self.place_x:.2f},{self.place_y:.2f}, yaw={self.place_yaw:.2f}) '
            f'frame={self.place_frame_id} | delay={self.delay}s '
            f'repeat={self.repeat} | 等订阅者={self.wait_sub}')

        self.timer = self.create_timer(0.1, self.timer_cb)

    # ------------------------------------------------------------------
    def _elapsed(self):
        return (self.get_clock().now() - self.start_time).nanoseconds / 1e9

    def _build_msg(self):
        msg = TaskCommand()
        msg.object_id = self.object_id
        msg.place_frame_id = self.place_frame_id
        msg.place_x = float(self.place_x)
        msg.place_y = float(self.place_y)
        msg.place_yaw = float(self.place_yaw)
        return msg

    def _publish_once(self):
        subs = self.pub.get_subscription_count()
        self.pub.publish(self._build_msg())
        self.sent += 1
        self.last_send_time = self.get_clock().now()
        self.get_logger().info(
            f'已下发任务 {self.sent}/{self.repeat}：'
            f'object_id={self.object_id} → '
            f'({self.place_x:.2f},{self.place_y:.2f}) | /task_cmd 订阅者数={subs}')

    def timer_cb(self):
        now = self.get_clock().now()

        # 1) 基础延时
        if self._elapsed() < self.delay:
            return

        # 2) 等 /task_cmd 出现订阅者（orchestrator 已就绪）
        if self.wait_sub and self.sent == 0:
            subs = self.pub.get_subscription_count()
            if subs == 0:
                if not self.waited_warned and self.wait_timeout > 0:
                    self.get_logger().info(
                        f'等待 task_orchestrator 订阅 /task_cmd（最多 '
                        f'{self.wait_timeout:.0f}s）…')
                    self.waited_warned = True
                # 超时保护：宁可按期下发（orchestrator 若稍后起来会漏掉这条），
                # 也不要让整个 demo 卡住不动。
                base = self.start_time
                if self.wait_timeout > 0 and \
                        (now - base).nanoseconds / 1e9 > self.delay + self.wait_timeout:
                    self.get_logger().warn(
                        f'等待订阅者超时（{self.wait_timeout:.0f}s），'
                        f'仍然下发任务 —— 请检查 task_orchestrator 是否启动')
                else:
                    return

        # 3) 多次发送的间隔
        if self.sent > 0 and self.period > 0 and self.last_send_time is not None:
            if (now - self.last_send_time).nanoseconds / 1e9 < self.period:
                return

        if self.sent >= self.repeat:
            self.done = True
            return

        self._publish_once()

        if self.sent >= self.repeat:
            self.get_logger().info('任务已全部下发，task_sender 退出')
            self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = TaskSender()
    try:
        # 用 spin_once 轮询而不是 spin()：发送结束后可以干净退出，
        # 避免在回调里调 rclpy.shutdown() 的时序问题。
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if node.exit_code != 0:
        raise SystemExit(node.exit_code)


if __name__ == '__main__':
    main()
