#!/usr/bin/env python3
"""M9 任务编排节点 task_orchestrator。

订阅 /task_cmd（TaskCommand: object_id + 放置点）后执行状态机：
  1. NAV_TO_OBJECT  导航到"物体观测点"（物体 -X 侧 obs_dist 处，面朝 +X 看 -X 面标签）
  2. 识别确认       等待 /object_detection 识别到目标；
                     识别超时 → 原地小幅转动对准，重试 max_retry 次
  3. PICKED         识别成功（PICK 动作占位，夹爪修好后在此接入）
  4. NAV_TO_PLACE   导航到放置点
  5. PLACED→DONE    （PLACE 动作占位）
全程发布 /task_status。

依赖（需同时运行）：
  - my_robot_nav_commander nav_commander（接收 /nav_cmd，发布 /nav_status）
  - my_robot_perception aruco_detector（发布 /object_detection）
"""

import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import (
    TaskCommand, TaskStatus, NavGoal, NavStatus, ObjectDetection)
from geometry_msgs.msg import Twist


class TaskOrchestrator(Node):
    # 对准动作序列（按 retry 顺序执行；识别超时一次取一个动作）：
    #   rot+ 左转 / rot- 右转 / back 后退 / fwd 前进
    # 序列设计：覆盖【左偏→回正→右偏→回正】两侧朝向 + 前后距离试探
    #   （原序列 rot- 只是回正，从不停留在右偏位置 → 需要右偏时永远扫不到）
    ALIGN_ACTIONS = ['rot+', 'rot-', 'rot-', 'rot+', 'back', 'fwd']

    def __init__(self):
        super().__init__('task_orchestrator')

        # 观测点（map 系）：直接用人工确认能识别的坐标，不做"物体-偏移"自动推算
        self.declare_parameter('obs_x', 2.2)         # 观测点 x
        self.declare_parameter('obs_y', 3.0)          # 观测点 y
        self.declare_parameter('obs_yaw', 0.0)        # 观测点朝向（面朝 +X 看标签）
        self.declare_parameter('obs_frame', 'map')
        self.declare_parameter('detect_timeout', 5.0)  # 每轮识别等待秒
        self.declare_parameter('max_retry', 6)         # 对准重试上限
        self.declare_parameter('align_vel', 0.5)       # 转动角速度 rad/s
        self.declare_parameter('rot_angle', 0.12)      # 每次转动角度 rad(≈7°)
        self.declare_parameter('move_vel', 0.15)       # 前后移动速度 m/s
        self.declare_parameter('move_duration', 0.6)   # 前后移动时长 s(≈9cm)
        self.declare_parameter('settle_time', 1.5)     # 对准停止后稳定等待 s

        self.obs_x = self.get_parameter('obs_x').value
        self.obs_y = self.get_parameter('obs_y').value
        self.obs_yaw = self.get_parameter('obs_yaw').value
        self.obs_frame = self.get_parameter('obs_frame').value
        self.detect_timeout = self.get_parameter('detect_timeout').value
        self.max_retry = self.get_parameter('max_retry').value
        self.align_vel = self.get_parameter('align_vel').value
        self.rot_angle = self.get_parameter('rot_angle').value
        self.move_vel = self.get_parameter('move_vel').value
        self.move_duration = self.get_parameter('move_duration').value
        self.settle_time = self.get_parameter('settle_time').value

        # 发布/订阅
        self.task_sub = self.create_subscription(
            TaskCommand, '/task_cmd', self.task_cmd_cb, 10)
        self.nav_status_sub = self.create_subscription(
            NavStatus, '/nav_status', self.nav_status_cb, 10)
        self.det_sub = self.create_subscription(
            ObjectDetection, '/object_detection', self.det_cb, 10)
        self.nav_pub = self.create_publisher(NavGoal, '/nav_cmd', 10)
        self.status_pub = self.create_publisher(TaskStatus, '/task_status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 状态
        self.state = TaskStatus.IDLE
        self.phase = '空闲'
        self.task = None          # 当前任务
        self.target_class = ''    # 要识别的物体类别
        self.retry = 0
        self.current_action = ''  # 当前对准动作
        self.align_target_angle = 0.0  # 本次转动目标角度（rad，带符号）
        self.det_deadline = None  # 识别等待截止（ROS 时间）
        self.align_start = None   # 对准开始（ROS 时间）
        self.settle_start = None  # 稳定等待开始（ROS 时间）
        self.object_pose = None   # 识别到的物体位姿（camera 系，抓取用）

        # 状态机轮询（0.1s）
        self.create_timer(0.1, self.timer_cb)

        self._pub_status()
        self.get_logger().info(
            f'task_orchestrator 启动：观测点=({self.obs_x},{self.obs_y}) '
            f'yaw={self.obs_yaw}')

    # ================= 状态发布 =================
    def _pub_status(self, state=None, phase=None):
        if state is not None:
            self.state = state
        if phase is not None:
            self.phase = phase
        msg = TaskStatus()
        msg.status = self.state
        msg.phase = self.phase
        self.status_pub.publish(msg)

    # ================= 任务入口 =================
    def task_cmd_cb(self, msg: TaskCommand):
        if self.state != TaskStatus.IDLE:
            self.get_logger().warn(f'任务进行中（状态={self.state}），忽略新命令')
            return
        self.task = msg
        self.target_class = msg.object_id
        self.retry = 0
        self.get_logger().info(
            f'收到任务：抓取 {msg.object_id} → 放置 '
            f'({msg.place_x:.2f},{msg.place_y:.2f})')
        self._nav_to_object()

    # ================= 导航 =================
    def _nav_to_object(self):
        # 直接导航到人工确认的观测点（观测点坐标系默认 map）
        g = NavGoal()
        g.frame_id = self.obs_frame
        g.x = self.obs_x
        g.y = self.obs_y
        g.yaw = self.obs_yaw
        self._pub_status(TaskStatus.NAV_TO_OBJECT, '导航到物体观测点')
        self.get_logger().info(
            f'导航到观测点 ({g.x:.2f},{g.y:.2f}) 朝向 {g.yaw:.2f}')
        self.nav_pub.publish(g)

    def _nav_to_place(self):
        g = NavGoal()
        g.frame_id = self.task.place_frame_id or 'map'
        g.x = self.task.place_x
        g.y = self.task.place_y
        g.yaw = self.task.place_yaw
        self._pub_status(TaskStatus.NAV_TO_PLACE, '导航到放置点')
        self.get_logger().info(
            f'导航到放置点 ({g.x:.2f},{g.y:.2f}) 朝向 {g.yaw:.2f}')
        self.nav_pub.publish(g)

    def nav_status_cb(self, msg: NavStatus):
        if self.state == TaskStatus.NAV_TO_OBJECT and \
                msg.status == NavStatus.ARRIVED:
            # 刚到位车还在微调/停稳 → 先稳定等待，再开识别窗口
            self.get_logger().info('已到达观测点，等待稳定后开始识别')
            self._pub_status(TaskStatus.RUNNING, '稳定等待')
            self.settle_start = self.get_clock().now()
        elif self.state == TaskStatus.NAV_TO_PLACE and \
                msg.status == NavStatus.ARRIVED:
            self.get_logger().info('已到达放置点（PLACE 动作占位）')
            self._pub_status(TaskStatus.PLACED, '放置完成')
            self._task_done()
        elif msg.status == NavStatus.FAILED:
            self.get_logger().error(f'导航失败（阶段={self.phase}）')
            self._fail('导航失败')

    # ================= 识别 =================
    def det_cb(self, msg: ObjectDetection):
        # 诊断：收到 /object_detection 时打印（节流 2s，避免刷屏）
        self.get_logger().info(
            f'[det_cb] 收到 class={msg.class_id} | state={self.state} '
            f'phase={self.phase} | target={self.target_class}',
            throttle_duration_sec=2.0)
        if (self.state == TaskStatus.RUNNING and self.phase == '识别中'
                and msg.class_id == self.target_class):
            self.object_pose = msg.pose
            self.get_logger().info(
                f'识别到 {msg.class_id}（PICK 动作占位，'
                f'位姿在 camera 系）')
            self._pub_status(TaskStatus.PICKED, '抓取完成（占位）')
            self._nav_to_place()

    # ================= 轮询：识别超时/对准 =================
    def timer_cb(self):
        now = self.get_clock().now()
        if self.state != TaskStatus.RUNNING:
            return
        if self.phase == '识别中' and self.det_deadline is not None and \
                now > self.det_deadline:
            # 识别超时 → 按序列取下一个对准动作
            if self.retry >= len(self.ALIGN_ACTIONS):
                self.get_logger().error(
                    f'识别超时，{self.retry} 次对准仍失败')
                self._fail('识别失败')
                return
            self._start_align(self.ALIGN_ACTIONS[self.retry], now)
        elif self.phase == '对准转' and self.align_start is not None:
            # 闭环：转动时长 = 目标角度 / 角速度（精确控制角度，不受轮询延迟影响）
            dur = abs(self.align_target_angle) / self.align_vel
            if (now - self.align_start).nanoseconds / 1e9 > dur:
                self.cmd_vel_pub.publish(Twist())      # 停止
                self.phase = '稳定等待'                 # 先等小车停稳/相机稳定
                self.settle_start = now
                self.get_logger().info(
                    f'对准动作 {self.current_action} 结束（转 '
                    f'{self.align_target_angle*57.3:.1f}°），等待稳定 '
                    f'{self.settle_time}s 后识别')
        elif self.phase == '对准平移' and self.align_start is not None and \
                (now - self.align_start).nanoseconds / 1e9 > self.move_duration:
            self.cmd_vel_pub.publish(Twist())
            self.phase = '稳定等待'
            self.settle_start = now
            self.get_logger().info(
                f'对准动作 {self.current_action} 结束（移动 '
                f'{self.move_vel*self.move_duration*100:.0f}cm），等待稳定 '
                f'{self.settle_time}s 后识别')
        elif self.phase == '稳定等待' and self.settle_start is not None and \
                (now - self.settle_start).nanoseconds / 1e9 > self.settle_time:
            # 小车已停稳 → 正式开始识别窗口
            self._pub_status(None, '识别中')
            self._reset_detect()
            self.get_logger().info('已稳定，开始识别')

    def _start_align(self, action: str, now):
        """执行一个对准动作：原地转 或 前后移动（cmd_vel 脉冲）"""
        t = Twist()
        if action == 'rot+':
            t.angular.z = self.align_vel
            self.align_target_angle = self.rot_angle      # 目标转动角（+）
        elif action == 'rot-':
            t.angular.z = -self.align_vel
            self.align_target_angle = -self.rot_angle     # 目标转动角（-）
        elif action == 'back':
            t.linear.x = -self.move_vel
        elif action == 'fwd':
            t.linear.x = self.move_vel
        self.retry += 1
        self.current_action = action
        self.cmd_vel_pub.publish(t)
        self.phase = '对准转' if 'rot' in action else '对准平移'
        self.align_start = now
        if 'rot' in action:
            self.get_logger().info(
                f'识别超时，执行对准动作 {action}（第 {self.retry} 次，'
                f'目标 {self.rot_angle*57.3:.1f}°）')
        else:
            self.get_logger().info(
                f'识别超时，执行对准动作 {action}（第 {self.retry} 次，'
                f'移动 {self.move_vel*self.move_duration*100:.0f}cm）')

    def _reset_detect(self):
        self.det_deadline = self.get_clock().now() + \
            rclpy.duration.Duration(seconds=self.detect_timeout)

    # ================= 收尾 =================
    def _task_done(self):
        self._pub_status(TaskStatus.IDLE, '任务完成')
        self.task = None

    def _fail(self, reason: str):
        self._pub_status(TaskStatus.FAILED, reason)
        self.state = TaskStatus.IDLE
        self.phase = '空闲'
        self.task = None


def main(args=None):
    rclpy.init(args=args)
    node = TaskOrchestrator()
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
