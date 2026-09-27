#!/usr/bin/env python3
"""M9 任务编排节点 task_orchestrator（M12/M13 起：真实抓取 + 放置）。

订阅 /task_cmd（TaskCommand: object_id + 放置点）后执行状态机：

  1. NAV_TO_OBJECT   导航到"物体观测点"（能看见 ArUco 标签的位置）
  2. 识别确认         等 /object_detection 认到目标；超时则原地小幅转动/前后试探，
                      最多 max_retry 次（ALIGN_ACTIONS 序列）
  3. NAV_TO_GRASP    从观测点沿朝向再前进 grasp_forward 米 —— 必须走这一步，
                      因为竖直下抓时方块在 base 系 x>0.50 就超出机械臂可达范围，
                      而观测点上方块在 x≈0.60
  4. GRASPING        预抓取 → 张开夹爪 → 笛卡尔直下 → 合拢 → [吸附] → 笛卡尔直上 → carry
  5. NAV_TO_PLACE    导航到放置点前方 place_standoff 处
  6. PLACING         预放置 → 笛卡尔下放 → 松爪 → [脱开] → 笛卡尔退回
  7. PLACED → IDLE

全程发布 /task_status（数值状态 + 中文 phase）。

为什么抓取用 action 而不是话题
------------------------------
原来的 /pose_cmd、/gripper_cmd 是 fire-and-forget，发完不知道成没成。
抓取必须能判断成功/超时/失败，否则只能 sleep 硬等、失败也无法重试，
所以 M11 起 my_robot_commander 提供了 /arm_task（ArmTask.action），
本节点用**异步** action client 驱动它：不阻塞执行器（节点还要处理
/joint_states、TF、导航反馈），也不会和 MoveIt 内部回调互相饿死。

依赖（需同时运行）：
  - my_robot_nav_commander nav_commander（/nav_cmd → /nav_status）
  - my_robot_perception aruco_detector（/object_detection）
  - my_robot_commander_cpp commander.launch.py（/arm_task action server）
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from my_robot_interfaces.msg import (
    TaskCommand, TaskStatus, NavGoal, NavStatus, ObjectDetection)
from my_robot_interfaces.action import ArmTask


def quat_from_rpy(roll, pitch, yaw):
    """欧拉角 → 四元数 (x, y, z, w)。"""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


class TaskOrchestrator(Node):
    # 对准动作序列（按 retry 顺序执行；识别超时一次取一个动作）：
    #   rot+ 左转 / rot- 右转 / back 后退 / fwd 前进
    # 序列设计：覆盖【左偏→回正→右偏→回正】两侧朝向 + 前后距离试探
    # 序列设计：先扫朝向（左右各偏一点），再**逐步后退**——"离得太近、标签出了
    # 相机画面"是最常见的失败模式（相机视场是有限的，越近仰角越大），所以后退
    # 排在前面且步长更大；最后才试探前进。
    ALIGN_ACTIONS = ['rot+', 'rot-', 'rot-', 'rot+',
                     'back', 'back', 'back',
                     'fwd', 'rot+', 'rot-']

    def __init__(self):
        super().__init__('task_orchestrator')

        # ================= 参数 =================
        # --- 观测点 ---
        self.declare_parameter('obs_x', 2.2)
        self.declare_parameter('obs_y', 3.0)
        self.declare_parameter('obs_yaw', -0.13)
        self.declare_parameter('obs_frame', 'map')
        # --- 识别与对准 ---
        self.declare_parameter('detect_timeout', 5.0)
        self.declare_parameter('max_retry', 10)
        self.declare_parameter('align_vel', 0.5)
        self.declare_parameter('rot_angle', 0.12)
        self.declare_parameter('move_vel', 0.15)
        self.declare_parameter('move_duration', 0.6)    # 前进时长 s（≈9cm）
        self.declare_parameter('back_duration', 1.2)    # 后退时长 s（≈18cm，步子更大）
        self.declare_parameter('settle_time', 1.5)
        # --- M12 抓取 ---
        self.declare_parameter('enable_pick', True)
        self.declare_parameter('grasp_forward', 0.175)   # 观测点→抓取点前进距离
        self.declare_parameter('tool_x', 0.42)           # 抓取时物体在 base 系 x
        self.declare_parameter('tool_y', 0.078)          # 抓取时物体在 base 系 y
        self.declare_parameter('pregrasp_z', 0.60)
        self.declare_parameter('grasp_z', 0.50)
        self.declare_parameter('lift_z', 0.70)
        self.declare_parameter('tool_roll', math.pi)     # 工具 +Z 朝下
        self.declare_parameter('tool_pitch', 0.0)
        self.declare_parameter('tool_yaw', 0.0)
        # --- M13 放置 ---
        self.declare_parameter('place_standoff', 0.42)   # 机器人在放置点前方多远
        self.declare_parameter('place_approach_z', 0.65)
        self.declare_parameter('place_down_z', 0.50)     # 指爪底端刚好贴台面 0.40
        # --- 通用 ---
        self.declare_parameter('arm_timeout', 60.0)
        self.declare_parameter('arm_server_timeout', 40.0)
        self.declare_parameter('nav_timeout', 150.0)
        # --- 吸附（DetachableJoint，仿真专用）---
        self.declare_parameter('attach_enabled', False)
        self.declare_parameter('attach_topic', '/red_cube/attach')
        self.declare_parameter('detach_topic', '/red_cube/detach')
        self.declare_parameter('detach_on_start', True)
        self.declare_parameter('attach_settle', 0.5)

        g = self.get_parameter
        self.obs_x = g('obs_x').value
        self.obs_y = g('obs_y').value
        self.obs_yaw = g('obs_yaw').value
        self.obs_frame = g('obs_frame').value
        self.detect_timeout = g('detect_timeout').value
        self.max_retry = g('max_retry').value
        self.align_vel = g('align_vel').value
        self.rot_angle = g('rot_angle').value
        self.move_vel = g('move_vel').value
        self.move_duration = g('move_duration').value
        self.back_duration = g('back_duration').value
        self.settle_time = g('settle_time').value
        self.enable_pick = bool(g('enable_pick').value)
        self.grasp_forward = g('grasp_forward').value
        self.tool_x = g('tool_x').value
        self.tool_y = g('tool_y').value
        self.pregrasp_z = g('pregrasp_z').value
        self.grasp_z = g('grasp_z').value
        self.lift_z = g('lift_z').value
        self.tool_rpy = (g('tool_roll').value, g('tool_pitch').value,
                         g('tool_yaw').value)
        self.place_standoff = g('place_standoff').value
        self.place_approach_z = g('place_approach_z').value
        self.place_down_z = g('place_down_z').value
        self.arm_timeout = g('arm_timeout').value
        self.arm_server_timeout = g('arm_server_timeout').value
        self.nav_timeout = g('nav_timeout').value
        self.attach_enabled = bool(g('attach_enabled').value)
        self.attach_topic = g('attach_topic').value
        self.detach_topic = g('detach_topic').value
        self.detach_on_start = bool(g('detach_on_start').value)
        self.attach_settle = g('attach_settle').value

        # ================= 接口 =================
        self.task_sub = self.create_subscription(
            TaskCommand, '/task_cmd', self.task_cmd_cb, 10)
        self.nav_status_sub = self.create_subscription(
            NavStatus, '/nav_status', self.nav_status_cb, 10)
        self.det_sub = self.create_subscription(
            ObjectDetection, '/object_detection', self.det_cb, 10)
        self.nav_pub = self.create_publisher(NavGoal, '/nav_cmd', 10)
        self.status_pub = self.create_publisher(TaskStatus, '/task_status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.attach_pub = self.create_publisher(Empty, self.attach_topic, 10)
        self.detach_pub = self.create_publisher(Empty, self.detach_topic, 10)
        self.arm_client = ActionClient(self, ArmTask, '/arm_task')

        # ================= 状态 =================
        self.state = TaskStatus.IDLE
        self.phase = '空闲'
        self.stage = 'IDLE'          # 内部状态机
        self.task = None
        self.target_class = ''
        self.retry = 0
        self.current_action = ''
        self.align_target_angle = 0.0
        self.det_deadline = None
        self.align_start = None
        self.settle_start = None
        self.settle_next = None
        self.stage_deadline = None
        self.object_pose = None
        # 手臂动作序列驱动状态
        self.ops = None
        self.op_idx = 0
        self.op_kind = None          # None / 'GOAL' / 'WAIT' / 'SERVER_WAIT'
        self.op_deadline = None
        self.op_wait_until = None
        self.op_goal_handle = None
        self.ops_done = None         # (state, phase)
        self.ops_then = None

        self.create_timer(0.1, self.timer_cb)

        # 上电先脱开吸附：DetachableJoint 插件在 Configure 时就会把方块
        # 和夹爪连起来，如果不管它，方块从第一帧起就被夹爪拖着走。
        if self.attach_enabled and self.detach_on_start:
            self.startup_detach_left = 3
            self.create_timer(1.0, self.startup_detach_cb)

        self._pub_status()
        self.get_logger().info(
            f'task_orchestrator 启动：观测点=({self.obs_x},{self.obs_y}) '
            f'yaw={self.obs_yaw} | 抓取前进 {self.grasp_forward}m | '
            f'抓取工具位 ({self.tool_x},{self.tool_y}) z={self.grasp_z} | '
            f'吸附={"开" if self.attach_enabled else "关"}')

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

    def _now(self):
        return self.get_clock().now()

    def _pub_empty(self, publisher, what):
        publisher.publish(Empty())
        self.get_logger().info(f'已发布 {what}')

    # ================= 任务入口 =================
    def task_cmd_cb(self, msg: TaskCommand):
        if self.stage != 'IDLE':
            self.get_logger().warn(f'任务进行中（stage={self.stage}），忽略新命令')
            return
        self.task = msg
        self.target_class = msg.object_id
        self.retry = 0
        self.object_pose = None
        self.get_logger().info(
            f'收到任务：抓取 {msg.object_id} → 放置 '
            f'({msg.place_x:.2f},{msg.place_y:.2f}) '
            f'yaw={msg.place_yaw:.2f} frame={msg.place_frame_id}')
        self._nav_to_object()

    # ================= 导航 =================
    def _send_nav(self, x, y, yaw, frame, state, phase, stage):
        g = NavGoal()
        g.frame_id = frame
        g.x = float(x)
        g.y = float(y)
        g.yaw = float(yaw)
        self._pub_status(state, phase)
        self.stage = stage
        self.stage_deadline = self._now() + rclpy.duration.Duration(
            seconds=self.nav_timeout)
        self.get_logger().info(
            f'{phase}：({g.x:.2f},{g.y:.2f}) yaw={g.yaw:.2f} frame={frame}')
        self.nav_pub.publish(g)

    def _nav_to_object(self):
        self._send_nav(self.obs_x, self.obs_y, self.obs_yaw, self.obs_frame,
                       TaskStatus.NAV_TO_OBJECT, '导航到物体观测点', 'NAV_OBS')

    def _nav_to_grasp(self):
        # 从观测点沿朝向再前进 grasp_forward 米（保持同一朝向）
        gx = self.obs_x + self.grasp_forward * math.cos(self.obs_yaw)
        gy = self.obs_y + self.grasp_forward * math.sin(self.obs_yaw)
        self._send_nav(gx, gy, self.obs_yaw, self.obs_frame,
                       TaskStatus.NAV_TO_GRASP, '导航到抓取点', 'NAV_GRASP')

    def _nav_to_place(self):
        # 放置点前方 standoff 处停车，物体落在 base 系 (standoff, 0)
        px = self.task.place_x - self.place_standoff * math.cos(self.task.place_yaw)
        py = self.task.place_y - self.place_standoff * math.sin(self.task.place_yaw)
        self._send_nav(px, py, self.task.place_yaw,
                       self.task.place_frame_id or 'map',
                       TaskStatus.NAV_TO_PLACE, '导航到放置点', 'NAV_PLACE')

    def nav_status_cb(self, msg: NavStatus):
        if msg.status == NavStatus.FAILED:
            self.get_logger().error(f'导航失败（stage={self.stage}）')
            self._fail('导航失败')
            return
        if msg.status != NavStatus.ARRIVED:
            return
        if self.stage == 'NAV_OBS':
            self._enter_settle(TaskStatus.RUNNING, '稳定等待', 'DETECT')
        elif self.stage == 'NAV_GRASP':
            self._enter_settle(TaskStatus.GRASPING, '抓取点稳定等待', 'PICK')
        elif self.stage == 'NAV_PLACE':
            self._enter_settle(TaskStatus.PLACING, '放置点稳定等待', 'PLACE')

    def _enter_settle(self, state, phase, next_stage):
        self.get_logger().info(f'已到位，等待稳定 {self.settle_time}s 后进入 {next_stage}')
        self._pub_status(state, phase)
        self.settle_start = self._now()
        self.stage = 'SETTLE'
        self.settle_next = next_stage

    # ================= 识别 =================
    def det_cb(self, msg: ObjectDetection):
        self.get_logger().info(
            f'[det_cb] 收到 class={msg.class_id} | stage={self.stage} '
            f'phase={self.phase} | target={self.target_class}',
            throttle_duration_sec=2.0)
        if (self.stage == 'DETECT' and self.phase == '识别中'
                and msg.class_id == self.target_class):
            self.object_pose = msg.pose
            self.get_logger().info(
                f'识别到 {msg.class_id}，位姿(camera 系)='
                f'({msg.pose.pose.position.x:.3f},{msg.pose.pose.position.y:.3f},'
                f'{msg.pose.pose.position.z:.3f})')
            self._begin_pick()

    def _begin_pick(self):
        if not self.enable_pick:
            # 调试模式：跳过机械臂，直接当"已抓起"（等价于 M9 的占位行为）
            self.get_logger().warn('enable_pick=false，跳过抓取，直接去放置点')
            self._pub_status(TaskStatus.PICKED, '抓取完成（已跳过）')
            self._nav_to_place()
            return
        self._nav_to_grasp()

    # ================= 手臂动作序列 =================
    def _goal_named(self, group, target):
        goal = ArmTask.Goal()
        goal.task_type = ArmTask.Goal.NAMED
        goal.group_name = group
        goal.named_target = target
        goal.timeout = float(self.arm_timeout)
        return goal

    def _goal_pose(self, x, y, z, cartesian):
        goal = ArmTask.Goal()
        goal.task_type = ArmTask.Goal.POSE
        goal.target_pose.position.x = float(x)
        goal.target_pose.position.y = float(y)
        goal.target_pose.position.z = float(z)
        qx, qy, qz, qw = quat_from_rpy(*self.tool_rpy)
        goal.target_pose.orientation.x = qx
        goal.target_pose.orientation.y = qy
        goal.target_pose.orientation.z = qz
        goal.target_pose.orientation.w = qw
        goal.use_cartesian = bool(cartesian)
        goal.timeout = float(self.arm_timeout)
        return goal

    @staticmethod
    def _op_goal(label, goal):
        return {'kind': 'goal', 'label': label, 'goal': goal}

    @staticmethod
    def _op_wait(label, seconds):
        return {'kind': 'wait', 'label': label, 'seconds': seconds}

    @staticmethod
    def _op_fn(label, fn):
        return {'kind': 'fn', 'label': label, 'fn': fn}

    def _pick_ops(self):
        ops = [
            self._op_goal('预抓取',
                          self._goal_pose(self.tool_x, self.tool_y,
                                          self.pregrasp_z, False)),
            self._op_goal('张开夹爪', self._goal_named('gripper', 'gripper_open')),
            self._op_goal('下压抓取',
                          self._goal_pose(self.tool_x, self.tool_y,
                                          self.grasp_z, True)),
            self._op_goal('合拢夹爪',
                          self._goal_named('gripper', 'gripper_grasp')),
        ]
        if self.attach_enabled:
            ops.append(self._op_fn('吸附物体', self._do_attach))
            ops.append(self._op_wait('吸附稳定', self.attach_settle))
        ops += [
            self._op_goal('抬起',
                          self._goal_pose(self.tool_x, self.tool_y,
                                          self.lift_z, True)),
            self._op_goal('收起搬运', self._goal_named('arm', 'carry')),
        ]
        return ops

    def _place_ops(self):
        ops = [
            self._op_goal('预放置',
                          self._goal_pose(self.place_standoff, 0.0,
                                          self.place_approach_z, False)),
            self._op_goal('下放',
                          self._goal_pose(self.place_standoff, 0.0,
                                          self.place_down_z, True)),
            self._op_goal('松开夹爪', self._goal_named('gripper', 'gripper_open')),
        ]
        if self.attach_enabled:
            ops.append(self._op_fn('脱开物体', self._do_detach))
            ops.append(self._op_wait('脱开稳定', self.attach_settle))
        ops += [
            self._op_goal('退回',
                          self._goal_pose(self.place_standoff, 0.0,
                                          self.place_approach_z, True)),
        ]
        return ops

    def _run_ops(self, ops, done_state, done_phase, then=None):
        self.ops = list(ops)
        self.op_idx = 0
        self.op_kind = None
        self.ops_done = (done_state, done_phase)
        self.ops_then = then
        self.op_server_deadline = self._now() + rclpy.duration.Duration(
            seconds=self.arm_server_timeout)
        self._op_start()

    def _op_start(self):
        if self.ops is None:
            return
        while True:
            if self.op_idx >= len(self.ops):
                st, ph = self.ops_done
                self.ops = None
                self.op_kind = None
                self._pub_status(st, ph)
                then, self.ops_then = self.ops_then, None
                if then:
                    then()
                return
            op = self.ops[self.op_idx]
            self._pub_status(None, op['label'])

            if op['kind'] == 'fn':
                self.get_logger().info(f'[手臂序列] {op["label"]}')
                try:
                    op['fn']()
                except Exception as e:  # noqa: BLE001
                    self._fail(f'{op["label"]} 异常: {e}')
                    return
                self.op_idx += 1
                continue

            if op['kind'] == 'wait':
                self.get_logger().info(
                    f'[手臂序列] {op["label"]}（等 {op["seconds"]}s）')
                self.op_wait_until = self._now() + rclpy.duration.Duration(
                    seconds=float(op['seconds']))
                self.op_kind = 'WAIT'
                return

            # goal
            if not self.arm_client.server_is_ready():
                self.get_logger().warn(
                    f'/arm_task 服务还没就绪，等待中（{op["label"]}）',
                    throttle_duration_sec=2.0)
                self.op_kind = 'SERVER_WAIT'
                return
            self.get_logger().info(f'[手臂序列] {op["label"]} → 发送 /arm_task')
            self.op_kind = 'GOAL'
            self.op_deadline = self._now() + rclpy.duration.Duration(
                seconds=self.arm_timeout)
            fut = self.arm_client.send_goal_async(op['goal'])
            fut.add_done_callback(self._on_goal_response)
            return

    def _on_goal_response(self, future):
        if self.ops is None:
            return
        try:
            handle = future.result()
        except Exception as e:  # noqa: BLE001
            self._fail(f'发送手臂目标异常: {e}')
            return
        if not handle.accepted:
            self._fail(f'{self.ops[self.op_idx]["label"]} 被拒绝（可能上一动作未结束）')
            return
        self.op_goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        if self.ops is None:
            return
        label = self.ops[self.op_idx]['label']
        try:
            wrapped = future.result()
            ok = bool(wrapped.result.success)
            msg = wrapped.result.message
        except Exception as e:  # noqa: BLE001
            self._fail(f'{label} 结果异常: {e}')
            return
        self.op_kind = None
        self.op_goal_handle = None
        if not ok:
            self._fail(f'{label} 失败: {msg}')
            return
        self.get_logger().info(f'[手臂序列] {label} 完成')
        self.op_idx += 1
        self._op_start()

    def _cancel_arm(self):
        if self.op_goal_handle is not None:
            try:
                self.op_goal_handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
        self.op_goal_handle = None

    # ================= 吸附（DetachableJoint） =================
    def _do_attach(self):
        self._pub_empty(self.attach_pub, f'吸附 attach → {self.attach_topic}')

    def _do_detach(self):
        self._pub_empty(self.detach_pub, f'脱开 detach → {self.detach_topic}')

    def startup_detach_cb(self):
        """上电后补几次脱开，抵消 DetachableJoint 在 Configure 时的自动连接。"""
        if self.startup_detach_left <= 0:
            return
        self.startup_detach_left -= 1
        self._pub_empty(self.detach_pub, '上电脱开 detach')

    # ================= 轮询 =================
    def timer_cb(self):
        now = self._now()

        # --- 手臂序列驱动 ---
        if self.ops is not None:
            if self.op_kind == 'WAIT' and now >= self.op_wait_until:
                self.op_idx += 1
                self._op_start()
                return
            if self.op_kind == 'SERVER_WAIT':
                if now >= self.op_server_deadline:
                    self._fail('等待 /arm_task 服务超时')
                    return
                self._op_start()
                return
            if self.op_kind == 'GOAL' and now >= self.op_deadline:
                self._cancel_arm()
                self._fail(f'{self.ops[self.op_idx]["label"]} 超时')
                return
            return

        # --- 稳定等待 ---
        if self.stage == 'SETTLE':
            if self.settle_start is not None and \
                    (now - self.settle_start).nanoseconds / 1e9 > self.settle_time:
                nxt, self.settle_next = self.settle_next, None
                if nxt == 'DETECT':
                    self.stage = 'DETECT'
                    self._pub_status(None, '识别中')
                    self._reset_detect()
                    self.get_logger().info('已稳定，开始识别')
                elif nxt == 'PICK':
                    self.stage = 'PICK'
                    self._run_ops(self._pick_ops(),
                                  TaskStatus.PICKED, '抓取完成，准备搬运',
                                  then=self._nav_to_place)
                elif nxt == 'PLACE':
                    self.stage = 'PLACE'
                    self._run_ops(self._place_ops(),
                                  TaskStatus.PLACED, '放置完成',
                                  then=self._task_done)
            return

        # --- 识别超时 / 对准 ---
        if self.stage != 'DETECT':
            return
        if self.phase == '识别中' and self.det_deadline is not None and \
                now > self.det_deadline:
            if self.retry >= len(self.ALIGN_ACTIONS):
                self.get_logger().error(f'识别超时，{self.retry} 次对准仍失败')
                self._fail('识别失败')
                return
            self._start_align(self.ALIGN_ACTIONS[self.retry], now)
        elif self.phase == '对准转' and self.align_start is not None:
            dur = abs(self.align_target_angle) / self.align_vel
            if (now - self.align_start).nanoseconds / 1e9 > dur:
                self.cmd_vel_pub.publish(Twist())
                self.phase = '稳定等待'
                self.settle_start = now
                self.get_logger().info(
                    f'对准动作 {self.current_action} 结束（转 '
                    f'{self.align_target_angle * 57.3:.1f}°），稳定 '
                    f'{self.settle_time}s 后继续识别')
        elif self.phase == '对准平移' and self.align_start is not None and \
                (now - self.align_start).nanoseconds / 1e9 > self.align_duration:
            self.cmd_vel_pub.publish(Twist())
            self.phase = '稳定等待'
            self.settle_start = now
            self.get_logger().info(
                f'对准动作 {self.current_action} 结束（移动 '
                f'{self.move_vel * self.align_duration * 100:.0f}cm），稳定 '
                f'{self.settle_time}s 后继续识别')
        elif self.phase == '稳定等待' and self.settle_start is not None and \
                (now - self.settle_start).nanoseconds / 1e9 > self.settle_time:
            self._pub_status(None, '识别中')
            self._reset_detect()
            self.get_logger().info('已稳定，继续识别')

    def _start_align(self, action, now):
        t = Twist()
        if action == 'rot+':
            t.angular.z = self.align_vel
            self.align_target_angle = self.rot_angle
        elif action == 'rot-':
            t.angular.z = -self.align_vel
            self.align_target_angle = -self.rot_angle
        elif action == 'back':
            t.linear.x = -self.move_vel
        elif action == 'fwd':
            t.linear.x = self.move_vel
        self.retry += 1
        self.current_action = action
        self.cmd_vel_pub.publish(t)
        self.phase = '对准转' if 'rot' in action else '对准平移'
        # 后退步子比前进大（"太近看不到"要一次退够）
        self.align_duration = (self.back_duration if action == 'back'
                               else self.move_duration)
        self.align_start = now
        if 'rot' in action:
            self.get_logger().info(
                f'识别超时，对准动作 {action}（第 {self.retry} 次，'
                f'目标 {self.rot_angle * 57.3:.1f}°）')
        else:
            self.get_logger().info(
                f'识别超时，对准动作 {action}（第 {self.retry} 次，'
                f'移动 {self.move_vel * (self.back_duration if action == "back" else self.move_duration) * 100:.0f}cm）')

    def _reset_detect(self):
        self.det_deadline = self._now() + rclpy.duration.Duration(
            seconds=self.detect_timeout)

    # ================= 收尾 =================
    def _task_done(self):
        self._pub_status(TaskStatus.IDLE, '任务完成')
        self.stage = 'IDLE'
        self.task = None
        self.object_pose = None

    def _fail(self, reason):
        self._cancel_arm()
        self.ops = None
        self.op_kind = None
        self.get_logger().error(f'任务失败：{reason}')
        # 若失败发生在"已吸附"之后（例如抓起后导航到放置点失败），必须补一次
        # detach，否则方块会一直粘在夹爪上，场景再也回不到初始状态。
        if self.attach_enabled:
            self._pub_empty(self.detach_pub, '失败收尾 detach')
        self._pub_status(TaskStatus.FAILED, reason)
        self.state = TaskStatus.IDLE
        self.phase = '空闲'
        self.stage = 'IDLE'
        self.task = None
        self.object_pose = None
        # 失败后把手臂收起来，避免挡着激光影响后续导航
        if self.arm_client.server_is_ready():
            self.arm_client.send_goal_async(self._goal_named('arm', 'carry'))


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
