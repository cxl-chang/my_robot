#!/usr/bin/env python3
"""夹爪开合单独验证脚本。

前置（两个终端）
----------------
    # 终端 1：仿真（gripper_mirror 会随它自动拉起，右指靠它跟随）
    ros2 launch my_robot_bringup my_robot_gazebo.launch.xml

    # 终端 2：手臂/夹爪执行入口
    ros2 launch my_robot_commander_cpp commander.launch.py

然后
----
    # 终端 3
    ros2 run my_robot_bringup check_gripper.py

它会依次下发 张开 → 夹住5cm → 完全闭合 → 张开，每步读取
/joint_states 里两个指关节的真实位置，与期望值比对并打印表格，
最后给出结论。退出码 0 = 全部通过，1 = 有失败项。

参数
----
timeout     等待 /arm_task 服务就绪的秒数，默认 30
settle      每个动作完成后等待稳定的秒数，默认 2.0
tol         关节位置允许误差（米），默认 0.008

期望值来源
----------
指根内距 = 0.12 - 2 × 左指值（SRDF 里两指约定一致，都是"正值=合拢"）：

    gripper_open         0      → 内距 0.12 m
    gripper_half_closed  0.03   → 内距 0.06 m
    gripper_grasp        0.035  → 内距 0.05 m（正好夹住 5cm 的 red_cube）
    gripper_closed       0.06   → 内距 0（完全闭合，抓 5cm 方块会硬顶）

右指在仿真里由 Gazebo 原生 JointPositionController 镜像驱动（因为
gz_ros2_control 对这个关节名的写通路有缺陷），所以它会比左指略滞后
（实测差 0.5~0.6mm），容差已考虑这一点。

如果只想手工用命令验证，等价做法见 PICK_PLACE.md 的"夹爪验证"一节。
"""

import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from my_robot_interfaces.action import ArmTask

LEFT = 'gripper_left_finger_joint'
RIGHT = 'gripper_right_finger_joint'
GAP_BASE = 0.12   # 两指全开时的指根内距

# (命名目标, 期望左指, 中文说明)
CASES = [
    ('gripper_open', 0.0, '全开（内距 12cm）'),
    ('gripper_grasp', 0.035, '夹住 5cm 方块（内距 5cm）'),
    ('gripper_half_closed', 0.03, '半闭（内距 6cm）'),
    ('gripper_closed', 0.06, '完全闭合（内距 0）'),
    ('gripper_open', 0.0, '回到全开'),
]


class GripperCheck(Node):

    def __init__(self):
        super().__init__('check_gripper')
        self.latest = {}
        self.stamp = 0.0
        self.create_subscription(JointState, '/joint_states', self.on_js, 50)
        self.client = ActionClient(self, ArmTask, '/arm_task')

    def on_js(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in (LEFT, RIGHT):
                self.latest[name] = float(pos)
        self.stamp = time.time()

    # ---------- 基础工具 ----------
    def spin_for(self, seconds):
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_joint_states(self, seconds=25.0):
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if LEFT in self.latest and RIGHT in self.latest:
                return True
        return False

    def send_named(self, target, timeout):
        goal = ArmTask.Goal()
        goal.task_type = ArmTask.Goal.NAMED
        goal.group_name = 'gripper'
        goal.named_target = target
        goal.timeout = float(timeout)
        fut = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        handle = fut.result()
        if handle is None or not handle.accepted:
            return False, '目标被拒绝（可能上一个动作还没结束）'
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=timeout)
        wrapped = res_fut.result()
        if wrapped is None:
            return False, '等待结果超时'
        return bool(wrapped.result.success), wrapped.result.message


def main():
    rclpy.init()
    node = GripperCheck()

    node.declare_parameter('timeout', 30.0)
    node.declare_parameter('settle', 2.0)
    node.declare_parameter('tol', 0.008)
    timeout = float(node.get_parameter('timeout').value)
    settle = float(node.get_parameter('settle').value)
    tol = float(node.get_parameter('tol').value)

    print('=' * 74)
    print('夹爪开合验证')
    print('=' * 74)

    print('[1/3] 等待 /joint_states 出现两个指关节 ...', end=' ')
    if not node.wait_joint_states(25.0):
        print('失败')
        print('  /joint_states 里看不到指关节。请确认：')
        print('   - Gazebo 已启动（ros2 launch my_robot_bringup '
              'my_robot_gazebo.launch.xml）')
        print('   - joint_state_broadcaster 是 active'
              '（ros2 control list_controllers）')
        node.destroy_node()
        rclpy.shutdown()
        return 1
    print(f'OK（左指={node.latest[LEFT]:+.4f} 右指={node.latest[RIGHT]:+.4f}）')

    print(f'[2/3] 等待 /arm_task 服务 ...', end=' ')
    if not node.client.wait_for_server(timeout_sec=timeout):
        print('失败')
        print('  服务没起来。请确认终端 2 跑的是：')
        print('   ros2 launch my_robot_commander_cpp commander.launch.py')
        print('  （不能用 ros2 run，缺 MoveIt 参数会直接崩溃）')
        node.destroy_node()
        rclpy.shutdown()
        return 1
    print('OK')

    print('[3/3] 依次下发动作：')
    print()
    header = (f'{"命名目标":<20}{"期望左指":>9}{"实际左指":>9}'
              f'{"实际右指":>9}{"内距":>9}  结论')
    print(header)
    print('-' * len(header))

    all_ok = True
    for target, expect, note in CASES:
        ok, msg = node.send_named(target, timeout + 30.0)
        node.spin_for(settle)          # 等稳定 + 刷新 /joint_states
        node.spin_for(0.3)
        left = node.latest.get(LEFT, float('nan'))
        right = node.latest.get(RIGHT, float('nan'))
        gap = GAP_BASE - left - right   # 两指各向内走了 left/right
        good = (ok
                and abs(left - expect) <= tol
                and abs(right - expect) <= tol)
        all_ok = all_ok and good
        verdict = 'PASS' if good else ('FAIL' if ok else f'FAIL({msg})')
        print(f'{target:<20}{expect:>9.3f}{left:>9.4f}{right:>9.4f}'
              f'{gap:>9.4f}  {verdict}  {note}')

    print()
    if all_ok:
        print('结论：夹爪开合正常 ✅  两个指关节都跟随指令。')
    else:
        print('结论：有失败项 ❌')
        print('  排查顺序：')
        print('   1. 左指不动 → gripper_controller 是否 active、'
              'URDF 里是否有该关节')
        print('   2. 右指不动 → gripper_mirror 是否在跑'
              '（ros2 node list | grep mirror）')
        print('   3. 数值对不上 → 是不是用了旧的 gripper_closed 抓 5cm 方块'
              '（那会硬顶）')
    print()

    node.destroy_node()
    rclpy.shutdown()
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
