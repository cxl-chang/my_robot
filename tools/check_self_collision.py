#!/usr/bin/env python3
"""用 MoveIt 自己的碰撞检查（/check_state_validity）扫关节构型，报告非法构型。

为什么需要它：几何验算（AABB 间距）在有旋转时会**误报**——旋转盒的轴对齐包围盒
会膨胀，两个本来不碰的零件会被算成 0mm。真正的判断必须用 FCL + SRDF 的 ACM，
这也正是规划器实际使用的判据。

前提：仿真 + move_group 已启动（`ros2 launch my_robot_bringup demo_all.launch.py`
或至少 `ros2 launch my_robot_moveit_config move_group.launch.py`）。

用法：
  # 默认：SRDF 里 arm 组的所有命名状态（home/carry/place_approach/...）
  python3 tools/check_self_collision.py

  # 额外做 joint6 整圈扫描（每 15°），基准姿态用 carry
  python3 tools/check_self_collision.py --sweep joint6 --step 15 --base carry

  # 同时扫 joint5 与 joint6
  python3 tools/check_self_collision.py --sweep joint5,joint6 --step 15 --base carry

  # 只看某个状态
  python3 tools/check_self_collision.py --state place_down
"""
import argparse
import math
import sys
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']


def load_srdf_states(group='arm', srdf_pkg='my_robot_moveit_config',
                     srdf_name='my_robot.srdf'):
    """从 SRDF 读该组的全部命名状态（避免在脚本里硬编码关节角）。"""
    path = f'{get_package_share_directory(srdf_pkg)}/config/{srdf_name}'
    root = ET.parse(path).getroot()
    out = {}
    for gs in root.findall('group_state'):
        if gs.get('group') != group:
            continue
        out[gs.get('name')] = {j.get('name'): float(j.get('value'))
                               for j in gs.findall('joint')}
    return out


class Checker(Node):
    def __init__(self, group):
        super().__init__('self_collision_checker')
        self.group = group
        self.cli = self.create_client(GetStateValidity, '/check_state_validity')
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                '/check_state_validity 不可用 —— 先启动 move_group')
            sys.exit(2)

    def check(self, pos):
        req = GetStateValidity.Request()
        req.group_name = self.group
        rs = RobotState()
        js = JointState()
        js.name = list(ARM_JOINTS)
        js.position = [float(v) for v in pos]
        rs.joint_state = js
        req.robot_state = rs
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        if res is None:
            return None, 'service timeout', []
        if res.valid:
            return True, '', []
        msgs, pairs = [], []
        for c in res.contacts[:4]:
            msgs.append(f'{c.contact_body_1}<->{c.contact_body_2} '
                        f'depth {c.depth * 1000:.1f}mm')
            pairs.append((c.contact_body_1, c.contact_body_2))
        for cp in res.constraint_result:
            if not cp.valid:
                msgs.append(f'{cp.constraint} 不满足')
        return False, ('; '.join(msgs) if msgs else 'invalid（原因未给出）'), pairs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--group', default='arm')
    ap.add_argument('--state', default=None, help='只检查这个命名状态')
    ap.add_argument('--base', default='carry', help='扫描用的基准命名状态')
    ap.add_argument('--sweep', default='', help='要扫描的关节名，逗号分隔，如 joint6 或 joint5,joint6')
    ap.add_argument('--step', type=float, default=15.0, help='扫描步进（度）')
    ap.add_argument('--only-camera', action='store_true',
                    help='只报告涉及 wrist_camera_* 的非法构型（过滤掉机械臂自身的自碰）')
    args = ap.parse_args()

    rclpy.init()
    states = load_srdf_states(args.group)
    if not states:
        print('SRDF 里没读到命名状态')
        return 1
    node = Checker(args.group)

    fail = 0
    if args.state:
        todo = {args.state: states[args.state]}
    else:
        todo = states
    print(f'=== 命名状态（{len(todo)} 个）===')
    for name, js in todo.items():
        pos = [js.get(j, 0.0) for j in ARM_JOINTS]
        ok, why, _pairs = node.check(pos)
        if ok is None:
            print(f'  {name:16s} 服务超时')
            fail += 1
        elif ok:
            print(f'  {name:16s} ✅ 有效')
        else:
            print(f'  {name:16s} ❌ {why}')
            fail += 1

    if args.sweep:
        joints = [s.strip() for s in args.sweep.split(',') if s.strip()]
        base = states.get(args.base, {})
        print(f'\n=== 扫描 {joints}（步进 {args.step}°，基准 {args.base}）===')
        import itertools
        if args.step <= 0:
            print('--step 必须为正数')
            return 1
        # 按真实关节限位决定扫描范围（度）；joint6 是 continuous ⇒ 整圈
        LIMITS_DEG = {
            'joint1': (-180.0, 180.0),
            'joint2': (-143.2, 34.4),      # -2.5 .. 0.6 rad
            'joint3': (-143.2, 143.2),     # ±2.5 rad
            'joint4': (-180.0, 180.0),
            'joint5': (-90.0, 90.0),       # ±1.57 rad
            'joint6': (-180.0, 180.0),     # continuous
        }
        grids = []
        for jn in joints:
            lo, hi = LIMITS_DEG.get(jn, (-180.0, 180.0))
            grids.append(range(int(lo), int(hi) + 1, int(args.step)))
        cnt = bad = cam_bad = other_bad = 0
        other_examples = []
        for combo in itertools.product(*grids):
            js = dict(base)
            lab = []
            for jn, deg in zip(joints, combo):
                js[jn] = math.radians(deg)
                lab.append(f'{jn}={deg}°')
            pos = [js.get(j, 0.0) for j in ARM_JOINTS]
            ok, why, pairs = node.check(pos)
            cnt += 1
            if ok is False:
                bad += 1
                is_cam = any('wrist_camera' in b for pair in pairs for b in pair)
                if is_cam:
                    cam_bad += 1
                    print(f'  {" ".join(lab):32s} ❌ [腕相机] {why}')
                else:
                    other_bad += 1
                    if len(other_examples) < 5:
                        other_examples.append(f'{" ".join(lab):32s} ❌ {why}')
        print(f'  扫描 {cnt} 个构型，非法 {bad} 个'
              f'（与腕相机相关 {cam_bad}，机械臂自身自碰 {other_bad}）')
        if other_bad and not args.only_camera:
            print('  机械臂自身自碰示例（与腕相机无关，规划器会绕开这些构型）：')
            for e in other_examples:
                print('   ', e)
        fail += cam_bad if args.only_camera else bad

    node.destroy_node()
    rclpy.shutdown()
    print('\n结论:', '全部有效 ✅' if fail == 0 else f'存在 {fail} 个非法构型 ❌')
    return 0 if fail == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
