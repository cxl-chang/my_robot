#!/usr/bin/env python3
"""my_robot 机械臂正运动学 + 数值反解工具（离线，不启动 ROS 节点）。

为什么需要它
------------
SRDF 里的命名姿态（carry 搬运收起姿态）、以及固定点抓取用的 grasp/pregrasp
关节角，都必须满足两个硬约束，靠"看着差不多"手填一定会踩坑：

  1. **被抓物体要高于激光扫描平面**。lidar 装在 base_footlink 系
     z = 0.1(base_joint) + 0.2/3*2 = 0.233 处。手臂抓着方块停在身前
     0.2~0.4m 高度时，2D 激光会把方块当成障碍物塞进 local costmap，
     Nav2 会直接卡死。所以 carry 姿态必须把方块抬到 z > 0.5。
  2. **关节角要在限位内**，不能顶死限位（顶死会让规划器找不到解）。

所以这里直接读 URDF 做 FK，并用 scipy 数值搜索满足目标位姿的关节角。

用法
----
    python3 arm_kinematics.py                 # 打印预设目标位姿的解
    python3 arm_kinematics.py --fk 0 0 0 0 0 0   # 指定关节角做正解
    python3 arm_kinematics.py --solve X Y Z      # 自定义目标位置（工具 Z 朝下）
"""

import argparse
import math
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import minimize

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
TOOL_LINK = 'tool_link'
ROOT_LINK = 'base_footlink'


# ----------------------------------------------------------------------
# 基础数学
# ----------------------------------------------------------------------
def rpy_to_mat(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def axis_angle_to_mat(axis, angle):
    ax = np.array(axis, dtype=float)
    n = np.linalg.norm(ax)
    if n < 1e-12:
        return np.eye(3)
    ax = ax / n
    K = np.array([[0, -ax[2], ax[1]],
                  [ax[2], 0, -ax[0]],
                  [-ax[1], ax[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)


def rot_angle_deg(Ra, Rb):
    """两个旋转矩阵之间的夹角（度）。"""
    dR = Ra.T @ Rb
    c = max(-1.0, min(1.0, (np.trace(dR) - 1.0) / 2.0))
    return math.degrees(math.acos(c))


# ----------------------------------------------------------------------
# URDF 解析与 FK
# ----------------------------------------------------------------------
def load_urdf_tree():
    """用 xacro 展开 my_robot.urdf.xacro，直接拿 stdout 解析（不落临时文件）。"""
    cmd = ('source /opt/ros/humble/setup.bash && '
           'xacro $(ros2 pkg prefix my_robot_description)/share/'
           'my_robot_description/urdf/my_robot.urdf.xacro')
    out = subprocess.run(['bash', '-lc', cmd], check=True,
                         capture_output=True, text=True)
    return ET.fromstring(out.stdout)


def parse_joints(root):
    joints = {}
    for j in root.findall('joint'):
        origin = j.find('origin')
        xyz = [0.0, 0.0, 0.0]
        rpy = [0.0, 0.0, 0.0]
        if origin is not None:
            if origin.get('xyz'):
                xyz = [float(v) for v in origin.get('xyz').split()]
            if origin.get('rpy'):
                rpy = [float(v) for v in origin.get('rpy').split()]
        axis = [0.0, 0.0, 1.0]
        ax = j.find('axis')
        if ax is not None and ax.get('xyz'):
            axis = [float(v) for v in ax.get('xyz').split()]
        lo = hi = None
        lim = j.find('limit')
        if lim is not None and j.get('type') in ('revolute', 'prismatic'):
            lo = float(lim.get('lower', 0.0))
            hi = float(lim.get('upper', 0.0))
        elif j.get('type') == 'continuous':
            # 连续关节没有限位；搜索时按 (-pi, pi] 处理（joint6 是绕 Z 的连续关节）
            lo, hi = -math.pi, math.pi
        joints[j.get('name')] = dict(
            type=j.get('type'),
            parent=j.find('parent').get('link'),
            child=j.find('child').get('link'),
            xyz=np.array(xyz, dtype=float), rpy=rpy,
            axis=np.array(axis, dtype=float),
            lower=lo, upper=hi)
    return joints


def chain_to(joints, target, root=ROOT_LINK):
    by_child = {v['child']: (k, v) for k, v in joints.items()}
    chain, link = [], target
    while link != root:
        if link not in by_child:
            raise KeyError(f'{link} 不在以 {root} 为根的链上（检查 URDF）')
        name, j = by_child[link]
        chain.append(name)
        link = j['parent']
    return list(reversed(chain))


def fk(joints, chain, q):
    """返回 root -> 链末端的 4x4 齐次变换。q: {joint_name: value}"""
    T = np.eye(4)
    for name in chain:
        j = joints[name]
        Tj = np.eye(4)
        Tj[:3, :3] = rpy_to_mat(*j['rpy'])
        Tj[:3, 3] = j['xyz']
        T = T @ Tj
        if name in q:
            Rj = np.eye(4)
            Rj[:3, :3] = axis_angle_to_mat(j['axis'], q[name])
            T = T @ Rj
    return T


# ----------------------------------------------------------------------
# 反解
# ----------------------------------------------------------------------
class ArmKinematics:

    def __init__(self):
        root = load_urdf_tree()
        self.joints = parse_joints(root)
        self.chain = chain_to(self.joints, TOOL_LINK)
        self.lo = np.array([self.joints[n]['lower'] for n in ARM_JOINTS])
        self.hi = np.array([self.joints[n]['upper'] for n in ARM_JOINTS])

    def tool_pose(self, qv):
        T = fk(self.joints, self.chain, dict(zip(ARM_JOINTS, qv)))
        return T[:3, 3], T[:3, :3]

    def solve(self, target_p, target_R, restarts=80, seed=0, w_ori=0.25):
        """多起点 L-BFGS-B 搜索。返回 (qv, pos_err_m, ori_err_deg)"""
        target_p = np.asarray(target_p, dtype=float)
        target_R = np.asarray(target_R, dtype=float)
        rng = np.random.default_rng(seed)
        best = None

        def cost(qv):
            p, R = self.tool_pose(qv)
            return (np.linalg.norm(p - target_p) ** 2
                    + (w_ori * math.radians(rot_angle_deg(target_R, R))) ** 2)

        for _ in range(restarts):
            x0 = self.lo + rng.random(len(self.lo)) * (self.hi - self.lo)
            x0 = np.clip(x0, self.lo + 1e-3, self.hi - 1e-3)
            res = minimize(cost, x0, method='L-BFGS-B',
                           bounds=list(zip(self.lo, self.hi)),
                           options=dict(maxiter=500, ftol=1e-14))
            if best is None or res.fun < best.fun:
                best = res
        q = best.x
        p, R = self.tool_pose(q)
        return q, float(np.linalg.norm(p - target_p)), rot_angle_deg(target_R, R)

    def report(self, label, target_p, target_R, **kw):
        q, ep, eo = self.solve(target_p, target_R, **kw)
        print(f'\n== {label} ==')
        print(f'  目标 {np.round(target_p, 3)}  '
              f'位置误差 {ep * 1000:.1f} mm  姿态误差 {eo:.2f} deg')
        print('  SRDF <joint> 片段:')
        for n, v in zip(ARM_JOINTS, q):
            print(f'      <joint name="{n}" value="{v:.4f}"/>')
        near = [(n, v, a, b) for n, v, a, b in
                zip(ARM_JOINTS, q, self.lo, self.hi)
                if min(v - a, b - v) < 0.05]
        for n, v, a, b in near:
            print(f'  ! {n}={v:.4f} 贴近限位 [{a:.4f}, {b:.4f}]')
        if ep > 0.005:
            print(f'  !! 位置误差偏大（{ep * 1000:.1f} mm），该目标可能不可达')
        return q, ep, eo


# 工具 +Z 朝下：夹爪指爪沿 tool_link 的 +Z 伸出，从上方下抓时 +Z 指地面
R_TOOL_DOWN = np.array([[1.0, 0, 0],
                        [0, -1.0, 0],
                        [0, 0, -1.0]])

# 默认目标几何（M12/M13 用）。
# 观测点 (2.2, 3.0, yaw=-0.13) 时方块在 base_footlink 系 ≈ (0.595, 0.078, 0.225)；
# 竖直下抓时方块在 x>0.50 就够不到（joint3 顶到限位），所以抓取前机器人要沿朝向
# 再前进 grasp_forward=0.175m，此时方块落到 base 系 ≈ (0.42, 0.078)。
# 工具目标高度按 方块中心 + 0.07 取（指爪关节 origin 从 0.02 抬到 0.03 后的新约定）。
GRASP_X = 0.42
GRASP_Y = 0.078
CUBE_Z = 0.425
TOOL_OFFSET_Z = 0.07   # 方块中心 → 工具目标（指爪关节抬高到 0.03 后的约定）
PRESETS = [
    ('pregrasp   抓取前（方块上方 10cm）',
     [GRASP_X, GRASP_Y, CUBE_Z + TOOL_OFFSET_Z + 0.10]),
    ('grasp      抓取位（笛卡尔下压终点）',
     [GRASP_X, GRASP_Y, CUBE_Z + TOOL_OFFSET_Z]),
    ('lift       抬起（方块上方 20cm）',
     [GRASP_X, GRASP_Y, CUBE_Z + TOOL_OFFSET_Z + 0.20]),
    ('carry      搬运收起（高于激光面）', [0.30, 0.0, 0.80]),
    ('place_down 放置位（放置台面 0.40 + 0.095）', [0.42, 0.0, 0.495]),
]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--fk', nargs=6, type=float, metavar='Q',
                    help='对给定 6 个关节角做正解，打印 tool_link 位姿')
    ap.add_argument('--solve', nargs=3, type=float, metavar=('X', 'Y', 'Z'),
                    help='求解使 tool_link 到达该位置（工具 Z 朝下）的关节角')
    ap.add_argument('--restarts', type=int, default=80,
                    help='多起点次数，默认 80（越大越可能找到全局解）')
    args = ap.parse_args()

    kin = ArmKinematics()
    print(f'链: {" -> ".join(kin.chain)}')
    for n, a, b in zip(ARM_JOINTS, kin.lo, kin.hi):
        print(f'  {n}: [{a:+.3f}, {b:+.3f}]')

    if args.fk:
        p, R = kin.tool_pose(np.array(args.fk))
        print(f'\ntool_link 位置 (base_footlink): {np.round(p, 4)}')
        print(f'tool_link 旋转矩阵:\n{np.round(R, 4)}')
        return 0

    if args.solve:
        kin.report('自定义目标', args.solve, R_TOOL_DOWN, restarts=args.restarts)
        return 0

    for label, target in PRESETS:
        kin.report(label, target, R_TOOL_DOWN, restarts=args.restarts)
    return 0


if __name__ == '__main__':
    sys.exit(main())
