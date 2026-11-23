"""腕相机对齐/复核的纯逻辑测试（不启动 ROS）——用假 self 调真方法。"""
import math
import types
import pathlib
import sys

# 从仓库根（tools 的上一级）导入源码包，避免用到 install/ 里的旧副本
ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'my_robot_task_orchestrator'))
from my_robot_task_orchestrator.orchestrator import TaskOrchestrator as T  # noqa: E402


class Log:
    def __init__(self):
        self.lines = []

    def info(self, m, **kw):
        self.lines.append(('I', m))

    def warn(self, m, **kw):
        self.lines.append(('W', m))

    def error(self, m, **kw):
        self.lines.append(('E', m))


def make_self(**kw):
    s = types.SimpleNamespace()
    s.log = Log()
    s.get_logger = lambda: s.log
    s.pick_tool_x, s.pick_tool_y = 0.430, 0.000
    s.grasp_x_min, s.grasp_reach_x_max, s.grasp_reach_y = 0.42, 0.45, 0.25
    s.wrist_align_max = 0.06
    s.wrist_verify_tol = 0.02
    s.attach_gate = True
    s.attach_enabled = True
    s.wrist_class = 'red_cube'
    s.wrist_collect_time = 0.5
    s.wrist_offset = None
    s.wrist_last = None
    s.failed = None
    s.attach_enable_gate = types.MethodType(T.attach_enable_gate, s)

    def _fail(reason, category=None):
        s.failed = (reason, category)

    s._fail = _fail
    for k, v in kw.items():
        setattr(s, k, v)
    return s


def show(title, s):
    print(f'--- {title}: tool=({s.pick_tool_x:.3f},{s.pick_tool_y:.3f}) '
          f'offset={s.wrist_offset} failed={s.failed}')


# 1) 对齐：测到偏 12mm → 目标被改到实测位置
s = make_self()
T._apply_wrist_align(s, (0.442, 0.012, 0.505))
show('对齐 12mm', s)
assert abs(s.pick_tool_x - 0.442) < 1e-9 and abs(s.pick_tool_y - 0.012) < 1e-9
assert math.hypot(*s.wrist_offset) < 1e-9, s.wrist_offset

# 2) 对齐：测到窗口外（x=0.47）→ 界幅到 0.45，残差 20mm 留给复核
s = make_self()
T._apply_wrist_align(s, (0.470, 0.0, 0.505))
show('对齐 界幅', s)
assert abs(s.pick_tool_x - 0.45) < 1e-9
assert abs(s.wrist_offset[0] - 0.02) < 1e-9

# 3) 对齐：测到的点离原目标 90mm > align_max → 不信，保持原目标
s = make_self()
T._apply_wrist_align(s, (0.520, 0.0, 0.505))
show('对齐 不信任', s)
assert abs(s.pick_tool_x - 0.430) < 1e-9 and s.wrist_offset is None

# 4) 对齐：没测到 → 保持原目标，不拦吸附
s = make_self()
T._apply_wrist_align(s, None)
show('对齐 无数据', s)
assert s.wrist_offset is None
assert T._wrist_gate_ok(s) is True

# 5) 复核：残差 8mm ≤ 20mm → 通过
s = make_self()
T._apply_wrist_verify(s, (0.438, 0.006, 0.505))
show('复核 通过', s)
assert T._wrist_gate_ok(s) is True and s.failed is None

# 6) 复核：残差 33mm > 20mm → 不吸附、判 pick 失败（就是"顶在方块上"那种）
s = make_self()
T._apply_wrist_verify(s, (0.430, 0.033, 0.505))
show('复核 超差', s)
assert T._wrist_gate_ok(s) is False
assert s.failed is not None and s.failed[1] == 'pick', s.failed
print('  失败原因:', s.failed[0])

# 7) 复核：没测到（被夹爪挡住）→ 放行，避免主流程跑不下去
s = make_self()
T._apply_wrist_verify(s, None)
show('复核 无数据', s)
assert T._wrist_gate_ok(s) is True

# 8) 门控关闭 attach_gate=False → 超差也放行
s = make_self(attach_gate=False)
T._apply_wrist_verify(s, (0.430, 0.033, 0.505))
assert T._wrist_gate_ok(s) is True

# 9) 中值滤波：三帧含一帧跳点
assert T._median_xyz([(1, 2, 3), (1.01, 2.02, 3.03), (9, 9, 9)]) == (1.01, 2.02, 3.03)

# 10) 序列分段：approach/grasp/tail 三段拼起来仍是"预抓取→张开→下压→合拢→吸附→抬起→搬运"
s = make_self()
s._goal_pose = lambda *a: 'POSE'
s._goal_named = lambda g, t: f'NAMED:{g}:{t}'
s._op_goal = lambda label, goal: {'kind': 'goal', 'label': label, 'goal': goal}
s._op_fn = lambda label, fn: {'kind': 'fn', 'label': label, 'fn': fn}
s._op_wait = lambda label, sec: {'kind': 'wait', 'label': label, 'seconds': sec}
s.pregrasp_z, s.grasp_z, s.lift_z = 0.68, 0.58, 0.70
s.attach_settle = 0.3
s._do_attach = lambda: None
for name in ('_pick_ops_approach', '_pick_ops_grasp', '_pick_ops_tail'):
    setattr(s, name, types.MethodType(getattr(T, name), s))
labels = ([o['label'] for o in s._pick_ops_approach()]
          + [o['label'] for o in s._pick_ops_grasp()]
          + [o['label'] for o in s._pick_ops_tail()])
print('--- 序列:', ' → '.join(labels))
assert labels == ['预抓取', '张开夹爪', '下压抓取', '合拢夹爪',
                  '吸附物体', '吸附稳定', '抬起', '收起搬运'], labels

print('ALL OK')
