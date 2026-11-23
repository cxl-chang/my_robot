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
    s.wrist_align_max = 0.15
    s.wrist_verify_tol = 0.02
    s.object_z = 0.505
    s.grasp_x_mid = 0.435
    s.creep_deadband = 0.005
    s.grasp_correct_max = 0.25
    s.grasp_correct_steps = 3
    s.grasp_correct_count = 0
    s.stage = 'PICK_ALIGN'
    s.creep_dist = 0.0
    s.base_bias = (0.0, 0.0)
    s.wrist_fail = ''
    s._, s._now = None, lambda: None
    # 信念（底盘视觉+定位）默认偏 -46mm，正是实测那一轮的情形
    s.belief = (0.427, 0.049)
    s._object_in_base = (lambda self: (self.belief[0] + self.base_bias[0],
                                       self.belief[1] + self.base_bias[1],
                                       0.43)).__get__(s)
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
assert T._apply_wrist_align(s, (0.442, 0.012, 0.505)) == 'grasp'
show('对齐 12mm', s)
assert abs(s.pick_tool_x - 0.442) < 1e-9 and abs(s.pick_tool_y - 0.012) < 1e-9
assert math.hypot(*s.wrist_offset) < 1e-9, s.wrist_offset

# 2) 对齐：测到窗口外（x=0.47）→ 界幅到 0.45，残差 20mm 留给复核
s = make_self()
assert T._apply_wrist_align(s, (0.470, 0.0, 0.505)) == 'creep'
show('对齐 偏出窗口→蠕动', s)
assert abs(s.creep_dist - (0.470 - 0.435)) < 1e-9, s.creep_dist
assert s.grasp_correct_count == 1

# 3) 对齐：测到的点离原目标 90mm < 150mm → 采纳（信念可能比测量更差）
s = make_self()
assert T._apply_wrist_align(s, (0.4495, 0.0, 0.505)) == 'grasp'
show('对齐 偏20mm 采纳', s)
assert abs(s.pick_tool_x - 0.4495) < 1e-9, s.pick_tool_x
assert s.wrist_offset is not None and math.hypot(*s.wrist_offset) < 1e-9
# 信念被校正为 实测−信念 = 0.4505−0.427
assert abs(s.base_bias[0] - (0.4495 - 0.427)) < 1e-9, s.base_bias
# 第二轮再测：偏差不叠加（同一实测值 → bias 不变）
assert T._apply_wrist_align(s, (0.4495, 0.0, 0.505)) == 'grasp'
assert abs(s.base_bias[0] - (0.4495 - 0.427)) < 1e-9, s.base_bias

# 3b) 对齐：测到的点离原目标 200mm > 150mm → 疑似认错物体，保持原目标
s = make_self()
assert T._apply_wrist_align(s, (0.630, 0.0, 0.505)) == 'grasp'
show('对齐 粗差挡下', s)
assert abs(s.pick_tool_x - 0.430) < 1e-9 and s.wrist_offset is None

# 3c) 对齐：偏出窗口且微调次数已用完 → fail（并给出可操作原因）
s = make_self(grasp_correct_count=3)
assert T._apply_wrist_align(s, (0.500, 0.0, 0.505)) == 'fail'
show('对齐 微调耗尽', s)
print('  原因:', s.wrist_fail)
assert '超出可达窗口' in s.wrist_fail

# 4) 对齐：没测到 → 保持原目标，不拦吸附
s = make_self()
assert T._apply_wrist_align(s, None) == 'grasp'
show('对齐 无数据', s)
assert s.wrist_offset is None
assert T._wrist_gate_ok(s) is True

# 5) 复核：残差 8mm ≤ 20mm → 通过
s = make_self()
T._apply_wrist_verify(s, (0.438, 0.006, 0.505))
show('复核 通过', s)
assert T._wrist_gate_ok(s) is True and s.failed is None

# 5b) 对齐已界幅留下 22mm 残差、复核又没测到 → 仍然要拦（本轮实测漏过去的洞）
s = make_self()
s.wrist_offset = (0.022, 0.0)
T._apply_wrist_verify(s, None)
show('对齐残差22mm+复核无数据', s)
assert T._wrist_gate_ok(s) is False, '复核无数据时不能把对齐残差抹掉'
assert s.failed is not None and s.failed[1] == 'pick'
print('  失败原因:', s.failed[0])

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
s.wrist_enable = True
s.wrist_settle = 0.5
s.object_z = 0.505
s._do_attach = lambda: None
for name in ('_pick_ops_approach', '_pick_ops_grasp', '_pick_ops_tail'):
    setattr(s, name, types.MethodType(getattr(T, name), s))
labels = ([o['label'] for o in s._pick_ops_approach()]
          + [o['label'] for o in s._pick_ops_grasp()]
          + [o['label'] for o in s._pick_ops_tail()])
print('--- 序列:', ' → '.join(labels))
assert labels == ['预抓取', '张开夹爪', '测量前稳定', '下压抓取',
                  '复核前稳定', '合拢夹爪', '吸附物体', '吸附稳定',
                  '抬起', '收起搬运'], labels

print('ALL OK')
