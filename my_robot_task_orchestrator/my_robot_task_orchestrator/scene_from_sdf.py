"""从 Gazebo SDF 世界文件里提取静态碰撞几何，供 MoveIt planning scene 使用。

为什么不在代码里手写台子坐标：那会把同一份几何抄两遍（SDF 一份、节点参数一份），
场景一改就得同步两处，忘了改的下场是"规划器以为台子在 A、真实在 B"。这里直接
读 SDF，让它当唯一真源。

只处理 <box> 几何（台面/支柱都是 box）；plane / cylinder / mesh 会被跳过并计数，
因为把整个地面平面塞进碰撞世界反而容易把规划搞死。
"""
import math
import xml.etree.ElementTree as ET

import numpy as np


def _rot(rpy):
    """SDF 的 rpy：先绕 X 滚、再绕 Y 俯、最后绕 Z 偏 → R = Rz·Ry·Rx。"""
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _transform(pose_text):
    """'x y z r p y' → 4x4 齐次变换；缺省当单位阵。"""
    T = np.eye(4)
    if not pose_text:
        return T
    v = [float(s) for s in pose_text.split()]
    if len(v) < 6:
        return T
    T[:3, :3] = _rot(v[3:6])
    T[:3, 3] = v[0:3]
    return T


def _quat_from_matrix(R):
    """旋转矩阵 → 四元数 (x, y, z, w)。"""
    t = np.trace(R)
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (x, y, z, w)


def load_static_boxes(sdf_path, models=None, shrink=0.0, logger=None):
    """读 SDF，返回静态模型里的所有 box 碰撞几何。

    参数
      sdf_path : 世界文件路径
      models   : 模型名白名单（None/空 = 全部静态模型）
      shrink   : 每个 box 的顶面压低量（m），用于"指尖贴台面不误报"
      logger   : 可选，接收 (level, text) 的日志回调

    返回 [(id, xyz, quat_xyzw, size), ...]
      xyz/quat/size 都在 SDF 的世界系里（= 本项目 map 系）。
    """
    def log(level, text):
        if logger is not None:
            logger(level, text)

    want = None
    if models:
        want = {m.strip() for m in models.split(',') if m.strip()}

    try:
        world = ET.parse(sdf_path).getroot().find('world')
    except (ET.ParseError, OSError) as e:
        log('warn', f'读取 SDF 失败（{sdf_path}）：{e}')
        return []
    if world is None:
        log('warn', f'SDF 里没有 <world>（{sdf_path}）')
        return []

    # 先按模型聚合几何，再决定 id：一个模型只有 1 个 box 时就用模型名
    # （好认、好删），多个 box 时用 "模型名/碰撞名" 区分（台面 + 支柱）。
    per_model = []
    skipped = 0
    for model in world.findall('model'):
        name = model.get('name') or ''
        if model.findtext('static', 'false').strip().lower() not in ('true', '1'):
            continue                      # 动态物体（方块）不在这里加
        if want is not None and name not in want:
            continue
        T_model = _transform(model.findtext('pose'))
        boxes = []
        for li, link in enumerate(model.findall('link')):
            T_link = T_model @ _transform(link.findtext('pose'))
            for ci, col in enumerate(link.findall('collision')):
                box = col.find('geometry/box')
                if box is None:
                    skipped += 1          # plane / cylinder / mesh：跳过
                    continue
                dims = [float(s) for s in (box.findtext('size') or '').split()]
                if len(dims) != 3:
                    skipped += 1
                    continue
                T_col = T_link @ _transform(col.findtext('pose'))
                size = list(dims)
                center = np.array(T_col[:3, 3], dtype=float)
                if shrink > 0.0:
                    size[2] = max(0.02, size[2] - shrink)
                    center[2] -= shrink / 2.0      # 顶面压低、底面不动
                boxes.append((col.get('name') or f'col{ci}',
                              tuple(float(v) for v in center),
                              _quat_from_matrix(T_col[:3, :3]),
                              tuple(size)))
        if boxes:
            per_model.append((name, boxes))

    out = []
    for name, boxes in per_model:
        multi = len(boxes) > 1
        for col_name, center, quat, size in boxes:
            obj_id = f'{name}/{col_name}' if multi else name
            out.append((obj_id, center, quat, size))
            log('debug', f'  {obj_id} @ {[round(v, 3) for v in center]} '
                         f'size={[round(v, 3) for v in size]}')
    if skipped:
        log('info', f'SDF 里有 {skipped} 个非 box 几何被跳过（plane/mesh 等）')
    return out
