"""tracker 原始位姿 → palm_link 6D 位姿（笛卡尔视图用）。

SDK 给的 `TrackerPose` 是 libsurvive 原始世界系（z 朝上，原点与朝向随每次基站几何标定而定）
下 tracker 本体坐标系的位姿：position 米，rotation 四元数 (w, x, y, z)。

模型里每只手都有一个固定挂在 palm_link 上的 tracker link（URDF `{hand}_tracker_joint` 的子 link
`{hand}_tracker_Link`，rpy=0、平移约 86 mm）。libsurvive 的 tracker 本体系与这个 link 的轴向不同，差一个常量旋转
`X_RAW_TO_LINK`（右乘）：

    R_link = R_raw · X
    R_palm = R_link · R_pt⁻¹              （R_pt / t_pt：tracker link 相对 palm_link 的固定位姿）
    p_palm = p_raw − R_palm · t_pt

X 与采集侧 tracker 节点的装配约定逐位一致（remap A = [[0,−1,0],[−1,0,0],[0,0,−1]]、
手部 frame_rotation C = Ry(+90°)，X = Aᵀ·C），把两手放进同一个 z 朝上的世界里，两手之间的
相对位姿就是真实的。世界系的原点与水平朝向没有物理意义——视图的相机跟随两掌中点。
"""

from __future__ import annotations

import math

X_RAW_TO_LINK = ((0.0, -1.0, 0.0),
                 (0.0, 0.0, -1.0),
                 (1.0, 0.0, 0.0))


def quat_to_mat(np, q):
    """四元数 (w, x, y, z) → 3×3 旋转矩阵（输入不必归一）。"""
    w, x, y, z = (float(v) for v in q)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def mat_to_quat(R) -> tuple[float, float, float, float]:
    """3×3 旋转矩阵 → 四元数 (w, x, y, z)，w ≥ 0。"""
    m = [[float(R[i][j]) for j in range(3)] for i in range(3)]
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = (0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s)
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        q = ((m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s)
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        q = ((m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s)
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        q = ((m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s)
    if q[0] < 0:
        q = tuple(-v for v in q)
    n = math.sqrt(sum(v * v for v in q))
    return tuple(v / n for v in q)


def tracker_body(mujoco, m, hand: str) -> int:
    """palm_link 下那个 tracker link 的 body id：直接挂在 `{hand}_palm_link` 下、名字以 `{hand}_tracker` 开头
    （`{hand}_tracker_Link`；URDF 里固定关节的名字进 MuJoCo 后就没了，只能按子 link 认）。"""
    palm = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"{hand}_palm_link")
    found = [b for b in range(m.nbody) if palm >= 0 and int(m.body_parentid[b]) == palm
             and (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or "").startswith(f"{hand}_tracker")]
    if len(found) != 1:
        raise ValueError(f"模型里没有（或不止一个）挂在 {hand}_palm_link 下的 {hand}_tracker link")
    return found[0]


class PalmFromTracker:
    """一只手：tracker 原始位姿 ↔ palm_link 世界位姿。tracker link 相对 palm_link 的位姿从编译后的模型里读，
    与 URDF 里的 `{hand}_tracker_joint` 是同一份数。"""

    def __init__(self, np, m, hand: str):
        import mujoco
        self.np = np
        self.hand = hand
        trk = tracker_body(mujoco, m, hand)
        self.t_pt = np.array(m.body_pos[trk], dtype=float)
        self.R_pt = quat_to_mat(np, m.body_quat[trk])
        self.X = np.array(X_RAW_TO_LINK, dtype=float)

    def palm_pose(self, position, rotation_wxyz):
        """tracker 原始位姿 → (p_palm, R_palm)，均为 numpy。"""
        np = self.np
        R_link = quat_to_mat(np, rotation_wxyz) @ self.X
        R_palm = R_link @ self.R_pt.T
        p_palm = np.asarray(position, dtype=float) - R_palm @ self.t_pt
        return p_palm, R_palm

    def tracker_raw(self, p_palm, R_palm):
        """palm_link 世界位姿 → tracker 原始位姿 (position, rotation wxyz)；palm_pose 的逆（演示源用）。"""
        np = self.np
        R_palm = np.asarray(R_palm, dtype=float)
        R_link = R_palm @ self.R_pt
        R_raw = R_link @ self.X.T
        p_raw = np.asarray(p_palm, dtype=float) + R_palm @ self.t_pt
        return tuple(float(v) for v in p_raw), mat_to_quat(R_raw)
