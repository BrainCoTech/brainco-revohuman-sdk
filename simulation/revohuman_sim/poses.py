"""Raw Tracker pose → palm_link 6D pose (for the Cartesian view).

The `TrackerPose` given by the SDK is the pose of the Tracker body frame in libsurvive's raw world frame (z
up; origin and heading are set by each base station geometry calibration): position in meters, rotation as a
quaternion (w, x, y, z).

In the model, each hand has a tracker link rigidly attached to palm_link (child link `{hand}_tracker_Link` of
URDF `{hand}_tracker_joint`, rpy=0, translation about 86 mm). libsurvive's Tracker body frame has different
axes from this link; they differ by a constant rotation `X_RAW_TO_LINK` (right-multiplied):

    R_link = R_raw · X
    R_palm = R_link · R_pt⁻¹              (R_pt / t_pt: fixed pose of the tracker link relative to palm_link)
    p_palm = p_raw − R_palm · t_pt

X is bit-for-bit identical to the mounting convention of the capture-side Tracker node (remap
A = [[0,−1,0],[−1,0,0],[0,0,−1]], hand frame_rotation C = Ry(+90°), X = Aᵀ·C). It puts both hands into the
same z-up world, so the relative pose between the two hands is real. The origin and horizontal heading of the
world frame have no physical meaning — the view camera follows the midpoint of the two palms.
"""

from __future__ import annotations

import math

X_RAW_TO_LINK = ((0.0, -1.0, 0.0),
                 (0.0, 0.0, -1.0),
                 (1.0, 0.0, 0.0))


def quat_to_mat(np, q):
    """Quaternion (w, x, y, z) → 3×3 rotation matrix (the input need not be normalized)."""
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
    """3×3 rotation matrix → quaternion (w, x, y, z), w ≥ 0."""
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
    """Body id of the tracker link under palm_link: attached directly to `{hand}_palm_link`, with a name
    starting with `{hand}_tracker` (`{hand}_tracker_Link`; URDF fixed-joint names are lost once in MuJoCo, so
    it can only be identified by the child link)."""
    palm = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"{hand}_palm_link")
    found = [b for b in range(m.nbody) if palm >= 0 and int(m.body_parentid[b]) == palm
             and (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or "").startswith(f"{hand}_tracker")]
    if len(found) != 1:
        raise ValueError(f"模型里没有（或不止一个）挂在 {hand}_palm_link 下的 {hand}_tracker link")
    return found[0]


class PalmFromTracker:
    """One hand: raw Tracker pose ↔ palm_link world pose. The pose of the tracker link relative to palm_link
    is read from the compiled model; it is the same numbers as `{hand}_tracker_joint` in the URDF."""

    def __init__(self, np, m, hand: str):
        import mujoco
        self.np = np
        self.hand = hand
        trk = tracker_body(mujoco, m, hand)
        self.t_pt = np.array(m.body_pos[trk], dtype=float)
        self.R_pt = quat_to_mat(np, m.body_quat[trk])
        self.X = np.array(X_RAW_TO_LINK, dtype=float)

    def palm_pose(self, position, rotation_wxyz):
        """Raw Tracker pose → (p_palm, R_palm), both numpy."""
        np = self.np
        R_link = quat_to_mat(np, rotation_wxyz) @ self.X
        R_palm = R_link @ self.R_pt.T
        p_palm = np.asarray(position, dtype=float) - R_palm @ self.t_pt
        return p_palm, R_palm

    def tracker_raw(self, p_palm, R_palm):
        """palm_link world pose → raw Tracker pose (position, rotation wxyz); inverse of palm_pose (used by
        the demo source)."""
        np = self.np
        R_palm = np.asarray(R_palm, dtype=float)
        R_link = R_palm @ self.R_pt
        R_raw = R_link @ self.X.T
        p_raw = np.asarray(p_palm, dtype=float) + R_palm @ self.t_pt
        return tuple(float(v) for v in p_raw), mat_to_quat(R_raw)
