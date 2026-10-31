"""触觉布局契约（241 点）与「贴到指节套环/套帽曲面上」的三维位置。

布局契约来自 assets/tactile_layout_241.yaml：每指一张 8 列网格 + 点号 → SDK 下标、所在指节 link。
三维位置不是标定值：对每个指节的 visual mesh 拟合套环（MCP/PIP，背侧开口的 C 形）/ 套帽（DIP，
闭合圆顶），逐行向曲面射线、按弧长把行列展开、再沿外法向抬 lift_mm 贴上去（见 PlacedLayout）。
拟合结果按「模型网格 + 布局表 + 手 + 算法版本」的哈希缓存在 ~/.cache/revohuman/tactile_layout。

随包的手部模型是简化外形，指节上没有套环/套帽可拟合；随包另带一份贴点表
assets/tactile_placement_241.json——在带套环/套帽的指节网格上按同一算法拟合、存成各指节 link 系里的
局部坐标。只要模型的指节运动学与它一致（指纹：每个指节 link 系里子 link 的原点与朝向、本关节轴），
就直接用这份表，不再拟合；不一致才回到拟合。
"""

from __future__ import annotations

import json
import pathlib
from typing import TYPE_CHECKING, NamedTuple

import yaml

from .paths import ASSETS_DIR, CACHE_ROOT

if TYPE_CHECKING:
    from .hand_model import HandModel

DEFAULT_LAYOUT = ASSETS_DIR / "tactile_layout_241.yaml"
BUNDLED_PLACEMENT = ASSETS_DIR / "tactile_placement_241.json"
BUNDLED_FORMAT = 1
FINGERPRINT_POS_MM = 0.5      # 指纹容差：子 link 原点
FINGERPRINT_ANGLE_DEG = 1.0   # 指纹容差：子 link 朝向、关节轴
FINGERS = ("thumb", "index", "middle", "ring", "little")


# ----------------------------------------------------------------------------- 布局契约（不依赖 mujoco）

class TactilePoint(NamedTuple):
    idx: int        # SDK 下标 0..240（= sdk_offset + point − 1）
    finger: str
    zone: str       # tip / mid / root
    point: int      # 点位图上印的点号（1 起，每指独立）
    x: int          # 点位图列 X0..X7
    y: int          # 点位图行 Y0（指尖）起
    link: str       # URDF link 后缀（不含 {hand}_）
    zone_row: int   # 该行在本区内的序号（0 = 本区最靠指尖的一行）
    zone_rows: int  # 本区行数


class TactileLayout:
    """布局契约 YAML 的解析 + 契约自检。"""

    def __init__(self, data: dict, path: pathlib.Path | None = None):
        self.data = data
        self.path = path
        self.point_count = int(data["point_count"])
        self.columns = int(data.get("columns", 8))
        self.placement = dict(data.get("placement", {}))
        self.value = dict(data.get("value", {}))
        self.points: list[TactilePoint] = []
        self.fingers: dict[str, dict] = {}
        problems: list[str] = []
        for finger, spec in data["fingers"].items():
            rows = [list(r) for r in spec["rows"]]
            zones = list(spec["zones"])
            off = int(spec["sdk_offset"])
            covered = set()
            for z in zones:
                lo, hi = int(z["rows"][0]), int(z["rows"][1])
                if lo > hi or hi >= len(rows):
                    problems.append(f"{finger}: zone {z['name']} rows {z['rows']} 越界")
                for yy in range(lo, hi + 1):
                    if yy in covered:
                        problems.append(f"{finger}: 行 Y{yy} 被两个区覆盖")
                    covered.add(yy)
            if covered != set(range(len(rows))):
                problems.append(f"{finger}: 有行不属于任何区：{sorted(set(range(len(rows))) - covered)}")
            pt = 1
            cells = set()
            zone_counts = {z["name"]: 0 for z in zones}
            for yy, row in enumerate(rows):
                if len(set(row)) != len(row) or any(not (0 <= int(x) < self.columns) for x in row):
                    problems.append(f"{finger}: 行 Y{yy} 列号非法 {row}")
                zone = next((z for z in zones if int(z["rows"][0]) <= yy <= int(z["rows"][1])), None)
                for x in row:
                    if (int(x), yy) in cells:
                        problems.append(f"{finger}: 格子 X{x}Y{yy} 重复")
                    cells.add((int(x), yy))
                    if zone is not None:
                        self.points.append(TactilePoint(
                            off + pt - 1, finger, str(zone["name"]), pt, int(x), yy, str(zone["link"]),
                            yy - int(zone["rows"][0]), int(zone["rows"][1]) - int(zone["rows"][0]) + 1))
                        zone_counts[zone["name"]] += 1
                    pt += 1
            n = pt - 1
            if n != int(spec["count"]):
                problems.append(f"{finger}: 网格数出 {n} 点，count 写的是 {spec['count']}")
            for z in zones:
                if "count" in z and zone_counts.get(z["name"]) != int(z["count"]):
                    problems.append(f"{finger}: 区 {z['name']} 数出 {zone_counts.get(z['name'])}，count 写的是 {z['count']}")
            self.fingers[finger] = {"sdk_offset": off, "count": n, "rows": rows, "zones": zones}
        self.points.sort(key=lambda p: p.idx)
        idxs = [p.idx for p in self.points]
        if idxs != list(range(self.point_count)):
            problems.append(f"SDK 下标不是 0..{self.point_count - 1} 连续：共 {len(idxs)} 个，"
                            f"缺 {sorted(set(range(self.point_count)) - set(idxs))[:8]}…")
        if problems:
            raise ValueError("布局表自检失败：\n  " + "\n  ".join(problems))

    @classmethod
    def load(cls, path: pathlib.Path = DEFAULT_LAYOUT) -> "TactileLayout":
        return cls(yaml.safe_load(pathlib.Path(path).read_text(encoding="utf-8")), pathlib.Path(path))

    def by_link(self) -> dict[str, list[TactilePoint]]:
        out: dict[str, list[TactilePoint]] = {}
        for p in self.points:
            out.setdefault(p.link, []).append(p)
        return out

    def finger_slice(self, finger: str) -> slice:
        f = self.fingers[finger]
        return slice(f["sdk_offset"], f["sdk_offset"] + f["count"])


# ----------------------------------------------------------------------------- 贴到指节网格的套环/套帽曲面上
PLACEMENT_VERSION = 2   # 贴法算法版本：参与布局缓存 key，改了算法就 +1（yaml 的 version 是契约版本，另算）

FIT_DEFAULTS = {   # placement.fit 的缺省值（mm / 度）
    "samples": 50000, "seed": 0, "slice_mm": 1.0, "min_slice_samples": 80, "inlier_mm": 0.8,
    "radius_range_mm": [4.0, 16.0], "min_coverage_deg": 150.0, "good_coverage_deg": 270.0,
    "radius_step_mm": 2.5, "center_step_mm": 3.5, "gap_mm": 3.0, "min_slices": 4,
    "max_tilt_deg": {"ring": 35.0, "cap": 30.0}, "outer_percentile": 95.0,
    # 壁面沿轴的真实范围：RANSAC 链只是起点，再按「壁带样本密度」向两端扩（掉到链内中位数的 extent_frac 以下为止）
    "extent_step_mm": 0.5, "extent_reach_mm": 4.0, "extent_frac": 0.5,
}
RING_DEFAULTS = {"max_wrap_deg": 100.0, "hit_margin_mm": 3.0}
CAP_DEFAULTS = {"apex_rho_mm": 2.5, "max_wrap_deg": 100.0, "hit_margin_mm": 3.0}   # 顶行离轴 2.5 mm（≈列距）：顶行 3 点横跨指尖，而不是围着轴心转
LINK_KNOBS = {"row_pitch_mm": 2.5, "column_pitch_mm": 2.5, "lift_mm": 1.2, "phi_offset_deg": 0.0,
              "row_offset_mm": 0.0, "apex_rho_mm": None}   # placement.overrides.<link> 可逐项覆盖


def link_mesh_mm(m, body_id: int):
    """某 body 的 visual mesh（group 1；没有就用全部 mesh geom）：顶点（body 系，mm）与三角面。"""
    import mujoco
    import numpy as np
    picked = [g for g in range(m.body_geomadr[body_id], m.body_geomadr[body_id] + m.body_geomnum[body_id])
              if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH]
    vis = [g for g in picked if m.geom_group[g] == 1]
    picked = vis or picked
    if not picked:
        raise ValueError(f"body {m.body(body_id).name} 没有 mesh 几何")
    Vs, Fs, base = [], [], 0
    for g in picked:
        mid = m.geom_dataid[g]
        v = m.mesh_vert[m.mesh_vertadr[mid]: m.mesh_vertadr[mid] + m.mesh_vertnum[mid]].astype(float)
        f = m.mesh_face[m.mesh_faceadr[mid]: m.mesh_faceadr[mid] + m.mesh_facenum[mid]].astype(int)
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, m.geom_quat[g])      # MuJoCo 会把 mesh 重心/主轴归一，geom_pos/quat 里是补偿量
        Vs.append(v @ R.reshape(3, 3).T + m.geom_pos[g])
        Fs.append(f + base)
        base += len(v)
    return np.vstack(Vs) * 1000.0, np.vstack(Fs)


def link_vertices_mm(m, body_id: int):
    """某 body 的 mesh 顶点（body 系，mm）。"""
    return link_mesh_mm(m, body_id)[0]


def sample_surface(V, F, n: int, rng):
    """按面积均匀在三角网格表面采样 n 个点。圆柱侧壁被切成通高的长三角形、顶点只在棱边上，
    直接拿顶点切片会漏掉整段侧壁，所以一律用面采样。"""
    import numpy as np
    v0, v1, v2 = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    area = 0.5 * np.linalg.norm(np.cross(v1 - v0, v2 - v0), axis=1)
    idx = rng.choice(len(F), size=n, p=area / area.sum())
    s = np.sqrt(rng.random(n))
    r2 = rng.random(n)
    return (1 - s)[:, None] * v0[idx] + (s * (1 - r2))[:, None] * v1[idx] + (s * r2)[:, None] * v2[idx]


def _kasa(P):
    import numpy as np
    A = np.column_stack([2 * P[:, 0], 2 * P[:, 1], np.ones(len(P))])
    b = (P ** 2).sum(1)
    x, *_ = np.linalg.lstsq(A, b, rcond=None)
    c = x[:2]
    return c, float(np.sqrt(max(x[2] + c @ c, 0.0)))


def _circle_ransac(P, inlier_mm: float, r_range, rng, iters: int = 150):
    """三点外接圆 RANSAC + Kasa 精修；精修后半径出界即判失败（否则会咬住 r≈1.4 的铰链销）。"""
    import numpy as np
    n = len(P)
    if n < 6:
        return None
    S = P if n <= 600 else P[rng.choice(n, 600, replace=False)]
    idx = rng.integers(0, len(S), size=(iters, 3))
    A, B, C = S[idx[:, 0]], S[idx[:, 1]], S[idx[:, 2]]
    d = 2 * (A[:, 0] * (B[:, 1] - C[:, 1]) + B[:, 0] * (C[:, 1] - A[:, 1]) + C[:, 0] * (A[:, 1] - B[:, 1]))
    ok = np.abs(d) > 1e-6
    d = np.where(ok, d, 1.0)
    a2, b2, c2 = (A ** 2).sum(1), (B ** 2).sum(1), (C ** 2).sum(1)
    ux = (a2 * (B[:, 1] - C[:, 1]) + b2 * (C[:, 1] - A[:, 1]) + c2 * (A[:, 1] - B[:, 1])) / d
    uy = (a2 * (C[:, 0] - B[:, 0]) + b2 * (A[:, 0] - C[:, 0]) + c2 * (B[:, 0] - A[:, 0])) / d
    r = np.hypot(A[:, 0] - ux, A[:, 1] - uy)
    ok &= (r >= r_range[0]) & (r <= r_range[1])
    if not ok.any():
        return None
    dist = np.abs(np.hypot(S[None, :, 0] - ux[:, None], S[None, :, 1] - uy[:, None]) - r[:, None])
    cnt = (dist < inlier_mm).sum(1)
    cnt[~ok] = -1
    best = int(np.argmax(cnt))
    if cnt[best] < 6:
        return None
    c, rr = np.array([ux[best], uy[best]]), float(r[best])
    inl = np.abs(np.hypot(P[:, 0] - c[0], P[:, 1] - c[1]) - rr) < inlier_mm
    for _ in range(3):
        if inl.sum() < 6:
            return None
        c, rr = _kasa(P[inl])
        inl = np.abs(np.hypot(P[:, 0] - c[0], P[:, 1] - c[1]) - rr) < inlier_mm
    if not (r_range[0] <= rr <= r_range[1]) or inl.sum() < 6:
        return None
    return c, rr, inl


def _basis(a):
    import numpy as np
    h = np.array([1.0, 0, 0]) if abs(a[0]) < 0.9 else np.array([0, 1.0, 0])
    u = np.cross(a, h)
    u /= np.linalg.norm(u)
    return u, np.cross(a, u)


def _coverage_deg(P, c, bins: int = 24) -> float:
    import numpy as np
    ang = np.arctan2(P[:, 1] - c[1], P[:, 0] - c[0])
    occ = np.zeros(bins, bool)
    occ[((ang + np.pi) / (2 * np.pi) * bins).astype(int) % bins] = True
    return occ.sum() * 360.0 / bins


def _slices(S, axis, origin, p: dict, rng):
    import numpy as np
    u, v = _basis(axis)
    rel = S - origin
    t, x, y = rel @ axis, rel @ u, rel @ v
    out = []
    h = float(p["slice_mm"])
    for t0 in np.arange(np.floor(t.min()), np.ceil(t.max()), h):
        sel = (t >= t0) & (t < t0 + h)
        if sel.sum() < p["min_slice_samples"]:
            continue
        P2 = np.column_stack([x[sel], y[sel]])
        res = _circle_ransac(P2, float(p["inlier_mm"]), p["radius_range_mm"], rng)
        if res is None:
            continue
        c, r, inl = res
        if inl.sum() < p["min_slice_samples"]:
            continue
        cov = _coverage_deg(P2[inl], c)
        if cov < p["min_coverage_deg"]:
            continue
        out.append({"t": t0 + h / 2, "r": r, "c3": origin + (t0 + h / 2) * axis + c[0] * u + c[1] * v,
                    "n": int(inl.sum()), "cov": cov})
    return out


def _chains(sl, p: dict, axis):
    """相邻（t 间隔 ≤ gap_mm）、半径连续（≤ radius_step_mm）、圆心连续（垂直于轴 ≤ center_step_mm）的片串成链。"""
    import numpy as np
    chains, cur = [], []
    for s in sorted(sl, key=lambda s: s["t"]):
        if cur:
            q = cur[-1]
            dc = s["c3"] - q["c3"]
            dc -= (dc @ axis) * axis
            if (s["t"] - q["t"] > p["gap_mm"] or abs(s["r"] - q["r"]) > p["radius_step_mm"]
                    or np.linalg.norm(dc) > p["center_step_mm"]):
                chains.append(cur)
                cur = []
        cur.append(s)
    if cur:
        chains.append(cur)
    return [c for c in chains if len(c) >= p["min_slices"]]


def fit_cuff(S, axis0, p: dict, kind: str) -> dict:
    """套环/套帽拟合。S：表面采样点（mm，link 系）；axis0：初始轴（指骨方向）；kind：ring / cap。
    沿轴每 slice_mm 切一片 RANSAC 拟圆，把相邻且半径/圆心连续的片串成链，取内点最多的链；
    每片会在内壁与外壁之间跳（Δr≈壁厚 1–2 mm，外壁是 C 形时圆心还偏 2–3 mm），链的容差按此放宽；
    轴与中心只用覆盖角 ≥ good_coverage_deg 的片（基本是内壁整圈）来估，倾角超过 max_tilt_deg 视为串到铰链耳、退回 axis0。
    返回 axis / c0（轴上一点）/ r_fit（内壁半径；套帽取底部）/ t_lo,t_hi（相对 c0 的壁面范围）/ tilt_deg / n_slices / coverage_deg。"""
    import numpy as np
    rng = np.random.default_rng(int(p.get("seed", 0)))
    axis0 = np.asarray(axis0, float) / np.linalg.norm(axis0)
    axis, origin, tilt, chain = axis0, np.zeros(3), 0.0, None
    max_tilt = float(p["max_tilt_deg"])
    for _ in range(3):
        chains = _chains(_slices(S, axis, origin, p, rng), p, axis)
        if not chains:
            raise RuntimeError("没有拟合出套环/套帽的圆截面")
        chain = max(chains, key=lambda c: sum(s["n"] for s in c))
        good = [s for s in chain if s["cov"] >= p["good_coverage_deg"]] or chain
        C = np.array([s["c3"] for s in good])
        w = np.array([s["n"] for s in good], float)
        origin = np.average(C, axis=0, weights=w)
        if len(good) >= 3 and np.abs(C[:, None, :] - C[None, :, :]).max() >= 3.0:
            _, _, vt = np.linalg.svd((C - origin) * np.sqrt(w)[:, None], full_matrices=False)
            d = vt[0] if vt[0] @ axis0 > 0 else -vt[0]
            cand = float(np.degrees(np.arccos(np.clip(d @ axis0, -1, 1))))
            axis, tilt = (d, cand) if cand <= max_tilt else (axis0, 0.0)
    ts = np.array([s["t"] for s in chain])
    good = [s for s in chain if s["cov"] >= p["good_coverage_deg"]] or chain
    if kind == "cap":   # 圆顶半径随高度递减，底部半径取链的下半段
        lower = [s for s in good if s["t"] <= np.median(ts)] or good
        r_fit = float(np.median([s["r"] for s in lower]))
    else:
        r_fit = float(np.median([s["r"] for s in good]))
    tc = np.array([(s["c3"] - origin) @ axis for s in chain])
    h = float(p["slice_mm"])
    return {"axis": axis, "c0": origin, "r_fit": r_fit, "t_lo": float(tc.min() - h / 2), "t_hi": float(tc.max() + h / 2),
            "n_slices": len(chain), "coverage_deg": float(np.median([s["cov"] for s in chain])), "tilt_deg": float(tilt)}


def _wall_extent(t, rho, r_lo: float, r_hi: float, t_lo: float, t_hi: float, p: dict) -> tuple[float, float]:
    """壁面沿轴的真实范围。RANSAC 链 [t_lo, t_hi] 会因为个别切片过不了覆盖门槛而随机短一截（左右手同一节能差 2 mm），
    这里改按壁带（r_lo ≤ ρ ≤ r_hi）里的样本密度：从链中心向两端逐 bin 扩，直到密度掉到链内中位数的 extent_frac 以下；
    最多向外扩 extent_reach_mm。铰链耳/底座只占一小段圆周，密度到不了一半，不会被吃进来。"""
    import numpy as np
    step, reach, frac = float(p["extent_step_mm"]), float(p["extent_reach_mm"]), float(p["extent_frac"])
    band = (rho >= r_lo) & (rho <= r_hi)
    edges = np.arange(t_lo - reach, t_hi + reach + step * 0.5, step)
    cnt, _ = np.histogram(t[band], bins=edges)
    centers = 0.5 * (edges[:-1] + edges[1:])
    inside = (centers >= t_lo) & (centers <= t_hi)
    ref = float(np.median(cnt[inside])) if inside.any() else 0.0
    if ref <= 0:
        return t_lo, t_hi
    ok = cnt >= frac * ref
    ic = int(np.argmin(np.abs(centers - 0.5 * (t_lo + t_hi))))
    if not ok[ic]:
        return t_lo, t_hi
    lo = ic
    while lo > 0 and ok[lo - 1]:
        lo -= 1
    hi = ic
    while hi + 1 < len(ok) and ok[hi + 1]:
        hi += 1
    return float(edges[lo]), float(edges[hi + 1])


def ray_hits_far(v0, e1, e2, o, D, t_lo: float | None = None, t_hi: float | None = None, chunk: int = 24):
    """同一起点 o、K 个方向 D（K×3）对全部三角形求交（Möller–Trumbore，向量化）；
    每条射线取 t 在 [t_lo, t_hi] 内最远的交点（从套环轴心往外射，最远 = 外壁；上限挡住铰链耳），没有交点给 nan。"""
    import numpy as np
    T = o - v0                       # M×3
    Q = np.cross(T, e1)              # M×3
    tq = (e2 * Q).sum(-1)            # M
    out = np.full(len(D), np.nan)
    for k0 in range(0, len(D), chunk):
        Dk = D[k0:k0 + chunk]
        Pk = np.cross(Dk[:, None, :], e2[None, :, :])        # k×M×3
        det = (e1[None] * Pk).sum(-1)
        ok = np.abs(det) > 1e-9
        inv = np.where(ok, 1.0 / np.where(ok, det, 1.0), 0.0)
        u = (T[None] * Pk).sum(-1) * inv
        v = (Dk @ Q.T) * inv
        t = tq[None] * inv
        hit = ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 1e-6)
        if t_lo is not None:
            hit &= t >= t_lo
        if t_hi is not None:
            hit &= t <= t_hi
        best = np.where(hit, t, -np.inf).max(1)
        out[k0:k0 + chunk] = np.where(np.isfinite(best), best, np.nan)
    return out


class _RowContour:
    """一行的曲面轮廓：从轴上一点 o 以仰角 psi、方位 φ∈[-max_wrap, max_wrap] 扫射线取最远交点，
    按弧长参数化（φ=0 处弧长为 0，正方向 = +ey）。at(s) 给弧长 s 处的曲面点与外法向（射线方向）。"""

    def __init__(self, v0, e1, e2, o, ex, ey, a, psi: float, max_wrap_deg: float, t_range, fallback_t, step_deg: float = 2.0):
        import numpy as np
        self.np = np
        self.phis = np.radians(np.arange(-max_wrap_deg, max_wrap_deg + 1e-6, step_deg))
        D = np.cos(psi) * (np.cos(self.phis)[:, None] * ex + np.sin(self.phis)[:, None] * ey) + np.sin(psi) * a
        th = ray_hits_far(v0, e1, e2, o, D, *t_range)
        miss = ~np.isfinite(th)
        self.misses = int(miss.sum())
        th = np.where(miss, fallback_t, th)
        self.H = o + th[:, None] * D
        self.D = D
        s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(self.H, axis=0), axis=1))])
        self.s = s - np.interp(0.0, self.phis, s)

    def at(self, s: float, lift: float):
        np = self.np
        s = float(np.clip(s, self.s[0], self.s[-1]))
        h = np.array([np.interp(s, self.s, self.H[:, k]) for k in range(3)])
        d = np.array([np.interp(s, self.s, self.D[:, k]) for k in range(3)])
        d /= np.linalg.norm(d)
        return h + lift * d, float(np.degrees(np.interp(s, self.s, self.phis)))


class PlacedLayout:
    """241 点在各指节 link 系里的局部坐标（m）+ 每帧算世界坐标。

    贴法（placement.method = cuff_surface）：
      每个指节的 visual mesh 就是实物上套在指骨外面的那个壳——MCP/PIP 是背侧开口的 C 形套环，
      DIP 是闭合的套帽（圆顶）；黑色触觉阵列贴在套环/套帽的掌侧曲面上。
      ① 面采样 + 逐片 RANSAC 拟圆串链（fit_cuff）→ 套环/套帽的轴 a、轴上中心 c0、内壁半径 r_fit、壁面 t 范围；
      ② 掌向 ex = 铰链轴 × a 中朝世界 +x（零姿态掌心朝向）的那一侧；列的 X0 侧按 columns_x0_side 由尺侧方向定；
      ③ 每一行：套环在 t = 行中心处、套帽在子午线弧长（顶点往下每行 row_pitch）定的仰角 ψ 处，
         从轴上往外扫射线取最远交点（外壁）得到这一行的轮廓，列按弧长 column_pitch 沿轮廓两侧展开，点再抬 lift_mm。
      未与实物核验的量：行距/列距（2.5 mm 是估计值）、X0 在尺侧、指尖第 0 行贴在顶点。
    结果按 (模型网格 + 布局表 + 手 + 算法版本) 的哈希缓存成 JSON。
    来源依次是：本机缓存 → 随包贴点表（bundled=BUNDLED_PLACEMENT，指纹与模型一致才用）→ 拟合。
    cache=False 两者都跳过、强制拟合（指节网格须带套环/套帽）。"""

    def __init__(self, layout: TactileLayout, fk: "HandModel", hand: str, cache: bool | pathlib.Path = True,
                 bundled: pathlib.Path | None = BUNDLED_PLACEMENT):
        import numpy as np
        self.np = np
        self.layout = layout
        self.hand = hand
        pl = layout.placement
        method = str(pl.get("method", "cuff_surface"))
        if method != "cuff_surface":
            raise ValueError(f"不认识的 placement.method={method!r}（只支持 cuff_surface）")
        self.cache_file = None
        self.from_cache = False
        self.bundled_file = None      # 用上了随包贴点表时是它的路径
        self.bundled_reject = None    # 随包贴点表没用上的原因（没有这份表时为 None）
        if cache:
            cache_dir = cache if isinstance(cache, pathlib.Path) else CACHE_ROOT / "tactile_layout"
            self.cache_file = cache_dir / f"{hand}_{self._cache_key(fk, layout, hand)}.json"
            if self._load_cache(fk):
                self.from_cache = True
            elif bundled is not None and bundled.is_file():
                self.bundled_reject = self._load_bundled(fk, bundled)
                if self.bundled_reject is None:
                    self.bundled_file = bundled
        if not self.from_cache and self.bundled_file is None:
            try:
                self._compute(fk)
            except RuntimeError as e:
                if self.bundled_reject:
                    raise RuntimeError(f"{e}（随包贴点表也用不上：{self.bundled_reject}）") from e
                raise
            if self.cache_file is not None:
                self._save_cache(fk)
        self.bodies = sorted(set(self.body_id.tolist()))
        self.world = np.zeros((layout.point_count, 3))

    # ---- 缓存
    @staticmethod
    def _cache_key(fk, layout: TactileLayout, hand: str) -> str:
        import hashlib
        m = fk.m
        h = hashlib.sha1()
        for arr in (m.mesh_vert, m.mesh_face, m.geom_pos, m.geom_quat, m.geom_group, m.body_pos, m.body_quat, m.jnt_axis):
            h.update(memoryview(arr).tobytes())
        h.update(json.dumps(layout.data, sort_keys=True, ensure_ascii=False).encode())
        h.update(f"|{hand}|v{PLACEMENT_VERSION}".encode())
        return h.hexdigest()[:16]

    def _load_cache(self, fk) -> bool:
        import mujoco
        np = self.np
        try:
            doc = json.loads(self.cache_file.read_text(encoding="utf-8"))
            ids = [mujoco.mj_name2id(fk.m, mujoco.mjtObj.mjOBJ_BODY, n) for n in doc["body"]]
            if min(ids) < 0 or len(ids) != self.layout.point_count:
                return False
            self.local = np.asarray(doc["local_mm"], float) / 1000.0
            self.body_id = np.asarray(ids, int)
            self.link_info = doc["links"]
            self.ulnar_sign = float(doc["ulnar_sign_y"])
            return True
        except (OSError, ValueError, KeyError, TypeError):
            return False

    # ---- 随包贴点表
    @staticmethod
    def layout_sha1(layout: TactileLayout) -> str:
        import hashlib
        return hashlib.sha1(json.dumps(layout.data, sort_keys=True, ensure_ascii=False).encode()).hexdigest()

    @staticmethod
    def fingerprint(m, link_names) -> dict:
        """各指节 link 的运动学指纹：link 系里每个子 link 的原点（mm）与朝向（四元数）、本关节轴。
        贴点是 link 系里的局部坐标，这几样一致 ⇒ link 系挂在实物上的方式一致，局部坐标照用。"""
        import mujoco
        out = {}
        for name in link_names:
            b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)
            if b < 0:
                raise ValueError(f"模型里没有 {name}")
            kids = {}
            for c in range(m.nbody):
                if c != b and int(m.body_parentid[c]) == b:
                    kids[mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, c)] = {
                        "pos_mm": [round(float(v) * 1000.0, 4) for v in m.body_pos[c]],
                        "quat": [round(float(v), 6) for v in m.body_quat[c]]}
            axis = None
            if int(m.body_jntnum[b]) > 0:
                axis = [round(float(v), 6) for v in m.jnt_axis[int(m.body_jntadr[b])]]
            out[name] = {"children": kids, "axis": axis}
        return out

    @staticmethod
    def fingerprint_mismatch(want: dict, have: dict) -> str | None:
        """两份指纹逐项比，返回第一处不符的说明；都在容差内返回 None。"""
        import numpy as np

        def angle_deg(u, v):
            u, v = np.asarray(u, float), np.asarray(v, float)
            return float(np.degrees(np.arccos(np.clip(abs(u @ v) / (np.linalg.norm(u) * np.linalg.norm(v)), -1.0, 1.0))))

        for name, w in want.items():
            h = have.get(name)
            if h is None:
                return f"模型里没有 {name}"
            if set(w["children"]) != set(h["children"]):
                return f"{name} 的子 link 是 {sorted(h['children'])}，贴点表按 {sorted(w['children'])}"
            for kid, wc in w["children"].items():
                hc = h["children"][kid]
                d = float(np.linalg.norm(np.subtract(wc["pos_mm"], hc["pos_mm"])))
                if d > FINGERPRINT_POS_MM:
                    return f"{kid} 在 {name} 系里的原点差 {d:.2f} mm"
                a = 2.0 * angle_deg(wc["quat"], hc["quat"])
                if a > FINGERPRINT_ANGLE_DEG:
                    return f"{kid} 在 {name} 系里的朝向差 {a:.1f}°"
            if (w["axis"] is None) != (h["axis"] is None):
                return f"{name} 的关节数与贴点表不符"
            if w["axis"] is not None:
                # 关节轴按有向比：轴反号时同一个关节角会把指节往另一边转
                u, v = np.asarray(w["axis"], float), np.asarray(h["axis"], float)
                a = float(np.degrees(np.arccos(np.clip(u @ v / (np.linalg.norm(u) * np.linalg.norm(v)), -1.0, 1.0))))
                if a > FINGERPRINT_ANGLE_DEG:
                    return f"{name} 的关节轴差 {a:.1f}°"
        return None

    def _ulnar_sign_of(self, m) -> float:
        import mujoco
        d0 = mujoco.MjData(m)
        mujoco.mj_forward(m, d0)
        ys = []
        for f in ("little", "index"):
            b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"{self.hand}_{f}_MPR_Link")
            if b < 0:
                raise ValueError(f"URDF 里找不到 {self.hand}_{f}_MPR_Link")
            ys.append(d0.xpos[b][1])
        return 1.0 if ys[0] > ys[1] else -1.0

    def _load_bundled(self, fk, path: pathlib.Path) -> str | None:
        """随包贴点表与本模型、本布局一致就装进来并返回 None；否则返回不能用的原因，什么也不改。"""
        import mujoco
        np = self.np
        try:
            doc = json.loads(path.read_text(encoding="utf-8"))
            if int(doc.get("format", 0)) != BUNDLED_FORMAT or int(doc.get("placement_version", 0)) != PLACEMENT_VERSION:
                return f"{path.name} 的格式/贴法版本与本程序不符"
            if doc.get("layout_sha1") != self.layout_sha1(self.layout):
                return f"{path.name} 不是按当前布局表 {self.layout.path.name} 生成的"
            entry = (doc.get("hands") or {}).get(self.hand)
            if entry is None:
                return f"{path.name} 里没有 {self.hand} 手"
            ids = [mujoco.mj_name2id(fk.m, mujoco.mjtObj.mjOBJ_BODY, n) for n in entry["body"]]
            if len(ids) != self.layout.point_count or min(ids) < 0:
                return "贴点表里的指节 link 在模型里找不到"
            why = self.fingerprint_mismatch(entry["fingerprint"], self.fingerprint(fk.m, entry["fingerprint"]))
            if why:
                return f"模型指节运动学与贴点表不符：{why}"
            ulnar = float(entry["ulnar_sign_y"])
            if ulnar != self._ulnar_sign_of(fk.m):
                return "模型的尺侧方向与贴点表不符"
            local = np.asarray(entry["local_mm"], float) / 1000.0
            if local.shape != (self.layout.point_count, 3):
                return "贴点表的点数不对"
            links = entry["links"]
        except (OSError, ValueError, KeyError, TypeError) as e:
            return f"{path.name} 读不了：{e}"
        self.local, self.body_id, self.link_info, self.ulnar_sign = local, np.asarray(ids, int), links, ulnar
        return None

    def bundle_entry(self, fk) -> dict:
        """把本次结果写成随包贴点表里一只手的那一段（生成随包贴点表时用）。"""
        return {"ulnar_sign_y": self.ulnar_sign,
                "body": [fk.m.body(int(b)).name for b in self.body_id],
                "local_mm": self.np.round(self.local * 1000.0, 4).tolist(),
                "links": self.link_info,
                "fingerprint": self.fingerprint(fk.m, list(self.link_info))}

    def _save_cache(self, fk) -> None:
        try:
            self.cache_file.parent.mkdir(parents=True, exist_ok=True)
            doc = {"placement_version": PLACEMENT_VERSION, "hand": self.hand, "ulnar_sign_y": self.ulnar_sign,
                   "body": [fk.m.body(int(b)).name for b in self.body_id],
                   "local_mm": self.np.round(self.local * 1000.0, 4).tolist(), "links": self.link_info}
            self.cache_file.write_text(json.dumps(doc, ensure_ascii=False), encoding="utf-8")
        except OSError:
            self.cache_file = None

    # ---- 计算
    def _compute(self, fk) -> None:
        import mujoco
        np = self.np
        m = fk.m
        layout = self.layout
        pl = layout.placement
        fitp = dict(FIT_DEFAULTS, **dict(pl.get("fit", {})))
        ringp = dict(RING_DEFAULTS, **dict(pl.get("ring", {})))
        capp = dict(CAP_DEFAULTS, **dict(pl.get("cap", {})))
        overrides = dict(pl.get("overrides", {}) or {})
        base_knobs = {k: pl.get(k, v) for k, v in LINK_KNOBS.items()}
        x0_side = str(pl.get("columns_x0_side", "ulnar"))
        d0 = mujoco.MjData(m)
        mujoco.mj_forward(m, d0)   # 零姿态：掌心朝世界 +x，尺侧看小指与食指 MPR 的 y

        def bid(name):
            b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)
            if b < 0:
                raise ValueError(f"URDF 里找不到 {name}")
            return b
        y_l = d0.xpos[bid(f"{self.hand}_little_MPR_Link")][1]
        y_i = d0.xpos[bid(f"{self.hand}_index_MPR_Link")][1]
        self.ulnar_sign = 1.0 if y_l > y_i else -1.0
        ulnar_w = np.array([0.0, self.ulnar_sign, 0.0])
        palmar_w = np.array([1.0, 0.0, 0.0])
        x0_sign = 1.0 if x0_side == "ulnar" else -1.0   # X0 在尺侧：列号增大 = 往桡侧走

        n = layout.point_count
        self.body_id = np.zeros(n, dtype=int)
        self.local = np.zeros((n, 3))
        self.link_info: dict[str, dict] = {}
        rng = np.random.default_rng(int(fitp.get("seed", 0)))
        for link, pts in layout.by_link().items():
            name = f"{self.hand}_{link}"
            b = bid(name)
            knobs = dict(base_knobs, **dict(overrides.get(link, {}) or {}))
            try:
                info = self._place_link(m, d0, b, pts, fitp, ringp, capp, knobs, rng, ulnar_w, palmar_w, x0_sign)
            except RuntimeError as e:
                raise RuntimeError(f"{name}: {e}") from e
            self.link_info[name] = info

    def _place_link(self, m, d0, b, pts, fitp, ringp, capp, knobs, rng, ulnar_w, palmar_w, x0_sign) -> dict:
        import mujoco
        np = self.np
        V, F = link_mesh_mm(m, b)
        S = sample_surface(V, F, int(fitp["samples"]), rng)
        children = [c for c in range(m.nbody) if m.body_parentid[c] == b]
        Rq = np.zeros(9)
        mujoco.mju_quat2Mat(Rq, m.body_quat[b])
        Rq = Rq.reshape(3, 3)
        if children:                      # 有下一节 → 套环，轴指向子关节
            kind, axis0 = "ring", m.body_pos[children[0]].astype(float)
        else:                             # 末节 → 套帽，轴 = 上一节指骨方向（父关节 → 本关节）在本 link 系里
            kind, axis0 = "cap", Rq.T @ m.body_pos[b].astype(float)
        fp = dict(fitp, max_tilt_deg=float(fitp["max_tilt_deg"][kind]) if isinstance(fitp["max_tilt_deg"], dict) else float(fitp["max_tilt_deg"]))
        fit = fit_cuff(S, axis0, fp, kind)
        a, c0 = fit["axis"], fit["c0"]
        h = m.jnt_axis[m.body_jntadr[b]].astype(float)
        ey = h - (h @ a) * a
        ey /= np.linalg.norm(ey)
        ex = np.cross(ey, a)              # 垂直于铰链轴与套环轴 = 指腹方向（正负待定）
        Rw = d0.xmat[b].reshape(3, 3)
        if ex @ (Rw.T @ palmar_w) < 0:
            ex, ey = -ex, -ey
        s_col = -x0_sign * float(np.sign(ey @ (Rw.T @ ulnar_w)))   # 列号增大时 φ 的符号
        rel = S - c0
        t = rel @ a
        rho = np.hypot(rel @ ex, rel @ ey)
        band = (t >= fit["t_lo"]) & (t <= fit["t_hi"]) & (np.abs(rho - fit["r_fit"]) < 2.5)
        r_outer = float(np.percentile(rho[band], float(fitp["outer_percentile"]))) if band.any() else fit["r_fit"] + 1.5
        chain_t = (fit["t_lo"], fit["t_hi"])
        ext_lo, ext_hi = _wall_extent(t, rho, fit["r_fit"] - 1.5, r_outer + 1.0, fit["t_lo"], fit["t_hi"], fitp)
        fit["t_lo"] = ext_lo
        if kind == "ring":                # 套帽的上端是圆顶，不按壁带密度定
            fit["t_hi"] = ext_hi
        band = (t >= fit["t_lo"]) & (t <= fit["t_hi"]) & (np.abs(rho - fit["r_fit"]) < 2.5)
        ang = np.degrees(np.arctan2(rel[band] @ ey, rel[band] @ ex))
        occ = np.zeros(36, bool)
        occ[(np.floor((ang + 180) / 10).astype(int)) % 36] = True
        gap_c, gap_w = _largest_gap(occ)
        v0 = V[F[:, 0]]
        e1, e2 = V[F[:, 1]] - v0, V[F[:, 2]] - v0
        row_pitch, col_pitch, lift = float(knobs["row_pitch_mm"]), float(knobs["column_pitch_mm"]), float(knobs["lift_mm"])
        phi0 = -s_col * np.radians(float(knobs["phi_offset_deg"]))   # 正 = 阵列中心往尺侧转
        zones = sorted({(p.zone, p.zone_rows) for p in pts})
        cols = {z: (min(p.x for p in pts if p.zone == z), max(p.x for p in pts if p.zone == z)) for z, _ in zones}
        info = {"kind": kind, "points": len(pts), "zones": [z for z, _ in zones], "tilt_deg": round(fit["tilt_deg"], 1),
                "r_fit_mm": round(fit["r_fit"], 2), "r_outer_mm": round(r_outer, 2),
                "t_lo_mm": round(fit["t_lo"], 1), "t_hi_mm": round(fit["t_hi"], 1), "n_slices": fit["n_slices"],
                "chain_t_mm": [round(chain_t[0], 1), round(chain_t[1], 1)],
                "coverage_deg": round(fit["coverage_deg"]), "gap_center_deg": gap_c, "gap_width_deg": gap_w,
                "c0_mm": np.round(c0, 2).tolist(), "axis": np.round(a, 4).tolist(), "ex": np.round(ex, 4).tolist(),
                "ey": np.round(ey, 4).tolist(), "col_sign": s_col, "row_pitch_mm": row_pitch, "column_pitch_mm": col_pitch,
                "lift_mm": lift, "phi_offset_deg": float(knobs["phi_offset_deg"]), "row_offset_mm": float(knobs["row_offset_mm"]),
                "faces": int(len(F)), "misses": 0}
        misses = 0
        if kind == "ring":
            wrap = float(ringp["max_wrap_deg"])
            t_range = (fit["r_fit"] - 1.5, r_outer + float(ringp["hit_margin_mm"]))
            t_c = 0.5 * (fit["t_lo"] + fit["t_hi"]) + float(knobs["row_offset_mm"])
            t_rows = []
            for zone, nrows in zones:
                x_c = 0.5 * sum(cols[zone])
                for i in range(nrows):
                    t_i = t_c + ((nrows - 1) / 2.0 - i) * row_pitch
                    t_rows.append(round(t_i, 2))
                    contour = _RowContour(v0, e1, e2, c0 + t_i * a, ex, ey, a, 0.0, wrap, t_range, r_outer)
                    misses += contour.misses
                    s_phi0 = float(np.interp(phi0, contour.phis, contour.s))
                    for p in pts:
                        if p.zone != zone or p.zone_row != i:
                            continue
                        pos, _ = contour.at(s_phi0 + s_col * (p.x - x_c) * col_pitch, lift)
                        self.local[p.idx] = pos / 1000.0
                        self.body_id[p.idx] = b
            info["t_rows_mm"] = t_rows
            info["wrap_deg"] = round(float(np.degrees((cols[zones[0][0]][1] - cols[zones[0][0]][0]) * col_pitch / (r_outer + lift))))
        else:
            wrap = float(capp["max_wrap_deg"])
            apex_rho = float(knobs["apex_rho_mm"]) if knobs.get("apex_rho_mm") is not None else float(capp["apex_rho_mm"])
            t_apex = float(t.max())
            o = c0 + fit["t_lo"] * a
            t_range = (0.5 * fit["r_fit"], (t_apex - fit["t_lo"]) + r_outer + float(capp["hit_margin_mm"]))
            # 子午线：从底面中心 o 沿仰角 ψ = 0..90° 射线，取外壁交点，按弧长参数化
            psis = np.radians(np.arange(0.0, 90.01, 1.0))
            e_phi0 = np.cos(phi0) * ex + np.sin(phi0) * ey
            D = np.cos(psis)[:, None] * e_phi0 + np.sin(psis)[:, None] * a
            th = ray_hits_far(v0, e1, e2, o, D, *t_range)
            good = np.isfinite(th)
            misses += int((~good).sum())
            if good.sum() < 10:
                raise RuntimeError("套帽子午线射线几乎全部落空")
            H = o + th[good][:, None] * D[good]
            ps_g = psis[good]
            s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(H, axis=0), axis=1))])
            rho_m = np.hypot((H - c0) @ ex, (H - c0) @ ey)
            k0 = int(np.argmax(rho_m < apex_rho)) if (rho_m < apex_rho).any() else len(rho_m) - 1
            # 子午线弧长 s 从套帽底(ψ=0)往顶点(ψ=90°)增大 ⇒ 这里跟套环那支一样用「+」，
            # row_offset_mm 正 = 往指尖挪（两支必须同号，否则同一个 override 在套环/套帽上方向相反）。
            s0 = float(s[k0]) + float(knobs["row_offset_mm"])
            psi_rows = []
            zone, nrows = zones[0]
            x_c = 0.5 * sum(cols[zone])
            for i in range(nrows):
                s_i = max(s0 - i * row_pitch, 0.0)
                psi_i = float(np.interp(s_i, s, ps_g))
                rho_i = max(float(np.interp(s_i, s, rho_m)), apex_rho)
                psi_rows.append(round(float(np.degrees(psi_i)), 1))
                contour = _RowContour(v0, e1, e2, o, ex, ey, a, psi_i, wrap, t_range, rho_i / max(np.cos(psi_i), 1e-3))
                misses += contour.misses
                s_phi0 = float(np.interp(phi0, contour.phis, contour.s))
                for p in pts:
                    if p.zone_row != i:
                        continue
                    pos, _ = contour.at(s_phi0 + s_col * (p.x - x_c) * col_pitch, lift)
                    self.local[p.idx] = pos / 1000.0
                    self.body_id[p.idx] = b
            info.update({"t_apex_mm": round(t_apex, 1), "meridian_mm": round(float(s[-1]), 1), "s0_mm": round(s0, 1),
                         "apex_rho_mm": apex_rho, "psi_rows_deg": psi_rows})
        info["misses"] = misses
        return info

    # ---- 每帧
    def update_world(self, fk: "HandModel"):
        for b in self.bodies:
            sel = self.body_id == b
            R = fk.d.xmat[b].reshape(3, 3)
            self.world[sel] = fk.d.xpos[b] + self.local[sel] @ R.T
        return self.world

    def describe(self) -> str:
        if self.from_cache:
            src = f"缓存 {self.cache_file}"
        elif self.bundled_file is not None:
            src = f"随包贴点表 {self.bundled_file.name}"
        else:
            src = "本次拟合"
        lines = [f"布局 {self.layout.path}（{self.layout.point_count} 点，贴法 cuff_surface v{PLACEMENT_VERSION}，{src}），"
                 f"尺侧 = {'+' if self.ulnar_sign > 0 else '-'}y，X0 在{self.layout.placement.get('columns_x0_side', 'ulnar')}侧"]
        for name, i in self.link_info.items():
            head = (f"   {name:<24} {i['points']:3d} 点 {'/'.join(i['zones']):<8} {'套环' if i['kind'] == 'ring' else '套帽'} "
                    f"r {i['r_fit_mm']:4.1f}/{i['r_outer_mm']:4.1f}  t {i['t_lo_mm']:5.1f}..{i['t_hi_mm']:5.1f}  倾 {i['tilt_deg']:4.1f}°  "
                    f"行/列距 {i['row_pitch_mm']:.1f}/{i['column_pitch_mm']:.1f}")
            if i["kind"] == "ring":
                tail = f"  行 t {i['t_rows_mm']}  绕 ±{i['wrap_deg'] // 2}°"
            else:
                tail = f"  子午线 {i['meridian_mm']}mm 顶行 s0 {i['s0_mm']}  ψ {i['psi_rows_deg']}"
            if i.get("misses"):
                tail += f"  射线落空 {i['misses']}"
            if i.get("phi_offset_deg") or i.get("row_offset_mm"):
                tail += f"  偏移 φ{i['phi_offset_deg']:+.0f}° 行{i['row_offset_mm']:+.1f}mm"
            lines.append(head + tail)
        return "\n".join(lines)


def _largest_gap(occ):
    """36 个 10° 扇区的占用表 → 最大空扇区段的中心角（°，相对 ex）与宽度（°）；没有空段给 (None, 0)。"""
    import numpy as np
    if occ.all():
        return None, 0
    e = np.concatenate([~occ, ~occ])
    best, run = (0, 0), 0
    for i, val in enumerate(e):
        run = run + 1 if val else 0
        if run > best[0]:
            best = (run, i)
    w = min(best[0], len(occ))
    end = best[1]
    start = end - best[0] + 1
    center = ((start + end) / 2 % len(occ)) * (360 / len(occ)) - 180 + 180 / len(occ)
    return round(float(center)), round(float(w * 360 / len(occ)))

