"""Tactile layout contract (241 points) and the 3D positions "attached to the ring/cap surfaces of the finger
segments".

The layout contract comes from assets/tactile_layout_241.yaml: per finger, one 8-column grid + point number →
SDK index and the finger-segment link it sits on.
The 3D positions are not calibration values: for each finger segment's visual mesh, a ring (MCP/PIP, a C shape
open on the dorsal side) / cap (DIP, a closed dome) is fitted; rays are cast at the surface row by row, rows
and columns are unrolled by arc length, and the points are then lifted by lift_mm along the outward normal
(see PlacedLayout).
Fit results are cached in ~/.cache/revohuman/tactile_layout, keyed by a hash of "model meshes + layout table +
hand + algorithm version".

The bundled hand models have simplified shapes, with no ring/cap on the finger segments to fit; a placement
table is bundled alongside, assets/tactile_placement_241.json — fitted with the same algorithm on
finger-segment meshes that have the rings/caps, and stored as local coordinates in each finger-segment link
frame. As long as the model's finger-segment kinematics match it (fingerprint: in each finger-segment link
frame, the origin and orientation of the child links, and the segment's own joint axis), this table is used
directly with no fitting; only on a mismatch does it fall back to fitting.
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
FINGERPRINT_POS_MM = 0.5      # fingerprint tolerance: child link origin
FINGERPRINT_ANGLE_DEG = 1.0   # fingerprint tolerance: child link orientation, joint axis
FINGERS = ("thumb", "index", "middle", "ring", "little")


# ----------------------------------------------------------------------------- layout contract (no mujoco)

class TactilePoint(NamedTuple):
    idx: int        # SDK index 0..240 (= sdk_offset + point − 1)
    finger: str
    zone: str       # tip / mid / root
    point: int      # point number printed on the point map (1-based, per finger)
    x: int          # point-map column X0..X7
    y: int          # point-map row, from Y0 (fingertip)
    link: str       # URDF link suffix (without {hand}_)
    zone_row: int   # index of the row within its zone (0 = the zone's row closest to the fingertip)
    zone_rows: int  # number of rows in the zone


class TactileLayout:
    """Parsing of the layout contract YAML + contract self-check."""

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


# ----------------------------------------------------------------------------- attaching to ring/cap surfaces
PLACEMENT_VERSION = 2   # placement algorithm version: part of the layout cache key, +1 whenever the algorithm
                        # changes (the yaml's version is the contract version, counted separately)

FIT_DEFAULTS = {   # defaults for placement.fit (mm / degrees)
    "samples": 50000, "seed": 0, "slice_mm": 1.0, "min_slice_samples": 80, "inlier_mm": 0.8,
    "radius_range_mm": [4.0, 16.0], "min_coverage_deg": 150.0, "good_coverage_deg": 270.0,
    "radius_step_mm": 2.5, "center_step_mm": 3.5, "gap_mm": 3.0, "min_slices": 4,
    "max_tilt_deg": {"ring": 35.0, "cap": 30.0}, "outer_percentile": 95.0,
    # true extent of the wall along the axis: the RANSAC chain is only the starting point, then it is extended
    # at both ends by "wall-band sample density" (until that drops below extent_frac of the in-chain median)
    "extent_step_mm": 0.5, "extent_reach_mm": 4.0, "extent_frac": 0.5,
}
RING_DEFAULTS = {"max_wrap_deg": 100.0, "hit_margin_mm": 3.0}
# top row 2.5 mm off the axis (≈ column pitch): the 3 top-row points span the fingertip instead of circling
# the axis center
CAP_DEFAULTS = {"apex_rho_mm": 2.5, "max_wrap_deg": 100.0, "hit_margin_mm": 3.0}
LINK_KNOBS = {"row_pitch_mm": 2.5, "column_pitch_mm": 2.5, "lift_mm": 1.2, "phi_offset_deg": 0.0,
              "row_offset_mm": 0.0, "apex_rho_mm": None}   # placement.overrides.<link> can override each item


def link_mesh_mm(m, body_id: int):
    """Visual mesh of a body (group 1; if none, all mesh geoms): vertices (body frame, mm) and triangles."""
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
        mujoco.mju_quat2Mat(R, m.geom_quat[g])      # MuJoCo normalizes the mesh centroid/principal axes;
                                                    # geom_pos/quat hold the compensation
        Vs.append(v @ R.reshape(3, 3).T + m.geom_pos[g])
        Fs.append(f + base)
        base += len(v)
    return np.vstack(Vs) * 1000.0, np.vstack(Fs)


def link_vertices_mm(m, body_id: int):
    """Mesh vertices of a body (body frame, mm)."""
    return link_mesh_mm(m, body_id)[0]


def sample_surface(V, F, n: int, rng):
    """Samples n points uniformly by area on the triangle-mesh surface. Cylinder side walls are cut into
    full-height long triangles with vertices only on the edges; slicing the vertices directly would miss whole
    stretches of side wall, so face sampling is always used."""
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
    """Three-point circumcircle RANSAC + Kasa refinement; a radius out of range after refinement counts as a
    failure (otherwise it latches onto the r≈1.4 hinge pin)."""
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
    """Strings slices that are adjacent (t spacing ≤ gap_mm), with continuous radius (≤ radius_step_mm) and
    continuous center (perpendicular to the axis ≤ center_step_mm), into chains."""
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
    """Ring/cap fit. S: surface sample points (mm, link frame); axis0: initial axis (phalanx direction); kind:
    ring / cap.
    Slices along the axis every slice_mm and fits a circle to each slice with RANSAC, strings adjacent slices
    with continuous radius/center into chains, and takes the chain with the most inliers;
    each slice may jump between the inner and outer wall (Δr ≈ wall thickness 1–2 mm; when the outer wall is
    C-shaped the center also shifts by 2–3 mm), so the chain tolerances are widened accordingly;
    the axis and center are estimated only from slices with coverage angle ≥ good_coverage_deg (essentially
    full circles of the inner wall); a tilt beyond max_tilt_deg is treated as having chained into a hinge lug,
    and it falls back to axis0.
    Returns axis / c0 (a point on the axis) / r_fit (inner-wall radius; for the cap, taken at the base) /
    t_lo,t_hi (wall extent relative to c0) / tilt_deg / n_slices / coverage_deg."""
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
    if kind == "cap":   # dome radius shrinks with height; base radius from the chain's lower half
        lower = [s for s in good if s["t"] <= np.median(ts)] or good
        r_fit = float(np.median([s["r"] for s in lower]))
    else:
        r_fit = float(np.median([s["r"] for s in good]))
    tc = np.array([(s["c3"] - origin) @ axis for s in chain])
    h = float(p["slice_mm"])
    return {"axis": axis, "c0": origin, "r_fit": r_fit, "t_lo": float(tc.min() - h / 2), "t_hi": float(tc.max() + h / 2),
            "n_slices": len(chain), "coverage_deg": float(np.median([s["cov"] for s in chain])), "tilt_deg": float(tilt)}


def _wall_extent(t, rho, r_lo: float, r_hi: float, t_lo: float, t_hi: float, p: dict) -> tuple[float, float]:
    """True extent of the wall along the axis. The RANSAC chain [t_lo, t_hi] randomly comes out a bit short
    when individual slices fail the coverage threshold (the same segment on the left and right hands can
    differ by 2 mm), so here it goes by the sample density within the wall band (r_lo ≤ ρ ≤ r_hi) instead:
    extend bin by bin from the chain center toward both ends until the density drops below extent_frac of the
    in-chain median; extend outward by at most extent_reach_mm. Hinge lugs/the base occupy only a short arc of
    the circumference, their density does not reach half, and they are not swallowed."""
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
    """Intersects K directions D (K×3) from the same origin o with all triangles (Möller–Trumbore,
    vectorized); each ray takes the farthest hit with t within [t_lo, t_hi] (casting outward from the ring
    axis, farthest = outer wall; the upper limit blocks the hinge lugs); nan when there is no hit."""
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
    """Surface contour of one row: from a point o on the axis, sweep rays at elevation psi and azimuth
    φ∈[-max_wrap, max_wrap], take the farthest hits, and parameterize by arc length (arc length is 0 at φ=0,
    positive direction = +ey). at(s) gives the surface point and outward normal (ray direction) at arc length
    s."""

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
    """Local coordinates (m) of the 241 points in each finger-segment link frame + world coordinates computed
    every frame.

    Placement (placement.method = cuff_surface):
      Each finger segment's visual mesh is the shell that sits over the phalanx on the physical hardware —
      MCP/PIP are C-shaped rings open on the dorsal side, DIP is a closed cap (dome); the black tactile arrays
      are attached to the palmar surface of the ring/cap.
      ① face sampling + per-slice RANSAC circle fits strung into chains (fit_cuff) → the ring/cap axis a,
         center on the axis c0, inner-wall radius r_fit, wall t extent;
      ② palmar ex = the side of hinge axis × a that faces world +x (the palm direction at zero pose); the X0
         side of the columns is set from the ulnar direction per columns_x0_side;
      ③ each row: for the ring at t = row center, for the cap at the elevation ψ set by the meridian arc
         length (row_pitch per row down from the apex), sweep rays outward from the axis and take the farthest
         hits (outer wall) to get the row's contour; columns are spread along the contour on both sides by
         column_pitch of arc length, and the points are then lifted by lift_mm.
      Quantities not verified against the physical hardware: row/column pitch (2.5 mm is an estimate), X0 on
      the ulnar side, fingertip row 0 attached at the apex.
    Results are cached as JSON keyed by a hash of (model meshes + layout table + hand + algorithm version).
    Sources in order: local cache → bundled placement table (bundled=BUNDLED_PLACEMENT, used only when the
    fingerprint matches the model) → fitting.
    cache=False skips both and forces fitting (the finger-segment meshes must have the rings/caps)."""

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
        self.bundled_file = None      # path of the bundled placement table when it is used
        self.bundled_reject = None    # why the bundled placement table was not used
                                      # (None when there is no such table)
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

    # ---- cache
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

    # ---- bundled placement table
    @staticmethod
    def layout_sha1(layout: TactileLayout) -> str:
        import hashlib
        return hashlib.sha1(json.dumps(layout.data, sort_keys=True, ensure_ascii=False).encode()).hexdigest()

    @staticmethod
    def fingerprint(m, link_names) -> dict:
        """Kinematic fingerprint of each finger-segment link: the origin (mm) and orientation (quaternion) of
        every child link in the link frame, and the link's own joint axis.
        Placement points are local coordinates in the link frame; if these items match ⇒ the link frame is
        attached to the physical hardware the same way, and the local coordinates apply as-is."""
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
        """Compares two fingerprints item by item and returns a description of the first mismatch; returns
        None when everything is within tolerance."""
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
                # joint axes are compared with their direction: with a reversed axis, the same joint angle
                # turns the finger segment the other way
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
        """If the bundled placement table matches this model and this layout, loads it and returns None;
        otherwise returns the reason it cannot be used and changes nothing."""
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
        """Writes this result as one hand's section of the bundled placement table (used when generating the
        bundled placement table)."""
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

    # ---- computation
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
        mujoco.mj_forward(m, d0)   # zero pose: palm faces world +x; the ulnar side is read from
                                   # the y of the little- and index-finger MPR links

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
        x0_sign = 1.0 if x0_side == "ulnar" else -1.0   # X0 on the ulnar side:
                                                        # increasing column number = toward the radial side

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
        if children:                      # has a next segment → ring, axis points to the child joint
            kind, axis0 = "ring", m.body_pos[children[0]].astype(float)
        else:                             # last segment → cap, axis = previous phalanx direction
                                          # (parent joint → this joint) in this link frame
            kind, axis0 = "cap", Rq.T @ m.body_pos[b].astype(float)
        fp = dict(fitp, max_tilt_deg=float(fitp["max_tilt_deg"][kind]) if isinstance(fitp["max_tilt_deg"], dict) else float(fitp["max_tilt_deg"]))
        fit = fit_cuff(S, axis0, fp, kind)
        a, c0 = fit["axis"], fit["c0"]
        h = m.jnt_axis[m.body_jntadr[b]].astype(float)
        ey = h - (h @ a) * a
        ey /= np.linalg.norm(ey)
        ex = np.cross(ey, a)              # perpendicular to the hinge axis and the ring axis
                                          # = finger-pad direction (sign not yet determined)
        Rw = d0.xmat[b].reshape(3, 3)
        if ex @ (Rw.T @ palmar_w) < 0:
            ex, ey = -ex, -ey
        s_col = -x0_sign * float(np.sign(ey @ (Rw.T @ ulnar_w)))   # sign of φ as the column number increases
        rel = S - c0
        t = rel @ a
        rho = np.hypot(rel @ ex, rel @ ey)
        band = (t >= fit["t_lo"]) & (t <= fit["t_hi"]) & (np.abs(rho - fit["r_fit"]) < 2.5)
        r_outer = float(np.percentile(rho[band], float(fitp["outer_percentile"]))) if band.any() else fit["r_fit"] + 1.5
        chain_t = (fit["t_lo"], fit["t_hi"])
        ext_lo, ext_hi = _wall_extent(t, rho, fit["r_fit"] - 1.5, r_outer + 1.0, fit["t_lo"], fit["t_hi"], fitp)
        fit["t_lo"] = ext_lo
        if kind == "ring":                # the cap's upper end is a dome, not set by wall-band density
            fit["t_hi"] = ext_hi
        band = (t >= fit["t_lo"]) & (t <= fit["t_hi"]) & (np.abs(rho - fit["r_fit"]) < 2.5)
        ang = np.degrees(np.arctan2(rel[band] @ ey, rel[band] @ ex))
        occ = np.zeros(36, bool)
        occ[(np.floor((ang + 180) / 10).astype(int)) % 36] = True
        gap_c, gap_w = _largest_gap(occ)
        v0 = V[F[:, 0]]
        e1, e2 = V[F[:, 1]] - v0, V[F[:, 2]] - v0
        row_pitch, col_pitch, lift = float(knobs["row_pitch_mm"]), float(knobs["column_pitch_mm"]), float(knobs["lift_mm"])
        phi0 = -s_col * np.radians(float(knobs["phi_offset_deg"]))   # positive = array center
                                                                     # rotated toward the ulnar side
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
            # meridian: rays from the base center o at elevation ψ = 0..90°, take the outer-wall hits,
            # parameterize by arc length
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
            # the meridian arc length s increases from the cap base (ψ=0) toward the apex (ψ=90°) ⇒ use "+"
            # here, same as the ring branch; positive row_offset_mm = shift toward the fingertip (the two
            # branches must share the sign, otherwise the same override would move in opposite directions on
            # ring and cap).
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

    # ---- per frame
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
    """Occupancy table of 36 10° sectors → center angle (°, relative to ex) and width (°) of the largest run
    of empty sectors; (None, 0) when there is no empty run."""
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

