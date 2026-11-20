"""Tactile heatmap: 241 real taxels → dense interpolated heat blobs + contact rings on finger-segment
surfaces.

Heat blobs are computed once from the MjData of the fixed-base view (`draw_heat`). The same hand in the
Cartesian view differs from it only by a rigid transform (same joint angles, only the palm_link pose differs),
so `copy_geoms_rigid` moves the whole span over without recomputing.

The `args` parameter reads only the attributes below; `HeatParams` and an argparse Namespace both satisfy it:
    contact  heat_max  heat_bulge_mm  heat_ring  heat_ring_seg  heat_ring_pad_mm  heat_cluster_mm
"""

from __future__ import annotations

import ctypes
import pathlib
import struct
import sys
import zlib
from dataclasses import dataclass

from .tactile_layout import FINGERS, PlacedLayout, TactileLayout


@dataclass
class HeatParams:
    contact: float = 20.0            # contact threshold (same unit as the tactile readings)
    heat_pitch_mm: float = 0.9       # fine-grid spacing
    heat_sigma_mm: float = 2.6       # interpolation Gaussian σ
    heat_reach_mm: float = 4.0       # the fine grid is laid only within this distance of the real taxels
    heat_bulge_mm: float = 1.6       # height the heat blob bulges along the outward normal at full scale
    heat_max: int = 6000             # max fine points drawn per hand per frame
    heat_cluster_mm: float = 9.0     # contact-ring clustering distance
    heat_ring: bool = True           # draw contact rings
    heat_ring_pad_mm: float = 1.5    # how far the contact ring extends beyond the heat region
    heat_ring_seg: int = 44          # number of contact-ring segments


class ScaleTracker:
    """Color-scale range: fixed when --scale>0; else peak hold + slow per-frame decay, never below floor."""

    def __init__(self, fixed: float, floor: float, decay: float):
        self.fixed = float(fixed)
        self.floor = float(floor)
        self.decay = float(decay)
        self.auto = self.floor

    def step(self, peak: float) -> float:
        if self.fixed > 0:
            return self.fixed
        self.auto = max(self.floor, peak, self.auto * self.decay)
        return self.auto

    def nudge(self, factor: float, current: float) -> None:
        self.fixed = max(1.0, current * factor)

    @property
    def mode(self) -> str:
        return "fixed" if self.fixed > 0 else "auto"


def draw_axes(mujoco, scn) -> None:
    for axis, rgba in (((0.05, 0, 0), (1, 0, 0, 1)), ((0, 0.05, 0), (0, 1, 0, 1)), ((0, 0, 0.05), (0, 0.4, 1, 1))):
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_ARROW, [0, 0, 0], [0, 0, 0], [1, 0, 0, 0, 1, 0, 0, 0, 1], rgba)
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_ARROW, 0.003, [0, 0, 0], list(axis))
        scn.ngeom += 1


# ----------------------------------------------------------------------------- contact heat region

HEAT_STOPS = ((0.00, 0.10, 0.28, 0.78), (0.25, 0.10, 0.76, 0.86), (0.50, 0.38, 0.90, 0.36),
              (0.75, 1.00, 0.78, 0.16), (1.00, 1.00, 0.26, 0.14))


def heat_rgb(u: float):
    """Heat color scale: blue → cyan → green → orange → red, pulled slightly toward white-hot near full scale.
    Outputs color only, no opacity — alpha is computed by draw_heat from "how far from the contact threshold",
    the color from "what fraction of the color-scale range". The two are kept separate: weak contact should
    still be clearly visible (alpha has a floor), but its color should be cool."""
    u = 0.0 if u < 0.0 else (1.0 if u > 1.0 else u)
    r = g = b = 0.0
    for k in range(len(HEAT_STOPS) - 1):
        u0, r0, g0, b0 = HEAT_STOPS[k]
        u1, r1, g1, b1 = HEAT_STOPS[k + 1]
        if u <= u1 or k == len(HEAT_STOPS) - 2:
            t = 0.0 if u1 <= u0 else (u - u0) / (u1 - u0)
            t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
            r, g, b = r0 + (r1 - r0) * t, g0 + (g1 - g0) * t, b0 + (b1 - b0) * t
            break
    if u > 0.88:                                  # add white-hot at the core so the strongest spots stand out
        t = 0.7 * (u - 0.88) / 0.12
        r, g, b = r + (1.00 - r) * t, g + (0.95 - g) * t, b + (0.85 - b) * t
    return (r, g, b)


def heat_rgb_vec(np, u):
    """Vectorized heat_rgb: computes n points at once; results are bit-identical to calling heat_rgb per point
    (checked against it by a unit test)."""
    u = np.clip(np.asarray(u, float), 0.0, 1.0)
    st = np.asarray(HEAT_STOPS, float)
    rgb = np.stack([np.interp(u, st[:, 0], st[:, c]) for c in (1, 2, 3)], axis=1)
    t = np.clip(0.7 * (u - 0.88) / 0.12, 0.0, None)[:, None]
    white = np.asarray([1.00, 0.95, 0.85])
    return np.where(t > 0, rgb + (white - rgb) * t, rgb)


class GeomSlab:
    """Maps the contiguous C memory of mjvScene.geoms to a numpy structured array, so the thousands of
    heat-blob spheres are written in one go.

    Why: one heat frame pushes 1–2 thousand spheres into user_scn; one mjv_initGeom + three attribute
    assignments per sphere comes to ~7000 pybind calls per frame. Every call spins on the GIL, which the
    stream-receive and render threads are also contending for — measured on the same frame: 12 ms without
    threads, 25 ms with two receive threads (p95 40 ms). The bulk write brings this under 0.2 ms.

    How it avoids writing to the wrong place: offsets are **not hard-coded**. At construction, the slot stride
    and the field offsets are derived from the addresses of the official wrapper's numpy views
    (size/pos/mat/rgba); type/emission/specular/shininess are found by writing sentinel values and searching
    for them. Then 3 random spheres go through both paths, "one mjv_initGeom each" and "bulk", and are
    compared byte by byte; on mismatch ok=False and the caller falls back to writing one by one.
    Template: first let mjv_initGeom write one sphere into slot 0 (dataid/objtype/category/texid… all take its
    defaults), copy the whole record to n slots, then change only the four fields size/pos/rgba/emission."""

    FIELDS = ("type", "size", "pos", "mat", "rgba", "emission", "specular", "shininess")

    def __init__(self, mujoco, np, scn, verbose: bool = False):
        self.np, self.mujoco, self.ok, self.why = np, mujoco, False, ""
        self._scn_id = None
        self.view = None
        self.tmpl = None
        try:
            self._map(scn)
            self._selfcheck(scn)
            self.ok = True
        except Exception as e:                                  # any step fails → disable entirely,
                                                                # no risky memory writes
            self.ok, self.why = False, f"{type(e).__name__}: {e}"
        if verbose:
            print(f"[slab] bulk geom write {'ON' if self.ok else 'OFF: ' + self.why}", file=sys.stderr)

    # ---- layout detection
    def _map(self, scn) -> None:
        np, mujoco = self.np, self.mujoco
        g0, g1 = scn.geoms[0], scn.geoms[1]
        base = g0.size.ctypes.data
        stride = g1.size.ctypes.data - base
        assert 100 < stride < 1024 and stride % 4 == 0, f"stride {stride}"
        off = {"size": 0, "pos": g0.pos.ctypes.data - base, "mat": g0.mat.ctypes.data - base,
               "rgba": g0.rgba.ctypes.data - base}
        assert g0.size.dtype == np.float32 and g0.pos.dtype == np.float32

        def probe(setter, val, dt):
            setter(val)
            raw = bytes((ctypes.c_ubyte * stride).from_address(base - 64))
            hit = np.flatnonzero(np.frombuffer(raw, np.uint8).view(dt) == dt.type(val))
            assert len(hit) == 1, f"probe {val}: {hit}"
            return int(hit[0]) * 4 - 64
        off["emission"] = probe(lambda v: setattr(g0, "emission", v), 12345.5, np.dtype("<f4"))
        off["specular"] = probe(lambda v: setattr(g0, "specular", v), 2345.5, np.dtype("<f4"))
        off["shininess"] = probe(lambda v: setattr(g0, "shininess", v), 345.5, np.dtype("<f4"))
        off["type"] = probe(lambda v: setattr(g0, "type", v), 77, np.dtype("<i4"))
        assert off["type"] < 0, "type 应是结构体第一个字段（在 size 之前）"
        rec0 = base + off["type"]
        dt = np.dtype({"names": list(self.FIELDS),
                       "formats": ["<i4", ("<f4", (3,)), ("<f4", (3,)), ("<f4", (9,)), ("<f4", (4,)),
                                   "<f4", "<f4", "<f4"],
                       "offsets": [off[k] - off["type"] for k in self.FIELDS], "itemsize": stride})
        n = int(scn.maxgeom)
        raw = (ctypes.c_ubyte * (stride * n)).from_address(rec0)
        self.view = np.frombuffer(raw, dtype=dt)               # writable view straight onto the C memory
        # Whole-record view of the same memory: copy the whole template record first, then change fields.
        # Assigning field by field misses unmapped bytes such as label — slot memory of a freshly created
        # scene is not zeroed, and leftover bytes get drawn as the geom label, which is the garbled text on
        # top of the heat blobs.
        self.rawview = np.frombuffer(raw, dtype=np.dtype((np.void, stride)))
        self.stride, self.rec0, self._scn_id = stride, rec0, id(scn)
        # Template slot: write a "zero sphere" via the official path, copying along the other default
        # fields that mjv_initGeom fills in
        # mjv_initGeom only writes label[0]=0 and leaves the other bytes as they are, so zero the whole record
        # first to make the template deterministic
        self.rawview[0] = np.void(bytes(stride))
        g = scn.geoms[0]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, [0, 0, 0], [0, 0, 0],
                            [1, 0, 0, 0, 1, 0, 0, 0, 1], [0, 0, 0, 0])
        g.specular, g.shininess, g.emission = 0.0, 0.0, 0.0
        self.tmpl = self.view[0].copy()
        self.tmpl_raw = self.rawview[0].copy()

    def _bytes(self, start: int, n: int) -> bytes:
        return bytes((ctypes.c_ubyte * (self.stride * n)).from_address(self.rec0 + self.stride * start))

    def _selfcheck(self, scn) -> None:
        np, mujoco = self.np, self.mujoco
        n = 3
        rng = np.random.default_rng(7)
        pos = rng.normal(size=(n, 3)); rad = rng.uniform(0.001, 0.003, n)
        rgba = rng.uniform(0, 1, (n, 4)); emis = rng.uniform(0, 1, n)
        ident = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        self.rawview[:n] = np.void(bytes(self.stride))         # start from all zeros, same as the template
        for k in range(n):                                     # official path
            g = scn.geoms[k]
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, [float(rad[k]), 0, 0], pos[k], ident,
                                [float(x) for x in rgba[k]])
            g.specular, g.shininess, g.emission = 0.0, 0.0, float(emis[k])
        ref = self._bytes(0, n)
        self.rawview[:n] = np.void(b"\xa5" * self.stride)     # fill whole records with garbage:
                                                              # any unwritten byte fails the comparison
        self._write(0, pos, rad, rgba, emis)                   # bulk path
        got = self._bytes(0, n)
        assert got == ref, "批量写与 mjv_initGeom 结果不一致"
        assert abs(float(scn.geoms[n - 1].emission) - float(emis[n - 1])) < 1e-6
        scn.ngeom = 0

    def _write(self, start: int, pos, rad, rgba, emis) -> None:
        self.rawview[start:start + len(rad)] = self.tmpl_raw
        sl = self.view[start:start + len(rad)]
        sl["size"] = self.np.asarray(rad, self.np.float32)[:, None]    # sphere: mjv_initGeom fills
                                                                       # all three sizes with the radius
        sl["pos"] = pos
        sl["rgba"] = rgba
        sl["emission"] = emis

    # ---- public
    def spheres(self, scn, pos, rad, rgba, emis) -> bool:
        """Writes len(rad) spheres starting at scn.ngeom in one go and advances ngeom; returns False when scn
        is a different object / unusable (the caller then writes one by one)."""
        if not self.ok or id(scn) is not self._scn_id and id(scn) != self._scn_id:
            return False
        n = len(rad)
        if n == 0:
            return True
        if scn.ngeom + n > scn.maxgeom:
            return False
        self._write(int(scn.ngeom), pos, rad, rgba, emis)
        scn.ngeom = int(scn.ngeom) + n
        return True


class _Surface:
    """Parametric surface of one finger-segment cuff, rebuilt from the fit values already stored in
    PlacedLayout.link_info.

    Parameter domain (row, ang):
      ring: row = axial coordinate mm,  ang = azimuth around the axis rad,  val = radius ρ there mm;
      cap:  row = elevation ψ rad,      ang = azimuth rad,                  val = sphere radius r there mm.
    row_scale = mm per unit of row; radius(row) = mm per radian of ang on that row. With these two rulers,
    distances in the parameter domain are measured as "arc length on the surface in mm" — laying out fine
    points, interpolation and drawing contact rings all use the same metric, so a ring really wraps around the
    finger segment instead of being a flat circle hanging in mid-air."""

    def __init__(self, np, kind: str, origin, a, ex, ey, P, gsig: float):
        self.np, self.kind, self.gsig = np, str(kind), float(gsig)
        self.o = np.asarray(origin, float)
        self.a, self.ex, self.ey = (np.asarray(v, float) for v in (a, ex, ey))
        rel = P - self.o
        if self.kind == "ring":
            self.row = rel @ self.a
            w = rel - self.row[:, None] * self.a
            self.val = np.linalg.norm(w, axis=1)
            self.ang = np.arctan2(w @ self.ey, w @ self.ex)
            self.rc = float(self.val.mean())                  # ring radius: azimuth → arc-length ruler
            self.row_scale, self.row_hi = 1.0, None
            self.tdir = w / np.maximum(self.val, 1e-9)[:, None]
        else:
            self.val = np.linalg.norm(rel, axis=1)
            d = rel / np.maximum(self.val, 1e-9)[:, None]
            self.row = np.arcsin(np.clip(d @ self.a, -1.0, 1.0))
            w = d - (d @ self.a)[:, None] * self.a
            self.ang = np.arctan2(w @ self.ey, w @ self.ex)
            self.rc = float(self.val.mean())                  # cap sphere radius
            self.row_scale, self.row_hi = self.rc, float(np.pi / 2)   # ψ stops at the apex:
                                                                      # past 90° the parameterization folds
            self.tdir = d

    # ---- parameter domain ↔ 3D
    def radius(self, row):
        np = self.np
        row = np.asarray(row, float)
        if self.kind == "ring":
            return np.full(row.shape, self.rc)
        return np.maximum(self.rc * np.cos(row), 1e-3)

    def _dir(self, row, ang):
        np = self.np
        e = np.cos(ang)[..., None] * self.ex + np.sin(ang)[..., None] * self.ey
        return e if self.kind == "ring" else np.cos(row)[..., None] * e + np.sin(row)[..., None] * self.a

    def value_at(self, row, ang):
        """Gaussian-weights this link's real taxels in the parameter domain → the radial value there, so fine
        points land on the same surface layer as the taxels (lift_mm included), neither sinking into the mesh
        nor floating."""
        np = self.np
        dphi = (ang[:, None] - self.ang[None, :] + np.pi) % (2 * np.pi) - np.pi
        d2 = ((row[:, None] - self.row[None, :]) * self.row_scale) ** 2 + (dphi * self.radius(row)[:, None]) ** 2
        w = np.exp(-0.5 * d2 / self.gsig ** 2) + 1e-12
        return (w @ self.val) / w.sum(axis=1)

    def at(self, row, ang):
        """Parameter-domain coordinates → (mm coordinates, outward normal)."""
        d = self._dir(row, ang)
        base = self.o + row[:, None] * self.a if self.kind == "ring" else self.o
        return base + self.value_at(row, ang)[:, None] * d, d

    def grid(self, P, pitch: float, reach: float):
        """Lays a grid in the parameter domain at pitch and drops grid points farther than reach from every
        real taxel → (mm coordinates, outward normal, row, ang).
        The parameter-domain coordinates are returned too: the contact ring sets its radius by "which of the
        drawn fine points are over the threshold", so it must be able to measure the arc length from grid
        points to the ring center."""
        np = self.np
        lo = float(self.row.min()) - reach / self.row_scale
        hi = float(self.row.max()) + reach / self.row_scale
        if self.row_hi is not None:
            hi = min(hi, self.row_hi)
        if hi <= lo:
            return np.zeros((0, 3)), np.zeros((0, 3)), np.zeros(0), np.zeros(0)
        a_lo, a_hi = float(self.ang.min()), float(self.ang.max())
        R, A = [], []
        for rg in np.arange(lo, hi + 1e-9, pitch / self.row_scale):
            rc = max(float(self.radius(rg)), 1e-3)            # nearer the apex, one radian is shorter
                                                              # → fewer points on that row
            angs = np.arange(a_lo - reach / rc, a_hi + reach / rc + 1e-9, pitch / rc)
            if len(angs):
                R.append(np.full(len(angs), rg))
                A.append(angs)
        if not R:
            return np.zeros((0, 3)), np.zeros((0, 3)), np.zeros(0), np.zeros(0)
        row, ang = np.concatenate(R), np.concatenate(A)
        pos, nrm = self.at(row, ang)
        keep = np.linalg.norm(pos[:, None, :] - P[None, :, :], axis=2).min(axis=1) <= reach
        return pos[keep], nrm[keep], row[keep], ang[keep]

    # ---- contact ring: equal-arc-length ellipse on the surface
    def _frame(self, row_c: float, ang_c: float):
        """Orthonormal frame (d0, u1, u2) of the cap at (row_c, ang_c): u1 = ∂d/∂ψ, u2 = ∂d/∂φ normalized."""
        np = self.np
        e_c = np.cos(ang_c) * self.ex + np.sin(ang_c) * self.ey
        return (np.cos(row_c) * e_c + np.sin(row_c) * self.a,
                -np.sin(row_c) * e_c + np.cos(row_c) * self.a,
                -np.sin(ang_c) * self.ex + np.cos(ang_c) * self.ey)

    def centroid(self, sel, w):
        """Centroid of a cluster of active taxels in the parameter domain (for the cap it is computed from
        direction vectors, to avoid degenerating near the apex)."""
        np = self.np
        if self.kind == "ring":
            return (float((self.row[sel] * w).sum() / w.sum()),
                    float(np.arctan2((np.sin(self.ang[sel]) * w).sum(), (np.cos(self.ang[sel]) * w).sum())))
        d = (self.tdir[sel] * w[:, None]).sum(axis=0)
        d = d / max(float(np.linalg.norm(d)), 1e-9)
        return float(np.arcsin(np.clip(d @ self.a, -1.0, 1.0))), float(np.arctan2(d @ self.ey, d @ self.ex))

    def offsets_at(self, row_c: float, ang_c: float, row, ang):
        """Surface offsets in mm of a batch of parameter-domain points relative to (row_c, ang_c), split into
        (row direction, ang direction).
        The ring is unrolled into a plane and measured directly; for the cap the geodesic distance rc·θ is
        split onto (u1, u2) along the tangent-plane direction — no fold-over near the apex either."""
        np = self.np
        row, ang = np.asarray(row, float), np.asarray(ang, float)
        if self.kind == "ring":
            dphi = (ang - ang_c + np.pi) % (2 * np.pi) - np.pi
            return (row - row_c) * self.row_scale, dphi * self.rc
        d0, u1, u2 = self._frame(row_c, ang_c)
        d = self._dir(row, ang)
        c = np.clip(d @ d0, -1.0, 1.0)
        t = d - c[..., None] * d0
        k = self.rc * np.arccos(c) / np.maximum(np.linalg.norm(t, axis=-1), 1e-12)
        return k * (t @ u1), k * (t @ u2)

    def arc_dist_at(self, row_c: float, ang_c: float, row, ang):
        """Surface arc length in mm from any batch of parameter-domain points to (row_c, ang_c)."""
        pp, qq = self.offsets_at(row_c, ang_c, row, ang)
        return self.np.hypot(pp, qq)

    def from_offsets(self, row_c: float, ang_c: float, pp, qq):
        """An offset curve relative to (row_c, ang_c) in the parameter domain ((row direction mm, ang
        direction mm)) → (3D mm coordinates, outward normal).
        For the ring the cuff is simply unrolled into a plane and walked directly; for the cap the walk
        follows geodesics, so the curve does not fold over even when it passes the apex.
        Offsets are always clamped to within ~1.2× the curvature radius first: any farther and this
        parameterization wraps around to the back of the finger segment."""
        np = self.np
        lim = 1.20 * self.rc
        pp, qq = np.clip(np.asarray(pp, float), -lim, lim), np.clip(np.asarray(qq, float), -lim, lim)
        if self.kind == "ring":
            return self.at(row_c + pp / self.row_scale, ang_c + qq / self.rc)
        d0, u1, u2 = self._frame(row_c, ang_c)
        m = np.maximum(np.hypot(pp, qq), 1e-9)
        al = m / self.rc
        d = np.cos(al)[:, None] * d0 + (np.sin(al) / m)[:, None] * (pp[:, None] * u1 + qq[:, None] * u2)
        d = d / np.maximum(np.linalg.norm(d, axis=1), 1e-9)[:, None]
        return self.at(np.arcsin(np.clip(d @ self.a, -1.0, 1.0)), np.arctan2(d @ self.ey, d @ self.ex))


class ContactField:
    """Dense point field of the contact heat region.

    There are only 241 physical taxels at 2.5 mm row/column pitch; drawn as one sphere per taxel, a contact
    region is just a small cluster of discrete dots that neither shows the contact shape nor looks good. Here
    a fine grid at pitch_mm is added on the ring/cap surface of each finger-segment link (see _Surface), laid
    only within reach_mm of the real taxels, and each fine point Gaussian-weights **this link's real taxels**:
    every frame, thousands of values are interpolated from the 241 true values and drawn as heat blobs with
    thickness, bulging out along the outward normal. The positions and number of the real taxels are not
    changed at all; points are only filled in between and around them.

    The surface is rebuilt directly from the fit values stored in link_info, **without re-running the ray
    fit** (so it does not touch PLACEMENT_VERSION, does not invalidate the layout cache, and opening the view
    does not cost the extra ~10 s).

    Coverage falloff: weights are first truncated at 3σ and summed into cov, then divided by max(cov,
    0.35·median cov). Fine points at the array edge have few neighbors and a low cov, so their interpolated
    values are scaled down proportionally — the heat blob fades out naturally at the array boundary instead of
    stopping abruptly at the outermost row."""

    def __init__(self, layout: TactileLayout, placed: PlacedLayout, hand: str, pitch_mm: float = 0.9,
                 reach_mm: float = 4.0, sigma_mm: float = 2.6, geom_sigma_mm: float = 1.6,
                 max_nodes: int = 40000):
        import numpy as np
        self.np = np
        self.layout = layout
        self.pitch_m = float(pitch_mm) / 1000.0
        self.sigma_mm = float(sigma_mm)
        self.reach_mm = float(reach_mm)
        self.groups: list[dict] = []
        loc, nrm, count = [], [], 0
        for link, pts in layout.by_link().items():
            name = f"{hand}_{link}"
            info = placed.link_info.get(name)
            if info is None:
                continue
            tax = np.array(sorted(p.idx for p in pts), dtype=int)
            body = int(placed.body_id[tax[0]])
            P = placed.local[tax] * 1000.0
            a = np.asarray(info["axis"], float)
            a = a / np.linalg.norm(a)
            ex, ey = np.asarray(info["ex"], float), np.asarray(info["ey"], float)
            c0 = np.asarray(info["c0_mm"], float)
            kind = str(info["kind"])
            o = c0 if kind == "ring" else c0 + float(info["t_lo_mm"]) * a
            surf = _Surface(np, kind, o, a, ex, ey, P, geom_sigma_mm)
            nodes, nn, nrow, nang = surf.grid(P, pitch_mm, reach_mm)
            if not len(nodes):
                continue
            d = np.linalg.norm(nodes[:, None, :] - P[None, :, :], axis=2)
            W = np.exp(-0.5 * (d / sigma_mm) ** 2)
            W[d > 3.0 * sigma_mm] = 0.0
            cov = W.sum(axis=1)
            floor = 0.35 * float(np.median(cov)) if len(cov) else 1.0
            W /= np.maximum(cov, max(floor, 1e-9))[:, None]
            self.groups.append({"sl": slice(count, count + len(nodes)), "tax": tax, "W": W, "body": body,
                                "name": name, "surf": surf, "P": P, "nrow": nrow, "nang": nang})
            loc.append(nodes / 1000.0)
            nrm.append(nn)
            count += len(nodes)
        self.count = count
        self.local = np.concatenate(loc) if loc else np.zeros((0, 3))
        self.normal = np.concatenate(nrm) if nrm else np.zeros((0, 3))
        self.world = np.zeros_like(self.local)
        self.wnorm = np.zeros_like(self.normal)
        self.overflow = count > max_nodes

    def update_world(self, fk, groups=None):
        """Transforms only the segments with contact into the world frame — finger segments without contact
        need no fine points computed at all."""
        for g in (self.groups if groups is None else groups):
            sl = g["sl"]
            R = fk.d.xmat[g["body"]].reshape(3, 3)
            self.world[sl] = fk.d.xpos[g["body"]] + self.local[sl] @ R.T
            self.wnorm[sl] = self.normal[sl] @ R.T
        return self.world

    def evaluate(self, vals, contact: float):
        """vals = the 241 shown values. Returns (fine-point value array, list of segments with contact).
        Links without contact are skipped entirely: typical contact falls on only 1–3 finger segments, so most
        frames compute only a few hundred fine points."""
        np = self.np
        v = np.asarray(vals, float)
        dv = np.zeros(self.count)
        hot = []
        for g in self.groups:
            vt = v[g["tax"]]
            if not len(vt) or float(vt.max()) <= contact:
                continue
            dv[g["sl"]] = g["W"] @ vt
            hot.append(g)
        return dv, hot

    def describe(self) -> str:
        return (f"[tac] 热区点场：{self.count} 个细点 / {len(self.groups)} 段（pitch {self.pitch_m * 1000:.1f}mm，"
                f"σ {self.sigma_mm:.1f}mm，reach {self.reach_mm:.1f}mm；真实触点 {self.layout.point_count} 个不变）")


def _clusters_of(np, pos, link_m: float):
    """Single-linkage clustering: active taxels within ≤ link_m of each other form one cluster. A link has at
    most 33 taxels, so the full distance matrix is computed directly."""
    n = len(pos)
    adj = np.linalg.norm(pos[:, None, :] - pos[None, :, :], axis=2) <= link_m
    seen = np.zeros(n, bool)
    out = []
    for s in range(n):
        if seen[s]:
            continue
        seen[s] = True
        stack, comp = [s], []
        while stack:
            i = stack.pop()
            comp.append(i)
            for j in np.where(adj[i] & ~seen)[0]:
                seen[j] = True
                stack.append(int(j))
        out.append(np.array(comp))
    return out


def _ring_geoms(mujoco, scn, pts, rgba, width: float, emission: float = 0.0) -> int:
    ident = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    n = 0
    for k in range(len(pts) - 1):
        if scn.ngeom >= scn.maxgeom:
            break
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, [width, 0, 0], [0, 0, 0], ident, list(rgba))
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, width, pts[k], pts[k + 1])
        g.specular, g.shininess, g.emission = 0.0, 0.0, float(emission)
        scn.ngeom += 1
        n += 1
    return n


def _heat_rings(mujoco, scn, field: ContactField, fk, vals, dv, scale: float, args, hot) -> int:
    """Outlines the contact region on each finger segment with contact by a ring. Rings are drawn per link: a
    contact spanning two finger segments gives two rings, which is more honest than forcing one large ring
    across the joint (once the joint bends, there is no surface a cross-joint ring could lie on).

    The radius is set by the **drawn fine points** that are over the threshold, not by the sparse real taxels:
    the interpolated heat region always spreads wider than the taxels themselves; circling only the taxels
    would leave a band of equally "over-threshold" green points outside the ring, and the ring would
    contradict itself."""
    np = field.np
    contact = float(args.contact)
    pad = float(args.heat_ring_pad_mm)
    seg = int(args.heat_ring_seg)
    # Lift the ring slightly above the fine points: the ring sits on the "just over threshold" band, where
    # fine points are small and faint, and drawn flush with the skin it gets buried by them (at a 0.5mm lift
    # only a few scattered pieces of the ring remain); but it does not follow the bulge either — the heat blob
    # center is much higher, and a ring lifted to the same height becomes a loop hanging in the air that no
    # longer looks drawn on the finger segment.
    # 2.5mm was found by comparison: on the cap side the fitted sphere and the real mesh separate outside the
    # taxel array; at a 1.3mm lift most of the fingertip ring is buried in the model and only a few pieces
    # remain; at 2.5mm the whole ring is intact, while the rings in the thumb/finger-segment frames still lie
    # on the finger segment without floating.
    up = 0.0025
    n_blob = 0
    for g in hot:
        vt = vals[g["tax"]]
        sel = np.where(vt > contact)[0]
        if not len(sel):
            continue
        surf, R3, T3 = g["surf"], fk.d.xmat[g["body"]].reshape(3, 3), fk.d.xpos[g["body"]]
        comps = _clusters_of(np, g["P"][sel] / 1000.0, float(args.heat_cluster_mm) / 1000.0)
        cens = [surf.centroid(sel[c], vt[sel[c]]) for c in comps]
        hn = np.where(dv[g["sl"]] > contact)[0]                     # fine points over threshold in this
                                                                    # segment (in-segment indices)
        offs = [surf.offsets_at(rc, ac, g["nrow"][hn], g["nang"][hn]) for rc, ac in cens] if len(hn) else []
        own = (np.stack([np.hypot(pp, qq) for pp, qq in offs]).argmin(axis=0)   # fine point → nearest cluster
               if offs else None)
        for k, (row_c, ang_c) in enumerate(cens):
            ii = sel[comps[k]]
            w = vt[ii]
            # Ring shape = the outline of this cluster's own hot fine points (when degenerate, at least
            # enclosing the cluster's real taxels) — neither a circle nor an ellipse.
            # In the parameter domain, split by azimuth into seg sectors, take the farthest hot fine point in
            # each sector as the radius, fill empty sectors from neighbors, then smooth once cyclically along
            # the azimuth and add pad: the result is a closed curve hugging the outer edge of the heat blob.
            # Why not a circle/ellipse: contact regions are often slanted bands or crescents, and a fitted
            # circle bulges out into large empty areas at four corners — under thumb load, an ellipse ring
            # came out nearly half a finger segment larger than the heat blob, with its lower part floating
            # right off the model.
            pq = np.stack(surf.offsets_at(row_c, ang_c, surf.row[ii], surf.ang[ii]), axis=1)
            if own is not None and bool((own == k).any()):
                m = own == k
                pq = np.concatenate([pq, np.stack([offs[k][0][m], offs[k][1][m]], axis=1)])
            th = np.linspace(0.0, 2 * np.pi, seg, endpoint=False)
            bi = np.floor(np.arctan2(pq[:, 1], pq[:, 0]) % (2 * np.pi) / (2 * np.pi / seg)).astype(int) % seg
            rad = np.zeros(seg)
            np.maximum.at(rad, bi, np.hypot(pq[:, 0], pq[:, 1]))
            if not rad.any():
                continue
            hit = np.flatnonzero(rad > 0)                       # fill empty sectors from the nearest
                                                                # non-empty one (nearest cyclically)
            if len(hit) < seg:
                d = (np.arange(seg)[:, None] - hit[None, :] + seg // 2) % seg - seg // 2
                rad = rad[hit[np.abs(d).argmin(axis=1)]]
            ker = np.hanning(max(seg // 6 | 1, 5) + 2)[1:-1]                    # cyclic smoothing,
                                                                                # removes sector spikes
            rad = np.convolve(np.r_[rad, rad, rad], ker / ker.sum(), "same")[seg:2 * seg] + pad
            pp, qq = rad * np.cos(th), rad * np.sin(th)
            pp, qq = np.r_[pp, pp[0]], np.r_[qq, qq[0]]         # close the loop
            # When the contact spreads around the finger segment beyond ±1.1rad (about 63°), **no ring is
            # drawn**: such a frame means "the whole segment is under load", not "there is a contact patch
            # here", and no ring on the surface can both lie on the segment and enclose it entirely (forcing
            # one gives a polyline that wraps to the back of the segment, half of it in mid-air). For the ring
            # kind only the around-axis direction counts — however long along the axis, it cannot wrap to the
            # back.
            if (np.abs(qq).max() if surf.kind == "ring" else rad.max()) > 1.10 * surf.rc:
                continue
            peak = float(np.clip(w.max() / scale, 0.0, 1.0)) if scale > 0 else 1.0
            # Ring color = the heat color of the cluster's peak, lifted toward white — a light press gives a
            # cool-white ring, a hard press a hot-white ring, so the ring itself carries information; a pure
            # white ring blends into the highlights on the gray back of the hand, and a touch of the color
            # scale makes it easier to spot.
            cr, cg, cb = heat_rgb(peak)
            col = (0.55 * cr + 0.42, 0.55 * cg + 0.40, 0.55 * cb + 0.36)
            # The ring is drawn as an **opaque** thin tube: capsules joined end to end form a smooth tube, but
            # when translucent, the two overlapping end hemispheres blend in one extra layer and the whole
            # ring turns into a string of beads (as at alpha 0.4). One tube per ring, with no outer glow
            # layered on — a glow is translucent as well and beads the same way, and one line is enough to
            # "outline the contact region".
            wd = 0.00021 + 0.00011 * peak                      # slightly thicker with the peak:
                                                               # light press thin, hard press thick
            pos, nn = surf.from_offsets(row_c, ang_c, pp, qq)
            ring = T3 + (pos / 1000.0) @ R3.T + (nn @ R3.T) * up
            _ring_geoms(mujoco, scn, ring, col + (1.0,), wd, emission=0.30)
            n_blob += 1
    return n_blob


def draw_heat(mujoco, scn, field: ContactField, placed: PlacedLayout, fk, shown, scale: float, args,
              selected: int = -1, slab: "GeomSlab | None" = None) -> tuple:
    """Draws only where there is contact: taxels without contact are not drawn at all (the heatmap shows only
    the loaded regions).
    Returns (number of fine points drawn, number of contact clusters, number cut off by --heat-max)."""
    np = field.np
    ident = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    contact = float(args.contact)
    if shown is None or not field.count:
        return 0, 0, 0
    vals = np.asarray(shown, float)
    dv, hot = field.evaluate(vals, contact)
    if not hot:
        return 0, 0, 0
    field.update_world(fk, hot)
    # Keep only a narrow sub-threshold fade band (0.7×threshold to threshold), so the visible extent of the
    # heat blob equals "the part actually in contact". In the variants widened to 0.3–0.4×threshold, the
    # interpolated field decays slowly and blue points can spread across the whole finger segment, which looks
    # as if the whole segment were loaded.
    lo = max(contact * 0.70, 1.0)
    idx = np.concatenate([np.arange(g["sl"].start, g["sl"].stop) for g in hot])
    idx = idx[dv[idx] > lo]
    cut = 0
    room = max(0, scn.maxgeom - scn.ngeom - 8 * args.heat_ring_seg - 4)
    cap = min(int(args.heat_max), room)
    if len(idx) > cap:                       # keep only the strongest; the status bar states
                                             # how many were cut, no silent truncation
        cut = len(idx) - cap
        idx = idx[np.argsort(dv[idx])[::-1][:cap]]
    u = np.clip(dv[idx] / scale, 0.0, 1.0) if scale > 0 else np.zeros(len(idx))
    # Edge fade: in the band below the threshold (lo..contact), alpha and radius shrink proportionally, so the
    # heat blob edge fades out instead of being a hard cut; below lo nothing at all is drawn — "no contact,
    # nothing shown" is the premise of this mode.
    e = np.clip((dv[idx] - lo) / max(contact - lo, 1e-6), 0.0, 1.0)
    lift = 0.00025 + (float(args.heat_bulge_mm) / 1000.0) * u ** 1.5     # outward-normal bulge = the 3D look
    pos = field.world[idx] + field.wnorm[idx] * lift[:, None]
    # Radius ≈ 0.9–1.9× the grid spacing (grid 0.9mm): enough to overlap into one continuous shell without
    # enlarging the heat blob by a band and swallowing the contact ring. Clamping the blob-center alpha to 1.0
    # is the same matter — where translucent spheres overlap, the outline of every sphere shows.
    rad = field.pitch_m * (0.45 + 0.50 * np.sqrt(u)) * (0.62 + 0.38 * e)
    alpha = np.clip(0.58 + 0.55 * u ** 0.7, 0.0, 1.0) * e ** 1.35   # the exponent on e hides stray points
                                                                    # outside the blob faster
    emis = 0.22 + 0.38 * u ** 1.2
    # Specular off, emission on: the default material's specular highlight outlines every sphere with a bright
    # rim and the heat blob breaks up into a tray of beads; emission follows the force, the harder the
    # "hotter", and it also keeps the heat blob independent of finger-segment orientation and lighting (the
    # side facing away from the light is just as visible).
    rgba = np.concatenate([heat_rgb_vec(np, u), alpha[:, None]], axis=1)
    if slab is None or not slab.spheres(scn, pos, rad, rgba, emis):
        for k in range(len(idx)):                 # one by one: fallback when slab self-check failed /
                                                  # no slab / too few slots
            g = scn.geoms[scn.ngeom]
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, [float(rad[k]), 0, 0], pos[k], ident,
                                [float(x) for x in rgba[k]])
            g.specular, g.shininess, g.emission = 0.0, 0.0, float(emis[k])
            scn.ngeom += 1
    n_draw = len(idx)
    n_blob = _heat_rings(mujoco, scn, field, fk, vals, dv, scale, args, hot) if args.heat_ring else 0

    if 0 <= selected < len(vals) and scn.ngeom < scn.maxgeom:
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, [0.0016, 0, 0], placed.world[selected], ident,
                            [1, 1, 1, 0.30])
        scn.ngeom += 1
    return n_draw, n_blob, cut


# ----------------------------------------------------------------------------- values / statistics

def shown_values(tac, baseline):
    if tac is None:
        return None
    if baseline is None:
        return [max(0, v) for v in tac]
    return [max(0, v - b) for v, b in zip(tac, baseline)]


def finger_stats(layout: TactileLayout, shown, threshold: float):
    out = []
    for f in FINGERS:
        sl = layout.finger_slice(f)
        vals = shown[sl] if shown is not None else []
        if not vals:
            out.append((f, 0, 0, 0, -1))
            continue
        act = sum(1 for v in vals if v > threshold)
        mx = max(vals)
        am = sl.start + vals.index(mx) if mx > threshold else -1
        out.append((f, act, mx, sum(vals), am))
    return out


def point_label(layout: TactileLayout, idx: int) -> str:
    p = layout.points[idx]
    return f"{p.finger} {p.zone} p{p.point} X{p.x} Y{p.y}"


def active_table(layout: TactileLayout, placed: PlacedLayout, shown, threshold: float, top: int) -> str:
    rows = [(shown[i], i) for i in range(len(shown)) if shown[i] > threshold]
    rows.sort(reverse=True)
    lines = [f"{'idx':>4} {'finger':<7} {'zone':<5} {'pt':>3} {'X':>2} {'Y':>3} {'link':<18} {'value':>8}  world x y z (mm)"]
    for v, i in rows[:top]:
        p = layout.points[i]
        w = placed.world[i] * 1000.0
        lines.append(f"{i:4d} {p.finger:<7} {p.zone:<5} {p.point:3d} {p.x:2d} {p.y:3d} {p.link:<18} {v:8.0f}  "
                     f"{w[0]:+7.1f} {w[1]:+7.1f} {w[2]:+7.1f}")
    if len(rows) > top:
        lines.append(f"   … 共 {len(rows)} 个 > {threshold:g}，只列前 {top}")
    if not rows:
        lines.append(f"   （没有 > {threshold:g} 的点）")
    return "\n".join(lines)


# ----------------------------------------------------------------------------- PNG (no PIL dependency)

def write_png(path: pathlib.Path, img) -> None:
    h, w, _ = img.shape
    raw = b"".join(b"\x00" + img[y].tobytes() for y in range(h))

    def chunk(tag: bytes, body: bytes) -> bytes:
        return struct.pack(">I", len(body)) + tag + body + struct.pack(">I", zlib.crc32(tag + body) & 0xFFFFFFFF)

    data = (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))
    pathlib.Path(path).write_bytes(data)


# ----------------------------------------------------------------------------- moving geoms across views

def _copy_rigid_bulk(np, src: GeomSlab, dst: GeomSlab, dst_scn, start: int, n: int, R, t) -> bool:
    if not (src.ok and dst.ok) or src.stride != dst.stride or id(dst_scn) != dst._scn_id:
        return False
    if dst_scn.ngeom + n > dst_scn.maxgeom:
        return False
    k0 = int(dst_scn.ngeom)
    # copy whole records verbatim (unmapped fields such as dataid / category / material come along), then
    # change only the pose
    ctypes.memmove(dst.rec0 + dst.stride * k0, src.rec0 + src.stride * start, dst.stride * n)
    sl = dst.view[k0:k0 + n]
    pos = sl["pos"].astype(float)
    mat = sl["mat"].astype(float).reshape(n, 3, 3)
    sl["pos"] = pos @ R.T + t
    sl["mat"] = (R @ mat).reshape(n, 9)
    dst_scn.ngeom = k0 + n
    return True


def copy_geoms_rigid(mujoco, np, src_scn, start: int, stop: int, dst_scn, R, t,
                     src_slab: "GeomSlab | None" = None, dst_slab: "GeomSlab | None" = None) -> int:
    """Appends src_scn.geoms[start:stop] to dst_scn via the rigid transform (R, t); returns the count moved.

    pos' = R·pos + t, mat' = R·mat (mat is the rotation from the geom frame to the world frame, row-major)."""
    n = max(0, min(int(stop), int(src_scn.ngeom)) - int(start))
    if n == 0:
        return 0
    R = np.asarray(R, float).reshape(3, 3)
    t = np.asarray(t, float).reshape(3)
    if src_slab is not None and dst_slab is not None and \
            _copy_rigid_bulk(np, src_slab, dst_slab, dst_scn, int(start), n, R, t):
        return n
    done = 0
    for k in range(int(start), int(start) + n):             # move one by one: fallback if slab is unavailable
        if dst_scn.ngeom >= dst_scn.maxgeom:
            break
        s = src_scn.geoms[k]
        mat = R @ np.asarray(s.mat, float).reshape(3, 3)
        g = dst_scn.geoms[dst_scn.ngeom]
        mujoco.mjv_initGeom(g, int(s.type), np.asarray(s.size, float), R @ np.asarray(s.pos, float) + t,
                            mat.reshape(9), np.asarray(s.rgba, float))
        g.specular, g.shininess, g.emission = float(s.specular), float(s.shininess), float(s.emission)
        dst_scn.ngeom += 1
        done += 1
    return done
