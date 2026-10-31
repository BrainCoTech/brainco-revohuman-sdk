"""触觉热力图：241 个真实触点 → 指节曲面上的密集插值热斑 + 接触圈。

热斑按固定基座视图的那份 MjData 算一次（`draw_heat`），笛卡尔视图里的同一只手与它只差一个
刚体变换（关节角相同、只有 palm_link 位姿不同），用 `copy_geoms_rigid` 整段搬过去，不重算。

`args` 形参只读下面这些属性，`HeatParams` 与 argparse 的 Namespace 都满足：
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
    contact: float = 20.0            # 接触阈值（与触觉读数同单位）
    heat_pitch_mm: float = 0.9       # 细网格间距
    heat_sigma_mm: float = 2.6       # 插值高斯 σ
    heat_reach_mm: float = 4.0       # 细网格只铺在真实触点周围这么远以内
    heat_bulge_mm: float = 1.6       # 满量程时热斑沿外法向鼓起的高度
    heat_max: int = 6000             # 每只手每帧最多画多少个细点
    heat_cluster_mm: float = 9.0     # 接触圈聚类距离
    heat_ring: bool = True           # 画接触圈
    heat_ring_pad_mm: float = 1.5    # 接触圈比热区外扩
    heat_ring_seg: int = 44          # 接触圈分段数


class ScaleTracker:
    """色标量程：--scale>0 定死；否则峰值保持 + 每帧慢衰减，且不低于 floor。"""

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


# ----------------------------------------------------------------------------- 接触热区

HEAT_STOPS = ((0.00, 0.10, 0.28, 0.78), (0.25, 0.10, 0.76, 0.86), (0.50, 0.38, 0.90, 0.36),
              (0.75, 1.00, 0.78, 0.16), (1.00, 1.00, 0.26, 0.14))


def heat_rgb(u: float):
    """热力色标：蓝 → 青 → 绿 → 橙 → 红，接近满量程再往白热收一点。
    只出颜色不出透明度 —— alpha 由 draw_heat 按「离接触阈值多远」算，色标按「占色标量程多少」算，
    两件事分开：弱接触也该看得清（alpha 有底），但颜色该是冷的。"""
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
    if u > 0.88:                                  # 核心补白热，让最强的地方跳出来
        t = 0.7 * (u - 0.88) / 0.12
        r, g, b = r + (1.00 - r) * t, g + (0.95 - g) * t, b + (0.85 - b) * t
    return (r, g, b)


def heat_rgb_vec(np, u):
    """heat_rgb 的向量版：一次算 n 个点，结果与逐点调用 heat_rgb 逐位一致（有单测对照）。"""
    u = np.clip(np.asarray(u, float), 0.0, 1.0)
    st = np.asarray(HEAT_STOPS, float)
    rgb = np.stack([np.interp(u, st[:, 0], st[:, c]) for c in (1, 2, 3)], axis=1)
    t = np.clip(0.7 * (u - 0.88) / 0.12, 0.0, None)[:, None]
    white = np.asarray([1.00, 0.95, 0.85])
    return np.where(t > 0, rgb + (white - rgb) * t, rgb)


class GeomSlab:
    """把 mjvScene.geoms 那块连续 C 内存映射成 numpy 结构化数组，热斑的几千个球一次写完。

    为什么：heat 一帧要往 user_scn 里塞 1–2 千个球，逐个 mjv_initGeom + 三个属性赋值就是每帧
    ~7000 次 pybind 调用；每次调用都要在 GIL 上打转，而收流线程、渲染线程都在抢它——
    实测同样一帧，无线程 12 ms、带两个收包线程 25 ms（p95 40 ms）。批量写把这段压到 0.2 ms 以内。

    怎么保证不写错：偏移**不写死**。构造时用官方 wrapper 的 numpy 视图（size/pos/mat/rgba）的地址
    推出槽位跨度与各字段偏移，type/emission/specular/shininess 靠写哨兵值再回找；然后拿 3 个随机球
    分别走「逐个 mjv_initGeom」和「批量」两条路，逐字节比对，不一致就 ok=False，调用方退回逐个写。
    模板：先让 mjv_initGeom 写一个球进 0 号槽（dataid/objtype/category/texid… 全是它给的默认值），
    整条记录复制到 n 个槽，再只改 size/pos/rgba/emission 四个字段。"""

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
        except Exception as e:                                  # 任何一步不对就整体禁用，不冒险写内存
            self.ok, self.why = False, f"{type(e).__name__}: {e}"
        if verbose:
            print(f"[slab] bulk geom write {'ON' if self.ok else 'OFF: ' + self.why}", file=sys.stderr)

    # ---- 布局探测
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
        self.view = np.frombuffer(raw, dtype=dt)               # 可写视图，直指 C 内存
        # 同一块内存的整条记录视图：先整条抄模板，再改字段。只按字段赋值会漏掉 label 等未映射的字节——
        # 新建场景的槽位内存没清零，残留字节被当成几何体标签画出来，就是热斑上那团乱码。
        self.rawview = np.frombuffer(raw, dtype=np.dtype((np.void, stride)))
        self.stride, self.rec0, self._scn_id = stride, rec0, id(scn)
        # 模板槽：官方路径写一个「零球」，把 mjv_initGeom 给的其他默认字段一并抄下来
        # mjv_initGeom 只写 label[0]=0，其余字节原样留着，所以先把整条清零，模板才是确定的
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
        self.rawview[:n] = np.void(bytes(self.stride))         # 与模板同样从全零出发
        for k in range(n):                                     # 官方路径
            g = scn.geoms[k]
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, [float(rad[k]), 0, 0], pos[k], ident,
                                [float(x) for x in rgba[k]])
            g.specular, g.shininess, g.emission = 0.0, 0.0, float(emis[k])
        ref = self._bytes(0, n)
        self.rawview[:n] = np.void(b"\xa5" * self.stride)     # 整条记录灌垃圾：有字节没被覆盖就比对不过
        self._write(0, pos, rad, rgba, emis)                   # 批量路径
        got = self._bytes(0, n)
        assert got == ref, "批量写与 mjv_initGeom 结果不一致"
        assert abs(float(scn.geoms[n - 1].emission) - float(emis[n - 1])) < 1e-6
        scn.ngeom = 0

    def _write(self, start: int, pos, rad, rgba, emis) -> None:
        self.rawview[start:start + len(rad)] = self.tmpl_raw
        sl = self.view[start:start + len(rad)]
        sl["size"] = self.np.asarray(rad, self.np.float32)[:, None]    # 球：mjv_initGeom 会把三个 size 都填成半径
        sl["pos"] = pos
        sl["rgba"] = rgba
        sl["emission"] = emis

    # ---- 对外
    def spheres(self, scn, pos, rad, rgba, emis) -> bool:
        """从 scn.ngeom 起一次写 len(rad) 个球并推进 ngeom；scn 换了对象/不可用时返回 False（调用方逐个写）。"""
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
    """一个指节 cuff 的参数曲面，用 PlacedLayout.link_info 里已经存好的拟合量复原。

    参数域 (row, ang)：
      ring（套环）row = 沿轴坐标 mm、ang = 绕轴方位角 rad、val = 该处半径 ρ mm；
      cap （套帽）row = 仰角 ψ rad、 ang = 方位角 rad、     val = 该处球半径 r mm。
    row_scale = row 一个单位合多少 mm，radius(row) = 该行上 ang 一弧度合多少 mm；
    有了这两把尺子，参数域里就能按「曲面上的弧长 mm」量距离 —— 铺细点、插值、画接触圈
    用的都是同一套度量，所以圈是真的贴着指节转，而不是一个悬在半空的平面圆。"""

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
            self.rc = float(self.val.mean())                  # 套环半径：方位角 → 弧长的换算尺
            self.row_scale, self.row_hi = 1.0, None
            self.tdir = w / np.maximum(self.val, 1e-9)[:, None]
        else:
            self.val = np.linalg.norm(rel, axis=1)
            d = rel / np.maximum(self.val, 1e-9)[:, None]
            self.row = np.arcsin(np.clip(d @ self.a, -1.0, 1.0))
            w = d - (d @ self.a)[:, None] * self.a
            self.ang = np.arctan2(w @ self.ey, w @ self.ex)
            self.rc = float(self.val.mean())                  # 套帽球半径
            self.row_scale, self.row_hi = self.rc, float(np.pi / 2)   # ψ 不越顶：>90° 这套参数化会翻折
            self.tdir = d

    # ---- 参数域 ↔ 三维
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
        """本 link 的真实触点在参数域里高斯加权 → 该处的径向量，于是细点落在触点所在的那层
        曲面上（已含 lift_mm 抬高），既不穿模也不浮空。"""
        np = self.np
        dphi = (ang[:, None] - self.ang[None, :] + np.pi) % (2 * np.pi) - np.pi
        d2 = ((row[:, None] - self.row[None, :]) * self.row_scale) ** 2 + (dphi * self.radius(row)[:, None]) ** 2
        w = np.exp(-0.5 * d2 / self.gsig ** 2) + 1e-12
        return (w @ self.val) / w.sum(axis=1)

    def at(self, row, ang):
        """参数域坐标 →（mm 坐标, 外法向）。"""
        d = self._dir(row, ang)
        base = self.o + row[:, None] * self.a if self.kind == "ring" else self.o
        return base + self.value_at(row, ang)[:, None] * d, d

    def grid(self, P, pitch: float, reach: float):
        """按 pitch 在参数域铺格，丢掉离任何真实触点超过 reach 的格点 →（mm 坐标, 外法向, row, ang）。
        参数域坐标一并交出来：接触圈要按「画出来的细点里哪些过了阈值」定半径，得能量格点到圈心的弧长。"""
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
            rc = max(float(self.radius(rg)), 1e-3)            # 越靠近顶点，一弧度越短 → 该行点越少
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

    # ---- 接触圈：曲面上的等弧长椭圆
    def _frame(self, row_c: float, ang_c: float):
        """套帽在 (row_c, ang_c) 处的正交标架 (d0, u1, u2)：u1 = ∂d/∂ψ，u2 = ∂d/∂φ 归一化。"""
        np = self.np
        e_c = np.cos(ang_c) * self.ex + np.sin(ang_c) * self.ey
        return (np.cos(row_c) * e_c + np.sin(row_c) * self.a,
                -np.sin(row_c) * e_c + np.cos(row_c) * self.a,
                -np.sin(ang_c) * self.ex + np.cos(ang_c) * self.ey)

    def centroid(self, sel, w):
        """一团活跃触点在参数域里的重心（套帽用方向向量求，免得在顶点附近退化）。"""
        np = self.np
        if self.kind == "ring":
            return (float((self.row[sel] * w).sum() / w.sum()),
                    float(np.arctan2((np.sin(self.ang[sel]) * w).sum(), (np.cos(self.ang[sel]) * w).sum())))
        d = (self.tdir[sel] * w[:, None]).sum(axis=0)
        d = d / max(float(np.linalg.norm(d)), 1e-9)
        return float(np.arcsin(np.clip(d @ self.a, -1.0, 1.0))), float(np.arctan2(d @ self.ey, d @ self.ex))

    def offsets_at(self, row_c: float, ang_c: float, row, ang):
        """参数域里一批点相对 (row_c, ang_c) 的曲面偏移 mm，分解成 (row 向, ang 向)。
        套环展开成平面直接量；套帽把测地距离 rc·θ 按切平面方向拆到 (u1, u2) —— 顶点附近也不翻折。"""
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
        """参数域里任意一批点到 (row_c, ang_c) 的曲面弧长 mm。"""
        pp, qq = self.offsets_at(row_c, ang_c, row, ang)
        return self.np.hypot(pp, qq)

    def from_offsets(self, row_c: float, ang_c: float, pp, qq):
        """参数域里一条相对 (row_c, ang_c) 的偏移曲线（(row 向 mm, ang 向 mm)）→（3D mm 坐标, 外法向）。
        套环就是把 cuff 展开成平面直接走；套帽按测地线走，所以曲线绕过顶点也不翻折。
        偏移一律先夹到 ~1.2 倍曲率半径内：再远这套参数化就绕到指节背面去了。"""
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
    """接触热区的密集点场。

    实物触觉点只有 241 个、行/列距 2.5 mm，一个点一个球画出来接触区就是一小撮离散点，
    既看不出接触形状也谈不上好看。这里在每个指节 link 的套环/套帽曲面（见 _Surface）上按
    pitch_mm 补一层细网格（只铺在真实触点周围 reach_mm 以内），每个细点对**本 link 的真实
    触点**做高斯加权：每帧由 241 个真值插出几千个值，画成带厚度、沿外法向鼓起来的热斑。
    真实触点的位置和数量一点没改，只是在它们之间、边上补点。

    曲面直接用 link_info 里存好的拟合量复原，**不重新射线拟合**（所以不碰 PLACEMENT_VERSION、
    不使布局缓存失效、起视图不多花那 ~10 s）。

    覆盖度衰减：权重先按 3σ 截断求和得 cov，再除以 max(cov, 0.35·cov 中位数)。
    阵列边缘的细点邻居少、cov 低，插出来的值被按比例压下去 —— 热斑到阵列边界自然淡出，
    而不是在最外一行戛然而止。"""

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
        """只把有接触的那几段换算到世界系——没接触的指节一个细点都不用算。"""
        for g in (self.groups if groups is None else groups):
            sl = g["sl"]
            R = fk.d.xmat[g["body"]].reshape(3, 3)
            self.world[sl] = fk.d.xpos[g["body"]] + self.local[sl] @ R.T
            self.wnorm[sl] = self.normal[sl] @ R.T
        return self.world

    def evaluate(self, vals, contact: float):
        """vals = 241 个 shown 值。返回 (细点值数组, 有接触的段列表)。
        整段跳过没接触的 link：典型接触只落在 1–3 个指节上，绝大多数帧只算几百个细点。"""
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
    """单链聚类：互相 ≤ link_m 的活跃触点归一团。一个 link 最多 33 个触点，整张距离矩阵直接算。"""
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
    """把每个有接触的指节上的接触区用圆圈框出来。圈按 link 分别画：一次接触跨两个指节就是两个圈，
    比硬凑一个跨关节的大圈诚实（关节一弯，跨关节的圈就没有一个曲面能贴）。

    半径按**画出来的细点**里过阈值的那些定，不是按稀疏真实触点定：插值出来的热区总比触点本身铺得开，
    只圈触点的话圈外还会留一圈同样「过了阈值」的绿点，圈就自相矛盾了。"""
    np = field.np
    contact = float(args.contact)
    pad = float(args.heat_ring_pad_mm)
    seg = int(args.heat_ring_seg)
    # 圈抬到细点之上一点：圈落在「刚过阈值」那一圈上，那里的细点又小又淡，贴着皮画会被它们埋掉
    # （抬 0.5mm 时圈只剩零星几截）；但也不跟着 bulge 走——热斑中心比这高得多，
    # 圈要是抬到同高就成了悬在空中的环，不再像画在指节上。
    # 2.5mm 是对比出来的：套帽那边拟合球与真网格在触点阵列之外会分家，抬 1.3mm 时指尖那圈大半
    # 埋进模型里、只剩几截；抬到 2.5mm 整圈才完整，而拇指/指节
    # 那几帧的圈仍然贴着指节没飘起来。
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
        hn = np.where(dv[g["sl"]] > contact)[0]                     # 本段里过阈值的细点（段内下标）
        offs = [surf.offsets_at(rc, ac, g["nrow"][hn], g["nang"][hn]) for rc, ac in cens] if len(hn) else []
        own = (np.stack([np.hypot(pp, qq) for pp, qq in offs]).argmin(axis=0)   # 细点归给最近的那一团
               if offs else None)
        for k, (row_c, ang_c) in enumerate(cens):
            ii = sel[comps[k]]
            w = vt[ii]
            # 圈的形状 = 这团热细点自己的轮廓（退化时至少圈住本团真实触点），不是正圆也不是椭圆。
            # 参数域里按方位分 seg 个扇区，每个扇区取最远的那个热细点作半径，空扇区拿邻居补上，
            # 再沿方位做一次循环平滑、放 pad，就是一条贴着热斑外沿的闭合曲线。
            # 为什么不用正圆/正椭圆：接触区常常是斜带或月牙，套正圆四个角会鼓出一大片空地
            # ——拇指受力时，正椭圆的圈比热斑大出小半个指节、底下一截直接飘出模型。
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
            hit = np.flatnonzero(rad > 0)                       # 空扇区补成最近的非空扇区（环形最近）
            if len(hit) < seg:
                d = (np.arange(seg)[:, None] - hit[None, :] + seg // 2) % seg - seg // 2
                rad = rad[hit[np.abs(d).argmin(axis=1)]]
            ker = np.hanning(max(seg // 6 | 1, 5) + 2)[1:-1]                    # 循环平滑，去掉扇区毛刺
            rad = np.convolve(np.r_[rad, rad, rad], ker / ker.sum(), "same")[seg:2 * seg] + pad
            pp, qq = rad * np.cos(th), rad * np.sin(th)
            pp, qq = np.r_[pp, pp[0]], np.r_[qq, qq[0]]         # 闭合
            # 接触绕着指节铺开超过 ±1.1rad（约 63°）时**不画圈**：这种帧的含义是「整节都在受力」，
            # 不是「这儿有一块接触区」，曲面上根本不存在又贴着指节、又把它整个框住的圈（硬画出来
            # 是一条绕到指节背面、半截悬空的折线）。套环只看绕轴方向——轴向再长也绕不到背面。
            if (np.abs(qq).max() if surf.kind == "ring" else rad.max()) > 1.10 * surf.rc:
                continue
            peak = float(np.clip(w.max() / scale, 0.0, 1.0)) if scale > 0 else 1.0
            # 圈的颜色 = 该团峰值的热力色往白里提 —— 轻按是冷白圈、重按是热白圈，圈本身也带信息；
            # 纯白圈在灰色手背上会跟高光混在一起，掺一点色标反而更认得出来。
            cr, cg, cb = heat_rgb(peak)
            col = (0.55 * cr + 0.42, 0.55 * cg + 0.40, 0.55 * cb + 0.36)
            # 圈画成**不透明**的细管：胶囊首尾相接本来是光滑管，但半透明时首尾两个半球叠在一起会
            # 多混一层，整圈就变成一串珠子（alpha 0.4 时即如此）。一圈一条，
            # 不再叠外层光晕 —— 光晕同样是半透明、同样起珠，而「圈出接触区」本来一条线就够。
            wd = 0.00021 + 0.00011 * peak                      # 按峰值加粗一点，轻按细、重按粗
            pos, nn = surf.from_offsets(row_c, ang_c, pp, qq)
            ring = T3 + (pos / 1000.0) @ R3.T + (nn @ R3.T) * up
            _ring_geoms(mujoco, scn, ring, col + (1.0,), wd, emission=0.30)
            n_blob += 1
    return n_blob


def draw_heat(mujoco, scn, field: ContactField, placed: PlacedLayout, fk, shown, scale: float, args,
              selected: int = -1, slab: "GeomSlab | None" = None) -> tuple:
    """只画接触到的地方：没接触的触觉点一个都不画（热力图只显示受力区域）。
    返回 (画了几个细点, 几团接触, 因 --heat-max 截掉几个)。"""
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
    # 只留窄窄一圈阈下淡出（0.7×阈值 ~ 阈值），热斑的可见范围才等于「真的接触到的那块」。
    # 放宽到 0.3–0.4×阈值那几版，插值场衰减得慢，蓝点能一路铺满整个指节，看着像整节都在受力。
    lo = max(contact * 0.70, 1.0)
    idx = np.concatenate([np.arange(g["sl"].start, g["sl"].stop) for g in hot])
    idx = idx[dv[idx] > lo]
    cut = 0
    room = max(0, scn.maxgeom - scn.ngeom - 8 * args.heat_ring_seg - 4)
    cap = min(int(args.heat_max), room)
    if len(idx) > cap:                       # 只留最强的那些；截了多少在状态栏里说清楚，不做静默截断
        cut = len(idx) - cap
        idx = idx[np.argsort(dv[idx])[::-1][:cap]]
    u = np.clip(dv[idx] / scale, 0.0, 1.0) if scale > 0 else np.zeros(len(idx))
    # 边缘淡出：阈值以下那一圈（lo..contact）按比例收 alpha 和半径，热斑边上是渐隐的，
    # 不是一刀切出来的硬边；而真正到了 lo 以下一个点都不画 —— 「没接触就不显示」是这个模式的前提。
    e = np.clip((dv[idx] - lo) / max(contact - lo, 1e-6), 0.0, 1.0)
    lift = 0.00025 + (float(args.heat_bulge_mm) / 1000.0) * u ** 1.5     # 沿外法向鼓起来 = 3D 感的来源
    pos = field.world[idx] + field.wnorm[idx] * lift[:, None]
    # 半径约等于网格间距的 0.9–1.9 倍（网格 0.9mm）：够叠成一层连续的壳，又不至于把热斑撑大一圈、
    # 把接触圈吞进去。斑心 alpha 压到 1.0 是同一件事 —— 半透明球互相叠，每个球的轮廓都会显出来。
    rad = field.pitch_m * (0.45 + 0.50 * np.sqrt(u)) * (0.62 + 0.38 * e)
    alpha = np.clip(0.58 + 0.55 * u ** 0.7, 0.0, 1.0) * e ** 1.35   # e 的指数让斑外零星点更快隐掉
    emis = 0.22 + 0.38 * u ** 1.2
    # 关掉高光、给自发光：默认材质的镜面高光会把每个球都描一圈亮边，热斑就散成一盘珠子；
    # 自发光按力度给，越重越「烫」，也让热斑不受指节朝向和打光影响（背光那面照样看得清）。
    rgba = np.concatenate([heat_rgb_vec(np, u), alpha[:, None]], axis=1)
    if slab is None or not slab.spheres(scn, pos, rad, rgba, emis):
        for k in range(len(idx)):                 # 逐个写：slab 自检没过 / 没给 slab / 槽位不够时的退路
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


# ----------------------------------------------------------------------------- 取值 / 统计

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


# ----------------------------------------------------------------------------- PNG（不依赖 PIL）

def write_png(path: pathlib.Path, img) -> None:
    h, w, _ = img.shape
    raw = b"".join(b"\x00" + img[y].tobytes() for y in range(h))

    def chunk(tag: bytes, body: bytes) -> bytes:
        return struct.pack(">I", len(body)) + tag + body + struct.pack(">I", zlib.crc32(tag + body) & 0xFFFFFFFF)

    data = (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))
    pathlib.Path(path).write_bytes(data)


# ----------------------------------------------------------------------------- 两个视图之间搬几何体

def _copy_rigid_bulk(np, src: GeomSlab, dst: GeomSlab, dst_scn, start: int, n: int, R, t) -> bool:
    if not (src.ok and dst.ok) or src.stride != dst.stride or id(dst_scn) != dst._scn_id:
        return False
    if dst_scn.ngeom + n > dst_scn.maxgeom:
        return False
    k0 = int(dst_scn.ngeom)
    # 整条记录照抄（dataid / category / 材质等未映射字段也一并带过去），再只改位姿
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
    """把 src_scn.geoms[start:stop] 经刚体变换 (R, t) 追加到 dst_scn，返回搬了几个。

    pos' = R·pos + t，mat' = R·mat（mat 是几何体系到世界系的旋转，行优先）。"""
    n = max(0, min(int(stop), int(src_scn.ngeom)) - int(start))
    if n == 0:
        return 0
    R = np.asarray(R, float).reshape(3, 3)
    t = np.asarray(t, float).reshape(3)
    if src_slab is not None and dst_slab is not None and \
            _copy_rigid_bulk(np, src_slab, dst_slab, dst_scn, int(start), n, R, t):
        return n
    done = 0
    for k in range(int(start), int(start) + n):             # 逐个搬：slab 不可用时的退路
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
