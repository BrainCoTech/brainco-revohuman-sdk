"""双视图：一个 MuJoCo 窗口左右两格——

  左：笛卡尔视图  两掌按 tracker 的 6D 位姿摆在同一个 z 朝上的世界里（地面网格 + 世界轴 + 掌根坐标轴）；
  右：固定基座视图 两掌固定（左 +y、右 −y），只看手指关节与触觉热力图。

一个模型、两份 MjData（手指关节逐位相同，只有掌根位姿不同）、两个 MjvScene、两个自由相机，
共用一个 MjrContext。触觉热力图只在固定基座场景里算一次，再按每只手的掌根刚体变换
（固定位姿 → tracker 位姿）整段搬进笛卡尔场景，两个视图的热斑逐点一致。

数据全部来自 SDK 广播（``live_client.LiveState``）：关节角是 SDK 换好的 URDF 角，位姿是 tracker 世界系绝对位姿。
没有位姿的手（没有 tracker、tracker 没起来、还没收到过位姿）在笛卡尔视图里不画，原因写在覆盖层；
位姿断流超过 POSE_STALE_S 的手停在最后一个位姿、半透明、标 LOST——绝不拿零位姿去画。
SDK 断开时两个视图都停在最后一帧、整体变淡，顶部横幅写原因，等 SDK 重新起来。

覆盖层字体只有 ASCII，所以屏幕上的字都是英文；控制台另打中文。
"""

from __future__ import annotations

import math
import time

from .hand_model import HandModel, home_position, root_joint_name, set_root
from .heatmap import (ContactField, GeomSlab, ScaleTracker, active_table, copy_geoms_rigid, draw_axes, draw_heat,
                      finger_stats, point_label, shown_values)
from .poses import PalmFromTracker, mat_to_quat
from .tactile_layout import FINGERS, PlacedLayout

POSE_STALE_S = 0.5                      # 超过这么久没有新位姿：保持上一位姿并标 LOST

DEFAULT_CAM = (180.0, -15.0, 0.40)      # 固定基座：从掌面一侧正对着看
BOTH_CAM = (180.0, -15.0, 0.62)         # 双手：两掌相距 0.27 m，退远一点才都在画面里
HAND_GROUP = {"left": 3, "right": 4}      # 两只手的可视几何各占一个 geomgroup，笛卡尔视图按手开关
LAYOUTS = ("split", "cart", "fixed")
CART_TITLE = "CARTESIAN  (tracker 6D pose)"
FIXED_TITLE = "FIXED BASE"
LOST_ALPHA = 0.28
OFFLINE_ALPHA = 0.35
GRID_HALF_M = 1.0
GRID_STEP_M = 0.1

KEYS_TACTILE = ("Left/Right point  PgUp/PgDn finger\n"
                "Up/Down scale  Home baseline  Delete clear  Insert print\n"
                "J joints/tactile  ")
KEYS_COMMON = "F follow  C reset cams  V layout\nmouse: L-drag rotate  R-drag pan  scroll zoom"


def parse_cam(text: str | None, multi: bool = False):
    """--cam "azimuth,elevation,distance" → 三元组；不给就用默认。"""
    if not text:
        return BOTH_CAM if multi else DEFAULT_CAM
    az, el, dist = (float(v) for v in text.split(","))
    return az, el, dist


def assign_hand_groups(mujoco, m, hands) -> None:
    """把每只手的可视几何（URDF visual，group 1）挪到这只手自己的 geomgroup；碰撞几何（group 0）不动。"""
    for g in range(m.ngeom):
        if int(m.geom_group[g]) == 0:
            continue
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, int(m.geom_bodyid[g])) or ""
        for hand in hands:
            if name.startswith(hand + "_"):
                m.geom_group[g] = HAND_GROUP[hand]


def geometry_note(status: dict | None, demo: bool = False) -> str:
    """SDK status 里的 tracker 状态 → 笛卡尔视图底部一行（ASCII）。"""
    if demo:
        return "demo"
    trk = (status or {}).get("tracker")
    if not trk:
        return ""
    if trk.get("fatal"):
        return f"tracker stopped: {trk['fatal']} (restart the SDK broadcast)"
    geo = trk.get("geometry") or {}
    if geo.get("error"):
        note = f"geometry check failed: {geo['error']}"
    elif not geo:
        note = ""
    else:
        issues = int(geo.get("errors") or 0) + int(geo.get("flags") or 0)
        pending = int(geo.get("pending") or 0)            # 还在持续期内：没过，只是还没定性
        if issues:
            note = f"geometry: {issues} issue(s)"
        elif pending:
            note = f"geometry: {pending} pending"
        else:
            note = "geometry ok" if geo.get("ok", True) else "geometry: not ok"
    gates = [str(g) for g in trk.get("start_gates") or () if g]
    if gates:
        note = f"startup gate: {', '.join(gates)} (not enforced)" + (f"  |  {note}" if note else "")
    return note


def rpy_deg(R) -> tuple[float, float, float]:
    """旋转矩阵 → (roll, pitch, yaw) 度，ZYX 顺序。"""
    pitch = -math.asin(max(-1.0, min(1.0, float(R[2][0]))))
    roll = math.atan2(float(R[2][1]), float(R[2][2]))
    yaw = math.atan2(float(R[1][0]), float(R[0][0]))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class RateMeter:
    """每秒按帧计数差算一次实际帧率。"""

    def __init__(self):
        self.t0 = None
        self.c0 = 0
        self.hz = 0.0

    def update(self, count: int, now: float) -> float:
        if self.t0 is None:
            self.t0, self.c0 = now, count
        elif now - self.t0 >= 1.0:
            self.hz = (count - self.c0) / (now - self.t0)
            self.t0, self.c0 = now, count
        return self.hz


class HandState:
    """一只手在两个视图里的全部状态。"""

    def __init__(self, np, m, d_fix, hand: str, hands, joints, spacing_m: float, tactile):
        self.hand = hand
        self.tag = hand[0].upper()
        self.fk = HandModel(m, d_fix, hand, joints)
        self.root = root_joint_name(hand, hands)
        self.home = np.array(home_position(hand, hands, spacing_m), dtype=float)
        self.conv = PalmFromTracker(np, m, hand)
        self.tactile = tactile                 # session.TactileDecision
        self.placed = None
        self.field = None
        self.baseline = None
        self.shown = None
        self.heat = (0, 0, 0)
        self.heat_range = (0, 0)
        self.cart = None                       # 最近一个有效掌根位姿 (p, R)
        self.cart_t = None                     # 它到达的 monotonic 秒
        self.pose_age = None
        self.pose_state = "none"               # live / lost / none
        self.rate = RateMeter()

    def build_tactile(self, layout, args, log=print) -> None:
        """触觉启用时才贴布局、建热区点场（首次拟合约 10 s，之后走缓存）。"""
        if not self.tactile.enabled:
            return
        self.placed = PlacedLayout(layout, self.fk, self.hand, cache=not args.refit)
        log(self.placed.describe().replace("\n", f"\n[{self.tag}] "))
        t0 = time.monotonic()
        self.field = ContactField(layout, self.placed, self.hand, pitch_mm=args.heat_pitch_mm,
                                  reach_mm=args.heat_reach_mm, sigma_mm=args.heat_sigma_mm,
                                  max_nodes=int(args.heat_max) * 8)
        log(f"[{self.tag}] {self.field.describe()}，建场 {(time.monotonic() - t0) * 1000:.0f} ms")


class DualView:
    """场景状态 + 每帧更新 + 两个场景的组装 + 覆盖层文本 + 画到给定的帧缓冲。窗口与离屏共用。"""

    def __init__(self, mujoco, np, m, d_fix, d_cart, hands: list, layout, live, args, preview=None):
        self.mujoco, self.np = mujoco, np
        self.m, self.d_fix, self.d_cart = m, d_fix, d_cart
        self.hands = hands
        self.layout = layout
        self.live = live                       # live_client.LiveState
        self.args = args
        self.gen = 0
        self.hello: dict = {}
        self.connected = False
        self.status = None
        self.multi = len(hands) > 1
        self.n_points = layout.point_count
        assign_hand_groups(mujoco, m, [h.hand for h in hands])

        self.opt_fix = mujoco.MjvOption()
        self.opt_cart = mujoco.MjvOption()
        for opt in (self.opt_fix, self.opt_cart):
            opt.geomgroup[0] = 0               # 碰撞几何不画
            for g in HAND_GROUP.values():
                opt.geomgroup[g] = 1
        self.pert = mujoco.MjvPerturb()
        per_hand = int(args.heat_max) + 16 * int(args.heat_ring_seg) + 16
        maxgeom = int(m.ngeom) + 256 + per_hand * len(hands)
        self.scn_fix = mujoco.MjvScene(m, maxgeom=maxgeom)
        self.scn_cart = mujoco.MjvScene(m, maxgeom=maxgeom)
        if args.lite:
            for scn in (self.scn_fix, self.scn_cart):
                for fl in (mujoco.mjtRndFlag.mjRND_SHADOW, mujoco.mjtRndFlag.mjRND_REFLECTION,
                           mujoco.mjtRndFlag.mjRND_SKYBOX, mujoco.mjtRndFlag.mjRND_FOG):
                    scn.flags[fl] = 0
        self.slab_fix = self.slab_cart = None
        if not args.no_bulk:
            self.slab_fix = GeomSlab(mujoco, np, self.scn_fix)
            self.slab_cart = GeomSlab(mujoco, np, self.scn_cart)
        self.geom_hand = {}
        for g in range(m.ngeom):
            grp = int(m.geom_group[g])
            for hand, hg in HAND_GROUP.items():
                if grp == hg:
                    self.geom_hand[g] = hand

        self.cam_fix = mujoco.MjvCamera()
        self.cam_cart = mujoco.MjvCamera()
        for cam in (self.cam_fix, self.cam_cart):
            cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.reset_fixed_cam()
        self.cam_cart.lookat[:] = (0.0, 0.0, 0.0)
        self.cam_cart.azimuth, self.cam_cart.elevation, self.cam_cart.distance = 180.0, -20.0, 1.0
        self.cart_cam_ready = False

        self.scale_tr = ScaleTracker(args.scale, float(layout.value.get("auto_scale_floor", 200.0)),
                                     float(layout.value.get("auto_scale_decay", 0.995)))
        self.scale = self.scale_tr.floor
        self.sel_hand = 0
        self.sel = 0
        self.msg, self.msg_t = "", 0.0
        self.layout_mode = args.view
        self.follow = True
        self.show_joints = False
        self.snaps = {}
        self.geometry_note = ""
        # 可选的关节角预览（见 app.load_preview_ext）：开着时写进模型的是变换后的 q，
        # 关节表的 fw deg 仍是原始读数，另加一列 delta（变换后 − 变换前，度）。
        self.preview = preview
        self.preview_xf = {h.hand: preview.bind(h.hand, h.fk.specs) for h in hands} if preview is not None else {}
        self.preview_delta = {h.hand: [None] * len(h.fk.specs) for h in hands}

    # ------------------------------------------------------------------ SDK 下发的静态信息

    def tracker_info(self) -> dict:
        return self.hello.get("tracker") or {}

    def tracker_on(self) -> bool:
        return bool(self.tracker_info().get("enabled")) and getattr(self.args, "tracker", "auto") != "off"

    def tracker_reason(self) -> str:
        if getattr(self.args, "tracker", "auto") == "off":
            return "off (--tracker off)"
        return str(self.tracker_info().get("reason") or "tracker not opened by the SDK")

    def product(self) -> dict:
        return self.hello.get("product") or {}

    # ------------------------------------------------------------------ 相机

    def reset_fixed_cam(self) -> None:
        self.cam_fix.lookat[:] = (0.0, 0.0, 0.10)
        self.cam_fix.azimuth, self.cam_fix.elevation, self.cam_fix.distance = parse_cam(self.args.cam, self.multi)

    def posed(self) -> list:
        return [h for h in self.hands if h.cart is not None]

    def palm_mid(self):
        posed = self.posed()
        if not posed:
            return None
        return sum(h.cart[0] for h in posed) / len(posed)

    def reset_cart_cam(self) -> None:
        """对准两掌中点；方位角取「与固定基座视图同一种看法」：面对掌面、左手在画面右侧。
        双手时由左→右掌连线定（对掌心朝向不敏感），单手时由掌心法向的水平投影定。"""
        posed = self.posed()
        if not posed:
            return
        np = self.np
        cam = self.cam_cart
        cam.lookat[:] = self.palm_mid()
        by = {h.hand: h for h in posed}
        if "left" in by and "right" in by:
            d = by["right"].cart[0] - by["left"].cart[0]
            span = float(np.hypot(d[0], d[1]))
            if span > 1e-3:
                cam.azimuth = math.degrees(math.atan2(-d[0], d[1]))
            cam.distance = min(3.0, max(0.8, 2.4 * float(np.linalg.norm(d))))
        else:
            n = posed[0].cart[1][:, 0]            # 掌心法向 = 掌根系 x 轴
            if math.hypot(n[0], n[1]) > 0.3:
                cam.azimuth = math.degrees(math.atan2(n[1], n[0])) + 180.0
            cam.distance = 0.6
        cam.elevation = -20.0
        self.cart_cam_ready = True

    # ------------------------------------------------------------------ 交互

    def note(self, text: str) -> None:
        self.msg, self.msg_t = text, time.monotonic()
        print(f"[view] {text}", flush=True)

    def cur(self) -> HandState:
        return self.hands[self.sel_hand]

    def key(self, name: str) -> None:
        h = self.cur()
        who = f"{h.tag} " if self.multi else ""
        n = self.n_points
        finger_idx = FINGERS.index(self.layout.points[self.sel].finger)
        if name == "TAB":
            if self.multi:
                self.sel_hand = (self.sel_hand + 1) % len(self.hands)
                self.note(f"hand -> {self.cur().hand}")
        elif name == "LEFT":
            self.sel = (self.sel - 1) % n
        elif name == "RIGHT":
            self.sel = (self.sel + 1) % n
        elif name == "PGUP":
            self.sel = self.layout.finger_slice(FINGERS[(finger_idx - 1) % len(FINGERS)]).start
        elif name == "PGDN":
            self.sel = self.layout.finger_slice(FINGERS[(finger_idx + 1) % len(FINGERS)]).start
        elif name == "UP":
            self.scale_tr.nudge(1.25, self.scale)
            self.note(f"scale -> {self.scale_tr.fixed:.0f} (fixed)")
        elif name == "DOWN":
            self.scale_tr.nudge(0.8, self.scale)
            self.note(f"scale -> {self.scale_tr.fixed:.0f} (fixed)")
        elif name == "HOME":
            tac = self.snaps.get(h.hand, {}).get("tac")
            if not h.tactile.enabled:
                self.note(f"{who}tactile disabled: {h.tactile.reason}")
            elif tac is None or len(tac) != n:
                self.note(f"{who}no tactile frame yet, cannot capture baseline")
            else:
                h.baseline = list(tac)
                self.note(f"{who}baseline captured (max {max(tac)})")
        elif name == "DELETE":
            h.baseline = None
            self.note(f"{who}baseline cleared")
        elif name == "INSERT":
            if h.shown is not None:
                print(f"[view] {h.hand}:", flush=True)
                print(active_table(self.layout, h.placed, h.shown, self.args.threshold, self.args.top), flush=True)
            else:
                self.note(f"{who}no tactile frame")
        elif name == "J":
            self.show_joints = not self.show_joints
            self.note("table -> joints" if self.show_joints else "table -> tactile")
        elif name == "F":
            self.follow = not self.follow
            self.note(f"follow {'ON' if self.follow else 'OFF'}")
        elif name == "C":
            self.reset_fixed_cam()
            self.reset_cart_cam()
            self.follow = True
            self.note("cameras reset, follow ON")
        elif name == "V":
            self.layout_mode = LAYOUTS[(LAYOUTS.index(self.layout_mode) + 1) % len(LAYOUTS)]
            self.note(f"layout -> {self.layout_mode}")
        elif name == "PREVIEW" and self.preview is not None:
            self.note(_ascii(self.preview.toggle()))

    # ------------------------------------------------------------------ 每帧

    def update(self, now: float | None = None) -> None:
        mj = self.mujoco
        now = time.monotonic() if now is None else now
        hello, _gen, self.connected = self.live.session()
        if hello is not None:
            self.hello = hello
        self.status = self.live.last_status()
        snaps = {h.hand: self.live.snapshot(h.hand) for h in self.hands}
        for h in self.hands:
            s = snaps[h.hand]
            h.fk.update(self._preview_q(h.hand, s["q"]), s["deg"], s["mask"])    # 写 d_fix 的手指关节
            h.rate.update(s["count"], now)
        mj.mj_forward(self.m, self.d_fix)
        for h in self.hands:
            h.fk.write(self.d_cart)                     # 笛卡尔那份：手指关节逐位相同
            pose = self.live.pose(h.hand) if self.tracker_on() else None
            if pose is not None:
                pos, q, age = pose
                h.pose_age = age
                if age is None or age <= POSE_STALE_S:
                    h.cart = h.conv.palm_pose(pos, q)
                    h.cart_t = now
            if h.cart is None:
                h.pose_state = "none"
            else:
                h.pose_state = "live" if h.cart_t is not None and now - h.cart_t <= POSE_STALE_S else "lost"
                p, R = h.cart
                set_root(self.m, self.d_cart, h.root, p, mat_to_quat(R))
        mj.mj_forward(self.m, self.d_cart)
        if not self.cart_cam_ready and self.posed():
            self.reset_cart_cam()
        self.geometry_note = geometry_note(self.status, self.product().get("source") == "demo")

        peak = 0.0
        for h in self.hands:
            tac = snaps[h.hand]["tac"]
            if h.placed is not None and h.tactile.enabled and tac is not None and len(tac) == self.n_points:
                h.shown = shown_values(tac, h.baseline)
                h.placed.update_world(h.fk)
                peak = max(peak, max(h.shown))
            else:
                h.shown = None
        self.scale = self.scale_tr.step(peak)
        self.snaps = snaps

    def _preview_q(self, hand: str, q):
        """预览开着时返回变换后的 q（并记下逐关节的 delta），否则原样返回。"""
        xf = self.preview_xf.get(hand)
        if xf is None or q is None or not self.preview.enabled:
            return q
        out = xf.apply(q)
        delta = self.preview_delta[hand]
        for k, (a, b) in enumerate(zip(q, out)):
            if k < len(delta) and a is not None and b is not None and math.isfinite(a) and math.isfinite(b):
                delta[k] = math.degrees(b - a)
        return out

    def preview_on(self) -> bool:
        return self.preview is not None and bool(self.preview.enabled)

    # ------------------------------------------------------------------ 场景

    def build_scenes(self) -> None:
        mj, np = self.mujoco, self.np
        cat = mj.mjtCatBit.mjCAT_ALL
        scn = self.scn_fix
        mj.mjv_updateScene(self.m, self.d_fix, self.opt_fix, self.pert, self.cam_fix, cat, scn)
        draw_axes(mj, scn)
        for k, h in enumerate(self.hands):
            start = int(scn.ngeom)
            if h.field is not None:
                sel = self.sel if k == self.sel_hand else -1
                h.heat = draw_heat(mj, scn, h.field, h.placed, h.fk, h.shown, self.scale, self.args, sel,
                                   self.slab_fix)
            h.heat_range = (start, int(scn.ngeom))

        scn = self.scn_cart
        posed = self.posed()
        for h in self.hands:
            self.opt_cart.geomgroup[HAND_GROUP[h.hand]] = 1 if h.cart is not None else 0
        if self.follow and posed:
            self.cam_cart.lookat[:] = self.palm_mid()
        mj.mjv_updateScene(self.m, self.d_cart, self.opt_cart, self.pert, self.cam_cart, cat, scn)
        lost = {h.hand for h in posed if h.pose_state == "lost"}
        if lost:
            for i in range(int(scn.ngeom)):
                g = scn.geoms[i]
                if int(g.objtype) == int(mj.mjtObj.mjOBJ_GEOM) and self.geom_hand.get(int(g.objid)) in lost:
                    g.rgba[3] = LOST_ALPHA
        for h in posed:
            p, R = h.cart
            copy_geoms_rigid(mj, np, self.scn_fix, h.heat_range[0], h.heat_range[1], scn, R, p - R @ h.home,
                             self.slab_fix, self.slab_cart)
        if posed:
            self._draw_world(scn, posed)
        if not self.connected:
            for s in (self.scn_fix, self.scn_cart):
                self._dim(s)

    def _dim(self, scn) -> None:
        """SDK 断开：手与热斑整体变淡（停在最后一帧）。"""
        mj = self.mujoco
        for i in range(int(scn.ngeom)):
            g = scn.geoms[i]
            if int(g.objtype) == int(mj.mjtObj.mjOBJ_GEOM) and int(g.objid) not in self.geom_hand:
                continue
            g.rgba[3] = min(float(g.rgba[3]), OFFLINE_ALPHA)

    def _line(self, scn, a, b, rgba, width: float) -> None:
        mj = self.mujoco
        if scn.ngeom >= scn.maxgeom:
            return
        g = scn.geoms[scn.ngeom]
        mj.mjv_initGeom(g, mj.mjtGeom.mjGEOM_LINE, [0, 0, 0], [0, 0, 0], [1, 0, 0, 0, 1, 0, 0, 0, 1], rgba)
        mj.mjv_connector(g, mj.mjtGeom.mjGEOM_LINE, width, [float(v) for v in a], [float(v) for v in b])
        scn.ngeom += 1

    def _arrow(self, scn, a, b, rgba, width: float = 0.004) -> None:
        mj = self.mujoco
        if scn.ngeom >= scn.maxgeom:
            return
        g = scn.geoms[scn.ngeom]
        mj.mjv_initGeom(g, mj.mjtGeom.mjGEOM_ARROW, [0, 0, 0], [0, 0, 0], [1, 0, 0, 0, 1, 0, 0, 0, 1], rgba)
        mj.mjv_connector(g, mj.mjtGeom.mjGEOM_ARROW, width, [float(v) for v in a], [float(v) for v in b])
        scn.ngeom += 1

    def _label(self, scn, pos, text: str, rgba=(1, 1, 1, 0)) -> None:
        mj = self.mujoco
        if scn.ngeom >= scn.maxgeom:
            return
        g = scn.geoms[scn.ngeom]
        mj.mjv_initGeom(g, mj.mjtGeom.mjGEOM_SPHERE, [0.004, 0, 0], [float(v) for v in pos],
                        [1, 0, 0, 0, 1, 0, 0, 0, 1], rgba)
        g.label = text
        scn.ngeom += 1

    def _draw_world(self, scn, posed) -> None:
        """z=0 平面上的网格（按 0.1 m 对齐世界坐标，手动了网格不跟着动）+ 网格角上的世界轴 +
        每只手的掌根坐标轴与到地面的铅垂线。"""
        np = self.np
        mid = self.palm_mid()
        cx = round(float(mid[0]) / GRID_STEP_M) * GRID_STEP_M
        cy = round(float(mid[1]) / GRID_STEP_M) * GRID_STEP_M
        n = int(round(GRID_HALF_M / GRID_STEP_M))
        grid = (0.62, 0.66, 0.72, 0.45)
        for i in range(-n, n + 1):
            x = cx + i * GRID_STEP_M
            y = cy + i * GRID_STEP_M
            self._line(scn, (x, cy - GRID_HALF_M, 0.0), (x, cy + GRID_HALF_M, 0.0), grid, 1.0)
            self._line(scn, (cx - GRID_HALF_M, y, 0.0), (cx + GRID_HALF_M, y, 0.0), grid, 1.0)
        o = np.array([cx - GRID_HALF_M, cy - GRID_HALF_M, 0.0])
        for axis, rgba in ((0, (1, 0.2, 0.2, 1)), (1, (0.2, 1, 0.2, 1)), (2, (0.3, 0.5, 1, 1))):
            e = np.zeros(3)
            e[axis] = 0.2
            self._arrow(scn, o, o + e, rgba, 0.006)
        self._label(scn, o + np.array([0.0, 0.0, 0.22]), "world (z up)")
        for h in posed:
            p, R = h.cart
            for axis, rgba in ((0, (1, 0.2, 0.2, 1)), (1, (0.2, 1, 0.2, 1)), (2, (0.3, 0.5, 1, 1))):
                self._arrow(scn, p, p + R[:, axis] * 0.06, rgba, 0.003)
            self._line(scn, p, (p[0], p[1], 0.0), (0.9, 0.9, 0.9, 0.5), 1.0)
            tag = h.tag if h.pose_state == "live" else f"{h.tag} LOST"
            self._label(scn, p + np.array([0.0, 0.0, 0.12]), tag)

    # ------------------------------------------------------------------ 覆盖层文本

    def _hand_status(self, h: HandState, now: float) -> list[str]:
        s = self.snaps.get(h.hand) or {}
        pre = f"{h.tag} " if self.multi else ""
        ind = "  " if self.multi else ""
        age = s.get("age")
        age_txt = f"{age * 1000:6.1f}ms" if age is not None else "   --  "
        mask = int(s.get("mask") or 0)
        invalid = [int(sp["index"]) for sp in h.fk.specs if not (mask >> int(sp["index"]) & 1)]
        lines = [f"{pre}{self._port(h.hand)}  {h.rate.hz:5.1f} fps  age {age_txt}  seq {s.get('seq', 0)}"
                 f"  invalid {invalid if s.get('deg') is not None and invalid else ('none' if s.get('deg') is not None else '(no frame)')}"]
        if s.get("error"):
            lines.append(f"{ind}!! stream stopped: {_ascii(s['error'])}")
        tac = f"{ind}tactile {h.tactile.reason}"
        if h.tactile.enabled:
            meta = s.get("tac_meta") or {}
            tac += f"  frames {s.get('tac_count', 0)}"
            if meta and not meta.get("link_ok", True):
                tac += "  LINK DOWN"
            if h.baseline is not None:
                tac += "  baseline ON"
            if h.field is not None:
                n_draw, n_blob, cut = h.heat
                tac += f"  blobs {n_blob} dots {n_draw}" + (f" (+{cut} capped)" if cut else "")
        lines.append(tac)
        return lines

    def _port(self, hand: str) -> str:
        return str(((self.hello.get("gloves") or {}).get(hand) or {}).get("port") or "?")

    def fixed_texts(self, now: float) -> list:
        mj = self.mujoco
        h = self.cur()
        out = []
        if h.tactile.enabled and not self.show_joints:
            left = [FIXED_TITLE, ""] + ([f"hand {h.tag} ({h.hand})"] if self.multi else [])
            right = ["", ""] + ([""] if self.multi else [])
            left += ["finger  act", "-----------"]
            right += ["      max       sum", "------------------"]
            for f, act, mx, sm, _am in finger_stats(self.layout, h.shown, self.args.threshold):
                left.append(f"{f:<7} {act:3d}")
                right.append(f"{mx:9.0f} {sm:9.0f}")
            p = self.layout.points[self.sel]
            tac = self.snaps.get(h.hand, {}).get("tac")
            raw = tac[self.sel] if tac is not None and len(tac) == self.n_points else "--"
            val = f"{h.shown[self.sel]:.0f}" if h.shown is not None else "--"
            left += ["", f"sel {self.sel:3d} {p.finger}", f"  {p.zone} pt{p.point} X{p.x} Y{p.y}", f"  {p.link}"]
            right += ["", f"raw {raw}", f"shown {val}", ""]
        else:
            left, right = self.joint_table(h)
            left = [FIXED_TITLE, ""] + left
            right = ["", ""] + right
        out.append((mj.mjtGridPos.mjGRID_TOPLEFT, "\n".join(left), "\n".join(right)))
        prod = self.product()
        src = "demo" if prod.get("source") == "demo" else f"sdk {self.hello.get('sdk', '?')} pid {self.hello.get('pid', '?')}"
        lines = self.preview_lines() + [f"product {prod.get('code', '?')} ({'+'.join(prod.get('sensors') or ())})  "
                                        f"identity {prod.get('source') or '-'}  source {src}"]
        if (self.status or {}).get("kit_fatal"):
            lines.append(f"!! kit stopped: {_ascii(self.status['kit_fatal'])}")
        for hk in self.hands:
            lines += self._hand_status(hk, now)
        if any(hk.field is not None for hk in self.hands):
            lines.append(f"scale {self.scale:.0f} ({self.scale_tr.mode})  contact >{self.args.contact:g}"
                         f"  layout {self.layout.path.name if self.layout.path else '-'}")
        out.append((mj.mjtGridPos.mjGRID_BOTTOMLEFT, "\n".join(lines), ""))
        return out

    def joint_table(self, h: HandState) -> tuple[list, list]:
        """关节角表（左列关节名，右列读数）。mjr_overlay 每段文本最多 mjMAXOVERLAY=500 字符，
        21 路关节要放得下：预览开着多一列 delta 时换成窄格式。* = 还没收到过有效值。"""
        left = [f"hand {h.tag} joints ({self.product().get('code', '?')})", "-" * 18]
        delta = self.preview_delta.get(h.hand) or []
        if not self.preview_on():
            right = ["  fw deg  urdf deg", "-" * 18]
            for name, fw, urdf, seen in h.fk.joint_rows():
                left.append(name)
                right.append((f"{fw:8.1f}" if fw is not None else "      --") + f" {urdf:8.1f}" + ("" if seen else " *"))
            return left, right
        right = ["    fw   urdf delta", "-" * 19]
        for k, (name, fw, urdf, seen) in enumerate(h.fk.joint_rows()):
            left.append(name)
            d = delta[k] if k < len(delta) else None
            right.append((f"{fw:6.1f}" if fw is not None else "    --") + f" {urdf:6.1f}"
                         + (f" {d:+5.1f}" if d is not None else "    --") + ("" if seen else "*"))
        return left, right

    def preview_lines(self) -> list[str]:
        """预览扩展的状态行（没有扩展时为空）。"""
        text = self.preview.overlay() if self.preview is not None else ""
        return [_ascii(text)] if text else []

    def cart_texts(self, now: float) -> list:
        mj = self.mujoco
        out = []
        lines = [CART_TITLE, ""]
        on = self.tracker_on()
        roles = self.tracker_info().get("roles") or {}
        if not on:
            lines.append(f"no pose: {_ascii(self.tracker_reason())}")
        for h in self.hands:
            pre = f"{h.tag} "
            if h.pose_state == "none":
                if on:
                    lines.append(pre + ("waiting for first pose" if h.hand in roles else "no tracker for this hand"))
                continue
            p, R = h.cart
            r, pch, y = rpy_deg(R)
            if h.pose_state == "live":
                age = f"{h.pose_age * 1000:5.0f}ms" if h.pose_age is not None else "--"
                head = f"{pre}live  age {age}"
            else:
                head = f"{pre}LOST {now - h.cart_t:4.1f}s (holding last pose)"
            lines.append(head)
            lines.append(f"  xyz {p[0]:+.3f} {p[1]:+.3f} {p[2]:+.3f} m")
            lines.append(f"  rpy {r:+6.1f} {pch:+6.1f} {y:+6.1f} deg")
        out.append((mj.mjtGridPos.mjGRID_TOPLEFT, "\n".join(lines), ""))
        foot = []
        if roles:
            foot.append("tracker " + "  ".join(f"{r[0].upper()}={s}" for r, s in sorted(roles.items())))
        if self.geometry_note and on:
            foot.append(_ascii(self.geometry_note))
        if on:
            foot.append(f"follow {'ON' if self.follow else 'OFF'}  (axes = palm frame, grid 0.1 m on z=0)")
        if self.layout_mode == "cart":                   # 固定基座视图不在屏上时，预览状态行挪到这里
            foot = self.preview_lines() + foot
        out.append((mj.mjtGridPos.mjGRID_BOTTOMLEFT, "\n".join(foot), ""))
        return out

    def keys_text(self, now: float) -> str:
        txt = ("Tab hand  " if self.multi else "") + \
            (KEYS_TACTILE if any(h.field is not None for h in self.hands) else "") + \
            (f"{self.preview.keys_hint}  " if self.preview is not None else "") + KEYS_COMMON
        if self.msg and now - self.msg_t < 4.0:
            txt = self.msg + "\n" + txt
        return txt

    # ------------------------------------------------------------------ 画到帧缓冲

    def viewports(self, width: int, height: int) -> list:
        """[(名字, MjrRect)]；左笛卡尔、右固定基座。"""
        mj = self.mujoco
        if self.layout_mode == "cart":
            return [("cart", mj.MjrRect(0, 0, width, height))]
        if self.layout_mode == "fixed":
            return [("fixed", mj.MjrRect(0, 0, width, height))]
        half = width // 2
        return [("cart", mj.MjrRect(0, 0, half, height)), ("fixed", mj.MjrRect(half, 0, width - half, height))]

    def viewport_at(self, x: float, width: int, height: int) -> str | None:
        for name, rect in self.viewports(width, height):
            if rect.left <= x < rect.left + rect.width:
                return name
        return None

    def scene_cam(self, name: str):
        return (self.scn_cart, self.cam_cart) if name == "cart" else (self.scn_fix, self.cam_fix)

    def render(self, ctx, width: int, height: int, now: float | None = None) -> None:
        mj = self.mujoco
        now = time.monotonic() if now is None else now
        ports = self.viewports(width, height)
        font = mj.mjtFont.mjFONT_NORMAL
        for k, (name, rect) in enumerate(ports):
            scn = self.scn_cart if name == "cart" else self.scn_fix
            mj.mjr_render(rect, scn, ctx)
            texts = self.cart_texts(now) if name == "cart" else self.fixed_texts(now)
            if k == len(ports) - 1:
                texts.append((mj.mjtGridPos.mjGRID_TOPRIGHT, self.keys_text(now), ""))
            for grid, t1, t2 in texts:
                mj.mjr_overlay(font, grid, rect, t1, t2, ctx)
        if len(ports) == 2:
            left = ports[1][1].left
            mj.mjr_rectangle(mj.MjrRect(left - 1, 0, 2, height), 0.85, 0.85, 0.85, 1.0)
        if not self.connected:                  # 两个视图都画完再压横幅，否则右半边会把它盖掉
            grid = getattr(mj.mjtGridPos, "mjGRID_TOP", mj.mjtGridPos.mjGRID_TOPLEFT)
            reason = _ascii(self.live.disconnect_reason or "connection lost")
            mj.mjr_overlay(mj.mjtFont.mjFONT_BIG, grid, mj.MjrRect(0, 0, width, height),
                           "SDK DISCONNECTED - " + reason, "showing the last frame, waiting for the SDK", ctx)


def _ascii(text) -> str:
    return "".join(c if 32 <= ord(c) < 127 else "?" for c in str(text))


def render_waiting(mujoco, ctx, width: int, height: int, text: str) -> None:
    """还没有场景（没连上 SDK / 正在建场景）时的画面：清屏 + 居中一段说明。"""
    mj = mujoco
    full = mj.MjrRect(0, 0, width, height)
    mj.mjr_rectangle(full, 0.12, 0.13, 0.15, 1.0)
    mj.mjr_overlay(mj.mjtFont.mjFONT_NORMAL, mj.mjtGridPos.mjGRID_TOPLEFT, full, text, "", ctx)


# ----------------------------------------------------------------------------- 离屏快照

def render_png(view: DualView, path, size=(1600, 800)) -> None:
    """离屏把两个视图（按当前 layout）画进一张 PNG。无显示器时由调用方设 MUJOCO_GL=egl。"""
    from .heatmap import write_png
    mj, np = view.mujoco, view.np
    w, h = size
    m = view.m
    m.vis.global_.offwidth = max(int(m.vis.global_.offwidth), w)
    m.vis.global_.offheight = max(int(m.vis.global_.offheight), h)
    gl = mj.GLContext(w, h)
    gl.make_current()
    try:
        ctx = mj.MjrContext(m, mj.mjtFontScale.mjFONTSCALE_100)
        try:
            mj.mjr_setBuffer(mj.mjtFramebuffer.mjFB_OFFSCREEN, ctx)
            view.build_scenes()
            view.render(ctx, w, h)
            rgb = np.zeros((h, w, 3), dtype=np.uint8)
            mj.mjr_readPixels(rgb, None, mj.MjrRect(0, 0, w, h), ctx)
            write_png(path, np.ascontiguousarray(np.flipud(rgb)))
        finally:
            ctx.free()
    finally:
        gl.free()


# ----------------------------------------------------------------------------- 窗口

class _Mouse:
    def __init__(self):
        self.left = self.right = self.middle = False
        self.x = self.y = 0.0
        self.owner = None


def window_keymap(glfw, preview=None) -> dict:
    """GLFW 键码 → DualView.key() 的名字。有预览扩展时再加它的切换键（名字 PREVIEW）。"""
    keymap = {glfw.KEY_TAB: "TAB", glfw.KEY_LEFT: "LEFT", glfw.KEY_RIGHT: "RIGHT", glfw.KEY_UP: "UP",
              glfw.KEY_DOWN: "DOWN", glfw.KEY_PAGE_UP: "PGUP", glfw.KEY_PAGE_DOWN: "PGDN",
              glfw.KEY_HOME: "HOME", glfw.KEY_DELETE: "DELETE", glfw.KEY_INSERT: "INSERT",
              glfw.KEY_J: "J", glfw.KEY_F: "F", glfw.KEY_C: "C", glfw.KEY_V: "V"}
    if preview is not None:
        pkey = getattr(glfw, f"KEY_{str(preview.key).upper()}", None)
        if pkey is None or pkey in keymap:
            raise ValueError(f"预览扩展的切换键 {preview.key!r} 不可用（不存在或已被占用）")
        keymap[pkey] = "PREVIEW"
    return keymap


def run_window(controller, stop, fps: float = 60.0, size=(1600, 820), title: str = "RevoHuman viewer") -> None:
    """GLFW 窗口主循环（渲染在主线程；订阅线程只写 LiveState）。stop：threading.Event。

    窗口一开就在：SDK 没起来时画等待画面，收到 hello、场景建好后切到双视图；场景换了（SDK 换了型号 /
    换了手）就按新模型重建渲染上下文。"""
    import glfw
    mj = controller.mujoco
    if not glfw.init():
        raise RuntimeError("GLFW 初始化失败（没有图形界面？可以用 --headless --snapshot）")
    glfw.window_hint(glfw.SAMPLES, 4)
    win = glfw.create_window(size[0], size[1], title, None, None)
    if not win:
        glfw.terminate()
        raise RuntimeError("GLFW 建窗失败（没有图形界面？可以用 --headless --snapshot）")
    glfw.make_context_current(win)
    glfw.swap_interval(1)
    xs, _ys = glfw.get_window_content_scale(win)
    fs = mj.mjtFontScale.mjFONTSCALE_200 if xs >= 1.75 else (
        mj.mjtFontScale.mjFONTSCALE_150 if xs >= 1.25 else mj.mjtFontScale.mjFONTSCALE_100)
    blank = mj.MjModel.from_xml_string("<mujoco/>")      # 等待画面只需要字体
    ctx = mj.MjrContext(blank, fs)
    ctx_view = None                                      # ctx 是按哪个场景建的
    mouse = _Mouse()
    keymap = window_keymap(glfw, getattr(controller, "preview", None))

    def cur_view():
        return controller.view if controller.view is ctx_view else None

    def fb_scale():
        ww, wh = glfw.get_window_size(win)
        fw, fh = glfw.get_framebuffer_size(win)
        return (fw / ww if ww else 1.0), (fh / wh if wh else 1.0), fw, fh

    def on_key(_w, key, _sc, action, _mods):
        view = cur_view()
        if view is not None and action in (glfw.PRESS, glfw.REPEAT) and key in keymap:
            view.key(keymap[key])

    def on_button(_w, button, action, _mods):
        mouse.left = glfw.get_mouse_button(win, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS
        mouse.right = glfw.get_mouse_button(win, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS
        mouse.middle = glfw.get_mouse_button(win, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS
        mouse.x, mouse.y = glfw.get_cursor_pos(win)
        view = cur_view()
        if action == glfw.PRESS and view is not None:
            sx, _sy, fw, fh = fb_scale()
            mouse.owner = view.viewport_at(mouse.x * sx, fw, fh)
        elif not (mouse.left or mouse.right or mouse.middle):
            mouse.owner = None

    def on_move(_w, x, y):
        dx, dy = x - mouse.x, y - mouse.y
        mouse.x, mouse.y = x, y
        view = cur_view()
        if view is None or mouse.owner is None or not (mouse.left or mouse.right or mouse.middle):
            return
        shift = (glfw.get_key(win, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
                 or glfw.get_key(win, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS)
        if mouse.right:
            action = mj.mjtMouse.mjMOUSE_MOVE_H if shift else mj.mjtMouse.mjMOUSE_MOVE_V
            if mouse.owner == "cart" and view.follow:
                view.follow = False
                view.note("follow OFF (F to re-enable)")
        elif mouse.left:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H if shift else mj.mjtMouse.mjMOUSE_ROTATE_V
        else:
            action = mj.mjtMouse.mjMOUSE_ZOOM
        _ww, wh = glfw.get_window_size(win)
        _scn, cam = view.scene_cam(mouse.owner)
        mj.mjv_moveCamera(view.m, action, dx / max(wh, 1), dy / max(wh, 1), cam)

    def on_scroll(_w, _xo, yo):
        view = cur_view()
        if view is None:
            return
        x, _y = glfw.get_cursor_pos(win)
        sx, _sy, fw, fh = fb_scale()
        name = view.viewport_at(x * sx, fw, fh)
        if name is not None:
            _scn, cam = view.scene_cam(name)
            mj.mjv_moveCamera(view.m, mj.mjtMouse.mjMOUSE_ZOOM, 0.0, -0.05 * yo, cam)

    glfw.set_key_callback(win, on_key)
    glfw.set_mouse_button_callback(win, on_button)
    glfw.set_cursor_pos_callback(win, on_move)
    glfw.set_scroll_callback(win, on_scroll)
    period = 1.0 / max(1.0, fps)
    frames, t_start = 0, time.monotonic()
    try:
        while not glfw.window_should_close(win) and not stop.is_set():
            t0 = time.monotonic()
            controller.poll()
            if controller.view is not ctx_view:          # 新场景：按它的模型重建渲染上下文
                ctx.free()
                ctx_view = controller.view
                ctx = mj.MjrContext(ctx_view.m if ctx_view is not None else blank, fs)
                mouse.owner = None
            view = cur_view()
            fw, fh = glfw.get_framebuffer_size(win)
            if view is not None:
                view.update(t0)
                view.build_scenes()
            if fw > 0 and fh > 0:
                if view is not None:
                    view.render(ctx, fw, fh, t0)
                else:
                    render_waiting(mj, ctx, fw, fh, controller.waiting_text())
                glfw.swap_buffers(win)
                frames += 1
            glfw.poll_events()
            dt = period - (time.monotonic() - t0)
            if dt > 0:
                time.sleep(dt)
    finally:
        ctx.free()
        glfw.destroy_window(win)
        glfw.terminate()
        dt = time.monotonic() - t_start
        print(f"[view] 窗口关闭：{frames} 帧 / {dt:.1f}s（{frames / max(dt, 1e-6):.1f} fps）", flush=True)


# ----------------------------------------------------------------------------- 无窗口汇报

def report_headless(view: DualView, seconds: float, stop, log=print) -> int:
    """跑 seconds 秒后打印每只手的统计。返回码：0 正常 / 2 有手一帧都没收到 / 3 触觉已启用但没收到触觉帧。"""
    view.update()
    base = {h.hand: (view.snaps[h.hand]["count"], view.snaps[h.hand]["tac_count"]) for h in view.hands}
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds and not stop.is_set():
        view.update()
        time.sleep(0.02)
    view.update()
    dt = time.monotonic() - t0
    rc = 0
    for h in view.hands:
        s = view.snaps[h.hand]
        pre = f"[{h.hand}] "
        n, n_tac = s["count"] - base[h.hand][0], s["tac_count"] - base[h.hand][1]
        log(f"{pre}{view._port(h.hand)}：{n} 帧 / {dt:.1f}s（{n / max(dt, 1e-6):.1f} Hz），带触觉 {n_tac} 帧，"
            f"valid_mask={int(s['mask']):#x}")
        if view.preview_on():
            ds = [(abs(d), d, sp["name"]) for d, sp in zip(view.preview_delta[h.hand], h.fk.specs) if d is not None]
            top = max(ds) if ds else None
            log(f"{pre}关节角预览开（{_ascii(view.preview.overlay())}）："
                + (f"delta 最大 {top[1]:+.2f}°（{top[2]}）" if top else "还没有 delta"))
        if s["error"]:
            log(f"{pre}数据流中断：{s['error']}")
        if s["q"] is None:
            log(f"{pre}一帧都没收到（快照按零姿态画）")
            rc = rc or 2
        log(f"{pre}触觉：{h.tactile.reason}")
        if h.tactile.enabled:
            if h.shown is None:
                log(f"{pre}触觉已启用但没收到触觉帧")
                rc = rc or 3
            else:
                log(f"{pre}色标量程 {view.scale:.0f}（{view.scale_tr.mode}）")
                log(f"{'finger':<7} {'active':>6} {'max':>8} {'sum':>9}  argmax")
                for f, act, mx, sm, am in finger_stats(view.layout, h.shown, view.args.threshold):
                    log(f"{f:<7} {act:6d} {mx:8.0f} {sm:9.0f}  "
                        + (f"idx {am} = {point_label(view.layout, am)}" if am >= 0 else "-"))
                log(active_table(view.layout, h.placed, h.shown, view.args.threshold, view.args.top))
        if h.pose_state == "none":
            reason = view.tracker_reason() if not view.tracker_on() else (
                "还没收到位姿" if h.hand in (view.tracker_info().get("roles") or {}) else "这只手没有 tracker")
            log(f"{pre}笛卡尔：没有位姿（{reason}）")
        else:
            p, R = h.cart
            r, pch, y = rpy_deg(R)
            log(f"{pre}笛卡尔：{h.pose_state}  掌根 xyz {p[0]:+.3f} {p[1]:+.3f} {p[2]:+.3f} m  "
                f"rpy {r:+.1f} {pch:+.1f} {y:+.1f}°")
    return rc
