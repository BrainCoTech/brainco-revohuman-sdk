"""Dual view: one MuJoCo window with a left and a right pane —

  left:  Cartesian view   both palms placed by the Tracker 6D pose in the same z-up world (ground grid +
                          world axes + palm_link coordinate axes);
  right: fixed-base view  both palms fixed (left +y, right −y), showing only the finger joints and the
                          tactile heatmap.

One model, two MjData (finger joints bit-for-bit identical, only the palm_link pose differs), two MjvScene,
two free cameras, sharing one MjrContext. The tactile heatmap is computed only once in the fixed-base scene,
then moved as a whole into the Cartesian scene by each hand's palm_link rigid transform (fixed pose → Tracker
pose), so the hot spots in the two views match point for point.

All data comes from the SDK broadcast (``live_client.LiveState``): joint angles are URDF angles already
converted by the SDK, and poses are absolute poses in the Tracker world frame.
Hands without a pose (no Tracker, Tracker not up, or no pose received yet) are not drawn in the Cartesian
view; the reason is written in the overlay;
a hand whose pose stream has been interrupted for longer than POSE_STALE_S stays at its last pose,
semi-transparent, marked LOST — a zero pose is never used for drawing.
When the SDK disconnects, both views stay on the last frame and fade as a whole, a top banner states the
reason, and the views wait for the SDK to come back.

The overlay holds only five things: data frame rate, real-time joint angles (degrees), pose quaternion
(w x y z), sampling timestamp (Unix seconds),
and arrival latency (arrival time − sampling time, median over the last 1 s). The Cartesian view shows the two
hands' quaternions at top left and the pose streams at bottom left;
arrival latency of a pose stream: when the SDK has a pack-delay calibration configured for this Tracker
(``hello.tracker.phi_pack_ms``), it includes that fixed delay;
when none is configured, it includes the host fixed overhead (``TRACKER_HOST_FIXED_MS``) but not the pack
delay, and the line ends with ``(excl. pack delay)``;
the fixed-base view shows the two hands' joint angles at top left and the glove data streams at bottom left;
the key hints are at the top right of the rightmost pane.
Only on errors (data stream interrupted, kit stopped, Tracker stopped, or geometry check failed) is there an
extra line starting with !!.
The overlay font is ASCII only, so all on-screen text is in English; the console additionally prints Chinese.
"""

from __future__ import annotations

import math
import time

from .hand_model import HandModel, home_position, root_joint_name, set_root
from .heatmap import (ContactField, GeomSlab, ScaleTracker, active_table, copy_geoms_rigid, draw_axes, draw_heat,
                      finger_stats, point_label, shown_values)
from .poses import PalmFromTracker, mat_to_quat
from .tactile_layout import FINGERS, PlacedLayout

POSE_STALE_S = 0.5                      # no new pose for longer than this: keep the previous pose,
                                        # mark LOST

# Host fixed overhead of the Tracker pose (ms): from the host kernel receiving the Tracker's USB report to the
# SDK's pose callback getting that frame, taken over the fastest batch of frames.
# When the SDK has no pack-delay calibration configured, the sampling time is aligned to the arrival time of
# the fastest batch of frames, so the pack delay together with this overhead falls into the alignment offset,
# and only the per-frame varying part remains in "arrival time − sampling time"; for display, this overhead is
# added back.
# Measured on an x86 host for two Trackers: 0.10 / 0.13 ms respectively (usbmon completion events, libusb
# callbacks and pose callbacks aligned frame by frame, 11k / 17k frames respectively).
TRACKER_HOST_FIXED_MS = 0.1

DEFAULT_CAM = (180.0, -15.0, 0.40)      # fixed base: looking straight on from the palm side
BOTH_CAM = (180.0, -15.0, 0.72)         # both hands: palms 0.30 m apart, so back off a little for both to fit
                                        # in the frame (the left side also has to leave room for the
                                        # joint-angle table)
FIX_CAM_SHIFT = 0.10                     # the joint-angle table occupies the top left: move the look-at point
                                         # 0.1×distance toward the left of the frame so the hand lands right
                                         # of the table (about 12% of viewport height in pixels)
HAND_GROUP = {"left": 3, "right": 4}      # each hand's visual geometry has its own geomgroup; the Cartesian
                                          # view toggles them per hand
LAYOUTS = ("split", "cart", "fixed")
LOST_ALPHA = 0.28
OFFLINE_ALPHA = 0.35
GRID_HALF_M = 1.0
GRID_STEP_M = 0.1

KEYS_COMMON = "F follow  V layout  C reset view"
KEYS_MOUSE = "drag rotate  right-drag pan  scroll zoom"
KEYS_TACTILE = "Left/Right point  PgUp/PgDn finger  Up/Down scale\nHome baseline  Delete clear  Insert print"
HUD_DESIGN_HEIGHT = 820                  # overlay laid out for 100% font size and a framebuffer this tall:
                                         # joint table 22 rows + 4–6 bottom rows, 23 px line spacing


def font_scale_pct(content_scale: float, fb_height: int) -> int:
    """Overlay font size step (100 / 150 / 200): taken from the display scaling, but no larger than the step
    that fits the framebuffer height.

    At font size 200 the line spacing is 42 px, so 820 px of height fits only 19 rows: the joint table
    overlaps the bottom status rows, and with both hands the table header also overlaps the shortcut-key row
    at the top right (this is what happens when opening the default window on a laptop with X11 and 200%
    display scaling)."""
    s = min(float(content_scale), fb_height / HUD_DESIGN_HEIGHT)
    return 200 if s >= 1.75 else (150 if s >= 1.25 else 100)


def hidpi_window_size(size, content_scale: float, fb_ratio: float, workarea=None) -> tuple[int, int] | None:
    """On X11, GLFW opens windows in physical pixels and does not follow the display scaling: when the scaling
    is ≥1.25 and the framebuffer is the same size as the window, enlarge the window by the scaling, then
    shrink it proportionally to fit within 90% of the work area. On Wayland / macOS the window size is already
    a logical size (the framebuffer is an integer multiple of the window), so it is left as is.
    Returns the resized dimensions; returns None when no change is needed."""
    if content_scale < 1.25 or fb_ratio > 1.1:
        return None
    w, h = size[0] * content_scale, size[1] * content_scale
    if workarea and workarea[2] > 0 and workarea[3] > 0:
        k = min(1.0, 0.9 * workarea[2] / w, 0.9 * workarea[3] / h)
        w, h = w * k, h * k
    return (int(w), int(h)) if int(w) > size[0] else None


def parse_cam(text: str | None, multi: bool = False):
    """--cam "azimuth,elevation,distance" → 3-tuple; the default is used when not given."""
    if not text:
        return BOTH_CAM if multi else DEFAULT_CAM
    az, el, dist = (float(v) for v in text.split(","))
    return az, el, dist


def assign_hand_groups(mujoco, m, hands) -> None:
    """Move each hand's visual geometry (URDF visual, group 1) to that hand's own geomgroup; collision
    geometry (group 0) is left unchanged."""
    for g in range(m.ngeom):
        if int(m.geom_group[g]) == 0:
            continue
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, int(m.geom_bodyid[g])) or ""
        for hand in hands:
            if name.startswith(hand + "_"):
                m.geom_group[g] = HAND_GROUP[hand]


def geometry_note(status: dict | None, demo: bool = False) -> str:
    """Tracker state in the SDK status → one line at the bottom of the Cartesian view (ASCII)."""
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
        pending = int(geo.get("pending") or 0)            # still within the persistence period: not passed,
                                                          # just not classified yet
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


def tracker_alert(status: dict | None, demo: bool = False) -> str:
    """Keep only the abnormal parts of geometry_note (Tracker stopped, geometry check failed, startup gate);
    normal and "still within the persistence period" are not reported."""
    note = geometry_note(status, demo)
    if not note or note == "demo":
        return ""
    parts = [p for p in note.split("  |  ") if p != "geometry ok" and not p.endswith(" pending")]
    return "  |  ".join(parts)


def fmt_stamp(ns) -> str:
    """wall ns → Unix seconds (to the microsecond). -- when absent."""
    if ns is None:
        return "--"
    return f"{int(ns) // 1_000_000_000}.{int(ns) % 1_000_000_000 // 1000:06d}"


def fmt_latency(ms) -> str:
    return "--" if ms is None else f"{ms:.1f} ms"


def stream_row(hz: float, count: int, sample_ns, latency_ms, note: str = "") -> str:
    """One row for a data stream: frame rate / sampling timestamp / arrival latency (``note`` follows the
    latency)."""
    if not count:
        return "no data yet"
    row = f"{hz:5.1f} Hz    sample {fmt_stamp(sample_ns)}    latency {fmt_latency(latency_ms)}"
    return f"{row} {note}" if note and latency_ms is not None else row


def fmt_quat(q) -> str:
    """(w, x, y, z) → 'w +0.7071  x +0.0000  y -0.7071  z +0.0000' (with a proportional font, alignment relies
    on the in-line labels)."""
    return "   ".join(f"{k} {float(v):+.4f}" for k, v in zip("wxyz", q))


def rpy_deg(R) -> tuple[float, float, float]:
    """Rotation matrix → (roll, pitch, yaw) in degrees, ZYX order."""
    pitch = -math.asin(max(-1.0, min(1.0, float(R[2][0]))))
    roll = math.atan2(float(R[2][1]), float(R[2][2]))
    yaw = math.atan2(float(R[1][0]), float(R[0][0]))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class RateMeter:
    """Computes the actual frame rate once per second from the difference in frame counts."""

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
    """All state of one hand in both views."""

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
        self.cart = None                       # most recent valid palm_link pose (p, R)
        self.cart_t = None                     # monotonic seconds when it arrived
        self.pose_q = None                     # Tracker quaternion (w, x, y, z) from the SDK broadcast;
                                               # this is what the overlay shows
        self.pose_age = None
        self.pose_state = "none"               # live / lost / none
        self.pose_stream = {"count": 0, "sample_ns": None, "latency_ms": None}
        self.rate = RateMeter()
        self.pose_rate = RateMeter()

    def build_tactile(self, layout, args, log=print) -> None:
        """Place the layout and build the heat point field only when tactile is enabled (placement prefers the
        placement table shipped with the package; when the model does not match it, a fit is run, about 10 s,
        cached afterwards)."""
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
    """Scene state + per-frame update + assembly of the two scenes + overlay text + drawing to a given
    framebuffer. Shared by the window and offscreen modes."""

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
            opt.geomgroup[0] = 0               # collision geometry is not drawn
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
        self.show_tactile = False              # J: the joint-angle table at top left is replaced
                                               # by the current hand's tactile table
        self.snaps = {}
        self.geometry_note = ""
        # Optional joint-angle preview (see app.load_preview_ext): when it is on, the transformed q is written
        # into the model; the joint table still shows the raw glove readings, with an extra delta column
        # (after − before the transform, degrees).
        self.preview = preview
        self.preview_xf = {h.hand: preview.bind(h.hand, h.fk.specs) for h in hands} if preview is not None else {}
        self.preview_delta = {h.hand: [None] * len(h.fk.specs) for h in hands}

    # ------------------------------------------------------------------ static information sent by the SDK

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

    def phi_pack_ms(self, hand: str):
        """Pack-delay calibration (ms) configured in the SDK for this hand's Tracker; None if not configured
        (or not reported by the SDK)."""
        serial = (self.tracker_info().get("roles") or {}).get(hand)
        val = (self.tracker_info().get("phi_pack_ms") or {}).get(serial)
        return float(val) if isinstance(val, (int, float)) and math.isfinite(val) else None

    def pose_latency_partial(self, hand: str) -> bool:
        """Pose arrival latency excludes the pack delay (the Tracker has no pack-delay calibration
        configured). Demo data, whose sampling time is its arrival time, does not count."""
        return self.product().get("source") != "demo" and self.phi_pack_ms(hand) is None

    def pose_latency_ms(self, hand: str, latency_ms):
        """Pose arrival latency for display: for a Tracker without a pack-delay calibration, the host fixed
        overhead ``TRACKER_HOST_FIXED_MS`` is added."""
        if latency_ms is None or not self.pose_latency_partial(hand):
            return latency_ms
        return latency_ms + TRACKER_HOST_FIXED_MS

    # ------------------------------------------------------------------ camera

    def reset_fixed_cam(self) -> None:
        az, el, dist = parse_cam(self.args.cam, self.multi)
        a, k = math.radians(az), FIX_CAM_SHIFT * dist           # screen right = (sin az, −cos az, 0)
        self.cam_fix.lookat[:] = (-k * math.sin(a), k * math.cos(a), 0.10)
        self.cam_fix.azimuth, self.cam_fix.elevation, self.cam_fix.distance = az, el, dist

    def posed(self) -> list:
        return [h for h in self.hands if h.cart is not None]

    def palm_mid(self):
        posed = self.posed()
        if not posed:
            return None
        return sum(h.cart[0] for h in posed) / len(posed)

    def reset_cart_cam(self) -> None:
        """Aim at the midpoint of the palms; the azimuth follows "the same viewpoint as the fixed-base view":
        facing the palm surface, with the left hand on the right side of the frame.
        With both hands it is set by the left→right palm line (insensitive to palm orientation); with one
        hand, by the horizontal projection of the palm normal."""
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
            n = posed[0].cart[1][:, 0]            # palm normal = x axis of the palm_link frame
            if math.hypot(n[0], n[1]) > 0.3:
                cam.azimuth = math.degrees(math.atan2(n[1], n[0])) + 180.0
            cam.distance = 0.6
        cam.elevation = -20.0
        self.cart_cam_ready = True

    # ------------------------------------------------------------------ interaction

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
            if not any(hk.tactile.enabled for hk in self.hands):
                self.note("no tactile on this product")
            else:
                self.show_tactile = not self.show_tactile
                self.note("table -> tactile" if self.show_tactile else "table -> joints")
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

    # ------------------------------------------------------------------ per frame

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
            h.fk.update(self._preview_q(h.hand, s["q"]), s["deg"], s["mask"])    # finger joints into d_fix
            h.rate.update(s["count"], now)
            h.pose_stream = self.live.pose_stream(h.hand)
            h.pose_rate.update(h.pose_stream["count"], now)
        mj.mj_forward(self.m, self.d_fix)
        for h in self.hands:
            h.fk.write(self.d_cart)                     # Cartesian copy: finger joints bit-identical
            pose = self.live.pose(h.hand) if self.tracker_on() else None
            if pose is not None:
                pos, q, age = pose
                h.pose_age = age
                h.pose_q = q
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
        """When the preview is on, return the transformed q (and record the per-joint delta); otherwise return
        q unchanged."""
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

    # ------------------------------------------------------------------ scenes

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
        """SDK disconnected: hands and hot spots fade as a whole (frozen at the last frame)."""
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
        """Grid on the z=0 plane (snapped to world coordinates in 0.1 m steps, so the grid does not move along
        with the hands) + world axes at the grid corner + each hand's palm_link coordinate axes and a plumb
        line to the ground."""
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

    # ------------------------------------------------------------------ overlay text

    def _tag(self, h: HandState, what: str) -> str:
        return f"{h.tag} {what}" if self.multi else what

    def _port(self, hand: str) -> str:
        return str(((self.hello.get("gloves") or {}).get(hand) or {}).get("port") or "?")

    def glove_rows(self) -> tuple[list, list]:
        """One glove data stream row per hand (frame rate / sampling timestamp / arrival latency); an extra !!
        row when the data stream is interrupted or the kit has stopped."""
        left, right = [], []
        for h in self.hands:
            s = self.snaps.get(h.hand) or {}
            left.append(self._tag(h, "glove"))
            right.append(stream_row(h.rate.hz, s.get("count", 0), s.get("sample_ns"), s.get("latency_ms")))
            if s.get("error"):
                left.append(f"!! {h.tag}")
                right.append(f"stream stopped: {_ascii(s['error'])}")
        if (self.status or {}).get("kit_fatal"):
            left.append("!!")
            right.append(f"kit stopped: {_ascii(self.status['kit_fatal'])}")
        return left, right

    def pose_rows(self) -> tuple[list, list]:
        """One pose stream row per hand that has a Tracker; a single row with the reason when the Tracker is
        not enabled, and an extra !! row when the Tracker has a problem."""
        if not self.tracker_on():
            return ["pose"], [f"none: {_ascii(self.tracker_reason())}"]
        roles = self.tracker_info().get("roles") or {}
        left, right = [], []
        for h in self.hands:
            if h.hand not in roles:
                continue
            ps = h.pose_stream
            left.append(self._tag(h, "pose"))
            right.append(stream_row(h.pose_rate.hz, ps["count"], ps["sample_ns"],
                                    self.pose_latency_ms(h.hand, ps["latency_ms"]),
                                    "(excl. pack delay)" if self.pose_latency_partial(h.hand) else ""))
        alert = tracker_alert(self.status, self.product().get("source") == "demo")
        if alert:
            left.append("!!")
            right.append(_ascii(alert))
        return left, right

    def quat_rows(self, now: float) -> tuple[list, list]:
        """Cartesian view, top left: one quaternion row per hand that has a Tracker (Tracker orientation from
        the SDK broadcast, w x y z)."""
        roles = self.tracker_info().get("roles") or {}
        left, right = [], []
        for h in self.hands:
            if h.hand not in roles:
                continue
            left.append(self._tag(h, "quat"))
            if h.pose_state == "none" or h.pose_q is None:
                right.append("waiting for first pose")
            elif h.pose_state == "live":
                right.append(fmt_quat(h.pose_q))
            else:
                right.append(f"{fmt_quat(h.pose_q)}    LOST {now - h.cart_t:.1f}s")
        return left, right

    def fixed_texts(self, now: float) -> list:
        mj = self.mujoco
        out = []
        h = self.cur()
        if self.show_tactile and h.tactile.enabled:
            left, right = self.tactile_table(h)
        else:
            left, right = self.joint_table()
        out.append((mj.mjtGridPos.mjGRID_TOPLEFT, "\n".join(left), "\n".join(right)))
        left, right = self.glove_rows()
        if self.layout_mode == "fixed":                  # Cartesian view off screen: pose streams go here
            pl, pr = self.pose_rows()
            left, right = left + pl, right + pr
        pv = self.preview_lines()                        # long text goes in the right column: in the
                                                         # left column it would push the entire right
                                                         # column off screen
        out.append((mj.mjtGridPos.mjGRID_BOTTOMLEFT, "\n".join([""] * len(pv) + left), "\n".join(pv + right)))
        return out

    def tactile_table(self, h: HandState) -> tuple[list, list]:
        """Tactile table when J is on (current hand): active point count / max / sum per finger + the selected
        point."""
        left = [f"tactile {h.tag}", "finger  act"]
        right = ["", "      max       sum"]
        for f, act, mx, sm, _am in finger_stats(self.layout, h.shown, self.args.threshold):
            left.append(f"{f:<7} {act:3d}")
            right.append(f"{mx:9.0f} {sm:9.0f}")
        p = self.layout.points[self.sel]
        tac = self.snaps.get(h.hand, {}).get("tac")
        raw = tac[self.sel] if tac is not None and len(tac) == self.n_points else "--"
        val = f"{h.shown[self.sel]:.0f}" if h.shown is not None else "--"
        left += ["", f"sel {self.sel:3d} {p.finger}", f"  {p.zone} pt{p.point}"]
        right += ["", f"raw {raw}", f"shown {val}"]
        return left, right

    def joint_table(self) -> tuple[list, list]:
        """Joint-angle table (degrees, readings reported by the glove): joint names in the left column, one
        column per hand in the right column, -- = no valid value received yet.
        mjr_overlay allows at most mjMAXOVERLAY=500 characters per text block: when the preview is on and adds
        a delta column, only the current hand is listed."""
        if self.preview_on():
            h = self.cur()
            delta = self.preview_delta.get(h.hand) or []
            left = [f"joint {h.tag}" if self.multi else "joint"]
            right = ["   deg  delta"]
            for k, (name, fw, _urdf, _seen) in enumerate(h.fk.joint_rows()):
                left.append(name)
                d = delta[k] if k < len(delta) else None
                right.append((f"{fw:6.1f}" if fw is not None else "    --")
                             + (f" {d:+6.1f}" if d is not None else "     --"))
            return left, right
        rows = [h.fk.joint_rows() for h in self.hands]
        left = ["joint"] + [name for name, *_ in rows[0]]
        right = ["   ".join(f"{h.tag + ' deg':>7}" for h in self.hands)]
        for k in range(len(rows[0])):
            right.append("   ".join((f"{r[k][1]:7.1f}" if k < len(r) and r[k][1] is not None else "     --")
                                    for r in rows))
        return left, right

    def preview_lines(self) -> list[str]:
        """Status lines of the preview extension (empty when there is no extension)."""
        text = self.preview.overlay() if self.preview is not None else ""
        return [_ascii(text)] if text else []

    def cart_texts(self, now: float) -> list:
        mj = self.mujoco
        out = []
        if self.tracker_on():
            left, right = self.quat_rows(now)
            out.append((mj.mjtGridPos.mjGRID_TOPLEFT, "\n".join(left), "\n".join(right)))
        left, right = self.pose_rows()
        if self.layout_mode == "cart":                   # fixed-base view off screen: glove streams and
                                                         # preview status go here
            gl, gr = self.glove_rows()
            pv = self.preview_lines()
            left, right = [""] * len(pv) + gl + left, pv + gr + right
        out.append((mj.mjtGridPos.mjGRID_BOTTOMLEFT, "\n".join(left), "\n".join(right)))
        return out

    def keys_text(self, now: float) -> str:
        tactile = any(h.tactile.enabled for h in self.hands)
        line = (f"{self.preview.keys_hint}  " if self.preview is not None else "") + KEYS_COMMON \
            + ("  J tactile" if tactile else "")
        if self.multi and (self.show_tactile or self.preview_on()):
            line = "Tab hand  " + line
        txt = line + "\n" + KEYS_MOUSE
        if tactile and self.show_tactile:
            txt += "\n" + KEYS_TACTILE
        if self.msg and now - self.msg_t < 4.0:
            txt = self.msg + "\n" + txt
        return txt

    # ------------------------------------------------------------------ drawing to the framebuffer

    def viewports(self, width: int, height: int) -> list:
        """[(name, MjrRect)]; Cartesian on the left, fixed base on the right."""
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
        if not self.connected:                  # draw the banner only after both views are drawn,
                                                # otherwise the right half would cover it
            grid = getattr(mj.mjtGridPos, "mjGRID_TOP", mj.mjtGridPos.mjGRID_TOPLEFT)
            reason = _ascii(self.live.disconnect_reason or "connection lost")
            mj.mjr_overlay(mj.mjtFont.mjFONT_BIG, grid, mj.MjrRect(0, 0, width, height),
                           "SDK DISCONNECTED - " + reason, "showing the last frame, waiting for the SDK", ctx)


def _ascii(text) -> str:
    return "".join(c if 32 <= ord(c) < 127 else "?" for c in str(text))


def render_waiting(mujoco, ctx, width: int, height: int, text: str) -> None:
    """Screen shown when there is no scene yet (not connected to the SDK / scene being built): clear + a
    centered message."""
    mj = mujoco
    full = mj.MjrRect(0, 0, width, height)
    mj.mjr_rectangle(full, 0.12, 0.13, 0.15, 1.0)
    mj.mjr_overlay(mj.mjtFont.mjFONT_NORMAL, mj.mjtGridPos.mjGRID_TOPLEFT, full, text, "", ctx)


# ----------------------------------------------------------------------------- offscreen snapshot

def render_png(view: DualView, path, size=(1600, 800)) -> None:
    """Render both views (per the current layout) offscreen into one PNG. Without a display, the caller sets
    MUJOCO_GL=egl."""
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


# ----------------------------------------------------------------------------- window

class _Mouse:
    def __init__(self):
        self.left = self.right = self.middle = False
        self.x = self.y = 0.0
        self.owner = None


def window_keymap(glfw, preview=None) -> dict:
    """GLFW key code → DualView.key() name. With a preview extension, its toggle key is included as well (name
    PREVIEW)."""
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
    """GLFW window main loop (rendering on the main thread; the subscription thread only writes LiveState).
    stop: threading.Event.

    The window is there as soon as it opens: while the SDK is not up it draws the waiting screen; after hello
    is received and the scene is built, it switches to the dual view; when the scene changes (the SDK switched
    product model / switched hands), the render context is rebuilt for the current model."""
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
    try:
        workarea = glfw.get_monitor_workarea(glfw.get_primary_monitor())
    except Exception:                                   # older GLFW versions have no work-area API:
                                                        # only enlarge by the scaling
        workarea = None
    ww0, _wh0 = glfw.get_window_size(win)
    fit = hidpi_window_size(size, glfw.get_window_content_scale(win)[0],
                            glfw.get_framebuffer_size(win)[0] / max(ww0, 1), workarea)
    if fit:
        glfw.set_window_size(win, *fit)

    def font_for(fh):                                   # the step changes after the window is rescaled
                                                        # or moved to another display
        pct = font_scale_pct(glfw.get_window_content_scale(win)[0], fh)
        return getattr(mj.mjtFontScale, f"mjFONTSCALE_{pct}")

    fs = font_for(glfw.get_framebuffer_size(win)[1])
    blank = mj.MjModel.from_xml_string("<mujoco/>")      # the waiting screen only needs the font
    ctx = mj.MjrContext(blank, fs)
    ctx_view = None                                      # which scene ctx was built for
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
            fw, fh = glfw.get_framebuffer_size(win)
            want = font_for(fh) if fh > 0 else fs
            if controller.view is not ctx_view or want != fs:   # scene changed / font step changed:
                                                                # rebuild the render context
                ctx.free()
                if controller.view is not ctx_view:
                    mouse.owner = None
                ctx_view, fs = controller.view, want
                ctx = mj.MjrContext(ctx_view.m if ctx_view is not None else blank, fs)
            view = cur_view()
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


# ----------------------------------------------------------------------------- headless report

def report_headless(view: DualView, seconds: float, stop, log=print) -> int:
    """Run for ``seconds`` seconds, then print per-hand statistics. Return code: 0 normal / 2 some hand
    received no frames at all / 3 tactile enabled but no tactile frames received."""
    view.update()
    base = {h.hand: (view.snaps[h.hand]["count"], view.snaps[h.hand]["tac_count"], h.pose_stream["count"])
            for h in view.hands}
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
        log(f"{pre}手套时间戳：最近采样刻 {fmt_stamp(s.get('sample_ns'))}，"
            f"到达延迟 {fmt_latency(s.get('latency_ms'))}（最近 1 s 中位数）")
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
            ps = h.pose_stream
            n_pose = ps["count"] - base[h.hand][2]
            log(f"{pre}笛卡尔：{h.pose_state}  palm_link xyz {p[0]:+.3f} {p[1]:+.3f} {p[2]:+.3f} m  "
                f"rpy {r:+.1f} {pch:+.1f} {y:+.1f}°")
            pack_ms = view.phi_pack_ms(h.hand)
            note = ("；含主机固定开销、不含 Tracker 打包延迟" if view.pose_latency_partial(h.hand)
                    else f"，含 Tracker 打包延迟 {pack_ms:g} ms" if pack_ms is not None else "")
            log(f"{pre}位姿：{n_pose} 帧 / {dt:.1f}s（{n_pose / max(dt, 1e-6):.1f} Hz），四元数 {fmt_quat(h.pose_q)}，"
                f"最近采样刻 {fmt_stamp(ps['sample_ns'])}，到达延迟 {fmt_latency(view.pose_latency_ms(h.hand, ps['latency_ms']))}（最近 1 s 中位数{note}）")
    return rc
