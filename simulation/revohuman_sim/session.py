"""The visualization scene for one "SDK session": the model is built, tactile is decided and the dual view is
built only once a hello has been received, based on that hello.

The product model, hands, joint table and tactile peripheral result in hello determine what the scene looks
like; an SDK restart with the same configuration (hello content unchanged after reconnecting) does not rebuild
the scene, it just keeps updating; only a configuration change (different product model / different hands /
changed tactile result) triggers a rebuild.
Building the scene means assembling the URDF and placing the tactile layout onto the hand model (a dozen-plus
seconds the first time, cached afterwards); in window mode this runs in a background thread while the window
keeps refreshing the waiting screen.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass

log = logging.getLogger("revohuman_sim")


# ----------------------------------------------------------------------------- tactile enable decision

@dataclass
class TactileDecision:
    enabled: bool
    reason: str                   # ASCII, shown in the overlay
    unit: str | None = None
    point_count: int | None = None


def decide_tactile(*, variant_code: str, variant_has_tactile: bool, hand_has_tactile: bool,
                   hand_note: str = "", point_count=None, unit=None, layout_points: int,
                   want: str = "auto") -> TactileDecision:
    """Whether one hand's tactile heatmap is on. Only the peripheral detection results sent by the SDK are
    considered, not whether the data stream contains non-zero values:

      product model has no tactile (P1)               → off
      model has tactile, but the hardware present on  → off (result of the firmware's peripheral detection;
      this hand did not report tactile                  reason copied verbatim from hand_note)
      point count reported by firmware ≠ layout       → off (the layout is drawn for a 241-point tactile
      point count                                       array; with a mismatched count the drawing is wrong)
      --tactile off                                   → off
    """
    if want == "off":
        return TactileDecision(False, "off (--tactile off)")
    if not variant_has_tactile:
        return TactileDecision(False, f"none ({variant_code} has no tactile)")
    if not hand_has_tactile:
        log.warning("型号 %s 应带触觉，但这只手的外设识别没报触觉：%s", variant_code, hand_note or "无说明")
        return TactileDecision(False, "off (firmware reports no tactile module)")
    if point_count is None:
        return TactileDecision(False, "off (tactile info unavailable)")
    n = int(point_count or 0)
    if n != layout_points:
        return TactileDecision(False, f"off ({n} points != layout {layout_points})", unit, n)
    return TactileDecision(True, f"on ({n} points, {unit})", unit, n)


def tactile_decisions(hello: dict, layout_points: int, want: str = "auto") -> dict:
    """hello → {hand: TactileDecision}."""
    code = str((hello.get("product") or {}).get("code") or "?")
    out = {}
    for h in hello.get("hands") or ():
        t = (hello.get("tactile") or {}).get(h) or {}
        out[h] = decide_tactile(variant_code=code, variant_has_tactile=bool(t.get("variant_has")),
                                hand_has_tactile=bool(t.get("hand_has")), hand_note=str(t.get("note") or ""),
                                point_count=t.get("point_count"), unit=t.get("unit"),
                                layout_points=layout_points, want=want)
    return out


def session_key(hello: dict, decisions: dict) -> tuple:
    """The part of hello that determines the scene structure: if it is unchanged, the scene is not rebuilt."""
    joints = hello.get("joints") or {}
    return (str((hello.get("product") or {}).get("code")), tuple(hello.get("hands") or ()),
            tuple((h, tuple((int(j["index"]), str(j["name"])) for j in joints.get(h) or ())) for h in sorted(joints)),
            tuple((h, d.enabled, d.point_count) for h, d in sorted(decisions.items())))


def check_hello(hello: dict) -> None:
    """Raise ValueError if hello lacks something needed to build the scene
    (the reason is written for humans)."""
    hands = hello.get("hands") or []
    if not hands:
        raise ValueError("SDK 广播里没有手")
    for h in hands:
        if h not in ("left", "right"):
            raise ValueError(f"不认识的手：{h!r}")
        if not (hello.get("joints") or {}).get(h):
            raise ValueError(f"SDK 广播里没有 {h} 手的关节表")


# ----------------------------------------------------------------------------- scene

def build_view(mujoco, np, hello: dict, decisions: dict, layout, args, gen: int, live, log_fn=print, preview=None):
    """Build the dual view from hello (no GL context needed). preview: optional joint-angle preview (see
    app.load_preview_ext)."""
    from .hand_model import load_model
    from .view import DualView, HandState
    check_hello(hello)
    hands = list(hello["hands"])
    m, d_fix, _urdf = load_model(hands, args.urdf_dir, args.spacing)
    d_cart = mujoco.MjData(m)
    d_cart.qpos[:] = d_fix.qpos
    mujoco.mj_forward(m, d_cart)
    states = [HandState(np, m, d_fix, h, hands, hello["joints"][h], args.spacing, decisions[h]) for h in hands]
    for s in states:
        s.build_tactile(layout, args, log=log_fn)
    view = DualView(mujoco, np, m, d_fix, d_cart, states, layout, live, args, preview=preview)
    view.gen = gen
    return view


class Controller:
    """Maintains the current scene following LiveState's hello. In window mode the scene is built in a
    background thread (``background=True``)."""

    def __init__(self, mujoco, np, live, client, layout, args, background: bool = True, preview=None):
        self.mujoco, self.np = mujoco, np
        self.preview = preview                 # kept across scene rebuilds (the on/off state goes with it)
        self.live = live
        self.client = client
        self.layout = layout
        self.args = args
        self.background = background
        self.view = None
        self.key = None
        self.gen = 0
        self.building = None                   # (gen, product model) being built
        self.build_error = ""
        self._lock = threading.Lock()
        self._done = None
        self._thread = None

    def poll(self) -> bool:
        """Check for a new hello and whether the background build has finished. Returns True = the current
        scene changed (the window must rebuild its render context)."""
        changed = False
        with self._lock:
            done, self._done = self._done, None
        if done is not None:
            view, key, gen, err = done
            self.building = None
            if view is not None:
                self.view, self.key, self.gen = view, key, gen
                self.build_error = ""
                changed = True
            else:
                self.gen = gen                   # this hello could not be built: wait for the next one
                self.build_error = err
        hello, gen, _connected = self.live.session()
        if hello is None or gen == self.gen or self.building is not None:
            return changed
        decisions = tactile_decisions(hello, self.layout.point_count, self.args.tactile)
        key = session_key(hello, decisions)
        if self.view is not None and key == self.key:
            self.gen = gen                       # SDK restarted but configuration unchanged: keep using the
                                                 # existing scene
            self.view.gen = gen
            log.info("SDK 重新连上（配置不变），接着显示")
            return changed
        self._report(hello, decisions)
        self.building = (gen, str((hello.get("product") or {}).get("code")))
        if self.background:
            self._thread = threading.Thread(target=self._build, args=(hello, decisions, key, gen), daemon=True,
                                            name="viewer-build")
            self._thread.start()
            return changed
        self._build(hello, decisions, key, gen)
        return self.poll() or changed

    def _report(self, hello: dict, decisions: dict) -> None:
        prod = hello.get("product") or {}
        gloves = hello.get("gloves") or {}
        print(f"[viewer] SDK {hello.get('sdk')}（pid {hello.get('pid')}）：型号 {prod.get('code')}"
              f"（{prod.get('name')}，识别来源 {prod.get('source') or '-'}），"
              + "，".join(f"{h} {(gloves.get(h) or {}).get('port') or '-'}" for h in hello.get("hands") or ()),
              flush=True)
        for h, d in decisions.items():
            print(f"[viewer] {h} 触觉：{d.reason}", flush=True)
        trk = hello.get("tracker") or {}
        if trk.get("enabled") and self.args.tracker != "off":
            print(f"[viewer] 笛卡尔视图：tracker {trk.get('roles')}", flush=True)
        else:
            print(f"[viewer] 笛卡尔视图不启用：{cart_reason(hello, self.args.tracker)}", flush=True)

    def _build(self, hello, decisions, key, gen) -> None:
        t0 = time.monotonic()
        try:
            view = build_view(self.mujoco, self.np, hello, decisions, self.layout, self.args, gen, self.live,
                              preview=self.preview)
        except Exception as exc:                 # noqa: BLE001  build failed: wait for the next hello, the
                                                 # window stays open
            log.exception("按 SDK 的 hello 建场景失败")
            with self._lock:
                self._done = (None, key, gen, f"{type(exc).__name__}: {exc}")
            return
        log.info("场景就绪（%.1f s）", time.monotonic() - t0)
        with self._lock:
            self._done = (view, key, gen, "")

    def waiting_text(self) -> str:
        """Waiting screen when there is no scene yet (ASCII)."""
        hello, _gen, connected = self.live.session()
        lines = ["RevoHuman viewer", ""]
        if self.building is not None:
            lines += [f"SDK connected ({self.building[1]}), building the scene ...",
                      "(first run fits the tactile layout to the hand model; cached afterwards)"]
        elif connected and hello is not None and self.build_error:
            lines += ["cannot build the scene from the SDK broadcast:", f"  {_ascii(self.build_error)}"]
        else:
            lines += ["waiting for the SDK broadcast ...", f"  socket: {self.client.describe()}", "",
                      "start it in another terminal (in the SDK's environment):",
                      "  revohuman-live      (or: python3 -m revohuman live)"]
            reason = self.live.disconnect_reason
            if reason:
                lines += ["", f"last: {_ascii(reason)}"]
        if self.live.error:
            lines += ["", f"!! {_ascii(self.live.error)}"]
        return "\n".join(lines)

    def close(self) -> None:
        if self._thread is not None:
            self._thread.join(timeout=0.1)


def cart_reason(hello: dict | None, want: str = "auto") -> str:
    if want == "off":
        return "off (--tracker off)"
    trk = (hello or {}).get("tracker") or {}
    if trk.get("enabled"):
        return ""
    return str(trk.get("reason") or "tracker not opened by the SDK")


def _ascii(text: str) -> str:
    """The overlay font is ASCII only: non-ASCII characters become ?
    (the full Chinese text is in the console)."""
    return "".join(c if 32 <= ord(c) < 127 else "?" for c in str(text))
