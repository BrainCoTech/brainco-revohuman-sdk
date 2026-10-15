"""一次「SDK 会话」对应的可视化场景：收到 hello 才按它建模型、判触觉、建双视图。

hello 里的型号、手、关节表、触觉外设结论决定场景长什么样；同一份配置的 SDK 重启（重连后 hello
内容不变）不重建，只接着更新；配置变了（换了型号 / 换了手 / 触觉结论变了）才重建。
建场景要组装 URDF、把触觉布局贴到手模型上（首次十几秒，之后走缓存），窗口模式下放在后台线程里做，
窗口照常刷新等待画面。
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass

log = logging.getLogger("revohuman_sim")


# ----------------------------------------------------------------------------- 触觉启用判定

@dataclass
class TactileDecision:
    enabled: bool
    reason: str                   # ASCII，覆盖层显示
    unit: str | None = None
    point_count: int | None = None


def decide_tactile(*, variant_code: str, variant_has_tactile: bool, hand_has_tactile: bool,
                   hand_note: str = "", point_count=None, unit=None, layout_points: int,
                   want: str = "auto") -> TactileDecision:
    """一只手的触觉热力图开不开。只看 SDK 下发的外设识别结果，不看数据流里有没有非零值：

      型号不带触觉（P1）            → 关
      型号带、这只手在位硬件没报触觉   → 关（固件外设探测的结论，原因照抄 hand_note）
      固件报的点数 ≠ 布局点数         → 关（布局是按 241 点的触觉阵列画的，点数对不上画出来是错的）
      --tactile off                 → 关
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
    """hello → {hand: TactileDecision}。"""
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
    """决定场景结构的那部分 hello：相同就不重建。"""
    joints = hello.get("joints") or {}
    return (str((hello.get("product") or {}).get("code")), tuple(hello.get("hands") or ()),
            tuple((h, tuple((int(j["index"]), str(j["name"])) for j in joints.get(h) or ())) for h in sorted(joints)),
            tuple((h, d.enabled, d.point_count) for h, d in sorted(decisions.items())))


def check_hello(hello: dict) -> None:
    """hello 缺了建场景要用的东西就抛 ValueError（原因写给人看）。"""
    hands = hello.get("hands") or []
    if not hands:
        raise ValueError("SDK 广播里没有手")
    for h in hands:
        if h not in ("left", "right"):
            raise ValueError(f"不认识的手：{h!r}")
        if not (hello.get("joints") or {}).get(h):
            raise ValueError(f"SDK 广播里没有 {h} 手的关节表")


# ----------------------------------------------------------------------------- 场景

def build_view(mujoco, np, hello: dict, decisions: dict, layout, args, gen: int, live, log_fn=print, preview=None):
    """按 hello 建双视图（不需要 GL 上下文）。preview：可选的关节角预览（见 app.load_preview_ext）。"""
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
    """跟着 LiveState 的 hello 维护当前场景。窗口模式在后台线程建场景（``background=True``）。"""

    def __init__(self, mujoco, np, live, client, layout, args, background: bool = True, preview=None):
        self.mujoco, self.np = mujoco, np
        self.preview = preview                 # 跨场景重建保留（开关状态跟着它走）
        self.live = live
        self.client = client
        self.layout = layout
        self.args = args
        self.background = background
        self.view = None
        self.key = None
        self.gen = 0
        self.building = None                   # (gen, 型号) 正在建
        self.build_error = ""
        self._lock = threading.Lock()
        self._done = None
        self._thread = None

    def poll(self) -> bool:
        """看有没有新 hello、后台有没有建好。返回 True = 当前场景换了（窗口要重建渲染上下文）。"""
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
                self.gen = gen                   # 这一份 hello 建不起来：等下一份
                self.build_error = err
        hello, gen, _connected = self.live.session()
        if hello is None or gen == self.gen or self.building is not None:
            return changed
        decisions = tactile_decisions(hello, self.layout.point_count, self.args.tactile)
        key = session_key(hello, decisions)
        if self.view is not None and key == self.key:
            self.gen = gen                       # SDK 重启但配置没变：接着用原场景
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
        except Exception as exc:                 # noqa: BLE001  建不起来：等下一份 hello，窗口不退
            log.exception("按 SDK 的 hello 建场景失败")
            with self._lock:
                self._done = (None, key, gen, f"{type(exc).__name__}: {exc}")
            return
        log.info("场景就绪（%.1f s）", time.monotonic() - t0)
        with self._lock:
            self._done = (view, key, gen, "")

    def waiting_text(self) -> str:
        """还没有场景时的等待画面（ASCII）。"""
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
    """覆盖层字体只有 ASCII：非 ASCII 字符换成 ?（完整中文在控制台）。"""
    return "".join(c if 32 <= ord(c) < 127 else "?" for c in str(text))
