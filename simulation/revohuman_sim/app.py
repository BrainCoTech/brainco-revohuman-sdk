"""Glove visualization entry point: one MuJoCo window, with the Cartesian view (Tracker 6D pose) on the left
and the fixed-base view (joints + tactile heatmap) on the right.

The visualization only subscribes to the SDK's local broadcast; it does not import the SDK and does not touch
the serial ports or the Trackers:

    revohuman-live                           # terminal 1, in the SDK's environment: open the gloves,
                                             #   start broadcasting (same as python3 -m revohuman live)
    simulation/run_viewer.sh                 # terminal 2: creates an isolated venv, installs dependencies,
                                             #   opens the window, waits for the broadcast
    simulation/run_viewer.sh --demo          # preview without gloves (built-in demo broadcast, same protocol)
    python3 simulation/viewer.py --headless --seconds 3 --snapshot out.png

The window comes up first and waits; the scene is built and updates start only once the SDK is up and its
hello has been received. The product model (P1 / PTV1), the joint table, whether each hand has tactile, and
whether the Tracker is on all follow the SDK's detection results in the hello. After the SDK exits, the view
stays on the last frame and waits for the SDK to come back.
"""

from __future__ import annotations

import argparse
import importlib
import importlib.util
import json
import logging
import os
import pathlib
import signal
import sys
import threading
import time

from . import logcolor
from .heatmap import HeatParams
from .tactile_layout import DEFAULT_LAYOUT, TactileLayout

log = logging.getLogger("revohuman_sim")

PREVIEW_MODULE = "joint_preview"


def load_preview_ext():
    """Optional joint-angle preview extension (the ``joint_preview`` module in this package): loaded if
    present, otherwise returns None.

    Without this module no arguments are added and no keys are bound; the visualization behaves exactly as if
    this hook did not exist. If the module exists but fails to import, the error is raised as usual, not
    silently treated as "absent".

    Extension contract: ``add_arguments(parser)`` adds its own arguments; ``create(args)`` returns a preview
    object or None (raises ValueError on a bad configuration). The preview object must have: ``key`` (a single
    letter, toggles it in the window), ``keys_hint``, ``enabled``, ``bind(hand, joints)`` (returns a per-hand
    transform with ``apply(q) -> list``; q is in joint-table order, in rad, may contain None),
    ``toggle() -> str``, ``overlay() -> str``, ``describe() -> str``; all overlay text is ASCII."""
    name = f"{__package__ or 'revohuman_sim'}.{PREVIEW_MODULE}"
    try:
        spec = importlib.util.find_spec(name)
    except ModuleNotFoundError:
        spec = None
    return None if spec is None else importlib.import_module(name)


def build_parser() -> argparse.ArgumentParser:
    hp = HeatParams()
    ap = argparse.ArgumentParser(
        prog="run_viewer.sh",
        description="RevoHuman 手套可视化：笛卡尔视图（tracker 6D）+ 固定基座视图（关节 + 触觉热力图）。"
                    "数据来自 SDK 的本机广播（revohuman-live，即 python3 -m revohuman live）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    g = ap.add_argument_group("数据源")
    g.add_argument("--socket", default=None,
                   help="SDK 广播的 socket 路径；默认按 SDK 的同一套规则找"
                        "（$REVOHUMAN_LIVE_SOCKET > $XDG_RUNTIME_DIR/revohuman/live.sock > /tmp/revohuman-<uid>/live.sock）")
    g.add_argument("--wait", type=float, default=10.0, help="--headless 下最多等 SDK 广播多少秒")
    g.add_argument("--tactile", choices=("auto", "off"), default="auto",
                   help="auto = 型号带触觉且这只手的外设识别报了触觉才画热力图")
    g.add_argument("--tracker", choices=("auto", "off"), default="auto",
                   help="auto = SDK 那边 tracker 起来了就显示笛卡尔视图；off = 不显示（tracker 开不开由 SDK 端决定）")
    g.add_argument("--demo", nargs="?", const="PTV1", choices=("P1", "PTV1"), default=None,
                   help="不接硬件：起一个内置演示广播（默认 PTV1），可视化照常订阅它")
    g.add_argument("--demo-hands", choices=("both", "left", "right"), default="both", help="演示广播里有哪几只手")

    g = ap.add_argument_group("显示")
    g.add_argument("--view", choices=("split", "cart", "fixed"), default="split",
                   help="split = 左笛卡尔 / 右固定基座；运行中按 V 切换")
    g.add_argument("--cam", default=None, help="固定基座视图相机 azimuth,elevation,distance")
    g.add_argument("--fps", type=float, default=60.0)
    g.add_argument("--lite", action="store_true", help="关阴影/反射/天空盒，低端显卡用")
    g.add_argument("--no-bulk", action="store_true", help="热斑逐个写几何体（排查渲染问题用）")

    g = ap.add_argument_group("触觉热力图")
    g.add_argument("--scale", type=float, default=0.0, help="色标满量程；0 = 自动（峰值保持 + 慢衰减）")
    g.add_argument("--contact", type=float, default=hp.contact, help="接触阈值，低于它的区域不画")
    g.add_argument("--threshold", type=float, default=0.0, help="手指统计与点表的计数阈值")
    g.add_argument("--top", type=int, default=20, help="Insert / --headless 打印的点表行数")
    g.add_argument("--heat-pitch-mm", type=float, default=hp.heat_pitch_mm)
    g.add_argument("--heat-sigma-mm", type=float, default=hp.heat_sigma_mm)
    g.add_argument("--heat-reach-mm", type=float, default=hp.heat_reach_mm)
    g.add_argument("--heat-bulge-mm", type=float, default=hp.heat_bulge_mm)
    g.add_argument("--heat-max", type=int, default=hp.heat_max, help="每只手每帧最多画多少个细点")
    g.add_argument("--heat-cluster-mm", type=float, default=hp.heat_cluster_mm)
    g.add_argument("--heat-ring-pad-mm", type=float, default=hp.heat_ring_pad_mm)
    g.add_argument("--heat-ring-seg", type=int, default=hp.heat_ring_seg)
    g.add_argument("--no-heat-ring", dest="heat_ring", action="store_false", help="不画接触圈")
    g.add_argument("--layout", type=pathlib.Path, default=DEFAULT_LAYOUT, help="触觉布局 yaml")
    g.add_argument("--refit", action="store_true", help="忽略缓存与随包贴点表，按模型网格重新拟合（指节网格须带套环/套帽）")

    g = ap.add_argument_group("模型")
    g.add_argument("--urdf-dir", type=pathlib.Path, default=None,
                   help="手部模型目录（urdf/ + meshes/），默认源码包根目录 models/ 里的手部模型")
    g.add_argument("--spacing", type=float, default=0.30, help="固定基座视图里两掌的间距（米）")

    g = ap.add_argument_group("无窗口")
    g.add_argument("--headless", action="store_true", help="不开窗口：跑 --seconds 秒后打印统计")
    g.add_argument("--seconds", type=float, default=3.0)
    g.add_argument("--snapshot", type=pathlib.Path, default=None, help="--headless 结束时把两个视图存成 PNG")
    g.add_argument("--size", default="1600x800", help="--snapshot 的图像尺寸 WxH")
    g.add_argument("--dump-layout", type=pathlib.Path, default=None,
                   help="--headless 结束时把触觉点的位置与读数存成 JSON（每只手一个文件，文件名加 _left/_right）")
    ext = load_preview_ext()
    if ext is not None:
        ext.add_arguments(ap)
    ap.set_defaults(heat_ring=True)
    return ap


def _dump_layout(view, base: pathlib.Path) -> list[pathlib.Path]:
    out = []
    for h in view.hands:
        if h.placed is None:
            continue
        path = base if len(view.hands) == 1 else base.with_name(f"{base.stem}_{h.hand}{base.suffix or '.json'}")
        pts = []
        for p in view.layout.points:
            pts.append({"idx": p.idx, "finger": p.finger, "zone": p.zone, "point": p.point, "x": p.x, "y": p.y,
                        "link": p.link, "local_mm": [round(float(v) * 1000, 3) for v in h.placed.local[p.idx]],
                        "world_mm": [round(float(v) * 1000, 3) for v in h.placed.world[p.idx]],
                        "value": None if h.shown is None else float(h.shown[p.idx])})
        doc = {"hand": h.hand, "layout": str(view.layout.path), "point_count": view.layout.point_count,
               "ulnar_sign_y": h.placed.ulnar_sign, "links": h.placed.link_info, "points": pts}
        path.write_text(json.dumps(doc, ensure_ascii=False, indent=1, default=str))
        out.append(path)
    return out


def _wait_for_view(controller, live, client, seconds: float, stop) -> bool:
    """--headless: wait for hello synchronously and build the scene."""
    deadline = time.monotonic() + seconds
    while not stop.is_set():
        controller.poll()
        if controller.view is not None:
            return True
        if controller.build_error:
            print(f"[viewer] 按 SDK 的广播建场景失败：{controller.build_error}", file=sys.stderr)
            return False
        if time.monotonic() > deadline:
            break
        time.sleep(0.05)
    hello, _gen, _conn = live.session()
    if live.error:
        print(f"[viewer] {live.error}", file=sys.stderr)
    elif hello is None:
        print(f"[viewer] {seconds:.0f} s 内没连上 SDK 广播（{client.describe()}）。\n"
              f"  先在 SDK 的环境里起广播：revohuman-live（或 python3 -m revohuman live）；"
              f"不接手套可用 --demo。", file=sys.stderr)
    return False


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    logcolor.basic_config(logging.INFO, "[%(name)s] %(message)s")
    logging.getLogger("OpenGL").setLevel(logging.WARNING)       # offscreen, PyOpenGL prints
                                                                # an unrelated INFO line
    if args.headless:
        # Offscreen always defaults to EGL (no display needed; GLX offscreen is unreliable on dual-GPU
        # laptops). Must be set before importing mujoco.
        os.environ.setdefault("MUJOCO_GL", "egl")
    try:
        import mujoco
        import numpy as np
    except ImportError as exc:
        print(f"[viewer] 缺依赖 {exc.name}：请用 simulation/run_viewer.sh 启动（自动建隔离 venv 并装依赖），"
              f"或在你自己的环境里 pip install -r simulation/requirements.txt", file=sys.stderr)
        return 2
    from .hand_model import MissingModelError, resolve_urdf_dir
    try:
        resolve_urdf_dir(args.urdf_dir)          # without a hand model nothing can be drawn (--demo needs it
                                                 # too): explain first, then exit
    except MissingModelError as exc:
        print(f"[viewer] {exc}", file=sys.stderr)
        return 2
    ext = load_preview_ext()
    try:
        preview = ext.create(args) if ext is not None else None
    except ValueError as exc:
        print(f"[viewer] {exc}", file=sys.stderr)
        return 2
    if preview is not None:
        print(f"[viewer] {preview.describe()}", flush=True)

    from .live_client import LiveClient, LiveState
    from .session import Controller
    from .view import render_png, report_headless, run_window

    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    demo = client = controller = None
    try:
        layout = TactileLayout.load(args.layout)
        path = args.socket
        if args.demo:
            from .demo import DemoBroadcaster
            hands = ("left", "right") if args.demo_hands == "both" else (args.demo_hands,)
            demo = DemoBroadcaster(product=args.demo, hands=hands, layout=layout, urdf_dir=args.urdf_dir).start()
            path = demo.path
            print(f"[viewer] 演示广播 {args.demo}（{'、'.join(hands)}）", flush=True)
        live = LiveState()
        client = LiveClient(live, path).start()
        controller = Controller(mujoco, np, live, client, layout, args, background=not args.headless,
                                preview=preview)
        if not args.headless:
            if not args.demo:
                print(f"[viewer] 等 SDK 广播：{client.describe()}\n"
                      f"[viewer] （SDK 的环境里起 `revohuman-live`；连上后自动开始更新）", flush=True)
            run_window(controller, stop, fps=args.fps)
            return 0
        if not _wait_for_view(controller, live, client, args.wait, stop):
            return 2
        view = controller.view
        rc = report_headless(view, args.seconds, stop)
        if args.snapshot is not None:
            w, h = (int(v) for v in args.size.lower().split("x"))
            render_png(view, args.snapshot, (w, h))
            print(f"[viewer] 快照 → {args.snapshot}", flush=True)
        if args.dump_layout is not None:
            for p in _dump_layout(view, args.dump_layout):
                print(f"[viewer] 触觉点 → {p}", flush=True)
        return rc
    except KeyboardInterrupt:
        return 130
    finally:
        for obj in (controller, client, demo):
            if obj is not None:
                try:
                    obj.close()
                except Exception:                           # noqa: BLE001
                    pass


if __name__ == "__main__":
    sys.exit(main())
