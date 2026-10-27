"""手套可视化入口：一个 MuJoCo 窗口，左边笛卡尔视图（tracker 6D 位姿）、右边固定基座视图（关节 + 触觉热力图）。

可视化只订阅 SDK 的本机广播，不 import SDK、不碰串口与 tracker：

    revohuman-live                           # 终端 1，SDK 的环境里：打开手套、开始广播
                                             #   （同 python3 -m revohuman live）
    simulation/run_viewer.sh                 # 终端 2：自动建隔离 venv、装依赖、起窗口、等广播
    simulation/run_viewer.sh --demo          # 不接手套先看效果（内置演示广播，同一套协议）
    python3 simulation/viewer.py --headless --seconds 3 --snapshot out.png

窗口先起来等着；SDK 起来、收到它的 hello 才建场景并开始更新。型号（P1 / PTV1）、关节表、每只手的
触觉有没有、tracker 开没开，都照 hello 里 SDK 的识别结果来。SDK 退出后停在最后一帧，等它再起来。
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
    """可选的关节角预览扩展（本包里的 ``joint_preview`` 模块）：有就加载，没有返回 None。

    没有这个模块时不加任何参数、不绑任何按键，可视化行为与没有这个钩子完全相同。
    模块在但自身导入出错时照常抛出，不悄悄当成「没有」。

    扩展约定：``add_arguments(parser)`` 加自己的参数；``create(args)`` 返回预览对象或 None
    （配置有误抛 ValueError）。预览对象要有：``key``（单个字母，窗口里切换用）、``keys_hint``、
    ``enabled``、``bind(hand, joints)``（返回带 ``apply(q) -> list`` 的逐手变换，q 按关节表顺序、
    单位 rad、可含 None）、``toggle() -> str``、``overlay() -> str``、``describe() -> str``；
    覆盖层用到的文本都是 ASCII。"""
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
                   help="手部模型目录（urdf/ + meshes/），默认源码包根目录的 description/")
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
    """--headless：同步等 hello 并建场景。"""
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
    logging.getLogger("OpenGL").setLevel(logging.WARNING)       # 离屏时 PyOpenGL 会打一条无关的 INFO
    if args.headless:
        # 离屏一律默认 EGL（不要显示器；GLX 离屏在双显卡笔记本上不可靠）。必须在 import mujoco 之前。
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
        resolve_urdf_dir(args.urdf_dir)          # 没有手部模型什么都画不了（--demo 也要它）：先说清楚再退出
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
