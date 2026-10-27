"""示例：整套自动装配（按 SN 与在位硬件识别 P1 / PTV1，启动时自动记零位）→ 读双手配对帧 → 录一段 episode。

    python3 examples/kit_quickstart.py              # 自动识别型号
    python3 examples/kit_quickstart.py PTV1         # 声明型号：与在位硬件不符时拒绝启动
    python3 examples/kit_quickstart.py auto bench   # 注册表里同一手别声明在多组时，指定组名

前置：基站已写进白名单并标定过几何（docs/ 下《RevoHuman SDK 参考手册》第 4 节「Tracker 与基站」）；录制需 pip install 'revohuman[recording]'。
"""

import pathlib
import queue
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))  # 未安装时从仓库根跑

from revohuman import RevoHumanError, format_error, open_kit, set_log_level


def main() -> None:
    set_log_level("INFO")
    product = sys.argv[1] if len(sys.argv) > 1 else "auto"
    set_name = sys.argv[2] if len(sys.argv) > 2 else None
    input("双手摆到零位后按回车（启动时按当时的位姿记零位）...")
    try:
        kit = open_kit(product, set_name=set_name)
    except RevoHumanError as exc:
        sys.exit(format_error(exc))
    with kit:
        print(kit, "识别依据:", kit.product.source)
        print("零位:", kit.tracker.reference.calibration_id)
        geometry = kit.geometry()
        if not geometry["ok"]:
            sys.exit(f"基站几何不达标，先重新标定：{geometry['errors']}")

        sub = kit.synchronizer().subscribe()
        for _ in range(5):
            try:
                pair = sub.recv(timeout=1.0)
            except queue.Empty:
                continue
            print(f"skew={pair.skew_ns}ns  L={pair.left.joints_deg[:3]}  R={pair.right.joints_deg[:3]}")

        rec = kit.recorder()
        handle = rec.start("episode_001.mcap")
        time.sleep(5.0)
        summary = handle.stop()
        print("录制:", summary["frames"], "帧", summary["per_channel"])
        rec.close()


if __name__ == "__main__":
    main()
