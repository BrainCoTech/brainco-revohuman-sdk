"""Example: full kit auto-assembly (identifies P1 / PTV1 from the SN and the hardware present, records the
zero pose automatically at startup) → read two-hand paired frames → record an episode.

    python3 examples/kit_quickstart.py              # auto-identify the product model
    python3 examples/kit_quickstart.py PTV1         # declare the product model: refuse to start if it
                                                    # does not match the hardware present

Prerequisites: the base stations have been added to the allowlist and their geometry has been calibrated
(*RevoHuman SDK Reference Manual* under docs/, section 4 "Tracker and base stations"); recording requires
pip install 'revohuman[recording]'.
"""

import pathlib
import queue
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))  # uninstalled: run from repo root

from revohuman import RevoHumanError, format_error, open_kit, set_log_level


def main() -> None:
    set_log_level("INFO")
    product = sys.argv[1] if len(sys.argv) > 1 else "auto"
    input("双手摆到零位后按回车（启动时按当时的位姿记零位）...")
    try:
        kit = open_kit(product)
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
