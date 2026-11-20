"""示例：双 tracker 采集 + 时戳对齐 + 左右配对，跑 30s。

用法：
    python3 examples/tracker_bimanual.py LHR-AAAA LHR-BBBB        # 左 右

前置（缺一条就跑不起来）：
  · 装好 libsurvive + pysurvive，基站通电且已标定；
  · serial 与左右手的对应关系**由人声明**（下面的 registry），不许按枚举顺序猜。
"""

import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))  # uninstalled: run from repo root

from revohuman import Handedness, set_log_level


def _lighthouses_from_env():
    """`REVOHUMAN_LIGHTHOUSES` → allowlist, **accepting only the 8 hex digits after `LHB-` in the base
    station broadcast name**.

    Channel numbers are not accepted: declaring by channel would treat a foreign base station that lands on
    the same channel as one of your own.
    """
    import os
    v = os.environ.get("REVOHUMAN_LIGHTHOUSES", "").strip()
    if not v:
        return None
    return [tok for tok in v.replace(",", " ").split()]


def main() -> None:
    # lazy import: this file can still be imported on machines without libsurvive (required by the CI gate)
    from revohuman.tracker import BimanualTrackerSynchronizer, SurviveTrackerSource

    set_log_level("INFO")
    if len(sys.argv) < 3:
        print(__doc__)
        return
    left_sn, right_sn = sys.argv[1], sys.argv[2]

    src = SurviveTrackerSource(
        registry={left_sn: Handedness.LEFT, right_sn: Handedness.RIGHT},
        # Allowlist of enrolled base stations: `REVOHUMAN_LIGHTHOUSES=xxxxxxxx,yyyyyyyy` (second half of the
        # BLE broadcast name `LHB-xxxxxxxx`). Unset = base station identity is not checked. Do not declare by
        # base station count: in a room with neighbors' base stations the count will not match, and
        # `healthy` will be wrongly judged false.
        lighthouses=_lighthouses_from_env(),
    )
    with src:
        sync = BimanualTrackerSynchronizer(src.hand(Handedness.LEFT),
                                           src.hand(Handedness.RIGHT))
        sub = sync.subscribe()
        t0 = time.monotonic()
        while time.monotonic() - t0 < 30.0:
            pair = sub.recv(timeout=1.0)
            if pair is None:
                continue
            if pair.left.seq % 250 == 0:
                # The pairing tier is not on the frame: to see which tier this run is on, read
                # sync.stats()["pairs_by_tier"] / ["pairs_blocked"].
                print(f"[{sync.stats()['pairs_by_tier']['worst']}] "
                      f"左右时差 {pair.skew_ns / 1e6:+.3f}ms")
        print("--- 收尾统计 ---")
        print(sync.stats())
        sync.close()


if __name__ == "__main__":
    main()
