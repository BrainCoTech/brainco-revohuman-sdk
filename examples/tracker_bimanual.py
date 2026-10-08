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

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))  # 未安装时从仓库根跑

from revohuman import Handedness, set_log_level


def _lighthouses_from_env():
    """`REVOHUMAN_LIGHTHOUSES` → 白名单，**只收基站广播名 `LHB-` 后面那 8 位 hex**。

    不接受信道号：按信道声明会把落到同一信道的外来基站当成自己的。
    """
    import os
    v = os.environ.get("REVOHUMAN_LIGHTHOUSES", "").strip()
    if not v:
        return None
    return [tok for tok in v.replace(",", " ").split()]


def main() -> None:
    # 惰性导入：没装 libsurvive 的机器上本文件仍然 import 得动（CI 门禁要求）
    from revohuman.tracker import BimanualTrackerSynchronizer, SurviveTrackerSource

    set_log_level("INFO")
    if len(sys.argv) < 3:
        print(__doc__)
        return
    left_sn, right_sn = sys.argv[1], sys.argv[2]

    src = SurviveTrackerSource(
        registry={left_sn: Handedness.LEFT, right_sn: Handedness.RIGHT},
        # 在册基站白名单：`REVOHUMAN_LIGHTHOUSES=xxxxxxxx,yyyyyyyy`（BLE 广播名
        # `LHB-xxxxxxxx` 后半截）。不设 = 不判基站身份。别按基站台数声明：
        # 有邻居基站的房间里台数对不上，会把 `healthy` 误判成 false。
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
                # 配对档不在帧上：要看这一轮跑在哪个档，读
                # sync.stats()["pairs_by_tier"] / ["pairs_blocked"]。
                print(f"[{sync.stats()['pairs_by_tier']['worst']}] "
                      f"左右时差 {pair.skew_ns / 1e6:+.3f}ms")
        print("--- 收尾统计 ---")
        print(sync.stats())
        sync.close()


if __name__ == "__main__":
    main()
