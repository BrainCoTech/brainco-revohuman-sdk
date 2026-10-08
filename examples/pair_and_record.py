"""示例：双手配对 + MCAP 录制 + 切 episode + 读回（设计文档 §4.6 全链）。

需要两只已注册手套在位；录制依赖可选 extra：pip install 'revohuman[recording]'

    python3 examples/pair_and_record.py [组名]   # 同一手别声明在多组时才要给组名
"""

import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))  # 未安装时从仓库根跑

from revohuman import BimanualSynchronizer, GloveManager, TopicRecorder, recording


def main() -> None:
    set_name = sys.argv[1] if len(sys.argv) > 1 else None
    mgr = GloveManager()
    with mgr.connect(handedness="left", set_name=set_name) as left, \
         mgr.connect(handedness="right", set_name=set_name) as right:
        sync = BimanualSynchronizer(left, right, mode="interpolate")
        rec = TopicRecorder(compression="lz4")
        rec.record(left.subscribe())         # 三路各归各的 channel
        rec.record(right.subscribe())
        rec.record(sync.subscribe())

        for episode in ("ep_001.mcap", "ep_002.mcap"):
            handle = rec.start(episode)      # 再次 start = 自动收上一段
            time.sleep(3.0)
            summary = handle.stop()          # 不可取消：Ctrl-C 也会把文件收完
            print(episode, summary["frames"], "帧", summary["per_channel"])

        print("配对质量:", sync.stats())
        sync.close()
        rec.close()

    with recording.open("ep_001.mcap") as reader:
        for i, msg in enumerate(reader):
            if i >= 3:
                break
            print(msg.topic, msg.log_time_ns, type(msg.frame).__name__)


if __name__ == "__main__":
    main()
