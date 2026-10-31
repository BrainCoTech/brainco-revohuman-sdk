"""示例：单手读流（最短路径）。

用法：python3 examples/read_frames.py                 # 全机恰好插着一只时可裸连
     python3 examples/read_frames.py right           # 按手别连

左右手只认 MCU 序列号的 L/R 手别位；序列号认不出手别的手套按手别连会报错，改用
`mgr.connect(port="/dev/ttyACM0")` 按串口连（帧不标手别）。
"""

import pathlib
import queue
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))  # 未安装时从仓库根跑

from revohuman import GloveManager, set_log_level


def main() -> None:
    set_log_level("INFO")
    mgr = GloveManager()
    hand = sys.argv[1] if len(sys.argv) > 1 else None
    with (mgr.connect(handedness=hand) if hand else mgr.connect()) as glove:
        # effective_rate_hz 由流的 started 事件回填，而起流是异步的：connect 刚返回时它还是
        # None（属性文档如此），要等第一帧到手才有值——所以这里只报口，速率留给下面。
        print(f"已连接 {glove.port}")
        sub = glove.subscribe()
        got = 0
        while got < 40:
            try:
                frame = sub.recv(timeout=1.0)
            except queue.Empty:
                continue                     # 流在重连；generation 变化 = 重连过
            got += 1
            if got == 1:
                print(f"实际速率 {glove.effective_rate_hz}Hz")
            if got % 10 == 0:
                print(f"#{frame.seq} gen={frame.generation} "
                      f"joints[0..2]={[round(j, 2) for j in frame.joints_deg[:3]]} "
                      f"tactile={frame.tactile.point_count if frame.tactile else 0}点")
        print("stats:", glove.stats().to_dict())


if __name__ == "__main__":
    main()
