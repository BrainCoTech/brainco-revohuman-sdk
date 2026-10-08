"""示例：asyncio 里读一只手套的帧（与 read_frames.py 同一件事，换成 await）。

    python3 examples/read_frames_async.py [left|right] [组名]

组名只在「注册表里同一手别声明在多组」时才需要给（不给会报错并列出有哪几组）。

recv_async 走事件循环原生唤醒（不占线程池），异常与 recv 一致：超时 queue.Empty、
关闭 Disconnected。也可以 `async for frame in sub:`，订阅关闭时循环自然结束。
"""

import asyncio
import queue
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # 未安装时从仓库根跑

from revohuman import GloveManager, Handedness


async def read(hand: Handedness, n_frames: int = 400, set_name: str | None = None) -> int:
    glove = GloveManager().connect(handedness=hand, set_name=set_name)
    got = 0
    try:
        with glove.subscribe() as sub:
            while got < n_frames:
                try:
                    frame = await sub.recv_async(timeout=1.0)
                except queue.Empty:
                    print("1s 没有帧（重连中 / 录制门关着）", file=sys.stderr)
                    continue
                got += 1
                if got % 100 == 0:
                    print(f"#{got} seq={frame.seq} sample_wall_ns={frame.sample_wall_ns}")
    finally:
        glove.close()
    return got


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    hand = Handedness(argv[0]) if argv else Handedness.LEFT
    set_name = argv[1] if len(argv) > 1 else None
    return asyncio.run(read(hand, set_name=set_name))


if __name__ == "__main__":
    main()
