"""Example: read frames from one glove in asyncio (the same thing as read_frames.py, with await).

    python3 examples/read_frames_async.py [left|right]

recv_async uses native event-loop wakeup (does not occupy the thread pool); exceptions are the same as
recv: queue.Empty on timeout, Disconnected on close. `async for frame in sub:` also works; the loop ends
naturally when the subscription is closed.
"""

import asyncio
import queue
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # uninstalled: run from repo root

from revohuman import GloveManager, Handedness


async def read(hand: Handedness, n_frames: int = 400) -> int:
    glove = GloveManager().connect(handedness=hand)
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
    return asyncio.run(read(hand))


if __name__ == "__main__":
    main()
