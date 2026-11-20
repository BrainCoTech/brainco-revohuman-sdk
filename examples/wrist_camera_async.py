"""Example: read the wrist camera in asyncio (same pattern as read_frames_async.py, for the camera).

    python3 examples/wrist_camera_async.py [left|right]

Camera subscriptions and glove subscriptions share the same async base class:
`await sub.recv_async(timeout=…)` uses native event-loop wakeup (does not occupy the thread pool), and
`async for frame in sub:` ends naturally when the camera stops / the subscription is closed;
a timeout raises queue.Empty, closing raises Disconnected (when the reader thread exits abnormally, the
exception text includes fatal_error).

This also demonstrates something only the camera has: a zero-copy subscription must return buffers even
inside `async for`, so the loop body is written as `with frame:`; leaving the with block returns the
driver buffer.
"""

import asyncio
import queue
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # uninstalled: run from repo root

from revohuman import GloveManager, Handedness, identity
from revohuman.camera import CameraConfig, WristCamera
from revohuman.errors import Disconnected

SECONDS = 5.0


def open_camera(hand: Handedness) -> WristCamera:
    """Search for the camera only under that hand's glove unit HUB (the MCU firmware determines
    handedness, so another hand's camera is never picked up).

    Serial numbers of recent firmware versions carry no handedness, so the handedness stored in the MCU
    firmware must be read once first (`scan(probe=True)`, the reading goes into the cache); if the glove is
    already open in another process (e.g. a ROS node), its port is not opened and the reading that process
    wrote into the cache is used."""
    GloveManager().scan(probe=True)
    usb_hub, _note = identity.hand_hub(hand)
    return WristCamera(CameraConfig(handedness=str(hand), usb_hub=usb_hub))


async def read(hand: Handedness) -> int:
    cam = open_camera(hand)
    sub = cam.subscribe(queue_size=16, loaned=True)   # subscribe before start()
    cam.start()
    got = 0
    stop = asyncio.get_running_loop().time() + SECONDS
    try:
        while asyncio.get_running_loop().time() < stop:
            try:
                frame = await sub.recv_async(timeout=1.0)
            except queue.Empty:
                print("1s 没有帧（相机在起流 / 映射未收敛）", file=sys.stderr)
                continue
            except Disconnected as exc:                # reader thread crashed: the cause is in the exception
                print("相机订阅已关闭：", exc, file=sys.stderr)
                break
            with frame:                                # zero-copy: leaving `with` returns
                                                       # the buffer to the driver
                got += 1
                if got % 30 == 0:
                    print(f"#{got} seq={frame.seq} {frame.width}x{frame.height} "
                          f"{frame.nbytes}B stamp={frame.sample_wall_ns}")
    finally:
        sub.close()
        cam.stop()
        print("零拷贝账:", cam.stats().stream["loan"])   # outstanding / starved should both be 0
    return got


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    hand = Handedness(argv[0]) if argv else Handedness.RIGHT
    return asyncio.run(read(hand))


if __name__ == "__main__":
    main()
