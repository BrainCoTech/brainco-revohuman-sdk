"""示例：asyncio 里读腕部相机（与 read_frames_async.py 同一套写法，换成相机）。

    python3 examples/wrist_camera_async.py [left|right] [组名]

相机订阅与手套订阅共用同一个异步基类：`await sub.recv_async(timeout=…)` 走事件循环原生
唤醒（不占线程池），`async for frame in sub:` 在相机停机/订阅关闭时循环自然结束；
超时抛 queue.Empty，关闭抛 Disconnected（读线程异常退出时异常文本里带 fatal_error）。

这里顺带演示一件只有相机才有的事：零拷贝订阅在 `async for` 里也得还缓冲，
所以循环体写成 `with frame:`——出了 with 就把驱动缓冲还回去。
"""

import asyncio
import queue
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # 未安装时从仓库根跑

from revohuman import GloveManager, Handedness
from revohuman.camera import CameraConfig, WristCamera
from revohuman.errors import Disconnected

SECONDS = 5.0


def open_camera(hand: Handedness, set_name: str | None) -> WristCamera:
    """相机限定在该手注册表声明的 HUB 下找（不会拿到别只手的相机）。"""
    found = GloveManager().registry.unit_of(hand, set_name)
    usb_hub = None if found is None else found[0]
    return WristCamera(CameraConfig(handedness=str(hand), usb_hub=usb_hub))


async def read(hand: Handedness, set_name: str | None = None) -> int:
    cam = open_camera(hand, set_name)
    sub = cam.subscribe(queue_size=16, loaned=True)   # 订阅先于 start()
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
            except Disconnected as exc:                # 读线程炸了：原因就在异常里
                print("相机订阅已关闭：", exc, file=sys.stderr)
                break
            with frame:                                # 零拷贝：出了 with 就把缓冲还给驱动
                got += 1
                if got % 30 == 0:
                    print(f"#{got} seq={frame.seq} {frame.width}x{frame.height} "
                          f"{frame.nbytes}B stamp={frame.sample_wall_ns}")
    finally:
        sub.close()
        cam.stop()
        print("零拷贝账:", cam.stats().stream["loan"])   # outstanding / starved 都该是 0
    return got


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    hand = Handedness(argv[0]) if argv else Handedness.RIGHT
    set_name = argv[1] if len(argv) > 1 else None
    return asyncio.run(read(hand, set_name))


if __name__ == "__main__":
    main()
