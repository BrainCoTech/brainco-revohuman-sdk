"""示例：手套 + 腕部相机（可选硬件）录进同一个 MCAP，相机走零拷贝借出（设计文档 §4.6 + §4.7）。

需要一只已注册的右手在位；腕部相机没接就只录手套（相机是可选硬件，缺席不算错）。
录制依赖可选 extra：pip install 'revohuman[recording]'

    python3 examples/wrist_camera_record.py [组名]   # 同一手别声明在多组时才要给组名
"""

import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))  # 未安装时从仓库根跑

from revohuman import GloveManager, Handedness, TopicRecorder, recording
from revohuman.camera import CameraConfig, WristCamera

HAND = "right"
EPISODE = "ep_cam_001.mcap"


def open_wrist_camera(mgr: GloveManager, set_name):
    """相机限定在该手注册表声明的 HUB 下找（和手套同一个 HUB，不会拿到别只手的相机）。
    返回 (相机, 零拷贝订阅)；没接相机返回 (None, None)。"""
    found = mgr.registry.unit_of(Handedness(HAND), set_name)
    usb_hub = None if found is None else found[0]
    cam = WristCamera(CameraConfig(handedness=HAND, usb_hub=usb_hub))
    sub = cam.subscribe(loaned=True, queue_size=16)   # 订阅先于 start()；帧是驱动缓冲上的视图，录完即归还
    try:
        cam.start()
    except (FileNotFoundError, OSError) as exc:      # 没相机 / 没权限：手套照录
        sub.close()
        print("腕部相机不可用，只录手套：", exc)
        return None, None
    return cam, sub


def main() -> None:
    set_name = sys.argv[1] if len(sys.argv) > 1 else None
    mgr = GloveManager()
    with mgr.connect(handedness=HAND, set_name=set_name) as glove:
        rec = TopicRecorder(compression="lz4")
        rec.record(glove.subscribe())                  # /glove/right
        cam, cam_sub = open_wrist_camera(mgr, set_name)
        if cam_sub is not None:
            rec.record(cam_sub)                        # /camera/right：原始 MJPG 字节，不解码、不滤

        handle = rec.start(EPISODE)
        time.sleep(5.0)
        summary = handle.stop()                        # 不可取消：Ctrl-C 也会把文件收完
        print(EPISODE, summary["frames"], "帧", summary["per_channel"])
        if cam is not None:
            print("零拷贝账:", cam.stats().stream["loan"])   # outstanding / starved 都该是 0
        rec.close()                                    # 先收录制器（归还队列里的借出帧），再停相机
        if cam is not None:
            cam.stop()

    with recording.open(EPISODE) as reader:
        for msg in reader:
            if msg.topic.startswith("/camera/"):
                f = msg.frame
                print(msg.topic, f"{f.width}x{f.height} {f.fourcc} {f.nbytes} B",
                      "sample_wall_ns", f.sample_wall_ns, "stamp_source", f.stamp_source)
                break


if __name__ == "__main__":
    main()
