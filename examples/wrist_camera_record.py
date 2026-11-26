"""Example: glove + wrist camera (optional hardware) recorded into the same MCAP; the camera uses
zero-copy loans.

Requires the right-hand glove present (the MCU serial number must identify the handedness); without a
wrist camera only the glove is recorded (the camera is optional hardware, its absence is not an error).
Recording depends on the optional extra: pip install 'revohuman[recording]'

    python3 examples/wrist_camera_record.py
"""

import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))  # uninstalled: run from repo root

from revohuman import GloveManager, TopicRecorder, recording
from revohuman.camera import CameraConfig, WristCamera

HAND = "right"
EPISODE = "ep_cam_001.mcap"


def open_wrist_camera(usb_hub):
    """Search for the camera only under the glove's HUB (same HUB as the glove, so another hand's camera
    is never picked up).
    Returns (camera, zero-copy subscription); returns (None, None) without a camera."""
    cam = WristCamera(CameraConfig(handedness=HAND, usb_hub=usb_hub))
    sub = cam.subscribe(loaned=True, queue_size=16)   # subscribe before start(); frames are views on
                                                      # driver buffers, returned once recorded
    try:
        cam.start()
    except (FileNotFoundError, OSError) as exc:      # no camera / no permission: glove still recorded
        sub.close()
        print("腕部相机不可用，只录手套：", exc)
        return None, None
    return cam, sub


def main() -> None:
    mgr = GloveManager()
    with mgr.connect(handedness=HAND) as glove:
        rec = TopicRecorder(compression="lz4")
        rec.record(glove.subscribe())                  # /glove/right
        cam, cam_sub = open_wrist_camera(glove.unit_id)
        if cam_sub is not None:
            rec.record(cam_sub)                        # /camera/right: raw MJPG bytes, undecoded, unfiltered

        handle = rec.start(EPISODE)
        time.sleep(5.0)
        summary = handle.stop()                        # uncancellable: even Ctrl-C finalizes the file
        print(EPISODE, summary["frames"], "帧", summary["per_channel"])
        if cam is not None:
            print("零拷贝账:", cam.stats().stream["loan"])   # outstanding / starved should both be 0
        rec.close()                                    # close the recorder first (returns loaned frames
                                                       # in its queue), then stop the camera
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
