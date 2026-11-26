"""Example: read a stream from one hand (the shortest path).

Usage: python3 examples/read_frames.py                 # bare connect if exactly one glove is plugged in
       python3 examples/read_frames.py right           # connect by handedness

Left and right hands are determined only by the MCU firmware (handedness stored in the firmware > the L/R
handedness character of the serial number); connecting by handedness to a glove whose handedness cannot be
identified raises an error; connect by serial port instead with
`mgr.connect(port="/dev/ttyACM0")` (frames are not tagged with handedness).
"""

import pathlib
import queue
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))  # uninstalled: run from repo root

from revohuman import GloveManager, set_log_level


def main() -> None:
    set_log_level("INFO")
    mgr = GloveManager()
    hand = sys.argv[1] if len(sys.argv) > 1 else None
    with (mgr.connect(handedness=hand) if hand else mgr.connect()) as glove:
        # effective_rate_hz is filled in by the stream's started event, and starting the stream is
        # asynchronous: right after connect returns it is still None (as the attribute docs say) and only
        # has a value once the first frame arrives, so only the port is reported here; the rate comes below.
        print(f"已连接 {glove.port}")
        sub = glove.subscribe()
        got = 0
        while got < 40:
            try:
                frame = sub.recv(timeout=1.0)
            except queue.Empty:
                continue                     # stream is reconnecting; generation change = it reconnected
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
