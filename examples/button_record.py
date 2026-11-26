"""Example: control recording with the button on the glove: press once to start recording an episode,
press again to end it.

Usage: python3 examples/button_record.py         # bare connect if exactly one glove is plugged in
       python3 examples/button_record.py right   # connect by handedness (MCU serial number decides L/R)

Ctrl-C exits (the episode being recorded is finalized first). Recording requires
pip install 'revohuman[recording]'.

Exit codes: 0 = normal exit; 1 = connection or recording error (one line with an error code on stderr);
            2 = this glove's firmware does not support reporting the button during capture.

The button is off by default; this example passes button=True explicitly when connecting.
The button state is maintained by the glove itself: each valid press toggles it once between "start" and
"end"; at power-up it is "end". The first state after connecting is only a baseline and does not count as
a press; so if the button was pressed before connecting, the first press may report "end"; press once
more to start.
"""

import pathlib
import queue
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))  # uninstalled: run from repo root

from revohuman import (ButtonMode, GloveManager, RevoHumanError, TopicRecorder, format_error,
                       set_log_level)

EXIT_UNSUPPORTED = 2


def main() -> None:
    set_log_level("INFO")
    hand = sys.argv[1] if len(sys.argv) > 1 else None
    mgr = GloveManager()
    try:
        glove = (mgr.connect(handedness=hand, button=True) if hand
                 else mgr.connect(button=True))
    except RevoHumanError as exc:
        sys.exit(format_error(exc))
    with glove:
        # Whether the button is available is known only after the stream first starts (starting the
        # stream is asynchronous): wait for the first frame first
        frames = glove.subscribe()
        while True:
            try:
                frames.recv(timeout=1.0)
                break
            except queue.Empty:
                continue
        frames.close()
        if not glove.button.supported:
            print(f"{glove.port}：固件不支持在采集期间上报按键", file=sys.stderr)
            sys.exit(EXIT_UNSUPPORTED)

        try:
            rec = TopicRecorder(compression="lz4")
        except RevoHumanError as exc:
            sys.exit(format_error(exc))
        rec.record(glove.subscribe())            # frames are dropped while not recording,
                                                 # written to disk once recording starts
        presses = glove.button.subscribe()
        print(f"已连接 {glove.port}，按一下手套上的按键开始录制，Ctrl-C 退出")

        handle, episode = None, 0
        try:
            while True:
                try:
                    ev = presses.recv(timeout=1.0)
                except queue.Empty:
                    continue
                if ev.missed_presses:
                    print(f"（有 {ev.missed_presses} 次按压没单独收到，以手套当前状态为准）")
                if ev.mode is ButtonMode.START and handle is None:
                    episode += 1
                    path = f"episode_{episode:03d}.mcap"
                    handle = rec.start(path)
                    print(f"开始录制 {path}")
                elif ev.mode is ButtonMode.END and handle is not None:
                    summary = handle.stop()
                    handle = None
                    print(f"结束录制：{summary['frames']} 帧")
                elif ev.mode is ButtonMode.END:
                    print("手套报「结束」（当前没在录），再按一下开始")
        except KeyboardInterrupt:
            pass
        except RevoHumanError as exc:
            sys.exit(format_error(exc))
        finally:
            presses.close()
            if handle is not None:
                summary = handle.stop()
                print(f"结束录制：{summary['frames']} 帧")
            rec.close()


if __name__ == "__main__":
    main()
