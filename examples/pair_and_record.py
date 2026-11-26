"""Example: two-hand pairing + MCAP recording + switching episodes + read back (the full chain).

Requires both left and right gloves present (the MCU serial numbers must identify left and right hands);
recording depends on the optional extra: pip install 'revohuman[recording]'

    python3 examples/pair_and_record.py
"""

import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))  # uninstalled: run from repo root

from revohuman import BimanualSynchronizer, GloveManager, TopicRecorder, recording


def main() -> None:
    mgr = GloveManager()
    with mgr.connect(handedness="left") as left, \
         mgr.connect(handedness="right") as right:
        sync = BimanualSynchronizer(left, right, mode="interpolate")
        rec = TopicRecorder(compression="lz4")
        rec.record(left.subscribe())         # each of the three streams goes to its own channel
        rec.record(right.subscribe())
        rec.record(sync.subscribe())

        for episode in ("ep_001.mcap", "ep_002.mcap"):
            handle = rec.start(episode)      # start again = automatically finalize the previous episode
            time.sleep(3.0)
            summary = handle.stop()          # uncancellable: even Ctrl-C finalizes the file
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
