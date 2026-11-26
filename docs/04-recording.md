# 04 Recording

English | [简体中文](zh-CN/04-数据录制.md)

This chapter uses the SDK to record the joint angles and Tracker poses of both P1 hands into MCAP files, read them back for a check, and export them to CSV.
The last section describes the options of the live broadcast command `revohuman-live` (*05 Simulation and Visualization* uses it to feed live data to the 3D visualization).
For recording with ROS 2, see *06 ROS 2*.

## 1. Pre-recording checks

Confirm each item:

1. All software is installed as described in *01 Installation and System Requirements*;
2. Both gloves are connected as described in *02 Hardware Connection*, and `python3 -m revohuman doctor` shows `fail=0`;
3. The base stations have been added to the allowlist and geometry-calibrated as described in *03 Base Stations and Pairing*, and have not been moved since;
4. No other program is holding the gloves or Trackers: close `revohuman-live`, the live mode of the visualization, and ROS 2 nodes first.
   Each glove, and the Tracker driver, can be opened by only one program at a time.

**Zero pose**: Tracker pose deltas are relative to the "zero pose". When the recording program starts, it automatically records a zero pose from the poses of both hands at that moment,
so before starting, place both hands in the zero pose (the same fixed posture for every recording, for example both hands flat on fixed marks on the table)
and keep them still until the program prints `零位已记下` (zero pose recorded). From running the command to this message takes about 20 seconds
(a 5-second countdown, plus about 15 seconds to open the gloves and Trackers and wait until the base stations and both Trackers are ready); do not move either hand during this time.

## 2. Recording script

Save the following script as `~/revohuman_record.py`:

```python
"""RevoHuman P1 数据录制：打开双手手套与 Tracker，连续录若干段 MCAP。

用法：python3 ~/revohuman_record.py <输出目录> <每段秒数> <段数>
例：  python3 ~/revohuman_record.py ~/revohuman_data/demo 10 2
"""
import logging
import pathlib
import sys
import time

from revohuman import RevoHumanError, format_error, open_kit


def countdown(text: str, seconds: int) -> None:
    for n in range(seconds, 0, -1):
        print(f"{text}：{n}", flush=True)
        time.sleep(1)


def main() -> int:
    if len(sys.argv) != 4:
        print(__doc__, file=sys.stderr)
        return 2
    out_dir = pathlib.Path(sys.argv[1]).expanduser()
    seconds = float(sys.argv[2])
    episodes = int(sys.argv[3])
    out_dir.mkdir(parents=True, exist_ok=True)
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    countdown("双手摆到零位并保持不动", 5)
    try:
        kit = open_kit("P1")
    except RevoHumanError as exc:
        print(format_error(exc), file=sys.stderr)
        return 1

    aborted = 0
    with kit:
        ref = kit.tracker.reference
        print(f"零位已记下（{ref.calibration_id[:8]}），可以开始动作", flush=True)
        geometry = kit.geometry()
        if not geometry["ok"]:
            print(f"基站几何不达标：{geometry['errors']}；按《03-基站使用与配对》重新标定后再录",
                  file=sys.stderr)
            return 1
        rec = kit.recorder()
        try:
            for i in range(1, episodes + 1):
                if i > 1:
                    countdown("下一段即将开始", 3)
                path = out_dir / f"episode_{i:03d}.mcap"
                handle = rec.start(str(path), metadata={
                    "product": "P1", "episode": i, "reference": ref.to_payload()})
                print(f"[{i}/{episodes}] 录制中：{path}（{seconds:g} 秒，Ctrl-C 提前结束）", flush=True)
                stop_all = False
                try:
                    time.sleep(seconds)
                except KeyboardInterrupt:
                    stop_all = True
                s = handle.stop()
                print(f"[{i}/{episodes}] 已保存：{s['frames']} 帧，{s['bytes'] / 1e6:.2f} MB，"
                      f"{s['duration_sec']:.1f} 秒；各话题 {s['per_channel']}", flush=True)
                if s["aborted"]:
                    aborted += 1
                    print(f"[{i}/{episodes}] 本段已作废：{s['abort_reason']}", flush=True)
                if stop_all:
                    print("已按 Ctrl-C 停止录制", flush=True)
                    break
        except RevoHumanError as exc:
            print(format_error(exc), file=sys.stderr)
            return 1
        finally:
            rec.close()
    return 1 if aborted else 0


if __name__ == "__main__":
    sys.exit(main())
```

The script's three arguments are, in order: the output directory, the seconds per episode, and the number of episodes. Each episode is saved as its own file: `episode_001.mcap`, `episode_002.mcap`, …;
the zero pose is recorded only once at startup and is shared by all episodes in the same run. Pressing `Ctrl-C` ends and saves the current episode, then exits.

Record two episodes of 10 seconds each:

```bash
python3 ~/revohuman_record.py ~/revohuman_data/demo 10 2
```

The output looks like this (the `INFO` log lines in between are omitted):

```text
双手摆到零位并保持不动：5
...
双手摆到零位并保持不动：1
零位已记下（019e0de4），可以开始动作
[1/2] 录制中：/home/<用户名>/revohuman_data/demo/episode_001.mcap（10 秒，Ctrl-C 提前结束）
[1/2] 已保存：11012 帧，1.65 MB，10.0 秒；各话题 {'/glove/right': 2002, '/tracker/right': 2505, '/tracker/left': 2501, '/paired': 2002, '/glove/left': 2002}
下一段即将开始：3
下一段即将开始：2
下一段即将开始：1
[2/2] 录制中：/home/<用户名>/revohuman_data/demo/episode_002.mcap（10 秒，Ctrl-C 提前结束）
[2/2] 已保存：11014 帧，1.65 MB，10.0 秒；各话题 {'/tracker/left': 2502, '/paired': 2002, '/glove/left': 2002, '/tracker/right': 2506, '/glove/right': 2002}
```

The zero-pose ID differs on every run.

The `已保存` (saved) line printed at the end of each episode gives the frame count, file size, duration, and message count per topic.
If `本段已作废` (this episode is aborted) appears, the episode's file is still kept but its data is unusable; see section 5.

Exit codes: `0` all succeeded; `1` open failed, base station geometry out of tolerance, or an episode was aborted; `2` wrong arguments.

## 3. Contents of a recording file

Each `.mcap` file contains the following topics:

| Topic | Frame type | Rate | Content |
|---|---|---|---|
| `/glove/left`, `/glove/right` | `GloveFrame` | 200 Hz | The 21 joint angles of one hand |
| `/paired` | `PairedFrame` | 200 Hz | Two-hand paired frame: the left and right frames from the same hardware synchronization cycle |
| `/tracker/left`, `/tracker/right` | `TrackerFrame` | about 250 Hz | Tracker pose in the base station coordinate frame |

Main fields:

| Frame | Field | Description |
|---|---|---|
| `GloveFrame` | `joints_deg` | 21 joint angles, in **degrees**; get the names with `revohuman.joint_map.joint_names()` |
| `GloveFrame` | `joints_valid_mask` / `all_joints_valid` | Whether each joint is valid |
| `GloveFrame` | `seq` / `generation` | Sequence number; a change in `generation` means this hand's data stream was reopened midway (after a reconnect, a self-test, or a rebuild of two-hand synchronization) |
| `PairedFrame` | `left` / `right` / `skew_ns` | The `GloveFrame` of the left and right hands; `skew_ns` is the difference between the two hands' sampling times, always 0 under hardware synchronization |
| `TrackerFrame` | `pose.position` | Position `(x, y, z)`, in **meters** |
| `TrackerFrame` | `pose.rotation` | Orientation quaternion, in the order **`(w, x, y, z)`** |
| `TrackerFrame` | `serial` / `handedness` | Tracker serial number / handedness |
| All | `sample_wall_ns` | **Sampling time**: the moment the sensor actually sampled, as Unix time in nanoseconds |
| All | `recv_wall_ns` | **Arrival time**: the moment the SDK received this frame, including USB link latency |

**Timestamps**: For analysis and for cross-hand and cross-device alignment, always use the frame's `sample_wall_ns` (for paired frames, `left.sample_wall_ns`).
The `log_time` that comes with each MCAP message is the arrival time (equal to `recv_wall_ns`); it reflects write order and lags the sampling time by one link latency. Do not use it for alignment.
For how to align gloves and Trackers, synchronization metrics, and time-related anomalies, see *09 Timestamp Synchronization*.
When a Tracker is occluded and has no pose, that frame's `pose` is `None`.

**Zero pose**: The Tracker poses in a recording file are absolute poses in the base station coordinate frame (defined in *03 Base Stations and Pairing* section 5.3).
Every base station calibration rebuilds this coordinate frame, so only absolute poses recorded under the same calibration can be compared directly.
The recording script in section 2 writes the zero pose recorded at startup into each episode file's metadata `revohuman.episode` → `reference`
(one 4×4 homogeneous matrix `tracker_ref` per hand); if you write your own recording program, you must include it yourself in `rec.start(..., metadata=...)`.
Deltas relative to the zero pose are computed as `ΔR = R_refᵀ · R`, `Δp = R_refᵀ · (p − p_ref)`, i.e. expressed in the Tracker's own coordinate frame at the zero-pose moment.

**Synchronization and abort flags**: Each episode file's metadata `revohuman.sync_health` records whether both hands were strictly synchronized throughout the episode (`sync_strict`)
and whether the episode was aborted (`aborted` / `abort_reason`); the read-back script in section 4 prints them.

## 4. Reading back and exporting

### 4.1 Inspecting a recording file

Save the following script as `~/revohuman_inspect.py`:

```python
"""读回 RevoHuman 录制文件：各话题条数、时长、速率、链路延迟，同步与作废标记，首帧示例。

用法：python3 ~/revohuman_inspect.py <文件.mcap>
"""
import statistics
import sys

from revohuman import recording
from revohuman.frames import PairedFrame


def sample_ns(frame):
    return frame.left.sample_wall_ns if isinstance(frame, PairedFrame) else frame.sample_wall_ns


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__, file=sys.stderr)
        return 2
    topics = {}
    with recording.open(sys.argv[1]) as reader:
        for msg in reader:
            t = topics.setdefault(msg.topic, {"n": 0, "stamps": [], "lag": [], "first": msg.frame})
            t["n"] += 1
            s = sample_ns(msg.frame)
            if s is not None:
                t["stamps"].append(s)
                t["lag"].append((msg.log_time_ns - s) / 1e6)
        meta = reader.metadata()

    print(f"{'话题':<16}{'条数':>8}{'时长(s)':>10}{'速率(Hz)':>10}{'链路延迟中位数(ms)':>20}")
    for name in sorted(topics):
        t = topics[name]
        span = (max(t["stamps"]) - min(t["stamps"])) / 1e9 if len(t["stamps"]) > 1 else 0.0
        rate = (len(t["stamps"]) - 1) / span if span > 0 else 0.0
        lag = statistics.median(t["lag"]) if t["lag"] else float("nan")
        print(f"{name:<16}{t['n']:>8}{span:>10.2f}{rate:>10.1f}{lag:>20.2f}")

    health = meta.get("revohuman.sync_health", {})
    episode = meta.get("revohuman.episode", {})
    print("双手全程严格同步：", "是" if health.get("sync_strict") else "否",
          "  本段作废：", "是（" + str(health.get("abort_reason")) + "）" if health.get("aborted") else "否")
    print("零位：", (episode.get("reference") or {}).get("calibration_id", "无"))

    if "/paired" in topics:
        f = topics["/paired"]["first"]
        print("/paired 首帧：skew_ns =", f.skew_ns,
              " 左手前 3 个关节角(度) =", [round(v, 2) for v in f.left.joints_deg[:3]])
    for hand in ("left", "right"):
        name = f"/tracker/{hand}"
        if name in topics and topics[name]["first"].pose is not None:
            p = topics[name]["first"].pose
            print(f"{name} 首帧：位置(米) =", [round(v, 4) for v in p.position],
                  " 姿态(w,x,y,z) =", [round(v, 4) for v in p.rotation])
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

Inspect the first episode just recorded:

```bash
python3 ~/revohuman_inspect.py ~/revohuman_data/demo/episode_001.mcap
```

The output looks like this:

```text
话题                    条数     时长(s)    速率(Hz)         链路延迟中位数(ms)
/glove/left         2002     10.01     200.0               12.35
/glove/right        2002     10.01     200.0                9.32
/paired             2002     10.01     200.0               12.35
/tracker/left       2501     10.01     249.9                0.53
/tracker/right      2505     10.00     250.3                0.45
双手全程严格同步： 是   本段作废： 否
零位： 019e0de4e6f34fc888e28286ac00917f
/paired 首帧：skew_ns = 0  左手前 3 个关节角(度) = [140.76, 195.05, 199.78]
/tracker/left 首帧：位置(米) = [0.0098, -0.0235, -0.0128]  姿态(w,x,y,z) = [-0.1665, 0.22, 0.9385, 0.2078]
/tracker/right 首帧：位置(米) = [0.1193, 0.2044, -0.0112]  姿态(w,x,y,z) = [-0.2703, -0.9474, -0.1712, 0.0066]
```

Pass criteria:

- All five topics are present; `/glove/left`, `/glove/right`, and `/paired` are about 200 Hz with equal counts or counts that differ by only a few, and both Trackers are about 250 Hz;
- `双手全程严格同步` (both hands strictly synchronized throughout) is `是` (yes) and `本段作废` (episode aborted) is `否` (no); the first `/paired` frame has `skew_ns = 0`.

`链路延迟` (link latency) is arrival time minus sampling time: about 10 ms for the gloves and under 1 ms for the Trackers. It shows why `log_time` must not be used for alignment:
aligning by arrival time would shift the glove data by about 10 ms relative to the Trackers. The joint angle and position values vary with how the hands are placed.

### 4.2 Exporting to CSV

Save the following script as `~/revohuman_export_csv.py`:

```python
"""把 RevoHuman 录制文件导出为 CSV：双手关节角一个文件，每只 Tracker 一个文件。

用法：python3 ~/revohuman_export_csv.py <文件.mcap>
输出（与录制文件同目录）：<名>_joints.csv、<名>_tracker_left.csv、<名>_tracker_right.csv
"""
import csv
import pathlib
import sys

from revohuman import recording
from revohuman.joint_map import joint_names


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__, file=sys.stderr)
        return 2
    src = pathlib.Path(sys.argv[1]).expanduser()
    stem = src.with_suffix("")
    names = joint_names()
    outputs = {}

    def writer(key, header):
        if key not in outputs:
            fp = open(f"{stem}_{key}.csv", "w", newline="", encoding="utf-8")
            w = csv.writer(fp)
            w.writerow(header)
            outputs[key] = [fp, w, 0]
        return outputs[key]

    with recording.open(src) as reader:
        for msg in reader:
            f = msg.frame
            if msg.topic == "/paired":
                out = writer("joints", ["sample_wall_ns"]
                             + [f"left_{n}_deg" for n in names] + [f"right_{n}_deg" for n in names])
                out[1].writerow([f.left.sample_wall_ns, *f.left.joints_deg, *f.right.joints_deg])
            elif msg.topic.startswith("/tracker/"):
                hand = msg.topic.rsplit("/", 1)[1]
                out = writer(f"tracker_{hand}",
                             ["sample_wall_ns", "x_m", "y_m", "z_m", "qw", "qx", "qy", "qz"])
                pose = [*f.pose.position, *f.pose.rotation] if f.pose is not None else [""] * 7
                out[1].writerow([f.sample_wall_ns, *pose])
            else:
                continue
            out[2] += 1
    for key, (fp, _, rows) in outputs.items():
        fp.close()
        print(f"{stem}_{key}.csv：{rows} 行")
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

Export the first episode:

```bash
python3 ~/revohuman_export_csv.py ~/revohuman_data/demo/episode_001.mcap
head -n 2 ~/revohuman_data/demo/episode_001_tracker_left.csv
```

This produces three files:

| File | Columns |
|---|---|
| `episode_001_joints.csv` | `sample_wall_ns`, 21 left-hand joint angles, 21 right-hand joint angles (degrees); one paired frame per row |
| `episode_001_tracker_left.csv` | `sample_wall_ns`, `x_m`, `y_m`, `z_m`, `qw`, `qx`, `qy`, `qz` |
| `episode_001_tracker_right.csv` | Same as above |

Joint angle columns are named like `left_<joint name>_deg`. In rows where the Tracker was occluded and had no pose, the pose columns are empty.

## 5. Aborted episodes

When any of the following occurs during recording, the SDK marks the current episode as aborted: the `已保存` (saved) line is followed by a `本段已作废` (this episode is aborted) line,
and the file is written out as usual, but `aborted` in its metadata is true. Do not use this episode's data.

| Situation | Cause | Afterwards |
|---|---|---|
| Glove timestamp quality degrades (unstable USB link, host clock jump) | The sampling times during this period cannot be trusted | Recording can continue after automatic recovery |
| Base stations occluded, or not enough usable base stations, for 3 seconds | The pose is unreliable | Recording can continue once the condition clears |
| A base station is knocked askew (tilt reaches 2°), is detected as moved, or its pose does not match the calibration record | The coordinate frame has changed | No more recording is possible in this run; recalibrate the base stations (*03 Base Stations and Pairing*), then run again |
| All Trackers stop producing data for 10 seconds | Tracker disconnected | Check the connections, then run again |
| The zero pose is recorded again during recording (calling `set_reference()`, or the ROS 2 `tracker/set_reference` service) | The two parts of this episode are not under the same zero pose | Recording can continue; the next episode uses the updated zero pose |

Translating a base station or rotating it horizontally cannot be detected, and the episode will not be marked as aborted; if a base station has been moved, recalibrate (see *03 Base Stations and Pairing* section 8).

When no more recording is possible in this run, the next episode fails with an error as it starts and the program exits; the error message carries an error code, whose meaning is given in *07 Logging and Error Codes*.

## 6. Data volume

A recording of both P1 hands plus two Trackers is about **10 MB/minute** (measured: 1.65 MB for a 10-second episode), i.e. about 0.6 GB/hour;
use this figure to estimate disk space. Recording files use lz4 compression.

The exported CSVs are close in size to the original recording file (measured: 1.44 MB for the three CSVs combined).

## 7. Live broadcast: revohuman-live

`revohuman-live` opens the whole set of gloves and Trackers and broadcasts joint angles and Tracker poses live to subscribers on the local machine
(the live mode of the 3D visualization; see *05 Simulation and Visualization*). It **does not record data**;
while it runs, it holds the gloves and Trackers, so recording and ROS 2 nodes must wait until it exits (`Ctrl-C`) before running.

| Option | Default | Description |
|---|---|---|
| `--product {auto,P1,PTV1}` | `auto` | Product model. `auto` identifies it automatically; with `P1`, a mismatch with the connected hardware is reported as an error and the program exits |
| `--hands {both,left,right}` | `both` | Which hands to open |
| `--no-tracker` | off | Do not open the Trackers; broadcast glove data only |
| `--tracker-gate {flag,error}` | `flag` | When the base station checks fail: `flag` broadcasts poses as usual and marks in the status which item failed; `error` does not broadcast poses |
| `--tracker-phi-pack SERIAL=毫秒` | none | Calibrated packing latency of one Tracker, in milliseconds (`毫秒`); `SERIAL` is the Tracker's serial number (in the form `LHR-XXXXXXXX`). Give one entry per Tracker; the option can be repeated. For each Tracker given, this latency is subtracted from the pose sampling timestamps. It is needed only for millisecond-level absolute alignment across modalities; when not given, it shows up as a fixed offset (see *RevoHuman SDK Reference Manual* section 6.1, the "Absolute phase" item) |
| `--sync {best-effort,on,off}` | `best-effort` | Two-hand hardware synchronization: `best-effort` keeps broadcasting even if synchronization fails; `on` reports an error when synchronization fails; `off` does no hardware synchronization |
| `--socket SOCKET` | automatic | Local socket path used for the broadcast; usually no need to change it |
| `-v` | off | Print debug logs |

Note that the default of `--tracker-gate` is `flag`, which makes it easier to troubleshoot base station problems while watching the view;
the recording script in section 2 uses `error` (the SDK default): when the base station checks fail, it refuses to start, so no data with an untrustworthy coordinate frame gets recorded.

To check glove data using only the left hand, without opening the Trackers:

```bash
revohuman-live --hands left --no-tracker
```

`revohuman-live` keeps running; end it with `Ctrl-C`. For everyday use, just run `revohuman-live`.

## 8. Starting/stopping recording with the glove button

The script in section 2 records for a fixed number of seconds. You can instead use the button on the glove: press once to start recording an episode, press again to end it, without going back to the keyboard.

The button is off by default; pass `glove_kwargs={"button": True}` explicitly when opening the whole set. Once enabled, the button state is reported together with the glove data,
without changing the sampling rate. The button requires glove firmware that supports it; if the firmware does not, explicitly enabling the button logs one `N2406`
(see *07 Logging and Error Codes* section 3), and the data stream continues as usual. In that case, the script in this section prints a message and exits with exit code `3`.

Save the following script as `~/revohuman_button_record.py`:

```python
"""RevoHuman P1 按键录制：打开双手手套与 Tracker，按一下手套上的按键开始录一段，再按一下结束。

用法：python3 ~/revohuman_button_record.py <输出目录> [right|left]
例：  python3 ~/revohuman_button_record.py ~/revohuman_data/button right
第二个参数选用哪只手套上的按键，默认 right。按 Ctrl-C 退出（正在录的那段会先保存）。
"""
import logging
import pathlib
import queue
import sys
import time

from revohuman import ButtonMode, RevoHumanError, format_error, open_kit

EXIT_BUTTON_UNSUPPORTED = 3


def countdown(text: str, seconds: int) -> None:
    for n in range(seconds, 0, -1):
        print(f"{text}：{n}", flush=True)
        time.sleep(1)


def wait_supported(button, timeout_s: float = 10.0):
    """Whether the button is usable is known only after the glove starts streaming data:
    True = supported, False = not supported, None = no data arrived in time."""
    deadline = time.monotonic() + timeout_s
    while button.supported is None and time.monotonic() < deadline:
        time.sleep(0.1)
    return button.supported


def report(i: int, s: dict) -> int:
    print(f"[{i}] 已保存：{s['frames']} 帧，{s['bytes'] / 1e6:.2f} MB，"
          f"{s['duration_sec']:.1f} 秒；各话题 {s['per_channel']}", flush=True)
    if s["aborted"]:
        print(f"[{i}] 本段已作废：{s['abort_reason']}", flush=True)
        return 1
    return 0


def main() -> int:
    if len(sys.argv) not in (2, 3) or sys.argv[2:] not in ([], ["left"], ["right"]):
        print(__doc__, file=sys.stderr)
        return 2
    out_dir = pathlib.Path(sys.argv[1]).expanduser()
    hand = sys.argv[2] if len(sys.argv) == 3 else "right"
    out_dir.mkdir(parents=True, exist_ok=True)
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    countdown("双手摆到零位并保持不动", 5)
    try:
        kit = open_kit("P1", glove_kwargs={"button": True})
    except RevoHumanError as exc:
        print(format_error(exc), file=sys.stderr)
        return 1

    aborted = 0
    with kit:
        ref = kit.tracker.reference
        print(f"零位已记下（{ref.calibration_id[:8]}），可以开始动作", flush=True)
        geometry = kit.geometry()
        if not geometry["ok"]:
            print(f"基站几何不达标：{geometry['errors']}；按《03-基站使用与配对》重新标定后再录",
                  file=sys.stderr)
            return 1
        button = kit.glove(hand).button
        supported = wait_supported(button)
        if supported is None:
            print(f"{hand} 手套 10 秒内没有数据，按《02-硬件连接》检查连接", file=sys.stderr)
            return 1
        if not supported:
            print(f"{hand} 手套的固件不支持在采集期间上报按键，请改用第 2 节的定时录制",
                  file=sys.stderr)
            return EXIT_BUTTON_UNSUPPORTED
        rec = kit.recorder()
        presses = button.subscribe()
        handle, episode = None, 0
        print(f"按一下 {hand} 手套上的按键开始录制，再按一下结束；Ctrl-C 退出", flush=True)
        try:
            while True:
                try:
                    ev = presses.recv(timeout=1.0)
                except queue.Empty:
                    continue
                if ev.missed_presses:
                    print(f"（有 {ev.missed_presses} 次按压没有单独收到，以手套当前状态为准）", flush=True)
                if ev.mode is ButtonMode.START and handle is None:
                    episode += 1
                    path = out_dir / f"episode_{episode:03d}.mcap"
                    handle = rec.start(str(path), metadata={
                        "product": "P1", "episode": episode, "reference": ref.to_payload()})
                    print(f"[{episode}] 录制中：{path}（再按一下按键结束）", flush=True)
                elif ev.mode is ButtonMode.END and handle is not None:
                    aborted += report(episode, handle.stop())
                    handle = None
                elif ev.mode is ButtonMode.END:
                    print("手套报「结束」，当前没在录；再按一下开始", flush=True)
        except KeyboardInterrupt:
            print("已按 Ctrl-C 停止录制", flush=True)
        except RevoHumanError as exc:
            print(format_error(exc), file=sys.stderr)
            aborted += 1
        finally:
            presses.close()
            if handle is not None:
                aborted += report(episode, handle.stop())
            rec.close()
    return 1 if aborted else 0


if __name__ == "__main__":
    sys.exit(main())
```

The two arguments are, in order: the output directory, and which glove's button to use (`right` or `left`, default `right`).
The zero pose and base station checks are the same as in section 2; each "start" press records another episode, saved in turn as `episode_001.mcap`, `episode_002.mcap`, …
Pressing `Ctrl-C` first saves the episode being recorded, then exits.

Control it with the button on the right glove:

```bash
python3 ~/revohuman_button_record.py ~/revohuman_data/button right
```

Button rules:

- The glove itself keeps two states, "start" and "end", and toggles between them once per press; a long press does not toggle repeatedly. At power-on the state is "end".
- The first state the script sees after starting serves only as a baseline and does not count as a press. If the button was pressed before the script started, the first press may report "end",
  and the script prints `当前没在录；再按一下开始` (not recording now; press again to start).
- When presses come too fast and some presses in between are not received individually, the script prints `有 n 次按压没有单独收到` (n presses were not received individually) and goes by the glove's current state:
  if the current state is "start", it starts recording (or keeps recording if already recording); if it is "end", it ends this episode.
- Only the button on the glove given by the argument is recognized; the button on the other glove has no effect.
  The button is enabled on both gloves, so if the other glove's firmware does not support it, one `N2406` is also logged; this does not affect recording.
- An episode starts and ends at the moment the script receives the button event, slightly later than the actual press. This is suitable for splitting episodes, not for precise time alignment. The button itself is not written to the recording file.
- After a USB disconnect and reconnect, or after the data stream is reopened by an operation such as a self-test, presses made in the meantime are not reported afterwards; the first state after reopening becomes the baseline again.

Exit codes: `0` normal exit; `1` open failed, base station geometry out of tolerance, an episode was aborted, or an error occurred while running; `2` wrong arguments;
`3` the glove firmware does not support reporting the button during capture.

## 9. Reading live data in your own program

The scripts in sections 2 and 8 only record. To use the data in your own program while capturing (real-time control, display, online analysis), read SDK subscriptions directly.
This section explains which calls block, how to choose among the three ways to read, and what happens when the reader cannot keep up, on disconnect, and on close.
For the full description of each interface, see *RevoHuman SDK Reference Manual* section 6.3.

### 9.1 Which calls block

SDK interfaces are all ordinary synchronous functions. Capture runs in the SDK's own background threads; your program does not need to poll the devices.
There are two kinds of interfaces for reading data:

- **Subscription** (`subscribe()`): get every frame, in order, without missing any. Supports three ways to read: blocking reads, callbacks, and asyncio.
- **Latest value**: non-blocking; immediately returns the current latest value.

| Call | Blocking | Notes |
|---|---|---|
| `open_kit()`, `GloveManager().connect()` / `connect_pair()` | Blocks | Opens the devices and runs the startup checks. `open_kit()` waits until the base stations and Trackers are ready, about 15 seconds. It returns without waiting for the first data frame |
| `kit.close()`, leaving `with` | Blocks | Stops capture and waits for the background threads to exit |
| `sub.recv(timeout=t)` | Blocks for at most `t` seconds | Returns immediately when a frame is available; raises `queue.Empty` if no frame arrives within `t` seconds. `timeout=0` does not wait; without `timeout`, it waits indefinitely |
| `await sub.recv_async(timeout=t)` | Does not block the event loop | Suspends only the current coroutine and does not occupy the thread pool; on timeout it likewise raises `queue.Empty` |
| `kit.tracker.latest(hand)` / `kit.tracker.delta(hand)` | Non-blocking | Immediately returns the latest frame of that hand's Tracker and its age (seconds) / the delta relative to the zero pose |
| `glove.button.read()` | Non-blocking | Immediately returns the latest button state (section 8) |
| `rec.start(...)` / `handle.stop()` | Blocks | `stop()` returns only after the file is completely written |
| `stats()` / `status()` | Non-blocking | Snapshot of the current statistics |

`open_kit()`, `connect()`, and `close()` have no asyncio versions. In an asyncio program, call them with `await asyncio.to_thread(...)`
so that they do not stall the event loop (see the example in 9.3).

### 9.2 A subscription is a bounded queue

- Each call to `subscribe()` returns an independent subscription, and every frame is delivered to every subscription.
  When several places need the same stream, call `subscribe()` once for each; do not let several threads read the same subscription: each frame is handed to only one of those readers.
- A subscription is a bounded queue. For gloves and two-hand paired frames, it holds at most 64 frames by default (about 0.3 seconds at 200 Hz); you can increase this with `subscribe(queue_size=...)`.
- When you read more slowly than data arrives, a full queue **drops the oldest frames**; capture itself is not affected. Check `sub.dropped_count` for how many were dropped.
- Gloves have no "latest frame" interface. When you only need the latest value, drain the frames accumulated in the queue in one go and keep the last one:

  ```python
  import queue

  def newest(sub, timeout=1.0):
      """Wait for at least one frame, then drain the backlog in the queue and return the newest frame."""
      f = sub.recv(timeout=timeout)
      while True:
          try:
              f = sub.recv(timeout=0)
          except queue.Empty:
              return f
  ```

- **No frames while the timestamp gate is closed.** The default subscription dispatches only frames whose timestamps pass the gate, so in two situations the glove dispatches no frames at all, and the reader only sees `queue.Empty`:
  - the brief warm-up right after the glove is opened;
  - while timestamp quality is degraded (log `N2102`, see *09 Timestamp Synchronization*).

  Check `glove.capture_ok` to see whether it is in this state; when it is `False`, `glove.capture_cause` gives the reason.
  If you only need live display and do not need precise timestamps, you can use `glove.subscribe(gated=False)`: frames keep coming during these periods, but their `gate_open` is `False`,
  and the precision of `sample_wall_ns` is not guaranteed. The recorder does not accept such a subscription, nor one with `joint_filter` (recordings store raw joint angles; do filtering in a separate subscription opened on the reading side).
- **No frames during disconnect and reconnect either.** Delivery resumes automatically after reconnecting, and the `generation` of frames after recovery increases by 1.

### 9.3 Three ways to read

**Blocking read**: use `recv(timeout=...)` in your own thread or main loop. In the example, each time a pair of glove frames arrives, the latest Tracker pose is fetched as well:

```python
import queue
from revohuman import open_kit

with open_kit("P1") as kit:
    sub = kit.synchronizer().subscribe()          # two-hand paired frames, 200 Hz
    for _ in range(2000):                         # about 10 seconds
        try:
            pf = sub.recv(timeout=1.0)            # wait at most 1 second
        except queue.Empty:
            gl, gr = kit.glove("left"), kit.glove("right")
            print("1 秒内没有数据，时间戳门：", gl.capture_ok, gr.capture_ok)
            continue
        tf, age_s = kit.tracker.latest("right")   # non-blocking
        if tf is not None and age_s < 0.1:
            print(pf.left.sample_wall_ns, pf.right.joints_deg[:3], tf.pose.position)
    print("丢帧：", sub.dropped_count)
```

**Callback**: `subscribe(callback=fn)`; the SDK starts a dedicated background thread for this subscription and calls the function frame by frame, in order:

```python
def on_pair(pf):
    ...                                           # runs in this subscription's own thread

sub = kit.synchronizer().subscribe(callback=on_pair)
```

- When the callback is slow, the oldest frames are likewise dropped according to the rules in the previous subsection; capture is not slowed down.
- Exceptions raised in the callback do not interrupt capture and are **not printed**; only the most recent one is kept in `sub.last_error`. Handle them yourself with `try` in the callback and log them.
- Do not close the subscription itself inside its callback; and do not call `recv()` on a callback-style subscription.

**asyncio**: `await sub.recv_async(timeout=...)` or `async for`. Put blocking calls such as open and close into `asyncio.to_thread`:

```python
import asyncio
from revohuman import open_kit

async def main():
    kit = await asyncio.to_thread(open_kit, "P1")
    try:
        sub = kit.synchronizer().subscribe()
        async for pf in sub:                      # loop ends when the subscription closes
            ...                                   # no long synchronous computation here
    finally:
        await asyncio.to_thread(kit.close)

asyncio.run(main())
```

### 9.4 Closing and errors

- `kit.close()` (or leaving `with`) closes all subscriptions. After that, `recv()` / `recv_async()` on a subscription raises `Disconnected` (error code `1101`), and `async for` ends normally.
- A synchronous `for f in sub:` raises `Disconnected` if it is waiting when the close happens; either wrap it in `try`, or, as in 9.3, use a `recv(timeout=...)` loop and check `sub.closed`.
- After getting `Disconnected`, do not call `recv()` on the same subscription again: the close notification is delivered only once, and calling again waits forever (with `timeout`, it raises `queue.Empty`).
  Use `sub.closed` to tell whether a subscription has been closed.
- A subscription can also be closed by an unrecoverable fault; then `recv()` raises the exception for that fault, and `async for` does not swallow it either.
  For example, when a base station is moved, the Tracker subscription raises `WorldFrameChanged` (`1902`). Catching `RevoHumanError` covers all of these; for their meanings, see *07 Logging and Error Codes* section 4.
- **`kit.tracker.latest()` / `delta()` do not raise exceptions.** When the Tracker is occluded, has failed, or has been closed, they still return the last pose received, only with an ever-growing age `age_s`.
  So use `age_s` to judge whether a pose is fresh (Trackers run at about 250 Hz; normally `age_s` is far below 0.1 seconds),
  and use `kit.fatal` (or `kit.raise_if_fatal()`) to judge whether an unrecoverable fault has occurred.
  After such a fault, only the Trackers stop; glove data keeps flowing. Recovering the whole set requires restarting the process; see *03 Base Stations and Pairing* section 7.
- A program that reads data holds the gloves and Trackers exclusively, just like the recording scripts, `revohuman-live`, and ROS 2 nodes, and cannot run at the same time as them.

## 10. Common issues

| Symptom | Cause and fix |
|---|---|
| Error code `1102`, saying the serial port is in use | Another program has the glove open (`revohuman-live`, the visualization's live mode, a ROS 2 node, or another recording script); close it first |
| Error code `1900` | The Tracker driver is already held by another program; same as above |
| Error code `1905`, Tracker not ready | The startup check failed. The error message lists the causes and what to do; most often the base stations are not powered on, are occluded, or need recalibration. See *03 Base Stations and Pairing* |
| Error code `1804`, product model mismatch | The connected gloves are not P1 gloves, or one hand's Tracker is not connected. First run `python3 -m revohuman product identify` to see the identification result |
| Error `left 手的手套不在位` (left glove not present), or the same for right | This glove is not connected properly; check it as described in *02 Hardware Connection* section 3. The recording scripts in this chapter require both gloves to be present |
| Error code `1600`, recording gate closed | The glove time base is not established yet; right after power-on, wait a few seconds before recording. If it keeps happening, check the USB cables and HUB |
| `基站几何不达标` (base station geometry out of tolerance) | A base station has been moved or the calibration is no longer valid; recalibrate as described in *03 Base Stations and Pairing* |
| The button-recording script exits with exit code `3`, and the log has `N2406` | This glove's firmware does not support reporting the button during capture; use the timed recording in section 2 instead |
| Pressing the button does not start recording | The button pressed is not on the glove given by the argument; or the button was pressed before startup and the first press reported "end": just press it once more |
| `本段已作废` (this episode is aborted) | See section 5 |

For all error codes and log descriptions, see *07 Logging and Error Codes*.
