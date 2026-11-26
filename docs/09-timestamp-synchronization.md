# 09 Timestamp Synchronization

English | [简体中文](zh-CN/09-时间戳同步.md)

This chapter explains what each timestamp in P1 data means, how to use them to align the left and right hands and the gloves with the Trackers,
which indicators to watch day to day to judge whether time synchronization is healthy, and what to do when time-related notices, exceptions and errors appear.
For the complete definitions of each field and interface, see sections 6.1, 6.2 and 6.6 of the *RevoHuman SDK Reference Manual*.

## 1. Three things to remember first

1. **Always align on the sampling time `sample_wall_ns`.** The following are all **arrival times** and must not be used for alignment:
   `recv_wall_ns` in a frame, the `log_time` carried by MCAP messages, the time rosbag stores with each message, and the time in the `revohuman-live` broadcast.
   Glove data reaches the host about 10 ms after sampling, Tracker data less than 1 ms after; if you align on arrival times, the gloves will be offset by about 10 ms relative to the Trackers.
2. **Every frame you receive is within the accuracy requirement.** The SDK does not dispatch frames whose timestamps do not meet the accuracy requirement, and does not output two-hand pairs whose pairing accuracy does not meet the requirement.
   Frames carry no "is this frame accurate" field, so there is no need to check frame by frame; the cost is that there is no data for the period when something goes wrong (section 4).
3. **Within one program, all `*_wall_ns` share the same time base**, so values across hands and across devices can be subtracted directly.
   Whether data from multiple hosts can be aligned depends on the hosts' time synchronization (section 7).

## 2. Where timestamps come from

All `*_wall_ns` are Unix time (UTC) in nanoseconds, the same clock as Python's `time.time_ns()`;
to convert one to a readable time, use `datetime.datetime.fromtimestamp(ns / 1e9)`.

The glove and the Tracker each have an internal device counter (1 ms per count on the glove, 48 MHz on the Tracker), and every frame records the counter value at the moment of sampling.
On the host, the SDK continuously measures how the device counter corresponds to the host clock, converts the counter to host time, and obtains `sample_wall_ns`.
This conversion is built on the host's hardware clock, so NTP gradually adjusting the host clock rate (rate adjustment) does not affect it; for the effect of host clock jumps, see section 7.

| Data | Sampling time | Arrival time | Raw device counter | Notes |
|---|---|---|---|---|
| Glove `GloveFrame` | `sample_wall_ns`: the moment the encoders were sampled | `recv_wall_ns`, about 10 ms later | `device_tick` (1 ms per count) | Sampling time accurate to within 0.2 ms |
| Two hands `PairedFrame` | `left.sample_wall_ns` / `right.sample_wall_ns` | Each one's own `recv_wall_ns` | — | `skew_ns` = right − left; always 0 under two-hand hardware sync |
| Tracker `TrackerFrame` | `sample_wall_ns` | `recv_wall_ns`, less than 1 ms later | `timecode` (wraps around about every 89.5 seconds) / `tc_unwrapped` (unwrapped value; use it to compare across frames) | Includes a fixed offset, see 3.3; when occluded, `pose` is `None` but the timestamp is still present |
| Two Trackers `PairedTrackerFrame` | `sample_wall_ns` of `left` / `right` respectively | — | — | The two Trackers sample independently; `skew_ns` is not 0, up to about ±2 ms |
| Glove button `ButtonEvent` | `sample_wall_ns`: the sampling time of the first glove data frame in which this press was seen | `recv_wall_ns` | — | `None` while the time base is not yet established; use `recv_wall_ns` in that case |

For product models with tactile sensing or wrist cameras, tactile and camera data each have their own sampling time; see section 6.1 of the reference manual.

## 3. How to align

### 3.1 Left and right hands: use paired frames directly

Each `PairedFrame` output by `kit.synchronizer().subscribe()` (the `/paired` topic in a recording file) already holds the left and right hands at the same moment.
When P1 is opened with `open_kit()`, two-hand hardware sync is established by default: both hands sample at the same moment, `skew_ns` is always 0, and no further alignment is needed.

- The default pairing accuracy requirement is 1.0 ms (`pair_accuracy_ms=1.0`). When hardware sync is not established or is lost midway, the SDK pairs by each hand's own sampling time instead;
  `skew_ns` is then not 0 but still within the 1.0 ms requirement; pairs that do not meet the requirement are not output.
- To accept only hardware-synced pairs, pass `kit.synchronizer(pair_accuracy_ms=0.0)` the first time you get the synchronizer.
  Only one synchronizer is created per `kit`, and arguments passed later have no effect, so get it once before calling `kit.recorder()` or any other interface.
- `sync_strict` under the recording file's metadata `revohuman.sync_health` records whether the two hands were hardware-synced throughout this segment (section 3 of *04 Recording*).

### 3.2 Gloves and Trackers: nearest match by sampling time

Gloves run at 200 Hz and Trackers at about 250 Hz; the two do not sample at the same moments, so find the nearest frame by `sample_wall_ns` (or interpolate between two frames).
The function below finds, for each glove time, the Tracker frame with the nearest sampling time; a difference larger than `max_gap_ns` is recorded as `-1` (this happens when a Tracker is occluded or its stream drops out):

```python
import numpy as np


def nearest(t_query, t_ref, max_gap_ns=10_000_000):
    """For each time in t_query, find the index in t_ref (ascending, at least 2 entries) with the nearest sampling time; a difference larger than max_gap_ns is recorded as -1."""
    t_query = np.asarray(t_query, dtype=np.int64)
    t_ref = np.asarray(t_ref, dtype=np.int64)
    i = np.clip(np.searchsorted(t_ref, t_query), 1, len(t_ref) - 1)
    i = np.where(t_query - t_ref[i - 1] <= t_ref[i] - t_query, i - 1, i)
    return np.where(np.abs(t_ref[i] - t_query) <= max_gap_ns, i, -1)
```

Usage (`paired` is a list of two-hand paired frames and `tracker_right` a list of right-hand Tracker frames, read back from a recording file or received through a live subscription):

```python
trk = [f for f in tracker_right if f.pose is not None]          # drop frames that are occluded and have no pose
idx = nearest([pf.right.sample_wall_ns for pf in paired], [f.sample_wall_ns for f in trk])
for pf, k in zip(paired, idx):
    if k >= 0:
        joints, pose = pf.right.joints_deg, trk[k].pose            # right-hand joint angles and right-hand pose at the same moment
```

The Tracker produces a frame about every 4 ms, so nearest matching introduces a time error of at most about 2 ms. For finer results, interpolate position linearly between two frames and orientation with spherical interpolation (slerp).

### 3.3 The Tracker's fixed offset

The Tracker's `sample_wall_ns` includes a fixed internal device latency that the SDK does not subtract by default, so the Tracker has a **fixed** offset relative to the gloves.
It stays the same over time on a given Tracker and does not affect the Tracker's own timing (velocity, acceleration, intervals between consecutive frames);
it only needs handling when you do sub-millisecond absolute alignment between the gloves and the Tracker. For how to subtract it, see "Absolute phase" in section 6.1 of the reference manual.

### 3.4 ROS 2 and rosbag

Every `header.stamp` published by the ROS 2 node is a sampling time; the two-hand `joint_states` uses the left hand's sampling time (identical for both hands under two-hand hardware sync).
The time rosbag records for each message is the arrival time; when aligning across topics, use the `header.stamp` inside the message (section 6 of *06 ROS 2*).

### 3.5 Live broadcast revohuman-live

The `revohuman-live` broadcast is for live display only: each frame carries the time the host received the data (`t_ns`) and the sampling time (`sample_ns`),
but it keeps broadcasting while the timestamp gate is closed (section 4), and the sampling times during that period are not trustworthy. For data that needs alignment, record it (*04 Recording*) or subscribe in your own program (section 9 of *04 Recording*).

## 4. Timestamp gate

Each glove has a **timestamp gate** (called `录制门` (recording gate) in the logs): when the SDK has confirmed that this glove's sampling times are trustworthy, the gate is open and frames are dispatched normally;
when they are not trustworthy, the gate closes, and frames during that time are not dispatched to subscribers, only counted. Once the reason for the closure goes away, the gate reopens automatically, without reconnecting.

**Right after a glove is opened**, the SDK first has to establish the time base, so the gate stays closed for a short while and the log shows N2104 (`untrusted`, level INFO), N2105 and N2101 in turn; this is normal.
So no data can be read for the first few tens of milliseconds after opening; give `recv(timeout=...)` enough time.

**When the gate closes during operation**:

- Subscribers receive no frames from that hand during this time, and two-hand paired frames stop as well; the log shows N2102 (with the reason), and N2103 when the gate reopens (with how long it was closed and how many frames were blocked).
- While recording, a recorder created with `kit.recorder()` invalidates the entire current segment (N2501, section 5 of *04 Recording*); after the gate reopens, the next segment can be recorded as usual.
- Calling `rec.start()` while the gate is closed raises `CaptureGateClosed` (error code 1600).

Reading the gate state:

| How to read | Meaning |
|---|---|
| `glove.capture_ok` | Whether the gate is open (`glove` is obtained from `kit.glove("left")`) |
| `glove.capture_cause` | Category of the reason the gate is closed (section 6.2); `None` while the gate is open |
| `glove.stamp_trust` | Timestamp trust level: `trusted` is normal; `coasting` means accuracy is slightly reduced but still within the requirement, the gate stays open and data flows as usual; `untrusted` means not trustworthy, and the gate closes accordingly |
| `glove.on_capture_gate = callback` | Each time the gate opens or closes, `callback(glove, ok, detail)` is called, with `detail["cause"]` the reason category; the callback runs in an SDK thread, so do not do time-consuming work in it |
| `kit.status()["gloves"]["left"]` | Its `capture_ok` and `stamp_trust` are the same as above |

A subscription obtained with `glove.subscribe(gated=False)` keeps receiving frames while the gate is closed; those frames have `gate_open` set to `False`,
and their `sample_wall_ns` is not trustworthy: use them only for display or troubleshooting, not for analysis or alignment.

The Tracker has no such gate: Tracker frames from before the time conversion is established are not dispatched (this only affects the first moments after startup), and after that frames are always dispatched.
Whether Tracker data is usable is guarded by the base station geometry check (section 7 of *03 Base Stations and Pairing*).

## 5. Checking synchronization status

The program below opens the whole kit, collects for 10 seconds, and then prints each time synchronization indicator (while it runs, the gloves and Trackers must not be in use by another program):

```python
import time
from revohuman import open_kit

with open_kit("P1") as kit:
    sync = kit.synchronizer()
    time.sleep(10)
    for hand in ("left", "right"):
        g = kit.glove(hand)
        st = g.stats()
        gate = st.stream["capture_gate"]
        print(hand, "门开:", g.capture_ok, "可信度:", g.stamp_trust, "采样率:", st.effective_rate_hz,
              "被拦帧:", st.stream["frames_blocked"], "门开关记录:", len(gate["transitions"]))
        print("   采样刻档位:", st.stream["frames_by_tier"])
        lat = st.latency
        print("   链路延迟 ms p05/p50/p95:", lat["p05_ms"], lat["p50_ms"], lat["p95_ms"],
              "告警:", lat["high_alarm"], lat["jump_alarm"])
    s = sync.stats()
    print("配对档位:", s["pairs_by_tier"], "被拦对子:", s["pairs_blocked"], s["pair_block_reason"])
    print("硬件同步:", kit.pair.synchronized, "丢失:", s["strict_sync_lost"], "重建次数:", s["resyncs"])
    for role, r in kit.status()["tracker"]["roles"].items():
        print("tracker", role, "在线:", r["online"], "最新位姿距今 s:", r["pose_age_s"])
```

Normal values of each indicator:

| Indicator | Where to read it | Normal | What an abnormal value means |
|---|---|---|---|
| Timestamp gate | `glove.capture_ok` | `True` | The gate is closed and this hand produces no data; see `glove.capture_cause` for the reason (section 4) |
| Trust level | `glove.stamp_trust` | `trusted` | `coasting` still produces data; for persistent `untrusted`, handle it according to the reason in section 6.2 |
| Sampling rate | `stats().effective_rate_hz` | 200 | If clearly lower, check the USB cable, HUB and host load |
| Blocked frames | `stats().stream["frames_blocked"]` | 0 | Total number of frames not dispatched while the gate was closed |
| Gate open/close records | `stats().stream["capture_gate"]["transitions"]` | Only 1 record: the gate opening at startup | The last 64 gate opens/closes; each record carries `ok` (open or closed), `cause` and `wall_ns` (when it happened); a record of the gate reopening also carries `closed_ms` (how long it was closed) and `blocked` (frames blocked in the meantime) |
| Sampling-time tiers | `stats().stream["frames_by_tier"]` | All in `sample` | Frames counted by the source of their sampling time: `sample` is a sampling time converted from the device counter; `sample_biased` includes a fixed offset that was not subtracted; `recv` is not converted and equals the arrival time |
| Link latency | `stats().latency` | `p50_ms` about 10 | Percentiles (in milliseconds) of arrival time minus sampling time. `high_alarm`: median above 25 ms; `jump_alarm`: median has jumped by more than 5 ms (one sampling period) compared with startup. Each triggers N2107 once |
| Pairing tiers | `sync.stats()["pairs_by_tier"]` | All in `hw_sync` | `independent` means a stretch was paired by each hand's own sampling time (hardware sync not established or lost), and `skew_ns` is not 0 |
| Blocked pairs | `sync.stats()["pairs_blocked"]` / `["pair_block_reason"]` | 0 / `None` | Number of pairs not output because they did not meet the pairing accuracy requirement, and the most recent reason |
| Two-hand sampling alignment | `sync.stats()["device_skew_degraded"]`; `["device_skew_p50_ms"]`, `["device_skew_abs_p99_ms"]`, `["device_skew_abs_max_ms"]`; `["device_skew_bad_windows_total"]`, `["device_skew_skipped_windows"]` | `False`; all absolute values below 1; 0, 0 | Under hardware sync, every 200 pairs (about 1 second) the SDK computes statistics of the difference between the two hands' `sample_wall_ns` at the same sampling time (in milliseconds, right minus left): the median, and the p99 and maximum of the absolute value. When the median exceeds 1 ms, N2205 is reported once as a reminder; this does not set the degraded state and has no corresponding N2206. When the median exceeds 2.5 ms 2 times in a row, `device_skew_degraded` is set (N2205); it clears automatically after 5 consecutive times back within 1 ms (N2206, counted in `device_skew_heals`), and after it clears, exceeding 1 ms again triggers a new reminder. `device_skew_bad_windows_total` is the number of times the statistic exceeded 2.5 ms; `device_skew_skipped_windows` is the number of times it was not evaluated because either hand's timestamp quality had dropped, with the reason in `device_skew_skip_reason` |
| Hardware sync | `kit.pair.synchronized`; `sync.stats()["strict_sync_lost"]`, `["resyncs"]` | `True`; `False`, 0 | After sync is lost, the SDK rebuilds it automatically in the background; the number of rebuilds is counted in `resyncs` (N2202–N2204 in section 6.1) |
| Tracker pose | `kit.status()["tracker"]["roles"][hand]["pose_age_s"]`, or the second item returned by `kit.tracker.latest(hand)` | Below 0.1 seconds | A steadily growing value means this Tracker is occluded or its stream has dropped |
| Recording file | Metadata `revohuman.sync_health` | `sync_strict` is true, `aborted` is false | This segment was not hardware-synced throughout / this segment was invalidated (sections 3 and 5 of *04 Recording*) |

The ROS 2 node puts the gloves' `capture_gate`, `frames_by_tier`, `mapper_quality.stamp_trust` and other fields in its status topic; see section 3 of *06 ROS 2*.

## 6. Exceptions and errors

### 6.1 Runtime notices

Situations related to time synchronization are reported as N-code log entries (for the format and the other notice codes, see section 3 of *07 Logging and Error Codes*):

| Code | What happened | What the SDK does automatically | What you need to do |
|---|---|---|---|
| N2101 | Timestamp gate opened, frames start flowing | — | Nothing |
| N2102 | Timestamp gate closed (with reason) | Stops dispatching this hand's frames; reopens automatically once the reason goes away | Handle according to the reason (6.2); if recording, this segment is invalidated |
| N2103 | Timestamp gate reopened (with how long it was closed and how many frames were blocked) | Resumes dispatching | Nothing; if it occurs frequently, investigate according to the reason |
| N2104 / N2105 | Timestamp trust level downgraded (with reason) / restored | When downgraded to `coasting`, frames keep flowing; when downgraded to `untrusted`, the gate closes | A single `untrusted` (INFO) right after a glove is opened is normal; if it recurs during operation, investigate according to the reason |
| N2106 | Establishing the time base has not finished; retrying | Retries automatically | If it does not finish for a long time, check the USB connection and the glove firmware |
| N2107 | Abnormal glove-to-host transfer latency | Reports only; frame output is not affected (sampling times are not affected by transfer latency) | Check the USB cable, HUB and host load |
| N2201 | Two-hand hardware sync not established, falling back to software pairing | Pairs by each hand's own sampling time; `skew_ns` is not 0 | If hardware sync is required, reopen the devices; see 3.1 |
| N2202 | Two-hand hardware sync lost (with which hand) | Rebuilds automatically in the background | If it occurs frequently, check USB power and cables |
| N2203 | Two-hand hardware sync rebuilt | Resumes hardware-sync pairing | Nothing |
| N2204 | Rebuilding two-hand hardware sync failed | Continues with software pairing | Reopen the devices; if it still fails, unplug and replug the glove USB |
| N2205 / N2206 | Deviation between the two hands' sampling times / recovered. N2205 has three causes: median above 1 ms (reminder, reported only once); median above 2.5 ms 2 times in a row (degraded); hardware-sync pairing check failed (with `暂按时间戳配对` (pairing by timestamp for now)) | Reminders and the degraded state are only reported, and pairs are still output; the degraded state clears automatically and N2206 is reported once the deviation has stayed within 1 ms for about 5 consecutive seconds (this check is paused while either hand's timestamp quality is reduced; see "Two-hand sampling alignment" in section 5). While the pairing check is failing, the two frames of the same sampling time are also paired by their own sampling times instead; once the check passes again, this recovers and N2206 is reported | During this period, do not subtract the left and right hands' `sample_wall_ns` from each other directly; rely on the paired frames; if it persists, reopen the devices to rebuild two-hand sync |
| N2301 | USB link unstable, timestamps temporarily not trustworthy | Closes the gate; recovers automatically once the link is stable | Check the cables, HUB and power supply |
| N2302 / N2303 / N2304 | USB link resetting / recovery failed / recovered | Resets the link automatically | If recovery fails, unplug and replug the glove USB |
| N2501 | Entire recording invalidated (with reason and file path) | Finishes writing the current segment's file and marks it invalid | Do not use this segment's data; once the reason goes away, continue with the next segment |
| N2502 | Recording aborted because strict two-hand sync was lost | The recorder stops and refuses to start again | Reopen the devices and rebuild two-hand sync before recording again (see 1700 in 6.3) |

### 6.2 Reasons for the timestamp gate closing

`glove.capture_cause` and the `cause` in the gate open/close records take one of the five values below; the N2102 / N2104 log lines show the corresponding Chinese description, not these English values:

| Reason | Meaning | Action |
|---|---|---|
| `clock_not_ready` | The time base has not been established yet or has expired, and is being rebuilt | Usually recovers automatically within a few seconds; no action needed |
| `clock_degraded` | The quality of the time base has dropped | If it persists, check the USB cable, HUB and host load |
| `host_clock_event` | Host clock jump (wake from sleep, manual time change, NTP step) | Recovers automatically; avoid during recording, see section 7 |
| `device_reset` | Glove reset | Recovers automatically; if it occurs frequently, check the power supply |
| `link_unstable` | USB link unstable | Check the cables, HUB and power supply |

### 6.3 Error codes

| Code | Exception | When it occurs | Action |
|---:|---|---|---|
| 1600 | `CaptureGateClosed` | `rec.start()` is called while the timestamp gate is closed | Wait for the gate to open (`glove.capture_ok` is true, or N2101 / N2103 appears in the log) before starting |
| 1700 | `StrictSyncLost` | `rec.start()` is called again after two-hand hardware sync was lost and the recorder was aborted (N2502). This only occurs if you connect the two yourself with `sync.on_sync_lost(rec.abort)`; `kit.recorder()` does not connect them by default and keeps recording while sync is lost, recording `sync_strict` as false for that segment | Reopen the devices (`open_kit()`) and rebuild two-hand sync before recording again |
| 1205 | `FirmwareIncompatible` | The glove firmware does not support, or is incompatible with, the time synchronization feature: this glove's data stream is closed and not reconnected, and N2405 appears in the log. The specific cause is 1203 / 1204 (`TimeProbeUnsupported` / `TimeProbeIncompatible`) | Upgrade the glove firmware (section 2.4 of the reference manual) |

If an accuracy requirement is set stricter than the device can achieve (for example a Tracker `stamp_accuracy_ms` below 0.2), a `ValueError` is raised right at creation, rather than the problem only being discovered at runtime.
For general use of error codes, see section 4 of *07 Logging and Error Codes*.

## 7. Host clock and multiple hosts

- **Rate adjustment has no effect**: when an NTP client gradually adjusts the host clock rate, sampling times are not affected; there is no need to turn time synchronization off.
- **Jumps cause a brief pause**: when the host clock jumps (NTP's first synchronization or a step during operation, a manual time change, wake from sleep),
  all `*_wall_ns` shift together by the same amount and remain comparable with each other; the gloves' timestamp gates close briefly (reason `host_clock_event`) and reopen automatically.
  If this happens during recording, the current segment is invalidated, so **do not change the system time or let the host sleep while recording**.
  When only the Trackers are open and no gloves, this shift is not applied: the data streams remain consistent with each other, but differ from the post-jump host time by the size of the jump.
- **Recommended time synchronization setup**: step only at boot, and only adjust the rate during operation. Taking chrony as an example, put `makestep 1 3` in `/etc/chrony/chrony.conf`
  (for the first 3 synchronizations after boot, a step is allowed when the offset exceeds 1 second; after that only the rate is adjusted), then check the synchronization status:

```bash
chronyc tracking
```

- **When data from multiple hosts needs to be aligned**, have them synchronize to the same NTP server. The alignment accuracy between two hosts is the time synchronization accuracy between them;
  the SDK neither checks nor compensates for time differences between hosts.
- Multiple programs on the same host (for example, one program recording the gloves and another recording other sensors) use the same host clock, so their `*_wall_ns` can be compared directly,
  provided the host clock has not jumped since both programs started (for the effect of jumps, see the second item above).

## 8. FAQ

**`recv_wall_ns` is about 10 ms later than `sample_wall_ns`. Is that normal?**
Yes. This is the glove-to-host transfer latency, which the sampling time already excludes. If the latency changes substantially, the SDK reports N2107.

**The log shows `untrusted（时间基准未就绪）` (untrusted (time base not ready)) right after a glove is opened?**
This is normal and appears once per hand; the N2105 and N2101 that follow indicate it is ready.

**A recorded segment is marked invalid, and the reason says the timestamp gate closed?**
The sampling times during this period are not trustworthy; do not use the data. Use the reason in N2102 (6.2) to troubleshoot; after the gate reopens, you can continue recording the next segment.

**`skew_ns` in `/paired` is not 0?**
Hardware sync was not established or was lost midway, and the SDK switched to software pairing; the pairs are still within the pairing accuracy requirement. Look for N2201 / N2202 in the log;
to accept only hardware-synced pairs, set `pair_accuracy_ms=0.0` as described in 3.1.

**There is a fixed difference between the gloves and the Trackers?**
This is the Tracker's fixed offset; see 3.3. It does not change over time; if you do not need sub-millisecond absolute alignment, you can ignore it.

**Is the time in the data local time or UTC?**
UTC Unix time (in nanoseconds). To display it as local time, use `datetime.datetime.fromtimestamp(ns / 1e9)`, which converts according to the host's time zone.
