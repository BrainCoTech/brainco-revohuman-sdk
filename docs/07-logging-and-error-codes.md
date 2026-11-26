# 07 Logging and Error Codes

English | [简体中文](zh-CN/07-日志与错误码.md)

This chapter describes where each RevoHuman program writes its logs, what the runtime notice codes (N codes) and the exception error codes mean,
and how to collect information when you run into a problem. It covers P1 gloves only.

## 1. Where the logs are

| Program | Log destination |
|---|---|
| `python3 -m revohuman …`, `revohuman-lighthouses …` command lines | Printed to the terminal (standard error); add `-v` to enable debug logging, with `-v` placed before the subcommand |
| `revohuman-live` (*05 Simulation and Visualization*) | Printed to the terminal that started it; add `-v` to enable debug logging |
| Your own Python programs (such as the scripts in *04 Recording*) | Determined by the program's `logging` setup; the SDK's logger names all start with `revohuman` |
| ROS 2 nodes (*06 ROS 2*) | Printed to the terminal running `ros2 launch`, and also written to `~/.ros/log/` |
| 3D visualization `run_viewer.sh` | Printed to the terminal that started it |
| Tracker and base station state files | `~/.local/state/revohuman/tracker/` (the directory given by `REVOHUMAN_TRACKER_STATE_DIR` when it is set; otherwise `$XDG_STATE_HOME/revohuman/tracker/` when `XDG_STATE_HOME` is set); view them with `python3 -m revohuman tracker status` (see *03 Base Stations and Pairing*) |
| Base station geometry | `~/.config/libsurvive/config.json`, written by base station calibration; do not edit or delete it by hand |

To save the complete log of one command to a file (debug level):

```bash
python3 -m revohuman -v doctor 2>&1 | tee ~/revohuman-doctor.log
```

### 1.1 Log colors

When you run `python3 -m revohuman`, `revohuman-live`, `revohuman-lighthouses` and the 3D visualization in a terminal, log lines are shown in different colors by level;
the whole log line is colored together with any exception traceback (the N codes in section 3 are also colored by their own levels):

| Level | Color |
|---|---|
| DEBUG (only with `-v`) | Dim |
| INFO | Not colored |
| WARNING | Yellow |
| ERROR | Red |
| CRITICAL | Bold red |

This is controlled by the environment variable `REVOHUMAN_LOG_COLOR`; values are case-insensitive, and any other value is treated as `auto`:

| Value | Effect |
|---|---|
| `auto` (default) | Colored when the output is a terminal, `NO_COLOR` is not set, and `TERM` is not `dumb` |
| `always` | Always colored, regardless of `NO_COLOR`. For example, `REVOHUMAN_LOG_COLOR=always python3 -m revohuman -v doctor 2>&1 \| less -R` to page through the colored log |
| `never` | Not colored |

- When the output is redirected to a file or a pipe (such as the `tee` above), it carries no color, and the content is byte-for-byte identical to uncolored output; all sample output in this chapter is in this form.
- In your own Python programs, the log format and colors are determined by the program's own `logging` setup; the SDK does not color it.
- ROS 2 node logs are colored by ROS 2 itself and are not controlled by `REVOHUMAN_LOG_COLOR` or `NO_COLOR`:
  colored when `ros2 run` runs directly in a terminal, not colored when started with `ros2 launch`; set `RCUTILS_COLORIZED_OUTPUT=1` to force color, `=0` to turn it off.

## 2. Information to collect when reporting a problem

When reporting a problem, run the commands below with the gloves connected, and provide the generated `~/revohuman-report.tgz` together with the terminal output from when the problem occurred:

```bash
cd ~
python3 -m revohuman doctor --json > revohuman-doctor.json
python3 -m revohuman tracker status > revohuman-tracker-status.txt 2>&1
tar czf revohuman-report.tgz revohuman-doctor.json revohuman-tracker-status.txt .local/state/revohuman/tracker .config/libsurvive/config.json
ls -l revohuman-report.tgz
```

The archive contains the glove health check results, the Tracker and base station state (including the allowlisted base stations) and calibration status, and the base station geometry; it contains no recording data.

## 3. Runtime notices (N codes)

Conditions the SDK encounters at runtime (time base changes, two-hand synchronization, the USB link, reconnection, invalidated recordings, etc.) are reported as log lines. Each one starts with `N` followed by four digits,
and the square brackets contain the serial port involved:

```text
2026-09-27 23:51:04,376 INFO revohuman.glove: N2104 [/dev/ttyACM0] 时间戳质量降为 untrusted（时间基准未就绪）
2026-09-27 23:51:04,408 INFO revohuman.glove: N2105 [/dev/ttyACM0] 时间戳质量恢复为 trusted
2026-09-27 23:51:04,408 INFO revohuman.glove: N2101 [/dev/ttyACM0] 录制门已开，开始出帧
```

Identify a notice by its leading code; there is no need to parse the Chinese text after it. The `录制门` (recording gate) in the notices is what other chapters call the timestamp gate. Notice codes that P1 uses:

| Code | Level | Meaning |
|---|---|---|
| N2101 | INFO | Recording gate opened; frames start being delivered |
| N2102 | WARNING | Recording gate closed (with the reason); frame dispatch is paused and resumes automatically once the gate opens |
| N2103 | INFO | Recording gate reopened (with how long it was closed and how many frames were held back in the meantime) |
| N2104 | WARNING | Timestamp quality downgraded (with the reason). Right after the gloves are opened, `untrusted（时间基准未就绪）` (time base not ready) appears once for each hand at INFO level; this is normal |
| N2105 | INFO | Timestamp quality restored |
| N2106 | WARNING | Time base setup not completed; retrying |
| N2107 | WARNING | Abnormal transfer latency from glove to host |
| N2201 | WARNING | Two-hand hardware synchronization not established; falling back to software pairing (lower pairing precision) |
| N2202 | ERROR | Two-hand hardware synchronization lost |
| N2203 | WARNING | Two-hand hardware synchronization re-established |
| N2204 | ERROR | Failed to re-establish two-hand hardware synchronization |
| N2205 / N2206 | WARNING / INFO | Two-hand sampling alignment deviates / has recovered. N2205 has three causes: the median difference between the two hands' sampling times exceeds 1 ms (a reminder, reported only once); it exceeds 2.5 ms twice in a row (downgrade); the hardware synchronization pairing check failed and pairing temporarily goes by timestamps. For the meaning and handling see section 6 of *09 Timestamp Synchronization* |
| N2301 | WARNING | USB link unstable; timestamps temporarily untrusted |
| N2302 / N2303 / N2304 | WARNING / WARNING / INFO | USB link resetting / recovery failed / recovered |
| N2401 / N2402 | WARNING | Reconnection failed, retrying with backoff / stream restored |
| N2403 | WARNING | Serial port name changed (USB re-enumeration); relocated automatically |
| N2404 | WARNING | Stream interrupted; reconnecting |
| N2405 | ERROR | Firmware incompatible; the stream is closed and will not reconnect; the glove firmware must be upgraded |
| N2406 | WARNING | The glove firmware does not support reporting button presses during capture; the button is unavailable, and data capture is not affected. Appears only when the button is explicitly enabled (`button=True`) |
| N2407 | ERROR | An SDK program error occurred while connecting; still retrying automatically. This is not a device or cable fault, and changing the cable or replugging will not help; the same kind of error is reported only once, and the cumulative count is in `glove.stats().stream["unexpected_errors"]` |
| N2501 | ERROR | The entire recording is invalidated (with the reason and file path); see *04 Recording* |
| N2502 | ERROR | Recording aborted because strict two-hand synchronization was lost |

Notices related to time quality report only the broad category of the cause:

| Cause | Meaning | Action |
|---|---|---|
| `clock_not_ready` | The time base has not been established yet or has expired; it is being rebuilt | Usually recovers on its own within a few seconds; no action needed |
| `clock_degraded` | Time base quality degraded | If it persists, check the USB cable and the host load |
| `host_clock_event` | Host clock jump (suspend/resume, manual time change, time sync step) | Recovers automatically; avoid during recording |
| `device_reset` | Glove reset | Recovers automatically; if it happens often, check the power supply |
| `link_unstable` | USB link unstable | Check the cable and power supply; plug the glove directly into the host |

For the symptoms of each timestamp gate and two-hand synchronization notice code, what the SDK does automatically and what you need to do, see section 6 of *09 Timestamp Synchronization*.

### 3.1 Writing notice codes as JSON

Each notice log record also carries structured fields: `record.revohuman_code` (such as `"N2101"`) and `record.revohuman_fields` (serial port, handedness, count, duration, etc.).
The program below opens both gloves, runs for the given number of seconds, and writes the notices it receives to a file, one JSON object per line. Save it as `~/revohuman_notices.py`:

```python
"""打开 P1 两只手套运行指定秒数，把 RevoHuman 运行期提示（N 码）按 JSON 行写进文件。

用法：python3 ~/revohuman_notices.py <秒数> [输出文件，默认 ~/revohuman-notices.jsonl]
"""
import json
import logging
import os
import sys
import time

from revohuman import RevoHumanError, error_code, format_error, open_kit


class NoticeJsonLines(logging.Handler):
    """Collect only log records that carry a notice code, writing each as one JSON line."""

    def __init__(self, path):
        super().__init__()
        self.file = open(path, "a", encoding="utf-8")

    def emit(self, record):
        code = getattr(record, "revohuman_code", None)
        if code is None:
            return
        line = {
            "time": record.created,
            "level": record.levelname,
            "code": code,
            "fields": getattr(record, "revohuman_fields", {}),
            "text": record.getMessage(),
        }
        self.file.write(json.dumps(line, ensure_ascii=False, default=str) + "\n")
        self.file.flush()


def main():
    if len(sys.argv) not in (2, 3):
        print(__doc__.strip(), file=sys.stderr)
        return 2
    seconds = float(sys.argv[1])
    path = sys.argv[2] if len(sys.argv) == 3 else os.path.expanduser("~/revohuman-notices.jsonl")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    logging.getLogger("revohuman").addHandler(NoticeJsonLines(path))
    try:
        with open_kit("P1", sensors={"encoder"}) as kit:
            print(f"已打开 {kit.hands}，运行 {seconds:g} 秒")
            time.sleep(seconds)
    except RevoHumanError as exc:
        print(f"打开失败：{format_error(exc)}（错误码 {int(error_code(exc))}）", file=sys.stderr)
        return 1
    print(f"提示已写入 {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

Run it for 5 seconds and view the result:

```bash
python3 ~/revohuman_notices.py 5
cat ~/revohuman-notices.jsonl
```

Sample output (the terminal log printed while the gloves are being opened comes first and is omitted; the file printed by `cat` has three entries per glove, N2104 → N2105 → N2101; only the left hand is shown here):

```text
已打开 [<Handedness.LEFT: 'left'>, <Handedness.RIGHT: 'right'>]，运行 5 秒
提示已写入 /home/<用户名>/revohuman-notices.jsonl
{"time": 1790553064.376964, "level": "INFO", "code": "N2104", "fields": {"state": "untrusted", "cause": "时间基准未就绪", "port": "/dev/ttyACM0"}, "text": "N2104 [/dev/ttyACM0] 时间戳质量降为 untrusted（时间基准未就绪）"}
{"time": 1790553064.408402, "level": "INFO", "code": "N2105", "fields": {"state": "trusted", "port": "/dev/ttyACM0"}, "text": "N2105 [/dev/ttyACM0] 时间戳质量恢复为 trusted"}
{"time": 1790553064.4084709, "level": "INFO", "code": "N2101", "fields": {"port": "/dev/ttyACM0"}, "text": "N2101 [/dev/ttyACM0] 录制门已开，开始出帧"}
```

The file is appended to, so repeated runs accumulate.

`sensors={"encoder"}` means only the gloves are opened and the Trackers are not started, so opening takes only a few seconds.

## 4. Error codes

Exceptions raised when a program fails all inherit from `revohuman.RevoHumanError`. `revohuman.error_code(exc)` gives the numeric error code,
and `revohuman.format_error(exc)` gives readable text that includes the code (this is how the program in the previous section prints it). When a command-line tool fails, it prints only the text description, without the code.

| Code | Exception | Scenario | Action |
|---:|---|---|---|
| 1000 | `RevoHumanError` | Generic error | Read the text description |
| 1100 | `TransportError` | USB communication with the glove failed | Check the USB connection, replug the glove and retry |
| 1101 | `Disconnected` | Connection lost / subscription closed | Check the USB connection, then reopen |
| 1102 | `PortBusyError` | The glove serial port is held by another program (the PID of the holder is included) | Close the program holding the glove (`revohuman-live`, a recording script, a ROS node, etc.) |
| 1200 | `ProtocolError` | The glove's reply does not conform to the communication protocol | Replug the glove and retry; if it still occurs, collect information as in section 2 and contact technical support |
| 1201 | — | Abnormal reply from the glove | Replug the glove and retry |
| 1202 | `UnsupportedFeatureError` | The firmware does not support this feature | Upgrade the glove firmware |
| 1203 / 1204 | `TimeProbeUnsupported` / `TimeProbeIncompatible` | The firmware does not support / is incompatible with the time synchronization feature | Upgrade the glove firmware |
| 1205 | `FirmwareIncompatible` | Incompatible firmware version | Upgrade the glove firmware |
| 1300 | `CalibrationError` | Glove joint zero-pose calibration failed, or only some of the encoders succeeded | Lay the glove flat with all five fingers straight, keep it still, and rerun `python3 -m revohuman glove calibrate --hand left` (`right` for the right hand); see section 5 of *02 Hardware Connection* |
| 1400 | `UnregisteredGloveError` | Neither the handedness recorded in the glove firmware nor the MCU serial number tells whether it is a left or right hand (the firmware has no handedness recorded, both gloves have the same serial number, or the serial number has no handedness character) | Specify by serial port instead: add `--port` to `firmware` and `glove calibrate`; in programs pass `port=` / `connect_pair(left_port=, right_port=)`; for ROS 2 nodes set the parameters `port` / `left_port`, `right_port` (these are not launch arguments; put them in `params_file` or pass them with `ros2 run … --ros-args -p`); if still unsure, collect information as in section 2 and contact technical support |
| 1401 | `GloveNotPresent` | The glove for the hand being opened is not plugged in (for example, both hands are opened by default but only one glove is plugged in) | Plug in that glove; when using only one hand, add `--hands left` or `--hands right` on the command line |
| 1403 | `HandConflictError` | The handedness recorded in a glove's MCU firmware contradicts its serial number | First confirm which glove is actually that hand, then specify by serial port (same action as 1400); if still unsure, collect information as in section 2 and contact technical support |
| 1500 | `StreamRateConflict` | The same glove is subscribed more than once at different rates | Use a single rate throughout the program |
| 1600 | `CaptureGateClosed` | Recording started while the timestamp gate is closed | Wait for the gate to open (N2101 / N2103) before starting; see *04 Recording* |
| 1700 | `StrictSyncLost` | Recording started again after two-hand hardware synchronization was lost and the recorder had aborted (see section 6.3 of *09 Timestamp Synchronization*) | Check the USB connections of both gloves, reopen the devices, then record again |
| 1800 | `TactileUnsupported` | The program requires tactile (`require_tactile=True`), but this glove's product model has no tactile sensing | Use a glove with tactile sensing, or do not require tactile |
| 1801 | `TactileUndetermined` | It cannot be determined yet whether this glove has tactile sensing (the device information has not been read, or the firmware is still probing peripherals) | Wait a moment and retry |
| 1802 | `VariantMismatch` | The left and right gloves are different product models (one with tactile, one without) | Use a pair of the same product model |
| 1803 | `ProductUnidentified` | The product model cannot be determined | Collect information as in section 2 and contact technical support |
| 1804 | `ProductMismatch` | The declared product model does not match the hardware present, or an expected sensor is missing (such as an unconnected Tracker) | Check that both glove units are fully connected; see *02 Hardware Connection* |
| 1805 | `TactileTareError` | Tactile tare did not succeed: the product model has no tactile sensing, the firmware does not support host-triggered tare, or the glove refused or timed out | Read the reason in the error; it is not resent automatically. Check the glove's light indication before retrying, or long-press the button on the glove for about 2 seconds to tare (*RevoHuman SDK Reference Manual* 6.4) |
| 1900 | `TrackerBusyError` | The Tracker is held by another program | Close the program holding the Tracker |
| 1901 | `TrackerStreamDead` | No Tracker data is received after startup, or the stream stops during operation | Check the Tracker connection, device permissions and base stations; see *03 Base Stations and Pairing* |
| 1902 | `WorldFrameChanged` | A base station was moved during capture, or a base station outside the allowlist was solved into the coordinate frame | Base station moved: secure the base stations, recalibrate and restart the program; foreign base station: turn it off or move it out of view, then restart the program |
| 1903 | `LighthouseError` | The base station allowlist, the channel preflight check or a base station write operation did not pass (each reason is listed) | Act on the reasons; see *03 Base Stations and Pairing* |
| 1904 | `CalibrationRejected` | Base station calibration or zero-pose calibration was rejected or not completed | Act on the reason; see *03 Base Stations and Pairing* |
| 1905 | `TrackerNotReady` | The Tracker startup check did not pass (with the reason and the action to take) | Follow the message; see *03 Base Stations and Pairing* |
| 1906 | `GeometryLost` | Base station geometry was lost during capture | Follow the message; see *03 Base Stations and Pairing* |

### 4.1 Example: glove in use (1102)

Only one program can open a glove at a time. In terminal 1, start the broadcast with only the gloves opened:

```bash
revohuman-live --no-tracker
```

While the broadcast is running, open the gloves again in terminal 2:

```bash
python3 ~/revohuman_notices.py 3 || echo "退出码 $?"
```

Sample output:

```text
2026-09-27 23:51:25,532 INFO revohuman.manager: left 手 → 单元 1-1 串口 /dev/ttyACM0（按 serial 定位）
2026-09-27 23:51:25,534 INFO revohuman.manager: right 手 → 单元 3-1 串口 /dev/ttyACM1（按 serial 定位）
打开失败：E1102 PortBusyError: 串口 /dev/ttyACM0 已被其他进程独占（占用进程 pid=144）（错误码 1102）
退出码 1
```

The number after `pid=` is the process ID of the program holding the glove; use `ps -p <PID> -o pid,cmd` to see which program it is.

After stopping the broadcast with `Ctrl+C` in terminal 1, the gloves open normally when you run it again.

## 5. Command-line exit codes

`python3 -m revohuman …`:

| Exit code | Meaning |
|---|---|
| 0 | Success |
| 1 | Failed, not ready, or blocked by the Tracker startup check |
| 2 | Usage error (wrong arguments), or an error in the tool itself (such as Bluetooth being unavailable) |
| 3 | Rejected, or product model mismatch |

`revohuman-live` (same as `python3 -m revohuman live`): pressing `Ctrl+C` after it prints `广播中` (broadcasting) ends it with exit code `0`;
pressing `Ctrl+C` before `广播中` has been printed gives exit code `130`; if the gloves cannot be opened, `--product` does not match the hardware present, or the broadcast socket cannot be used, the exit code is `2`.
A Tracker that cannot be opened does not affect the exit code: `revohuman-live` prints the reason and broadcasts the glove data as usual.

`revohuman-lighthouses` exit codes: `0` success; `1` there are gaps or conflicts, the plan has not been executed yet, or writing to a base station failed; `2` Bluetooth unavailable or busy, or a usage error. A ROS 2 node exiting with exit code 5 indicates a Tracker mode switch and is restarted automatically by launch; see *06 ROS 2*.

## 6. Common problems

| Symptom | Action |
|---|---|
| `LIBUSB_ERROR_ACCESS` or `libusb couldn't open USB device … Permission denied` is reported, followed by `TrackerStreamDead` (1901) | The Tracker device rule is not installed or not in effect: install the rule as in section 6 of *01 Installation and System Requirements*, run `sudo udevadm control --reload-rules && sudo udevadm trigger`, then replug the glove |
| `Permission denied: '/dev/ttyACM0'` is reported | The current user is not in the `dialout` group: add it as in section 5 of *01 Installation and System Requirements*, then log out and log back in |
| A command in a message is written as `python -m revohuman …`, and running it reports `python: command not found` | On Ubuntu 22.04 the command is `python3`; replace `python` with `python3` and run it again |
| `revohuman-live: command not found` | `~/.local/bin` is not in `PATH`: open a new terminal and try again (see section 7 of *01 Installation and System Requirements*) |
| N2301 / N2404 appear repeatedly in the log | Unstable USB link: change the cable, do not go through a hub, and check the host power supply |
| N2407 appears in the log | An issue in the SDK itself, not the hardware: keep the complete log (preferably with debug logging enabled) and report it to us; until then this glove keeps retrying and produces no data |
| N2501 appears during recording | That segment of data is invalidated; see *04 Recording* |
