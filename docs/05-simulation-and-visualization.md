# 05 Simulation and Visualization

English | [简体中文](zh-CN/05-仿真与可视化.md)

The `simulation/` directory of the source bundle contains a 3D visualization program that shows a pair of P1 gloves live in one window:

| View | Content |
|---|---|
| Left: Cartesian view | Both hands are placed in one world frame at the positions and orientations given by the Trackers, and the fingers move with the joint angles; the upper-left corner lists the pose quaternions |
| Right: fixed-base view | The two hands are placed at fixed positions and only the finger joints are shown; the upper-left corner lists a joint-angle table |

The data comes from the SDK's local broadcast: the command `revohuman-live` opens the gloves and Trackers and broadcasts the joint angles and poses to subscribers on the same host;
the visualization only subscribes to this broadcast and **does not access the gloves or Trackers directly**. The two are therefore started and stopped separately,
and you can also open several visualization windows on the same broadcast at once.

## 1. Before you start

- You have completed all steps in *01 Installation and System Requirements*, including section 9, "Install the 3D visualization environment";
  or you used the one-step install into a virtual environment in section 11 of *01 Installation and System Requirements*. With a virtual environment, `run_sim_host.sh` in section 4.1 uses it automatically;
  before running `revohuman-live` directly in a terminal (section 4.2), first run `source ~/.venvs/revohuman_sdk/bin/activate`.
- With real gloves, you also need to connect the gloves as described in *02 Hardware Connection* and complete base station pairing as described in *03 Base Stations and Pairing*.
  The demo mode in section 3 needs no hardware; use it first to confirm that the visualization itself works.

All later commands in this chapter are run in the visualization directory; change into it first:

```bash
cd ~/revohuman-release/revohuman-0.1.4/simulation
```

## 2. How the visualization works

```text
Terminal 1                                        Terminal 2
revohuman-live          ── local broadcast ──▶    ./run_viewer.sh
Opens gloves and Trackers, starts broadcasting    Subscribes to the broadcast and displays it
```

- **Either one can be started first.** If the visualization starts first, the window shows `waiting for the SDK broadcast ...` and lists the command to run in terminal 1;
  as soon as the broadcast starts, the window connects automatically and shows both hands.
- **The window does not exit when the SDK exits**: the view freezes on the last frame and dims, and a banner at the top states the reason for the disconnect;
  it reconnects automatically once `revohuman-live` is started again.
- Only one `revohuman-live` can broadcast at a time; there can be several subscribers (visualization windows).
- If you do not want two terminals, use `run_sim_host.sh` from section 4.1 to start and stop both with one command.

## 3. Demo mode (no gloves)

Open the window with the visualization's built-in demo data, without any hardware connected:

```bash
./run_viewer.sh --demo P1
```

Do not omit the `P1` after `--demo`; it specifies the product model to demo.

Once the window opens you should see: on the left, two hands each tracing small circles in space, bobbing up and down and slowly rotating at the wrist, above a ground grid;
on the right, two hands at fixed positions with the fingers flexing and extending slightly, and the joint-angle table in the upper-left corner changing with them. The terminal prints:

```text
[viewer] 演示广播 P1（left、right）
[viewer] SDK demo（pid 12345）：型号 P1（demo P1，识别来源 demo），left demo，right demo
[viewer] left 触觉：none (P1 has no tactile)
[viewer] right 触觉：none (P1 has no tactile)
[viewer] 笛卡尔视图：tracker {'left': 'DEMO-L', 'right': 'DEMO-R'}
[revohuman_sim] 场景就绪（0.1 s）
```

P1 has no tactile sensors; `触觉：none (P1 has no tactile)` (tactile: none) is a normal message.

Close the window or press `Ctrl+C` in the terminal to exit; on exit the terminal prints the frame count and refresh rate of the session, for example
`[view] 窗口关闭：3580 帧 / 60.0s（59.7 fps）` (window closed: 3580 frames / 60.0 s).

To demo only one hand, add `--demo-hands left` (or `right`).

## 4. With real gloves

### 4.1 Starting with one command

```bash
cd ~/revohuman-release/revohuman-0.1.4/simulation
./run_sim_host.sh
```

The script first starts the broadcast `revohuman-live` in the background, waits until the broadcast is ready (about half a minute, as it waits for base station data and the first Tracker pose), and then opens the visualization window;
while it waits, the broadcaster's output is shown in the terminal as well. It waits at most 90 seconds (the environment variable `REVOHUMAN_SIM_WAIT` changes the number of seconds); if the broadcast is still not ready by then, it opens the window anyway,
and the window connects automatically once the broadcast is ready. Once the window is connected to the broadcast, the relative position and orientation of the two hands in the Cartesian view on the left should match the real hands, and the fingers on the right flex and extend with the real hands.

- **Closing the window, pressing `Ctrl+C` or closing the terminal** also makes the script stop the broadcast it started.
- The broadcaster's output goes to the log `~/.local/state/revohuman/sim_live.log`. If the broadcast fails to start (for example, because the glove is held by another program),
  the script prints the last few lines of the log and exits; act on the error shown there (see section 9).
- If a broadcast is already running on this host (`revohuman-live` in another terminal or in a container), the script does not start another one; it only opens the window to show that broadcast's data,
  and prints `检测到已有后端在运行（pid …，型号 …，手 …，socket …），只启动可视化` (an existing backend is running; starting the visualization only). Closing the window then does not stop that broadcast;
  `--hands`, `--product` and the arguments after `--` take effect only when the script starts the broadcast, so they have no effect this time (the script says so); to change these settings, stop that broadcast first.
- The script looks for the SDK in this order: the virtual environment given by `--venv` → `~/.venvs/revohuman_sdk` created in section 11 of *01 Installation and System Requirements*
  (or the directory given by the environment variable `REVOHUMAN_VENV`; used only if it exists) → the current terminal's `python3` (an install into the user directory per sections 3–10 of *01 Installation and System Requirements* falls into this case).
  If none is found, it asks you to run `install_venv.sh` first and exits. A virtual environment does not need to be activated first.

Without gloves, you can first use demo data to confirm that the script itself works (demo mode does not start the broadcast and does not need the SDK):

```bash
./run_sim_host.sh --demo P1 --headless --seconds 2 --snapshot ~/sim-demo.png
```

The output is the same as in section 5; the last line is `[viewer] 快照 → /home/<用户名>/sim-demo.png` (snapshot → …; `<用户名>` is your user name).

Common options (see `./run_sim_host.sh --help` for all options):

| Option | Effect |
|---|---|
| `--hands left\|right\|both` | Open only one hand (default `both`) |
| `--product auto\|P1\|PTV1` | Specify the product model (default `auto`, detected automatically) |
| `--no-tracker` | The broadcaster does not open the Trackers; the Cartesian view draws no hands, and only the finger joints are shown |
| `--demo [P1\|PTV1]` | Do not start the broadcast; show demo data only (as in section 3); without a model, `--product` is used, and PTV1 if neither is given |
| `--log FILE` | Location of the broadcaster's log (default `~/.local/state/revohuman/sim_live.log`, or under `$XDG_STATE_HOME/revohuman/` when `XDG_STATE_HOME` is set; appended on each run) |
| `--venv DIR` | The virtual environment containing the SDK; `--venv none` means using the current terminal's `python3` directly |
| `--socket PATH` | The broadcast socket; the broadcaster and the visualization use the same one (found by a fixed rule by default; usually no need to set it) |

All other arguments are passed to the visualization unchanged, so any parameter from section 6 can be appended directly, for example `./run_sim_host.sh --lite`;
arguments after `--` are passed to `revohuman-live`.

### 4.2 Starting in two terminals

The broadcast and the visualization can also be started and stopped separately; this needs two terminals.

**Terminal 1: start the broadcast**

```bash
revohuman-live
```

Startup takes about half a minute (it waits for base station data and the first Tracker pose); in the meantime it prints, in order:

```text
[live] 型号 P1（编码器手套 + tracker，识别来源 sn），left /dev/ttyACM0，right /dev/ttyACM1
[revohuman.live] 打开 tracker {...}（基站门 flag 档；等基站数据与首帧位姿，可能要十几秒）
...
[live] tracker：已打开 {...}
[live] tracker 打包延迟：left <序列号> …，right <序列号> …
[live] 广播中：/run/user/1000/revohuman/live.sock（Ctrl+C 退出）
```

Once `广播中：` (broadcasting:) appears, the broadcast is ready (the `1000` in the path is the current user's uid and differs from user to user).
The mapping between the serial ports `/dev/ttyACM0`, `/dev/ttyACM1` and handedness is detected by the SDK automatically; go by what is actually printed.

`revohuman-live` is fully equivalent to `python3 -m revohuman live`. For options such as opening only one hand or not opening the Trackers, see *04 Recording*.

**Terminal 2: open the visualization**

```bash
cd ~/revohuman-release/revohuman-0.1.4/simulation
./run_viewer.sh
```

To exit, first close the window in terminal 2 (or press `Ctrl+C`), then press `Ctrl+C` in terminal 1 to stop the broadcast; the reverse order also works.

### 4.3 One-step self-check

Without opening a window, run "start the broadcast → visualization receives for 10 seconds → stop the broadcast" in one terminal, to confirm that the whole chain works:

```bash
cd ~/revohuman-release/revohuman-0.1.4/simulation
./run_sim_host.sh --headless --seconds 10 --snapshot ~/viewer-check.png
```

`--headless --seconds 10 --snapshot` are passed to the visualization: it opens no window, receives for 10 seconds, saves the image and exits; after the visualization exits, the script stops the broadcast. When everything is normal, the output ends with:

```text
[left] /dev/ttyACM0：2004 帧 / 10.0s（200.0 Hz），带触觉 0 帧，valid_mask=0x1fffff
[left] 手套时间戳：最近采样刻 1791977031.930631，到达延迟 8.0 ms（最近 1 s 中位数）
[left] 触觉：none (P1 has no tactile)
[left] 笛卡尔：live  palm_link xyz +0.183 +0.492 +0.156 m  rpy -159.1 +61.5 -67.8°
[left] 位姿：2347 帧 / 10.0s（234.2 Hz），四元数 w +0.2286   x +0.9693   y +0.0110   z +0.0905，最近采样刻 1791977031.940243，到达延迟 0.5 ms（最近 1 s 中位数；含主机固定开销、不含 Tracker 打包延迟）
[right] /dev/ttyACM1：2004 帧 / 10.0s（200.0 Hz），带触觉 0 帧，valid_mask=0x1fffff
[right] 手套时间戳：最近采样刻 1791977031.930516，到达延迟 8.4 ms（最近 1 s 中位数）
[right] 触觉：none (P1 has no tactile)
[right] 笛卡尔：live  palm_link xyz +0.039 +0.505 +0.154 m  rpy +145.7 +74.9 -111.1°
[right] 位姿：2354 帧 / 10.0s（234.9 Hz），四元数 w -0.1164   x -0.9848   y -0.1135   z +0.0611，最近采样刻 1791977031.940950，到达延迟 0.6 ms（最近 1 s 中位数；含主机固定开销、不含 Tracker 打包延迟）
[viewer] 快照 → /home/<用户名>/viewer-check.png
[run_sim_host] 停止广播（pid 12345）
```

Pass criteria:

- each of the two gloves runs at about **200 Hz**;
- both hands show `笛卡尔：live` (Cartesian: live), and each has a `位姿：` (pose:) line (the position, orientation and quaternion vary with how the hands are placed);
- the glove arrival latency is on the order of a few milliseconds (about 8 ms in the example above); the pose arrival latency is much smaller than the glove's (0.5–0.6 ms in the example above; see section 8 for why);
- the command does not exit with an error.

The screenshot of the two views is saved to `~/viewer-check.png`, and the broadcaster's log is in `~/.local/state/revohuman/sim_live.log`.

## 5. Headless mode

A machine without a display (for example, when logged in over SSH) cannot open a window; use headless mode instead: it runs for the given number of seconds, prints statistics,
and saves the two views as one image. Try it once with demo data:

```bash
./run_viewer.sh --demo P1 --headless --seconds 5 --snapshot ~/viewer-demo.png
```

The output should end with:

```text
[left] demo：489 帧 / 5.0s（97.7 Hz），带触觉 0 帧，valid_mask=0x1fffff
[left] 手套时间戳：最近采样刻 1791976299.250077，到达延迟 0.0 ms（最近 1 s 中位数）
[left] 触觉：none (P1 has no tactile)
[left] 笛卡尔：live  palm_link xyz +0.305 +0.137 +1.090 m  rpy +32.7 +0.0 +8.4°
[left] 位姿：244 帧 / 5.0s（48.8 Hz），四元数 w +0.6691   x -0.4432   y +0.5510   z -0.2287，最近采样刻 1791976299.239952，到达延迟 0.0 ms（最近 1 s 中位数）
[right] demo：489 帧 / 5.0s（97.7 Hz），带触觉 0 帧，valid_mask=0x1fffff
[right] 手套时间戳：最近采样刻 1791976299.250124，到达延迟 0.0 ms（最近 1 s 中位数）
[right] 触觉：none (P1 has no tactile)
[right] 笛卡尔：live  palm_link xyz +0.377 -0.215 +1.065 m  rpy +3.3 -0.0 -15.3°
[right] 位姿：244 帧 / 5.0s（48.8 Hz），四元数 w +0.4170   x -0.5978   y +0.3777   z -0.5711，最近采样刻 1791976299.239983，到达延迟 0.0 ms（最近 1 s 中位数）
[viewer] 快照 → /home/<用户名>/viewer-demo.png
```

Demo data runs at about 100 Hz, real gloves at about 200 Hz. For demo data the sampling time and the arrival time are the same instant, so the arrival latency is always 0.0 ms. With real gloves, drop `--demo P1` and start the broadcast first (see section 4).

Exit codes in headless mode:

| Exit code | Meaning |
|---|---|
| `0` | Normal |
| `2` | Startup failed, no connection to the broadcast within `--wait` seconds, or no glove data received during the run |
| `3` | Tactile is enabled, but no tactile frames were received during the run |
| `130` | Interrupted by `Ctrl-C` |

## 6. Common parameters

`./run_viewer.sh --help` lists all parameters (if the visualization environment has not been set up yet, it sets it up first; see section 9 of *01 Installation and System Requirements*). Commonly used ones:

| Parameter | Effect |
|---|---|
| `--demo P1` | Show the built-in P1 demo data without hardware |
| `--demo-hands left\|right\|both` | Which hands the demo data contains (default `both`) |
| `--view split\|cart\|fixed` | Layout at startup: both views / Cartesian view only / fixed-base view only (default `split`; press `V` to switch while running) |
| `--tracker off` | The Cartesian view draws no hands (it shows `pose none: off (--tracker off)`), and only the finger joints are shown; add `--view fixed` as well to keep only the fixed-base view. The broadcaster still opens the Trackers; to keep the broadcaster from opening the Trackers as well, use `--no-tracker` from section 4.1 (when you start the broadcast yourself, see section 7 of *04 Recording*) |
| `--lite` | Turn off shadows and reflections; use this when there is no dedicated graphics card or rendering stutters |
| `--fps N` | Upper limit of the window refresh rate (default 60) |
| `--spacing M` | Distance between the two hands in the fixed-base view, in meters (default 0.30) |
| `--headless` | Headless mode |
| `--seconds S` | Number of seconds to run in headless mode |
| `--wait S` | Maximum number of seconds to wait for the broadcast in headless mode (default 10) |
| `--snapshot FILE` | Save the two views as a PNG at the end of headless mode |
| `--urdf-dir DIR` | Directory containing the hand model (with `urdf/` and `meshes/`); defaults to the PTV1 hand model in `models/` at the root of the source bundle |
| `--size WxH` | Screenshot size (default `1600x800`) |
| `--socket PATH` | Broadcast address. Uses the same one as `revohuman-live` by default; usually no need to set it |

Parameters for the tactile heat map and for troubleshooting:

| Parameter | Effect |
|---|---|
| `--tactile off` | Do not draw the tactile heat map. Default `auto`: drawn only when the product model has tactile sensing and tactile was detected for that hand |
| `--scale N` | Full scale of the color scale; default `0` means automatic (holds the peak and slowly falls back) |
| `--contact N` | Contact threshold; areas with readings below it are not drawn (default 20, same unit as the tactile readings) |
| `--threshold N` | Per-finger statistics and the point table count only points with readings above it (default 0) |
| `--top N` | Number of rows of the point table printed on `Insert` or in headless mode (default 20) |
| `--no-heat-ring` | Do not draw contact rings |
| `--cam AZIMUTH,ELEVATION,DISTANCE` | Initial camera of the fixed-base view |
| `--layout FILE` | Tactile layout yaml; the layout shipped with the bundle is used by default |
| `--refit` | Ignore the cache and the shipped point-placement table, and refit the taxel positions to the model mesh |
| `--dump-layout FILE` | At the end of headless mode, save the taxel positions and readings as JSON (one file per hand, with `_left` / `_right` appended to the file name) |
| `--no-bulk` | Write heat spots into the scene one by one (for troubleshooting rendering problems; slower) |

Fine-tuning parameters for the heat-spot shape (`--heat-pitch-mm`, `--heat-sigma-mm`, etc.) are listed in `./run_viewer.sh --help`; they usually do not need changing.

Log lines in the terminal are colored by level: warnings yellow, errors red, ordinary messages uncolored; output redirected to a file has no color.
The visualization and `revohuman-live` honor the same environment variable `REVOHUMAN_LOG_COLOR` (`auto` / `always` / `never`);
for example, `REVOHUMAN_LOG_COLOR=never ./run_viewer.sh` turns colors off. See section 1.1 of *07 Logging and Error Codes* for details.

## 7. Window controls

| Action | Effect |
|---|---|
| `V` | Switch layout: both views → Cartesian view only → fixed-base view only |
| `F` | Make the Cartesian view's camera follow the midpoint between the two palms (on / off; turned off automatically when you drag to pan) |
| `C` | Reset the camera of both views and turn following back on |
| Left mouse drag | Rotate the camera |
| Right mouse drag | Pan the camera |
| Mouse wheel | Zoom |

Mouse actions apply to the view under the mouse pointer.

When the tactile heat map is drawn for PTV1, the following additional keys are available (they apply to the fixed-base view):

| Key | Effect |
|---|---|
| `J` | Switch the table in the upper-left corner between joint angles and tactile statistics |
| `Tab` | Choose which hand the tactile statistics show (with both hands) |
| `←` / `→` | Select the previous / next taxel |
| `PgUp` / `PgDn` | Jump to the previous / next finger |
| `↑` / `↓` | Increase / decrease the color-scale range (switches to a fixed range) |
| `Home` | Use the current readings as the tactile baseline; afterwards, values minus the baseline are shown |
| `Delete` | Clear the baseline |
| `Insert` | Print the taxel table of the current hand to the terminal |

## 8. On-screen information

The screen shows only five kinds of data: data frame rate, live joint angles, pose quaternions, sampling timestamps and arrival latency. `L` / `R` at the start of a line means left hand / right hand.

**Cartesian view (left)**

- Upper left, `L quat` / `R quat`: the quaternion of each hand's Tracker pose, in `w x y z` order, the same order as the Tracker pose quaternion output by the SDK.
- Lower left, `L pose` / `R pose`: frame rate, sampling timestamp `sample` and arrival latency `latency` of each hand's pose data.
- For each hand, the palm_link coordinate axes and a vertical line down to the ground are drawn; the ground grid spacing is 0.1 m.
- A hand with no new pose for more than 0.5 seconds (for example, because its Tracker is occluded) stays at its last position and is drawn semi-transparent,
  with `LOST` and the number of seconds since it was lost shown after the quaternion; it recovers automatically once the base stations are visible again.
- The world frame has its z axis pointing up; **its origin and horizontal orientation depend on how the base stations are placed and calibrated, and have no physical meaning in themselves**;
  the relative position and orientation between the two hands are real.

**Fixed-base view (right)**

- Upper-left joint-angle table: one row per joint; the two columns `L deg` / `R deg` are the joint angles (degrees) reported by the left and right gloves; `--` means no valid reading has been received yet.
- Lower left, `L glove` / `R glove`: frame rate (about 200 Hz), sampling timestamp `sample` and arrival latency `latency` of each glove's data.

**Timestamps and latency**

- `sample` is the sampling time of this frame, shown as Unix time in seconds (to the microsecond); it is the same value as the sampling timestamp output by the SDK.
- `latency` is the time at which the SDK received this frame minus the sampling time, taken as the median over the last 1 second.
- When the Tracker pose `latency` line ends with `(excl. pack delay)`, it excludes the Tracker's internal packing delay and is smaller than the true arrival latency;
  use it to look at jitter and stalls, not as end-to-end latency.
- Before any data has been received, the line shows `no data yet`; when no sampling timestamp is available, it shows `--`.

**When something is wrong**: only then does an extra line starting with `!!` appear, for example `tracker stopped: …` when the broadcaster's Trackers stop outputting poses,
or `startup gate: …` or `geometry: … issue(s)` when the base station check did not pass (see section 9).
When the broadcaster has not enabled the Trackers, the lower left shows `pose none: <reason>`.

**When disconnected from the broadcast**: both views dim, the banner `SDK DISCONNECTED - …` at the top states the reason, and the last frame stays on screen.

## 9. Common issues

| Symptom | Cause and fix |
|---|---|
| Startup error `找不到手部模型目录` (hand model directory not found) | `models/` (the hand models) at the root of the source bundle has been moved or deleted: unpack the source bundle again, or use `--urdf-dir` to point to the directory that contains the hand model |
| Startup error `手部模型目录不完整` (hand model directory incomplete) | The model directory must contain two subdirectories: `urdf/` (with exactly one `.urdf` file) and `meshes/` |
| The window keeps showing `waiting for the SDK broadcast ...` | The broadcast has not been started, or is still starting (about half a minute). Check that terminal 1 has printed `广播中：` (broadcasting:); if it has, check whether the `socket:` path listed in the window matches the path printed in terminal 1 |
| `run_sim_host.sh` prints the last few lines of the broadcast log and exits | The broadcast failed to start. Look up the error from the log in the other rows of this table (the "Terminal 1 reports …" rows below apply as well); the full log is in `~/.local/state/revohuman/sim_live.log` |
| `run_sim_host.sh` waited 90 seconds, the broadcast was still not ready, and the window was opened anyway | The broadcast is still waiting for base station data or the first Tracker pose: the window connects automatically once it is ready; if it never connects, check in the log `~/.local/state/revohuman/sim_live.log` which step it is stuck at, and handle base-station-related problems as described in *03 Base Stations and Pairing* |
| After the window opens, the terminal prints `广播已退出，窗口不再更新` (the broadcast has exited; the window no longer updates) | The broadcaster exited partway through, and the window stays on the last frame. Act on the log lines printed after it, then run `run_sim_host.sh` again |
| `run_sim_host.sh` asks you to run `install_venv.sh` first | The SDK was not found: run `install_venv.sh` as described in section 11 of *01 Installation and System Requirements*, or use `--venv` to point to a virtual environment where the SDK is installed; if you installed per sections 3–10 of *01 Installation and System Requirements*, confirm that `python3 -c "import revohuman"` runs without error |
| Terminal 1 reports `上已经有一个广播在运行` (a broadcast is already running on it) | Another `revohuman-live` is still open; close it first |
| Terminal 1 reports `手套串口被别的程序占着` (the glove serial port is held by another program) | The recording program or another `revohuman-live` is using the glove; only one program can open the glove at a time, so close it first |
| Terminal 1 reports `当前用户没有手套串口的读写权限` (the current user has no read/write permission on the glove serial port) | Add the user to the `dialout` group as described in section 5 of *01 Installation and System Requirements*, then log out and log back in |
| `revohuman-live: command not found` | Installed with a virtual environment: run `source ~/.venvs/revohuman_sdk/bin/activate` first. Installed into the user directory: `~/.local/bin` is not in `PATH`; open a new terminal and try again (see section 7 of *01 Installation and System Requirements*), or use `python3 -m revohuman live` instead |
| The Cartesian view shows `pose none: pysurvive not installed` | The broadcaster cannot find the Tracker driver; install it as described in section 6 of *01 Installation and System Requirements*. The view on the right works as usual in this case |
| The Cartesian view shows `pose none: tracker in use by another process` | The Trackers are being used by another program; close it and restart `revohuman-live` |
| `!! startup gate: …` or `!! geometry: … issue(s)` appears in the lower left of the Cartesian view | The base station check did not pass, so the relative pose of the two hands is not necessarily trustworthy. Check the log in terminal 1 and handle it as described in *03 Base Stations and Pairing* |
| One hand is semi-transparent, with `LOST` after its quaternion | That hand's Tracker has had no new pose for more than 0.5 seconds, usually because it is occluded or has left the base stations' field of view; it recovers automatically once the base stations are visible again |
| Both hands show `LOST`, and `!! tracker stopped: …` appears in the lower left | The broadcaster's Trackers have stopped outputting poses (for example, a base station was moved, or all Trackers went offline); this does not recover automatically; the view on the right works as usual. Terminal 1 prints `tracker 已停：…` (tracker stopped: …); handle the Tracker error in terminal 1 (section 7 of *03 Base Stations and Pairing*), then restart `revohuman-live` |
| In the Cartesian view the hands are far from the grid origin, or their orientation is not intuitive | The world origin and orientation depend on the base stations and have no physical meaning; press `C` to reset the camera and `F` to turn following on |
| Startup error `GLFW 初始化失败（没有图形界面？可以用 --headless --snapshot）` (GLFW initialization failed; no graphical interface?) | The current session has no graphical interface (for example, logged in over SSH); use the headless mode in section 5 instead |
| Headless mode exits with an error that mentions `eglQueryString` or `glGetError` | The OpenGL runtime libraries are missing: run `sudo apt install -y libegl1 libgl1` (section 4 of *01 Installation and System Requirements*) |
| `run_viewer.sh` reports `依赖安装失败（网络 / 镜像？…）` (dependency installation failed; network / mirror?) | The first run downloads dependencies from PyPI. On networks in mainland China, prefix the command with `PIP_INDEX_URL=https://mirrors.aliyun.com/pypi/simple/` and try again |
| The window stutters | Add `--lite` to turn off shadows and reflections |
| Error `unsupported broadcast schema` | The visualization and the SDK versions do not match: use release files of the same version for both the SDK and the source bundle |
