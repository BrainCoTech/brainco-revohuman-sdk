# Glove visualization (simulation/)

English | [简体中文](README.zh-CN.md)

A single MuJoCo window with a left and a right view, showing a pair of gloves at the same time:

| View | Content |
|---|---|
| Left: **Cartesian view** | Both hands are placed in one z-up world according to the **6D pose** from the Trackers; finger joints and the tactile heatmap move with the hands. Each hand draws its palm_link coordinate axes and a vertical line to the ground; the ground grid is 0.1 m. |
| Right: **fixed-base view** | The two palms are placed at fixed positions (spacing `--spacing`, default 0.30 m), showing only the finger joints and the **tactile heatmap**. |

- **The product model follows the SDK**: both P1 and PTV1 are supported. The SDK tells the visualization the product model, the hands and the joint table when it connects, and both product models go through the same path.
- **Tactile display is decided by peripheral detection**: the heatmap is drawn only if the product model has tactile sensing, the firmware peripheral detection for this hand reported a tactile module, and the point count and layout match (241 points);
  otherwise only the joints are drawn for this hand, and the terminal states the reason at startup. Whether the data contains non-zero values is not considered.
- **The Cartesian view follows the SDK-side Tracker**: it is enabled only once the Tracker is up on the SDK side; otherwise the left view shows the reason,
  and the right view works as usual.

## Relationship with the SDK: the SDK broadcasts, the visualization only subscribes

The visualization **does not import the SDK**, and does not touch the serial ports or the Trackers. There is only one data path, a local Unix socket:

```
Terminal 1 (SDK environment)                   Terminal 2 (visualization's own environment)
revohuman-live             ──local socket──▶   simulation/run_viewer.sh
opens gloves + tracker, starts broadcasting    subscribes only: updates on receipt
```

- **Either side can be started first.** When the visualization starts first, the window shows `waiting for the SDK broadcast ...` and gives the command for terminal 1;
  once the SDK is ready (the socket appears), it connects automatically, and after receiving the product model and joint table it builds the scene and starts updating.
- **The window does not exit when the SDK exits**: it keeps the last frame and marks the reason for the disconnection, and reconnects automatically when the SDK comes back.
  If the SDK configuration has not changed (same product model, same pair of hands), display simply continues; if it has changed, the scene is rebuilt for the new configuration.
- The two sides are installed in their own Python environments: the visualization side **does not need the SDK installed**, and the SDK side **does not need mujoco**.
  The SDK virtual environment built by `install_venv.sh` in the source bundle root (default `~/.venvs/revohuman_sdk`) also installs the visualization dependencies, so both sides can share it.
- If you do not want to open two terminals, use `run_sim_host.sh` to bring up both sides with one command (see the next section).

The broadcaster is the SDK's own command-line entry point `revohuman-live` (installed together with the SDK);
it is exactly equivalent to `python3 -m revohuman live`, so use the latter if the command is not found
(the command entry point is not on `PATH`: for an SDK installed with `pip install --user`, add `~/.local/bin` to `PATH`; for one installed in a venv, activate the venv first):

```bash
revohuman-live                               # the full set of gloves + tracker
revohuman-live --hands left                  # left hand only
revohuman-live --no-tracker                  # broadcast gloves only (Cartesian view not enabled)
revohuman-live --help                        # all options
python3 -m revohuman live --hands left       # same as revohuman-live --hands left
```

Both sides locate the socket by the same rules, so usually you do not need to care: `$REVOHUMAN_LIVE_SOCKET` →
`$XDG_RUNTIME_DIR/revohuman/live.sock` → `/tmp/revohuman-<uid>/live.sock`.
The directory is 0700 and the socket 0600, so only the current user can connect. To change the location, set the same `REVOHUMAN_LIVE_SOCKET` on both sides
(or add `--socket <path>` to the visualization). The protocol is defined in the module documentation of the SDK's `revohuman/live.py`.

## Bringing up the broadcast and visualization with one command

```bash
simulation/run_sim_host.sh                   # starts revohuman-live in the background and opens the window once it is ready; closing the window also stops the broadcast
simulation/run_sim_host.sh --hands left      # left hand only
simulation/run_sim_host.sh --lite -- --tracker-gate error   # visualization options go first; everything after -- is passed to revohuman-live
simulation/run_sim_host.sh --demo P1         # no broadcast, demo only (SDK not needed)
simulation/run_sim_host.sh --help            # all options
simulation/run_sim_docker.sh                 # same as above, with both the broadcast and the window in a container (build the image first as described in docker/README.md)
```

For how the SDK is located, waiting and exiting, handling of an already running broadcast, log locations and all options, see
section 4.1 of [*05 Simulation and Visualization*](../docs/05-simulation-and-visualization.md); for `run_sim_docker.sh`, see also
section 8 of [*08 Running in Containers*](../docs/08-running-in-containers.md).

## Quick start

```bash
simulation/run_viewer.sh --demo              # no gloves: built-in demo broadcast (PTV1, both hands + tracker)
simulation/run_viewer.sh --demo P1           # demo of P1 (no tactile)
simulation/run_viewer.sh                     # real gloves: start revohuman-live in another terminal first
simulation/run_viewer.sh --help              # all options
simulation/run_viewer.sh --urdf-dir <目录>   # specify this when the hand model is not in the default location (models/ in the source bundle root)
```

`--demo` starts a private demo broadcast inside the visualization process, which goes through exactly the same receiving, scene-building and rendering path as the real SDK.

On its first run, `run_viewer.sh` creates an **isolated Python environment** in `simulation/.venv`
and installs the dependencies listed in `simulation/requirements.txt` (`mujoco` / `glfw` / `numpy` / `PyYAML`);
it does not touch the system Python and needs neither a container nor ROS; after that, it reinstalls only when `requirements.txt` changes.
The positions of the 241 taxels of a tactile glove on each finger segment of the hand model are taken from the bundled placement table
`revohuman_sim/assets/tactile_placement_241.json`, which is loaded directly when the scene is built, with no fitting on the spot.
When you switch to another hand model with `--urdf-dir`, this table is used only if the finger-segment kinematics match it; otherwise the placement is fitted on the spot from the model meshes
(the finger-segment meshes must include rings / caps; this takes a dozen or so seconds), and the result is cached in `~/.cache/revohuman/` (in
`$XDG_CACHE_HOME/revohuman/` when `XDG_CACHE_HOME` is set). `--refit` ignores the cache and the bundled placement table and refits from the model meshes;
it likewise requires meshes with rings / caps, so it cannot be used with the bundled hand model.

| Environment variable | Effect |
|---|---|
| `REVOHUMAN_SIM_VENV=<dir>` | venv location (default `simulation/.venv`) |
| `REVOHUMAN_VENV=<dir>` | SDK virtual environment that `run_sim_host.sh` looks for by default (default `~/.venvs/revohuman_sdk`; the default of `install_venv.sh --venv` is also taken from it) |
| `REVOHUMAN_SIM_WAIT=<seconds>` | Maximum number of seconds `run_sim_host.sh` waits for the broadcast to be ready; when the time is up, it opens the window first (default 90) |
| `REVOHUMAN_SIM_PYTHON=<python>` | Interpreter used to build the venv (default `python3`, requires ≥ 3.10) |
| `REVOHUMAN_SIM_PRIME=0` | Do not set the NVIDIA PRIME variables (by default, on machines with an NVIDIA driver, the window is rendered on the discrete GPU) |
| `REVOHUMAN_LIVE_SOCKET=<path>` | Broadcast socket (set the same value on both sides) |
| `REVOHUMAN_LOG_COLOR=auto\|always\|never` | Color terminal log lines by level (default `auto`: color when the output is a terminal, `NO_COLOR` is not set and `TERM` is not `dumb`; no color when redirected to a file). The SDK command line honors the same variable |
| `PIP_INDEX_URL=<url>` | pip mirror |

If you prefer not to use the scripts, you can also install into your own environment and run it directly:

```bash
pip install -r simulation/requirements.txt
python3 simulation/viewer.py --demo
```

## Prerequisites

The following are all prerequisites on **the SDK side** (`revohuman-live`); the visualization side only needs OpenGL:

- **Serial port permissions**: the current user must be able to read and write the glove serial ports. Without permission, the broadcaster prints how to fix it; use one of the two options:
  ```bash
  sudo usermod -aG dialout $USER        # then log in again
  # or install a udev rule
  echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5710", MODE="0666"' \
    | sudo tee /etc/udev/rules.d/99-revohuman.rules
  sudo udevadm control --reload && sudo udevadm trigger
  ```
- **Tracker (Cartesian view)**: pysurvive, the Python binding of libsurvive, is installed in **the SDK's environment**; the visualization side does not need it.
  Install `pysurvive 1.1.21+revohuman.3`, which ships with the SDK; for the command, see step ② of the installation in the main README. Do not install the original
  `pysurvive==1.1.21` from PyPI, which lacks three fixes in the bundled version. The SDK ships with this one and is tested and released on it;
  with any other version, the Tracker's base station check fails its self-check and degrades.
  The base station geometry must first be calibrated according to step 3 of Quick start in the main README (for details, see section 4 of the [*RevoHuman SDK Reference Manual*](../docs/reference-manual.md));
  the broadcaster only reads poses: it does not calibrate, does not adopt the base station geometry, and does not record a zero pose (the Cartesian view draws absolute poses and has no use for the zero pose).
  When pysurvive is not installed, or the Tracker is held by another process, the broadcaster broadcasts gloves only (without taking over the Tracker), and the Cartesian view shows the reason.
- **Base station check level** (broadcaster `--tracker-gate`): the default is `flag`, which broadcasts poses as usual even if the base station check fails;
  the lower-left corner of the Cartesian view marks with `!!` which item failed (e.g. `startup gate: S0 whitelist_missing`, `geometry: 2 issue(s)`),
  and the specific reason is in the Tracker log in the broadcaster's terminal (in this case the relative pose between the two hands is not necessarily trustworthy);
  with `error`, poses are not broadcast if the check fails.
- **Single-process exclusive access**: the glove serial ports and the Tracker can only be opened by one process at a time. If a recording program is running, the broadcaster reports that they are in use.
  Only one broadcast can run on a given socket; **there can be multiple subscribers** (opening several visualization windows is fine).
- **Graphics card**: OpenGL is required. On machines without a discrete GPU, add `--lite` (turns off shadows and reflections).

## Controls

| Key | Action |
|---|---|
| `V` | Switch layout: both views → Cartesian only → fixed base only |
| `F` | Cartesian view camera follows the midpoint of the two palms (on / off; turned off automatically when you drag to pan) |
| `C` | Reset the cameras of both views and turn following back on |
| `J` | Table in the upper-left corner of the fixed-base view: joint angles ↔ tactile statistics (when a hand has tactile enabled) |
| `Tab` | Which hand the tactile statistics show (with two hands) |
| `←` / `→` | Select the previous / next taxel |
| `PgUp` / `PgDn` | Jump to the previous / next finger |
| `↑` / `↓` | Enlarge / shrink the color scale range (switches to a fixed range) |
| `Home` | Use the current readings as the tactile baseline (values shown afterwards have the baseline subtracted) |
| `Delete` | Clear the baseline |
| `Insert` | Print the current hand's taxel table to the terminal |
| Mouse | Left-drag to rotate, right-drag to pan, scroll wheel to zoom (acts on the view under the mouse) |

The tactile-related keys are listed only while the fixed-base view is showing tactile statistics.

## Overlay

Only five kinds of data are shown: data frame rate, live joint angles, pose quaternion, sampling timestamp and arrival latency. `L` / `R` at the start of a line means left / right hand.

- **Cartesian view**: in the upper-left corner, one line per hand with the Tracker pose quaternion (`w x y z`, i.e. `TrackerPose.rotation` as broadcast by the SDK);
  in the lower-left corner, one line per hand for the pose stream `pose  <Hz>  sample <sampling time>  latency <arrival latency>`;
  when the line ends with `(excl. pack delay)`, this arrival latency does not include the Tracker's internal packing delay and is smaller than the true arrival latency.
  A hand with no new pose for more than 0.5 s keeps its last pose and is drawn semi-transparent, with `LOST <seconds>` after the quaternion.
- **Fixed-base view**: in the upper-left corner, the joint angle table (`L deg` / `R deg`, the readings reported by the glove; `J` switches to the current hand's tactile statistics and the selected taxel's reading);
  in the lower-left corner, one line per hand for the glove stream `glove  <Hz>  sample <sampling time>  latency <arrival latency>`.
- **Timestamps**: `sample` is the sampling time given by the SDK (Unix seconds, to the microsecond); `latency` is the time the SDK received the frame minus the sampling time,
  as the median over the last 1 s; `--` is shown when the sampling time is not available.
- **An extra `!!` line appears only when something is wrong**: the Tracker has stopped outputting poses (`tracker stopped: <reason>`), the base station geometry check failed
  (`startup gate: …` / `geometry: … issue(s)`), or the glove data stream was interrupted. When the broadcaster has not opened the Tracker, the pose line reads `none: <reason>`.
- **Key hints** are in the upper-right corner; tactile-related keys are listed only while tactile statistics are shown.
- **When disconnected from the SDK**: both views dim, a banner at the top states the reason for the disconnection, and the last frame stays on screen.

## Coordinate conventions

- The world frame is libsurvive's world frame (z up); **its origin and horizontal orientation are determined by the base station calibration and have no physical meaning**;
  the relative pose between the two hands is real. The Cartesian view camera follows the midpoint of the two palms by default.
- The palm_link pose is derived from the Tracker pose through the fixed mounting relationship: `R_palm = R_tracker · X · R_ptᵀ`,
  `p_palm = p_tracker − R_palm · t_pt`, where `R_pt / t_pt` is the mounting pose of the model's Tracker link
  (`left_tracker_Link` / `right_tracker_Link`, about 86 mm from the palm_link origin) relative to palm_link,
  and `X` is the fixed axis conversion from the Tracker body frame to this link (see `revohuman_sim/poses.py`).

## Headless mode

On remote machines or for automated checks, you can run without a window: it runs for a few seconds, prints statistics, and can save both views as an image:

```bash
simulation/run_viewer.sh --demo --headless --seconds 3 --snapshot out.png
simulation/run_viewer.sh --headless --wait 20 --seconds 5 --dump-layout points.json   # taxel positions and readings
```

Headless mode uses EGL off-screen rendering by default (`MUJOCO_GL=egl`) and needs no display; it waits at most `--wait` seconds to connect to the SDK broadcast.
Exit codes: `0` normal; `2` startup failed, could not connect to the broadcast, or received no glove data during the run; `3` tactile enabled but no tactile frames received; `130` interrupted by `Ctrl-C`.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Startup reports `找不到手部模型目录` (hand model directory not found) | `models/` (the hand model) in the source bundle root has been moved or deleted: extract the source bundle again, or specify the directory with `--urdf-dir` |
| Startup reports `手部模型目录不完整` (hand model directory incomplete) | The model directory must contain `urdf/` (with exactly one `.urdf`) and `meshes/` |
| Running `viewer.py` directly reports `缺依赖 …` (missing dependency …) | The visualization dependencies are not installed in your own environment: use `run_viewer.sh` instead, or `pip install -r simulation/requirements.txt` |
| The window stays at `waiting for the SDK broadcast` | The broadcast is not running, or the two sides use different sockets: check whether the path listed in the window matches the `广播中：<path>` (broadcasting: <path>) printed when the broadcaster started; if not, set the same `REVOHUMAN_LIVE_SOCKET` on both sides |
| `run_sim_host.sh` prints the last few lines of the broadcast log and exits | The broadcaster did not start: look up the error in the log in the other rows of this table; the full log is at `~/.local/state/revohuman/sim_live.log` |
| `run_sim_host.sh` says to run `install_venv.sh` first | The SDK was not found: run `install_venv.sh` in the source bundle root, or point `--venv` at a virtual environment with the SDK installed |
| The broadcaster reports `已经有一个广播在运行` (a broadcast is already running) | Another `revohuman-live` is still running; close it, or give the new one a different socket |
| `revohuman-live: command not found` | The command entry point is not on `PATH`: for an SDK installed with `pip install --user`, add `~/.local/bin` to `PATH` (`export PATH="$HOME/.local/bin:$PATH"`); for one installed in a venv, activate the venv first; if the SDK was not installed with pip, this command does not exist. In all three cases you can use `python3 -m revohuman live` instead |
| `unsupported broadcast schema` | The visualization and SDK versions do not match: switch the SDK and the source bundle to release files of the same version |
| The Cartesian view shows `pysurvive not installed` / `tracker in use by another process` | See the Tracker item under "Prerequisites" on the broadcaster side; nothing needs to change on the visualization side |
| Both hands are marked `LOST`, and the lower-left corner shows `!! tracker stopped: …` | The SDK-side Tracker has stopped outputting poses and will not recover automatically; the broadcaster terminal prints `tracker 已停：…` (tracker stopped: …). Follow the hints in the broadcaster's Tracker log (if a base station has been moved, calibrate again), then restart `revohuman-live` |
| A hand shows `off (firmware reports no tactile module)` | That hand's firmware peripheral detection did not report a tactile module: this is a hardware identification result, and the visualization shows it as is |

## Directory

```
simulation/
  run_viewer.sh          launcher (isolated venv + dependencies + start)
  run_sim_host.sh        one command to bring up broadcast + visualization (host)
  run_sim_docker.sh      one command to bring up broadcast + visualization (container)
  requirements.txt       all visualization dependencies (SDK not included)
  viewer.py              no-install entry point
  revohuman_sim/
    app.py               options and startup flow
    paths.py             fixed paths and the local cache directory
    logcolor.py          color terminal log lines by level (REVOHUMAN_LOG_COLOR)
    live_client.py       subscribe to the SDK broadcast (reconnect, frame decoding, latest state)
    session.py           build the scene from the SDK's hello; decide whether tactile is enabled
    demo.py              built-in demo broadcast (--demo)
    hand_model.py        two-hand URDF assembly and forward kinematics (model taken from models/ in the source bundle root)
    poses.py             tracker pose → palm_link pose
    tactile_layout.py    taxel layout and fitting
    heatmap.py           tactile heatmap
    view.py              two-view rendering and interaction
    assets/              taxel layout and the bundled placement table (241 points)
```
