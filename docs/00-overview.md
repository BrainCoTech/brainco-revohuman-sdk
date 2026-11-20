# RevoHuman P1 User Manual

English | [简体中文](zh-CN/00-总览.md)

This manual is for users of **RevoHuman P1** (encoder glove + Vive Tracker). Starting from a freshly installed Ubuntu 22.04,
it covers connecting the hardware, setting up the base stations, recording data, viewing the simulation, connecting to ROS 2, and what to check when something goes wrong.

## What P1 is

| Part | Description |
|---|---|
| Glove unit ×2 | One each for the left and right hands: encoder glove (21 joint angles per hand, 200 Hz) + a Vive Tracker on the back of the hand (pose, about 250 Hz); one USB HUB inside the unit, connected to the host with one USB cable |
| Base station | SteamVR Base Station 2.0; the Tracker relies on it for positioning |
| Host | Ubuntu 22.04, x86_64 or aarch64 |

P1 has no full-finger tactile sensing and no wrist cameras.

## Reading order

| Chapter | Contents | When to read |
|---|---|---|
| [01 Installation and System Requirements](01-installation-and-requirements.md) | System requirements; installing ROS 2 Humble, libsurvive, the RevoHuman SDK, the ROS 2 packages and 3D visualization; without ROS 2, `install_venv.sh` can do a one-step install into a virtual environment | On first use, once per host |
| [02 Hardware Connection](02-hardware-connection.md) | Wiring, Tracker power-on and indicator, health check, confirming the product model and left/right hands, wearing the gloves | Run the health check first every time the gloves are connected |
| [03 Base Stations and Pairing](03-base-stations-and-pairing.md) | Turning on the Tracker and checking its indicator, base station allowlist, channels, base station geometry calibration, startup self-check, runtime protection; `calibrate_lighthouses.sh` can do the calibration in one step | On first use, when changing sites, after base stations are moved |
| [04 Recording](04-recording.md) | Recording MCAP with the SDK, controlling recording with the glove buttons, checking recordings and exporting CSV, live broadcast with `revohuman-live`, reading live data in your own program (blocking and non-blocking reads) | Capturing data, writing programs |
| [05 Simulation and Visualization](05-simulation-and-visualization.md) | MuJoCo 3D visualization: demo mode, `run_sim_host.sh` starting the broadcast and the window with one command, connecting real gloves, windowless screenshots | Checking whether the glove data looks right, demos |
| [06 ROS 2](06-ros2.md) | Starting the nodes, topics and services, Tracker zero pose, rosbag recording, single-hand mode, parameters | Using glove data in ROS 2 |
| [07 Logging and Error Codes](07-logging-and-error-codes.md) | Log locations, information to collect when reporting a problem, runtime notice codes (N codes), error codes, exit codes | When something goes wrong |
| [08 Running in Containers](08-running-in-containers.md) | Running the SDK, ROS 2 and visualization from a Docker image: building, starting, where data and calibration are stored, devices and GUI | When you do not want to install ROS 2 and Python dependencies on the host |
| [09 Timestamp Synchronization](09-timestamp-synchronization.md) | Where each data stream's timestamps come from, how to align across hands and devices, which metrics to look at, anomalies and errors, host time synchronization settings | For cross-modal alignment and timing analysis, or when timestamp-related notices appear during recording |

On first use, work through 01 → 02 → 03 in order; after that, read 04, 05, 06 and 09 as needed.
Without ROS 2, you only need sections 1, 2 and 11 ("One-step install into a virtual environment") of 01.
When using a container, use *08 Running in Containers* instead of 01 (for host preparation and building the image, see sections 2 and 3 of 08; `docker/install.sh` does this in one step); after that, run the commands of each chapter inside the container, with a few exceptions listed in 08; the visualization can be started in the container from the host with one command, `simulation/run_sim_docker.sh`.

For the list of release files (SDK and pysurvive packages for both platforms, source bundle, checksums), see section 2 of *01 Installation and System Requirements*.
When writing your own programs that call the SDK, see the *RevoHuman SDK Reference Manual* in the same directory for all interfaces, data fields, ROS 2 topics and parameters, and error codes.

## Things to keep in mind before use

- **Only one program at a time can open the gloves and Trackers.** Do not have a recording script, `revohuman-live`, a ROS node and the visualization read the gloves at the same time;
  when they are in use, error code `1102` (glove) or `1900` (Tracker) is reported; see *07 Logging and Error Codes*.
- **Left and right hands are identified by the handedness stored in the glove firmware** (for older gloves whose firmware has none stored, by the handedness character in the USB serial number), regardless of which USB port is used or
  the `/dev/ttyACM` number; do not hard-code serial ports in your programs.
- **Do not move the base stations after they are calibrated.** If one is moved, calibrate again; when the SDK detects that a base station has been moved, it stops capture and prompts for recalibration,
  but translating a base station or rotating it horizontally is not detected (with one base station or several); see sections 7 and 8 of *03 Base Stations and Pairing*.
- Command-line tools are all invoked as `python3 -m revohuman …`; `revohuman-lighthouses` and `revohuman-live` are two separate commands.

## Conventions in this manual

- All command blocks are run in a regular user's terminal; commands that need administrator privileges include `sudo` explicitly.
- Device serial numbers in sample output have been replaced with `X`: glove `BCRH1PLXXXXXXXXXXX`, Tracker `LHR-XXXXXXXX`, base station `FAXXXXXXXXXX`;
  go by the actual output on your own devices. Timestamps and values differ on every run.
- The commands in this manual were verified before release on a release candidate build, using a clean Ubuntu 22.04 and real hardware with two P1 gloves, Trackers and base stations connected.
