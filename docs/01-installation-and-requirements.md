# 01 Installation and System Requirements

English | [简体中文](zh-CN/01-环境安装与系统要求.md)

Starting from a freshly installed Ubuntu 22.04, this chapter installs all the software RevoHuman P1 needs:
ROS 2 Humble, the Tracker driver libsurvive, the RevoHuman SDK (including recording, base station Bluetooth and 3D visualization) and the ROS 2 packages.
Run the steps in order; every step is required, do not skip any.

If you only use the Python API, command line, recording and 3D visualization, **without the ROS 2 nodes**, you can use the one-step install script in section 11 instead,
which installs all dependencies except ROS 2 into a separate virtual environment; to run with Docker, see *08 Running in Containers*.

## 1. System requirements

| Item | Requirement |
|---|---|
| Operating system | Ubuntu 22.04 LTS, 64-bit (x86_64 or aarch64) |
| Python | The system Python 3.10 (`/usr/bin/python3`). The SDK only provides packages for Python 3.10; pip rejects other Python versions (including those installed with conda / pyenv) |
| ROS 2 | Humble (installed in section 3 of this chapter; needed only when using the ROS 2 nodes) |
| Account | A regular user with `sudo` privileges |
| USB | Each glove uses 1 USB port on the host, 2 for both hands; plug directly into the host, not through a passive HUB |
| Bluetooth | The host needs a working Bluetooth adapter (used when reading base station serial numbers and channels) |
| Disk | Installation needs about 3 GB; recorded data is extra, see *04 Recording* |
| Network | Installation needs access to the Ubuntu package repositories, the ROS package repository, PyPI and GitHub |

Each hand of the P1 glove is one glove unit: the encoder glove's MCU and the Vive Tracker are attached to the same USB HUB inside the unit,
and the host sees both devices through one USB cable. For how to connect the hardware, see *02 Hardware Connection*.

The commands below are run in a regular user's terminal by default; commands that need administrator privileges include `sudo` explicitly.

## 2. Prepare the release files

There are six release files; put them in the same directory (`~/revohuman-release` is used as the example below):

| File | Contents |
|---|---|
| `revohuman-0.1.4-cp310-cp310-linux_x86_64.whl` | SDK, for x86_64 hosts |
| `revohuman-0.1.4-cp310-cp310-linux_aarch64.whl` | SDK, for aarch64 (arm64) hosts |
| `pysurvive-1.1.21+revohuman.3-cp310-cp310-linux_x86_64.whl` | Tracker driver libsurvive, for x86_64 hosts |
| `pysurvive-1.1.21+revohuman.3-cp310-cp310-linux_aarch64.whl` | Tracker driver libsurvive, for aarch64 (arm64) hosts |
| `revohuman-0.1.4-bundle.tar.gz` | Source bundle: ROS 2 packages, hand models (URDF), examples, docs, simulation, Docker image build files, one-step install script |
| `SHA256SUMS` | Checksums |

Enter that directory and verify file integrity; every file should show `OK`:

```bash
cd ~/revohuman-release
sha256sum -c SHA256SUMS
```

## 3. Install ROS 2 Humble

Add the ROS 2 package repository:

```bash
sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

> If `packages.ros.org` is slow from networks in China, replace `http://packages.ros.org/ros2/ubuntu` above
> with `https://mirrors.aliyun.com/ros2/ubuntu`, leaving everything else unchanged.

Install ROS 2 and the build tools, and have every new terminal load the ROS environment automatically:

```bash
sudo apt update
sudo apt install -y ros-humble-ros-base ros-dev-tools
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash
```

## 4. Install system dependencies

The following installs, in one go, the system packages needed by the SDK, base station Bluetooth and 3D visualization:

```bash
sudo apt install -y python3-pip python3-venv bluez libgl1 libegl1
```

`python3-venv` creates the separate Python environment for 3D visualization; `libgl1` and `libegl1` are its OpenGL runtime libraries; both are required.

## 5. Device permissions

The glove appears as the serial port `/dev/ttyACM*`, which belongs to the `dialout` group. Add the current user to that group:

```bash
sudo usermod -aG dialout $USER
```

**After adding the group, log out and log back in** (or reboot) for it to take effect.

## 6. Install the Tracker driver libsurvive

The Tracker is read by the open-source driver libsurvive; the SDK calls it through its Python package `pysurvive`.
The release files include **pysurvive 1.1.21+revohuman.3**: it is based on upstream pysurvive 1.1.21 (libsurvive `v1.01-21-gd89876a`),
with three fixes: reception resumes automatically after a USB transfer timeout, so a Tracker that has been occluded for a long time recovers optical tracking as soon as it is visible again;
the base station geometry is kept when the gravity reading broadcast by the base station jitters by less than 2° (the upstream version clears it);
during capture only calibrated base stations are accepted, so other people's base stations nearby (or base stations whose channel was changed) are not merged into the current coordinate frame.
It also includes an optional feature that is off by default: the Tracker orientation-jump gate; for how to enable it, see section 4.8 of the *RevoHuman SDK Reference Manual*; when it is not enabled, the behavior is the same as the previous version.
**Install the bundled copy, not `pysurvive==1.1.21` from PyPI**: 1.1.21 from PyPI passes the self-check but lacks the three fixes above; on machines where it is already installed, running the command below replaces it.
If a pysurvive with any other version number is installed, the Tracker's base station diagnostics fail their self-check and are degraded:

```bash
cd ~/revohuman-release
python3 -m pip install --user --no-deps pysurvive-1.1.21+revohuman.3-cp310-cp310-linux_$(uname -m).whl
```

`--no-deps` must not be omitted: pysurvive declares a GUI dependency (wxPython) that the SDK does not use and that usually cannot be installed on Linux.

Install the Tracker device access rules so that regular users can read and write the Tracker:

```bash
sudo tee /etc/udev/rules.d/81-revohuman-tracker.rules > /dev/null <<'RULES'
# Vive Tracker（USB 有线，28de:2300）
SUBSYSTEM=="usb", ATTRS{idVendor}=="28de", ATTRS{idProduct}=="2300", MODE="0666"
KERNEL=="hidraw*", SUBSYSTEM=="hidraw", ATTRS{idVendor}=="28de", ATTRS{idProduct}=="2300", MODE="0666"
RULES
```

Apply the rules (Trackers already plugged in must be unplugged and plugged in again once):

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## 7. Install the RevoHuman SDK

Select the wheel for this machine's architecture and install the SDK together with all components (recording, camera decoding, base station Bluetooth, 3D visualization) in one go:

```bash
cd ~/revohuman-release
WHL=revohuman-0.1.4-cp310-cp310-linux_$(uname -m).whl
python3 -m pip install --user "$WHL[recording,camera,ble,viewer]"
```

> If PyPI is slow from networks in China, append `-i https://mirrors.aliyun.com/pypi/simple/` to the `pip install` above.

The SDK command-line tools are installed in `~/.local/bin`; add it to `PATH`:

```bash
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
export PATH="$HOME/.local/bin:$PATH"
```

## 8. Build the ROS 2 packages

Extract the source bundle, put its ROS 2 packages into a workspace and build them:

```bash
cd ~/revohuman-release
tar xzf revohuman-0.1.4-bundle.tar.gz
mkdir -p ~/ros2_ws/src
ln -s ~/revohuman-release/revohuman-0.1.4/ros2/* ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --symlink-install
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/ros2_ws/install/setup.bash
```

The extracted `~/revohuman-release/revohuman-0.1.4/` also contains `examples/` (example programs) and `simulation/` (3D visualization),
which later chapters use.

## 9. Install the 3D visualization environment

The 3D visualization comes with its own separate Python environment under `simulation/` (`simulation/.venv`), independent of the SDK.
Running the launcher once creates it and installs all dependencies (about 1–2 minutes the first time, starts in seconds afterwards):

```bash
~/revohuman-release/revohuman-0.1.4/simulation/run_viewer.sh --help
```

> If PyPI is slow from networks in China, prefix the command with `PIP_INDEX_URL=https://mirrors.aliyun.com/pypi/simple/`.

Seeing the option help means the environment is ready. For usage, see *05 Simulation and Visualization*.

## 10. Check the installation

Open a **new terminal** (so that the settings in `~/.bashrc` take effect) and run the following one by one:

```bash
python3 -c "import revohuman, pysurvive, mcap, bleak, mujoco; print('revohuman', revohuman.__version__)"
python3 -m pip show pysurvive | grep Version
python3 -c "import pysurvive; print('libsurvive', pysurvive.survive_build_tag().decode())"
python3 -m revohuman --help
ros2 pkg list | grep revohuman
ros2 interface show revohuman_msgs/srv/Calibrate
```

You should see:

- the first command prints `revohuman 0.1.4`, with no `ModuleNotFoundError`;
- `pip show` shows `Version: 1.1.21+revohuman.3`, and the next command prints `libsurvive v1.01-21-gd89876a+revohuman.3`;
- `ros2 pkg list` lists the three packages `revohuman_bringup`, `revohuman_msgs` and `revohuman_ros`.

After connecting the gloves, run a health check (for connecting the gloves, see *02 Hardware Connection*):

```bash
python3 -m revohuman doctor --probe
python3 -m revohuman product identify
```

`--probe` briefly opens the serial ports to read the handedness stored in the firmware and the firmware information; do not add it while capturing.
When the health check passes, `doctor` shows `fail=0` and lists the two glove units and their handedness; `product identify` shows the product model of the connected gloves (`型号 P1` or `型号 PTV1`, i.e. model P1 or model PTV1).

## 11. Alternative install: one-step install into a virtual environment

When you do not use the ROS 2 nodes, `install_venv.sh` in the source bundle root can replace sections 3–10 and install all dependencies except ROS 2 in one go:
the SDK, the bundled pysurvive and the 3D visualization dependencies all go into a separate Python virtual environment `revohuman_sdk` (default `~/.venvs/revohuman_sdk`),
without modifying the system Python or the Python packages in the user directory. A host already set up according to sections 3–10 can also install this additional copy; the two do not affect each other.

After completing section 2 (preparing the release files), run as a regular user (do not prefix `sudo`; the script asks separately before steps that need `sudo`):

```bash
cd ~/revohuman-release
tar xzf revohuman-0.1.4-bundle.tar.gz
revohuman-0.1.4/install_venv.sh
```

The script runs six steps in order and prints the result of each:

1. System dependencies: if missing, installs with `apt` the OpenGL / EGL runtime libraries (`libgl1`, `libegl1` from section 4), GLib, the X11 libraries used by the visualization window,
   `bluez` and `python3.10-venv`; installs the Tracker device rules from section 6; adds the current user to the `dialout` group as in section 5.
   Steps that need `sudo` ask one by one before running; steps already done are skipped.
2. Creates the virtual environment.
3. Verifies against `SHA256SUMS` and installs the bundled pysurvive 1.1.21+revohuman.3 (the same copy as in section 6).
4. Installs the SDK for this machine's architecture, with the four extras for recording, camera image decoding, base station Bluetooth and 3D visualization (same as section 7).
5. Installs the 3D visualization dependencies (mujoco at version 3.1 or later and below 3.13).
6. Self-check: imports each module and confirms that the command-line tools start and the visualization can render an image (windowless demo; no gloves or display needed).

When run again, already installed parts are skipped automatically; if a step fails, the script explains the cause and how to fix it, then exits; after fixing it, just run the script again.
All output is also saved to a log, `~/.local/state/revohuman-venv/install-<date>-<time>.log`; when you run into a problem, send it to technical support. Common options:

| Option | Description |
|---|---|
| `-y` | Do not ask before steps that need `sudo`; run them directly |
| `--venv DIR` | Location of the virtual environment, default `~/.venvs/revohuman_sdk` (can also be set with the environment variable `REVOHUMAN_VENV`) |
| `--wheels DIR` | Directory holding the release files, default the parent directory of the source bundle (i.e. `~/revohuman-release`) |
| `--python INTERPRETER` | Python used to create the virtual environment; must be CPython 3.10; by default looks for `python3.10`, then `python3` |
| `--mirror cn` | Use the Alibaba Cloud PyPI mirror (for networks in China); `--mirror official` uses the official index; when not given, pip's own settings are kept |
| `--no-tracker` | Do not install the Tracker driver and device rules |
| `--no-system` | Do not modify the system: no packages or device rules installed, no `dialout` group change; only check and print hints |
| `--force` | Delete the existing virtual environment and create it again |
| `--dry-run` | Only check and print the commands that would be run; do not modify the system or create the virtual environment |

For all options, see `install_venv.sh --help`. When not run in a terminal (for example when called by another script) and `-y` is not given, all steps that need `sudo` are skipped;
the script prints the commands to run manually and continues; if what is missing is `python3.10-venv`, which is required to create the virtual environment, it exits immediately.

The script first checks the run conditions (not running as root, Linux, x86_64 / aarch64) and whether the release files are complete, then checks the Python version; if it cannot find CPython 3.10 it exits and explains how to install it. The system Python on Ubuntu 22.04 (x86_64 / aarch64) is 3.10,
so no option is needed; on other distributions (for example Ubuntu 24.04, which ships 3.12), first install a CPython 3.10, then point `--python` at it;
on non-Debian distributions also add `--no-system` (the script only checks and does not install system packages), and install the packages from section 4 yourself with the distribution's package manager.

If PyPI is slow from networks in China, add `--mirror cn`. If pip settings such as `PIP_INDEX_URL` are already configured, the script keeps using them when `--mirror` is not given.

**Activate the virtual environment before use**; after that, `python3 -m revohuman`, `revohuman-live` and `revohuman-lighthouses` can be used directly.
Open a new terminal and check:

```bash
source ~/.venvs/revohuman_sdk/bin/activate
python3 -c "import revohuman, pysurvive, mcap, bleak, mujoco; print('revohuman', revohuman.__version__)"
python3 -m pip show pysurvive | grep Version
revohuman-live --help
deactivate
```

The first command prints `revohuman 0.1.4`, and `pip show` shows `Version: 1.1.21+revohuman.3`.

- After connecting the gloves, activate the virtual environment and run `python3 -m revohuman doctor --probe` for a health check (see section 3.2 of *02 Hardware Connection*).
- Every new terminal must first run `source ~/.venvs/revohuman_sdk/bin/activate`; `deactivate` exits. You can also skip activation and use the full path,
  for example `~/.venvs/revohuman_sdk/bin/revohuman-live`.
- After being added to the `dialout` group for the first time, you must log out and log back in once before the glove serial ports can be opened (as in section 5); for glove units that were already plugged in before the rules were installed,
  the Tracker device rules take effect only after unplugging and plugging them in again once.
- The virtual environment **does not contain ROS 2** and cannot be used by the ROS 2 nodes; to use *06 ROS 2*, install according to sections 3–10.
- Configuration and calibration such as the base station allowlist, base station geometry and zero pose are stored in the home directory (`~/.config/revohuman`, `~/.config/libsurvive`, `~/.local/state/revohuman`);
  both install methods share one copy, so switching to the other does not require pairing or calibrating again.
- The 3D visualization can be started together with the broadcast with one command, `simulation/run_sim_host.sh`; see section 4.1 of *05 Simulation and Visualization*.
