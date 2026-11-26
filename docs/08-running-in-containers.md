# 08 Running in Containers

English | [简体中文](zh-CN/08-容器运行.md)

The `docker/` directory of the source bundle provides the one-step build script `install.sh`, the Docker image build files and the launch script `run.sh`.
The image has ROS 2 Humble, the Tracker driver libsurvive, the RevoHuman SDK, the ROS 2 packages and the 3D visualization environment installed following the steps in *01 Installation and System Requirements*;
the host only needs Docker and one Tracker device rule.

Containers suit these cases: the host is not Ubuntu 22.04; you do not want to install ROS 2 and the Python dependencies on the host; you need exactly the same environment on several hosts.
The commands and directory layout inside the container are the same as in the other chapters of the manual; this chapter covers only what is specific to containers.

## 1. Host requirements

| Item | Requirement |
|---|---|
| Operating system | 64-bit Linux (x86_64 or aarch64) |
| Docker | 23.0 or later (builds use BuildKit by default); 25 or later is recommended, so that plugging and unplugging gloves after the container is up does not require recreating the container (see section 6) |
| Account | A regular user with `sudo` rights (user ID greater than 100) who is in the `docker` group; **do not run the container as root** |
| Disk | The image is about 2.6 GB; recording data is extra, see *04 Recording* |
| Network | Building the image requires access to the Ubuntu package sources, the ROS package sources, PyPI and GitHub (to fetch the signing key of the ROS package source); with `--mirror cn` all of these go through the Alibaba Cloud mirrors instead, and GitHub access is not needed. Running after the build needs no network |
| USB, Bluetooth | Same as section 1 of *01 Installation and System Requirements* |

## 2. Host preparation

Do the following three steps once on the host. When you build with `install.sh` from section 3, the script checks the Docker installation, the `docker` group and the Tracker device rule,
and adds whatever is missing (asking before it does so), so you can skip directly to section 3; this section is the manual procedure.

**Docker user group.** After installing Docker according to the official Docker documentation, add the current user to the `docker` group:

```bash
sudo usermod -aG docker $USER
```

The group membership takes effect only after you log out and log back in (or reboot). It is fine once `docker info` no longer reports `permission denied`.

**Tracker device rule.** The container accesses the Tracker through the host's device nodes, so this rule must be installed on the **host** (it is the same rule as in section 6 of *01 Installation and System Requirements*; skip this if it is already installed):

```bash
sudo tee /etc/udev/rules.d/81-revohuman-tracker.rules > /dev/null <<'RULES'
# Vive Tracker（USB 有线，28de:2300）
SUBSYSTEM=="usb", ATTRS{idVendor}=="28de", ATTRS{idProduct}=="2300", MODE="0666"
KERNEL=="hidraw*", SUBSYSTEM=="hidraw", ATTRS{idVendor}=="28de", ATTRS{idProduct}=="2300", MODE="0666"
RULES
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Replug any glove units that were already plugged in once after the rule is installed.

**Bluetooth.** Needed only for pairing and calibrating base stations inside the container (see section 7). Follow section 2 of *03 Base Stations and Pairing* to confirm that the host's Bluetooth service is running and the adapter is powered on.

The host does **not** need to add the user to the `dialout` or `video` groups: `run.sh` automatically adds the groups that own the glove serial ports and wrist cameras to the user inside the container.

## 3. Building the image

Put the release files in one directory (section 2 of *01 Installation and System Requirements*; `~/revohuman-release` is used as the example below as well).

**One-step build.** Extract the source bundle and run `install.sh` in the `docker/` directory:

```bash
cd ~/revohuman-release
tar xzf revohuman-0.1.4-bundle.tar.gz
cd revohuman-0.1.4/docker
./install.sh
```

On networks in mainland China, add `--mirror cn` (Ubuntu, ROS 2 and PyPI all use the Alibaba Cloud mirrors). The script runs six steps in order and prints the result of each:

1. Verify the release files: check the two wheels and the source bundle one by one against `SHA256SUMS`. A missing file, an incomplete download, a corrupted file, or a `SHA256SUMS` that does not belong to this version is reported together with the file concerned.
2. Check Docker: install it with `apt` if it is missing; add the current user to the `docker` group if not already in it (effective temporarily for this run, permanently after logging out and back in);
   confirm that the Docker service is running, the version is recent enough, and the local Docker is the one in use. Docker Desktop, rootless mode, Docker with userns-remap enabled
   and Podman cannot access the gloves or have mismatched user IDs; the script says so directly and does not go on to build.
3. Tracker device rule: installed if the host has no rule that lets all users read and write the Tracker USB device (`SUBSYSTEM=="usb"`, `MODE="0666"`);
   the content is the same as in section 2. A rule that only opens the hidraw nodes, or that does not grant the permission to all users, does not count.
4. Pre-build checks: disk space, the base image (must be Ubuntu 22.04), and whether the package sources are reachable. If proxy variables such as `http_proxy` are set in the current shell, the build also goes through that proxy.
5. Build the image; `USER_UID` / `USER_GID` are taken automatically from the current user.
6. Check the image: run the same checks as section 10 of *01 Installation and System Requirements* inside the image and confirm that files written in the container are owned by the current user; then use `run.sh` to actually create a check container,
   run one command in it, and delete that container immediately once it passes.

Finally it prints `镜像 revohuman:0.1.4 已就绪` (image revohuman:0.1.4 is ready) and the next command to run.

- Run it as a regular user; **do not add `sudo`**. The steps that need `sudo` (installing Docker, joining the `docker` group, installing the Tracker device rule) ask one by one before running;
  add `-y` to run them without asking, `--skip-host-setup` to leave the host unchanged and only check and build, or `--dry-run` to only check and print the commands that would run.
- If a step fails, the script explains the reason and the fix, then exits; rerun it after fixing the problem. If the image has already been built for the current user from the same release files and the same base image,
  it is not rebuilt and only the step 6 checks are rerun; add `--rebuild` to force a rebuild, or `--no-cache` to build from scratch without Docker's build cache either.
  If the base image has been pulled again since the last build, the script points this out; again, add `--rebuild`.
- By default the release files are expected in the directory one level above the source bundle (the directory where you extracted it); if they are elsewhere, specify it with `--release-dir DIR`.
- When upgrading from a previous version, containers created by the previous version need no manual handling: the next time you run `./run.sh`, it switches to the newly built image automatically,
  keeping the `--data`, `--ble`, `-e` and other settings given when the container was created; if programs are still running in the container, it tells you first and does not interrupt them; see section 4.
  The replaced old images are not deleted automatically; those no longer used by any container are listed at the end of the script together with the command to delete them.
- A copy of all output is saved as a log (`~/.local/state/revohuman-docker/install-<time>.log`; the path is printed at the start); attach it when you ask for technical support.
- For all options see `./install.sh --help`.

**Manual build.** Without the script, verify the release files and extract the source bundle, then build in the directory holding the release files:

```bash
cd ~/revohuman-release
sha256sum -c SHA256SUMS
tar xzf revohuman-0.1.4-bundle.tar.gz
docker build -f revohuman-0.1.4/docker/Dockerfile \
    --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g) \
    -t revohuman:0.1.4 .
```

If access to the package sources is slow on networks in mainland China, add the mirror arguments:

```bash
docker build -f revohuman-0.1.4/docker/Dockerfile \
    --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g) \
    --build-arg UBUNTU_MIRROR=http://mirrors.aliyun.com \
    --build-arg ROS_APT_MIRROR=https://mirrors.aliyun.com/ros2/ubuntu \
    --build-arg ROS_GPG_URL=https://mirrors.aliyun.com/rosdistro/ros.key \
    --build-arg PIP_INDEX_URL=https://mirrors.aliyun.com/pypi/simple/ \
    -t revohuman:0.1.4 .
```

Notes:

- The first build takes 10–20 minutes, mostly spent downloading, depending on the network. The build automatically selects the wheel for the host architecture and checks each one against `SHA256SUMS`;
  the last step runs the same checks as section 10 of *01 Installation and System Requirements*, and if any of them fails, the build fails, so no faulty image is produced.
- **`USER_UID` / `USER_GID` must be those of the user who runs the container** (`$(id -u)`, `$(id -g)` above).
  Ownership of the mounted directories and the device-in-use checks are both based on the user ID; when `run.sh` finds a mismatch, it refuses to start and asks you to rebuild.
  `install.sh` takes the current user's values automatically. When several users on the same host use containers, each user builds their own copy, distinguished by different image names
  (`install.sh --tag NAME` or `docker build -t NAME`), and then enters with `./run.sh --image NAME`.
- The build uses only the two wheels, the source bundle and `SHA256SUMS` from the release directory; other files in the directory are not sent into the build.

## 4. Entering the container

```bash
cd ~/revohuman-release/revohuman-0.1.4/docker
./run.sh
```

On the first run, `run.sh` creates and starts a container named `revohuman` in the background, then enters a shell in it; the terminal prints:

```text
已启动容器 revohuman（数据目录：/home/<用户名>/revohuman-data；插拔手套不用重建容器）
```

With Docker older than 25, this line lists the glove serial ports that were plugged in when the container was created, for example `（手套串口：/dev/ttyACM0 /dev/ttyACM1；数据目录：…）` (glove serial ports: …; data directory: …),
and a separate reminder says that after plugging or unplugging gloves you need `./run.sh --restart` (see section 6).

The container keeps running in the background. **Running `./run.sh` in another terminal enters the same container**,
so for the scenarios in the manual that need two terminals (for example `revohuman-live` plus visualization in *05 Simulation and Visualization*, or a node plus topic inspection in *06 ROS 2*), just run `./run.sh` once in each of two terminals;
the broadcast plus visualization from *05 Simulation and Visualization* can also be done with a single command, `simulation/run_sim_docker.sh` (see section 8).

You can also run a single command in the container without entering the shell:

```bash
./run.sh python3 -m revohuman doctor
./run.sh python3 -m revohuman product identify
```

After entering the container, run the health check as in section 3 of *02 Hardware Connection*; after that, run the commands of the other chapters as written.
The ROS 2 and RevoHuman workspace environments are already loaded in the container; no further `source` is needed. The following points differ from running on the host:

- `ls -l /dev/ttyACM*` in section 3.1 of *02 Hardware Connection*: with Docker 25 or later, the container's `/dev` has no glove serial ports (the SDK looks under `/run/host/dev`),
  so this command reporting `No such file or directory` is normal; go by the result of `python3 -m revohuman doctor`.
- *03 Base Stations and Pairing*: the Bluetooth check and enabling are done on the host, and the container must be created with `--ble`; for the complete steps see section 7.
- Scripts, recording files, logs and screenshots that the manual places in the home directory (`~/…`) go under `~/data` in the container instead; see section 5.
- `chronyc tracking` in *09 Timestamp Synchronization* is run on the host: the container uses the host's clock, and chrony is not installed in the image.

`run.sh` options:

| Option | Description |
|---|---|
| `--restart` | Delete the running container, recreate it strictly with the options given this time, then enter it (the original settings are not carried over; see below the table). Add it whenever you change any of the options below; with Docker older than 25, also add it after plugging or unplugging gloves |
| `--stop` | Stop and delete the container. Data and calibration are in host directories and are not affected |
| `--data DIR` | Host directory mounted at `~/data` in the container; default `~/revohuman-data` |
| `--state DIR` | Host directory that stores calibration, configuration and runtime state; default `~/.local/share/revohuman-docker` (`$XDG_DATA_HOME/revohuman-docker` when `XDG_DATA_HOME` is set) |
| `--ble` | Allow the container to use the host's Bluetooth (section 7) |
| `--no-gui` | Do not connect to the host's graphical display |
| `-e NAME=VALUE`, `--env NAME=VALUE` | Add an environment variable to the container; can be repeated |
| `--image IMAGE`, `--name NAME` | Image to use (default `revohuman:0.1.4`) and container name (default `revohuman`) |
| `--dry-run` | Only print the `docker` commands that would be run, without running them |

Options take effect only when the container is created. In the following cases `run.sh` explains why and, where the container needs to be recreated, gives the complete command; just run it as given:

- **Carrying over the original settings.** When the container has to be recreated (for example the container stopped after a host reboot, or it is switched to a new image as described below), any of
  `--data`, `--state`, `--ble`, `-e` not given this time are carried over from the original container, and `沿用原来容器的设置：…` (carrying over the original container's settings: …) is printed; those given this time take precedence.
  `--no-gui` is not carried over. With `--restart`, the container is created strictly with the options given this time, with nothing carried over.
- **Changed options.** If the container is already running and the options given this time differ from those used at creation, `run.sh` enters the original container as usual and prints
  `注意：容器 revohuman 是按另一组设置创建的…` (note: container revohuman was created with a different set of settings…) and the recreate command.
- **Switching to the new image after an upgrade.** The container was created from a previous version's image (a lower version number, or an image of the same name before it was rebuilt):
  when no programs are running in the container, `run.sh` automatically recreates the container from the new image, carrying over the original settings, and prints
  `容器 revohuman 原来是用镜像 … 创建的，换成 revohuman:0.1.4 重新创建…` (container revohuman was created from image …; recreating it from revohuman:0.1.4…);
  when programs are still running in the container (recording, visualization, a shell open in another terminal, etc.), it does not interrupt them. This time it enters the old container, prints
  `注意：容器 revohuman 是用镜像 … 创建的，里面运行的还是旧版本的程序` (note: container revohuman was created from image …; the programs running in it are still the old version), lists the running programs, and gives the command to switch now
  (those programs will be terminated); once they have all exited, run `./run.sh` again and it switches automatically.
  The replaced old image is not deleted automatically; when no container uses it, `run.sh` prints the `docker image rm` command to delete it. Delete it after confirming that the new version works.
- **Other images.** If the container was created from an image with a higher version number, or from an image with a different name, `run.sh` does not switch automatically and prints
  `注意：容器 revohuman 是用镜像 … 创建的，不是 …` (note: container revohuman was created from image …, not …) and the recreate command.
- **Gloves plugged or unplugged.** With Docker older than 25, if gloves were plugged or unplugged after the container was created, `run.sh` prints
  `注意：容器 revohuman 创建之后插拔过手套…` (note: gloves were plugged or unplugged after container revohuman was created…) and the recreate command.

## 5. Where data, configuration and calibration are stored

The following directories in the container are mounted from the host; deleting or recreating the container does not affect their contents:

| In the container | On the host | Contents |
|---|---|---|
| `~/data` (the current directory when entering the container) | `~/revohuman-data` | Recording data, your own scripts |
| `~/.config/revohuman` | `~/.local/share/revohuman-docker/config/revohuman` | Base station allowlist `lighthouses.yaml`, product model rules `products.yaml` |
| `~/.config/libsurvive` | `~/.local/share/revohuman-docker/config/libsurvive` | Base station geometry `config.json` |
| `~/.local/state/revohuman` | `~/.local/share/revohuman-docker/state/revohuman` | Tracker and base station state files |
| `~/.ros` | `~/.local/share/revohuman-docker/ros` | ROS 2 node logs |

**Files anywhere else in the container are lost after `--restart` or `--stop`.** Files that the manual writes to the home directory should be written under `~/data` in the container instead:

- Save the recording script from *04 Recording* as `~/data/revohuman_record.py`, and put the output directory under `~/data` as well, for example
  `python3 ~/data/revohuman_record.py ~/data/demo 10 2`;
- For the `revohuman-report.tgz` generated in section 2 of *07 Logging and Error Codes*, run `cp ~/revohuman-report.tgz ~/data/` afterwards and pick it up from `~/revohuman-data` on the host;
- Files written to the home directory in other chapters likewise go under `~/data`: the other scripts of *04 Recording* and the `~/revohuman_data/…` output directories
  (for example `~/data/demo/episode_001.mcap`), the bag recording directory in section 6 of *06 ROS 2* (`~/revohuman_bags` becomes `~/data/revohuman_bags`),
  `~/revohuman-doctor.log` in section 1 of *07 Logging and Error Codes*, `revohuman_notices.py` in section 3 of *07 Logging and Error Codes* and its output
  (pass `~/data/revohuman-notices.jsonl` as the second argument when running it),
  and visualization screenshots (`--snapshot ~/data/….png`);
- Scripts can be edited in the container with `nano`, or placed directly into `~/revohuman-data` on the host.

**Configuration and calibration are a separate copy for the container**, independent of the SDK installed directly on the host (`~/.config/revohuman`, `~/.config/libsurvive`).
If base stations have already been paired and calibrated on the host and you want the container to reuse them, stop the container first, then copy the three directories over from the host:

```bash
cd ~/revohuman-release/revohuman-0.1.4/docker
./run.sh --stop
S=~/.local/share/revohuman-docker
mkdir -p $S/config $S/state
cp -a ~/.config/revohuman ~/.config/libsurvive $S/config/
cp -a ~/.local/state/revohuman $S/state/
```

Afterwards, `python3 -m revohuman tracker status` in the container should match what you see on the host.
Conversely, if you have never calibrated on the host, pair and calibrate directly in the container following *03 Base Stations and Pairing*.
Base stations that have been moved need recalibration; this is no different from running on the host.

## 6. Gloves and Trackers

- **Gloves and wrist cameras**: with Docker 25 or later, `run.sh` lets the container see the host's glove serial ports and wrist cameras directly; gloves plugged in, or replugged, after the container is up
  work as usual in the container, without recreating it. The only devices the container can open are glove serial ports, video devices (wrist cameras), USB devices, and the GPU when the graphical display is connected (section 8); other devices on the host cannot be opened.
  With Docker older than 25, `run.sh` can only hand the glove serial ports and video devices plugged in at creation time to the container: **gloves plugged in, or replugged, after the container was created cannot be used in the container**;
  recreate the container with the command `run.sh` suggests (do so when no programs are running in the container; the suggested command includes the `--data`, `--ble`, `-e` and other options given when the container was created).
  In this case the health check in the container lists the glove unit but notes that its device node is unavailable, and suggests recreating the container.
- **Tracker**: no container recreation is needed after plugging or unplugging. If a hand has no Tracker serial number in `product identify`,
  check whether the Tracker device rule from section 2 is installed on the host; `run.sh` also prints a notice when, while creating the container, it finds that the current user cannot read and write the Tracker.
- **Device occupancy is shared inside and outside the container**: the same glove or the same set of Trackers can be used by only one program at a time,
  whether that program runs in the container or on the host. For example, while `revohuman-live` is running on the host, a recording script in the container also reports `1102` / `1900`.
- **USB power management**: the SDK in the container can read the gloves' USB power management state but cannot change it, the same as running as a regular user on the host.
  The health check item `USB 拓扑与电源管理` (USB topology and power management) showing `autosuspend 已关` (autosuspend off) is normal.

## 7. Base stations: Bluetooth, pairing and calibration

Pairing base stations and the channel check before calibration require Bluetooth (*03 Base Stations and Pairing*). The container uses the adapter through the host's Bluetooth service; Bluetooth is not connected by default, so the container must be created with `--ble`.
Only `revohuman-lighthouses`, used for pairing base stations, and the channel preflight check `python3 -m revohuman tracker preflight` use Bluetooth; calibration, the startup self-check, recording, broadcast and visualization do not.
Below are the complete steps for pairing and calibrating from scratch in the container; for the meaning of each step's output and what to do on errors, see the corresponding subsections of *03 Base Stations and Pairing*.

### 7.1 On the host: confirm Bluetooth and recreate the container with Bluetooth

The container does not run systemd, so the Bluetooth check and enable commands from section 2 of *03 Base Stations and Pairing* must be run in a terminal on the **host**:

```bash
systemctl is-active bluetooth
bluetoothctl show | grep Powered
```

You should see `active` and `Powered: yes`; if not, enable them on the host as in section 2 of *03 Base Stations and Pairing*. Then recreate the container with `--ble` and enter it
(recreate it only when no programs are running in the container; `--restart` creates it strictly with the options given this time, so include any options such as `--data`, `-e` that were given when the container was created):

```bash
cd ~/revohuman-release/revohuman-0.1.4/docker
./run.sh --restart --ble
```

### 7.2 In the container: pairing

Confirm that the container can see the Bluetooth adapter and that both gloves are connected, then scan for base stations and write out the allowlist configuration file:

```bash
bluetoothctl show | grep Powered
python3 -m revohuman doctor
revohuman-lighthouses scan
revohuman-lighthouses init
```

`doctor` should show `fail=0`. On the first scan there is no allowlist yet, so at the end `scan` says that the allowlist does not exist or has no entries; this is normal.
Fill in the serial numbers of the base stations to use in the allowlist (for how, see section 4.3 of *03 Base Stations and Pairing*; if there are many base stations and you cannot tell which is which, first make them blink to identify them as in section 4.2 of *03 Base Stations and Pairing*):

```bash
nano ~/.config/revohuman/lighthouses.yaml
```

This file is mounted from `~/.local/share/revohuman-docker/config/revohuman/lighthouses.yaml` on the host (section 5),
so you can also edit that file on the host with your preferred editor; both sides are the same file. Once it is filled in, check that it is read correctly (no Bluetooth needed; returns immediately):

```bash
revohuman-lighthouses show
```

If there is no `✗`, it is correct.

### 7.3 In the container: calibration and self-check

Calibration requires a successful scan within the last 5 minutes. Confirm that the indicator lights of both Trackers are green (section 1.2 of *03 Base Stations and Pairing*), then scan first:

```bash
revohuman-lighthouses scan
```

When the last line is `✓ 白名单 N 台都扫到，信道无冲突；信道预检已刷新（5 分钟内有效）` (all N allowlisted base stations seen, no channel conflict; channel preflight check refreshed, valid for 5 minutes), start calibration right away, walking around wearing the gloves as in section 5.2 of *03 Base Stations and Pairing*.
If a same-channel conflict is reported, run `revohuman-lighthouses plan` and `revohuman-lighthouses apply --yes` in the container as in section 4.5 of *03 Base Stations and Pairing*, then scan again;
for other messages see the table in section 4.4 of *03 Base Stations and Pairing*.

```bash
python3 -m revohuman tracker calibrate
```

When the last line is `标定完成：在册基站均已解出：…` (calibration complete: all allowlisted base stations solved: …), it is done; if it fails or is rejected, follow the table at the end of section 5.2 of *03 Base Stations and Pairing*. Then open the Trackers once more to actually verify the base stations and pose:

```bash
python3 -m revohuman tracker check --live
python3 -m revohuman tracker status
```

It is fine when `check --live` shows `✓ S0 静态门 通过` (S0 static gate passed) and `✓ S1 起流后门（…） 通过` (S1 post-stream-start gate (…) passed), and the `基站几何` (base station geometry) line of `status` is `可用` (available).
Both the allowlist and the calibration results are stored in directories mounted from the host (section 5), so deleting or recreating the container later does not require redoing them.

**All in one command**: once the allowlist is filled in, scanning, calibration and the self-check can also be handed to `calibrate_lighthouses.sh` in the root directory of the source bundle (section 5.4 of *03 Base Stations and Pairing*).
Run it on the **host** with `--docker`; it enters the container and runs these three steps there in order; if the container is not running, it creates it with `--ble`:

```bash
cd ~/revohuman-release/revohuman-0.1.4
./calibrate_lighthouses.sh --docker
```

- If the container is already running but was created without `--ble`, `run.sh` reports that the container was created with a different set of settings, and the scan then reports that Bluetooth is unavailable.
  When no programs are running in the container, recreate it with `--restart`: `./calibrate_lighthouses.sh --docker --restart`
  (include any options such as `--data`, `-e` that were given when the container was created).
- `--data`, `--state`, `-e`, `--image`, `--name` are passed to `run.sh` as is. When `--data`, `--state`, `-e` are not given, the original container's settings are carried over;
  when you are not using the default container name or image, `--name`, `--image` must be given.
- If the allowlist has not been filled in yet, the script scans once, writes out the allowlist and exits with exit code `4`, printing the allowlist's directory on the host
  (by default `~/.local/share/revohuman-docker/config/revohuman/`); fill it in on the host as in section 4.3 of *03 Base Stations and Pairing*, then rerun.
- If you end the script on the host side with `Ctrl-C`, by closing the terminal or with `kill`, it stops the calibration in the container and waits for the rollback to finish (at most about 30 seconds) before exiting.
- What runs in the container is the script of the same name in the image's copy of the source bundle. If you are told that the source bundle in the container has no `calibrate_lighthouses.sh`,
  it is most likely that programs were still running in the container after an upgrade, so `run.sh` did not switch to the new image this time (section 4): rerun after they have all exited and it switches automatically,
  or add `--restart` as suggested to switch now; if it still happens after recreating, rebuild the image with `docker/install.sh` from this source bundle on the host.
- When you are already in the container, do not add `--docker`; run `~/revohuman-release/revohuman-0.1.4/calibrate_lighthouses.sh` directly.

The exit codes are the same as in section 5.4 of *03 Base Stations and Pairing*.

After calibration, you can run `./run.sh --restart` on the host to switch back to a container without Bluetooth (do so when no programs are running in the container;
include any options such as `--data`, `-e` that were given when the container was created).
To keep using the container with Bluetooth, just run `./run.sh` as usual afterwards: options not given are carried over from the original container, and Bluetooth is kept when the container is recreated (for example after a host reboot or an upgrade).
After moving base stations or changing channels, redo everything from 7.1.

## 8. Graphical display

The windows of the 3D visualization (*05 Simulation and Visualization*) appear on the host desktop: by default `run.sh` hands the host's graphical display and GPU to the container,
so just run `./run.sh` in a terminal opened on the desktop; no other setup is needed. The visualization directory is the same as with a host installation:

```bash
cd ~/revohuman-release/revohuman-0.1.4/simulation
./run_viewer.sh --demo
```

With real gloves, you can also launch the broadcast and the visualization window in the container with a single command on the host, without entering the container:

```bash
cd ~/revohuman-release/revohuman-0.1.4/simulation
./run_sim_docker.sh
```

Usage is the same as `run_sim_host.sh` in section 4.1 of *05 Simulation and Visualization*: it first starts the broadcast in the background and opens the window once the broadcast is ready; when you close the window, press `Ctrl+C` or close the terminal,
the broadcast and visualization it started in the container are stopped together, while the container itself keeps running in the background. It also has the options `--image`, `--name`, `--restart`, `--data`, `--state`, `--ble`, `-e`, which are passed to `run.sh` as is and mean the same as the `run.sh` options of the same name;
the container always uses the SDK shipped in the image, so `--venv` is not accepted. For all options see `./run_sim_docker.sh --help`.
If the container is not running, the script first creates it with `run.sh`. As with `run.sh`, any of `--data`, `--state`, `--ble`, `-e` not given this time are carried over from the original container,
and after an upgrade it likewise switches to the new image automatically (section 4); with `--restart` the container is recreated strictly with the options given this time. When you are already in the container, run `./run_sim_host.sh` directly.

- The container includes drivers only for Intel and AMD GPUs. Whether the window can use a GPU depends on which GPU drives the host desktop:
  - Desktop driven by an Intel/AMD GPU: hosts with only these two kinds of GPU, and laptops with both an Intel/AMD integrated GPU and an NVIDIA discrete GPU whose desktop is still driven by the integrated GPU.
    Windows in the container are rendered by the Intel/AMD GPU, typically at up to 60 frames per second.
  - Desktop driven by an NVIDIA GPU: hosts with only an NVIDIA GPU, and laptops switched to use only the discrete GPU. Windows in the container still open,
    but are rendered by the CPU instead, at under 10 frames per second; at startup the terminal prints `libGL error: failed to load driver: nouveau`.
    On such hosts, run the visualization on the host (see the next item); windows on the host are rendered by the NVIDIA GPU at about 60 frames per second.
- The visualization only receives local broadcasts; **the visualization on the host can display data from `revohuman-live` in the container**, and vice versa:
  `run.sh` makes the container and the host use the same broadcast location. So you can run only the visualization on the host and keep the SDK in the container:
  the host only needs the source bundle extracted and `simulation/run_viewer.sh` run (see section 9 of *01 Installation and System Requirements*), without installing the SDK or ROS 2.
  When the container is already broadcasting, running `simulation/run_sim_host.sh` on the host likewise only opens the window and does not start another broadcast.
- When the window closes, the terminal prints the frame rate, for example `[view] 窗口关闭：3580 帧 / 60.0s（59.7 fps）` (window closed: 3580 frames / 60.0 s (59.7 fps)). If it is clearly below 60,
  the window is not using the GPU; in that case, too, run the visualization on the host instead (see the previous item).
- When logged in over SSH without a desktop, use the windowless mode from section 5 of *05 Simulation and Visualization*; a container created with `./run.sh --no-gui` does not connect to the graphical display.

## 9. ROS 2

The container shares the host's network, so ROS 2 topics are visible both inside and outside the container. When creating the container, `run.sh` carries the host's `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY` and
`RMW_IMPLEMENTATION` into the container; if you change these variables on the host, recreate the container with `--restart` for the change to take effect in the container.

After the nodes from *06 ROS 2* start in the container, if ROS 2 Humble is installed on the host you can see their topics directly with `ros2 topic list`.

## 10. Common problems

| Symptom | Action |
|---|---|
| `run.sh: 无法连接 Docker` (cannot connect to Docker) | The Docker service is not started, or the current user is not in the `docker` group or has not logged in again after joining it: see section 2 |
| `run.sh: 找不到镜像 revohuman:0.1.4` (image revohuman:0.1.4 not found) | The image has not been built yet (run `./install.sh`, see section 3), or a different image name was used when building (specify it with `--image`) |
| `run.sh: 镜像里的用户是 …，当前用户是 …` (the user in the image is …, the current user is …) | The image was built for another user: as the current user, run `./install.sh --tag <your-image-name>` to build a separate copy, then specify it with `./run.sh --image` |
| `run.sh: 注意：没有发现手套串口` (note: no glove serial port found) | Docker is older than 25 and the gloves were not plugged in when the container was created. Plug them in, then run the recreate command given in the message |
| The health check in the container notes that the glove serial port node is unavailable and suggests recreating the container | Docker is older than 25 and the gloves were plugged in or replugged after the container was created: run the recreate command shown when entering with `run.sh` |
| `腕部相机不可用` (wrist camera unavailable) is reported in the container, or `Operation not permitted` occurs when opening `/dev/video…` (`/run/host/dev/video…`) | The container was created with a different set of settings, or Docker is older than 25 and the camera was plugged in after the container was created: run the recreate command shown when entering with `run.sh` |
| `run.sh: 注意：容器 revohuman 是用镜像 … 创建的，里面运行的还是旧版本的程序` (note: container revohuman was created from image …; the programs running in it are still the old version) | After an upgrade, programs are still running in the container (the message lists which ones), so this time `run.sh` enters the old container first without interrupting them: once they have all exited, running `./run.sh` again switches to the new image automatically; to switch now, run the command given in the message (those programs will be terminated) |
| `install.sh: 不要用 root 或 sudo 运行本脚本` (do not run this script as root or with sudo) | Run `./install.sh` as a regular user; the script asks separately for the steps that need `sudo` |
| `install.sh: 在 … 里缺少这些发布文件`, `… 与 SHA256SUMS 对不上` (these release files are missing in …; … does not match SHA256SUMS) | Add or re-download the listed files as the message says, and put them in the directory where you extracted the source bundle (the same directory as `SHA256SUMS`) |
| `install.sh` ends by saying that the `docker` group takes effect only after you log out and log back in | The current user was just added to the `docker` group: this build has completed as usual; until you log out and back in once, use `sg docker -c './run.sh'` instead of `./run.sh` as the message says |
| `install.sh: 基础镜像 … 不是 Ubuntu 22.04` (base image … is not Ubuntu 22.04) | The image given with `--base-image` is not an Ubuntu 22.04 image: switch to an Ubuntu 22.04 image (for example `ubuntu:22.04` on a mirror registry), or drop `--base-image` to use the default |
| `install.sh: … run.sh 用它创建容器失败` (… run.sh failed to create a container with it) | The image's own checks have passed; the problem is in the container creation step: act on the `run.sh` output included in the message; it is most likely a Docker or host configuration issue |
| pip or apt downloads time out during the build | Unstable network; just rerun `./install.sh`, and completed steps are not redone; on networks in mainland China add `--mirror cn` |
| `revohuman-lighthouses scan` in the container reports `蓝牙是必需的：…` (Bluetooth is required: …) | The container was created without `--ble`, or the host's Bluetooth service is not running: see section 7.1 |
| Windows in the container do not open, reporting `cannot open display` or similar | Run `./run.sh --restart` in a terminal opened on the host desktop; when logged in over SSH, use the windowless mode instead |
| Windows in the container open but are very sluggish, and `libGL error: failed to load driver` is printed at startup | The host desktop is driven by an NVIDIA GPU, so the container can only render with the CPU: run the visualization on the host instead; see section 8 |
| `amdgpu: os_same_file_description couldn't determine …` and `amdgpu: drmGetDevice2 failed.` are printed when a window opens in the container | Does not affect use: the container's security restrictions block one check that the AMD GPU driver performs at startup, and the window is still rendered by the GPU. It is normal if the frame rate printed when the window closes is close to 60 |
| `run_sim_docker.sh` reports `容器里的源码包（revohuman-0.1.0）没有 run_sim_host.sh` (the source bundle in the container (revohuman-0.1.0) has no run_sim_host.sh) | After an upgrade, other programs are still running in the container, so `run.sh` did not switch to the new image this time and the container still has the previous version's programs: rerun after they have all exited and it switches automatically; to switch now, run `./run_sim_docker.sh --restart` (include options such as `--ble`, `--data` that were given when the container was created). If it still happens after recreating, rebuild the image with `docker/install.sh` from this source bundle on the host |
| `calibrate_lighthouses.sh --docker` reports `容器里的源码包（revohuman-0.1.1）没有 calibrate_lighthouses.sh` (the source bundle in the container (revohuman-0.1.1) has no calibrate_lighthouses.sh) | Same as above: rerun after the other programs in the container have all exited and it switches automatically; to switch now, run the command given on the line after the error (the original command plus `--restart`). If it still happens after recreating, rebuild the image with `docker/install.sh` from this source bundle on the host; see section 7.3 |
| `groups: cannot find name for group ID …` is printed when entering the container | Does not affect use: `run.sh` brings the user groups of host devices such as the GPU and glove serial ports into the container, and these groups have no names in the image. Images built with `./install.sh` from the current source bundle no longer print it |
| Recording files or configuration are gone after `--restart` | They were written to a location outside the table in section 5 and were cleared when the container was recreated |
| Other error codes | Same as running directly on the host; see *07 Logging and Error Codes* |
