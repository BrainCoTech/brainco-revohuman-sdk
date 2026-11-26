# Running RevoHuman in a container

English | [简体中文](README.zh-CN.md)

This directory provides a one-step build script, the Dockerfile for building the Docker image, and a launch script. The built image has Ubuntu 22.04, ROS 2 Humble, the RevoHuman SDK, the ROS 2 packages and the visualization environment installed,
so the host only needs Docker, without installing ROS 2 or Python dependencies. For the full description (prerequisites, where directories and calibrations are stored, devices and the graphical interface, FAQ),
see [*08 Running in Containers*](../docs/08-running-in-containers.md) in the user manual.

| File | Purpose |
|---|---|
| `install.sh` | One-step image build: checks and prepares the host (Docker, the `docker` group, Tracker device rules), verifies the release files, builds the image, and runs a check inside the image after building |
| `Dockerfile` | Builds the image from the release files (two wheels, the source bundle, `SHA256SUMS`) with the same steps as *01 Installation and System Requirements* in the manual, running item-by-item self-checks before the build completes |
| `Dockerfile.dockerignore` | Sends only the release files to Docker during the build |
| `run.sh` | Starts the container and enters it: mounts the gloves, Trackers, graphical interface and data directory |

## Build

In the directory where the release files are stored (the same as the download directory in section 2 of *01 Installation and System Requirements* in the manual), extract the source bundle, then run `install.sh`:

```bash
cd ~/revohuman-release
tar xzf revohuman-0.1.4-bundle.tar.gz
cd revohuman-0.1.4/docker
./install.sh                # networks in mainland China: ./install.sh --mirror cn
```

Run it as a regular user, without `sudo`; each step that needs `sudo` (installing Docker, joining the `docker` group, installing the Tracker device rules) asks before it runs,
and with `--skip-host-setup` the script only checks and does not modify the host. On error, it explains the cause and how to fix it; after fixing, simply run it again:
if the image has already been built for the current user from the same release files and the same base image, it is not rebuilt, and only the checks are rerun. For all options, see `./install.sh --help`.

You can also build directly in the directory where the release files are stored, without the script:

```bash
cd ~/revohuman-release
docker build -f revohuman-0.1.4/docker/Dockerfile \
    --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g) \
    -t revohuman:0.1.4 .
```

For the mirror options for networks in mainland China, see section 3 of [*08 Running in Containers*](../docs/08-running-in-containers.md).

## Run

```bash
cd ~/revohuman-release/revohuman-0.1.4/docker
./run.sh                                  # enter a shell in the container
./run.sh python3 -m revohuman doctor      # or run a single command directly
```

The container keeps running in the background; running `./run.sh` in another terminal enters the same container. Recorded data is written to `~/revohuman-data` on the host (`~/data` in the container),
and calibrations and runtime state are stored in `~/.local/share/revohuman-docker` on the host; deleting the container does not affect them.
With Docker 25 or later, plugging or unplugging gloves after the container is up does not require recreating the container; with earlier versions, after plugging or unplugging gloves, recreate the container with the command that `run.sh` prints.
After upgrading from an earlier version and rebuilding the image, the next `./run.sh` switches to the new image automatically, keeping the `--data`, `--ble`, `-e` and other settings given when the container was created;
if programs are still running in the container, it tells you first and does not interrupt them. `--restart` recreates the container strictly with the options given this time and does not carry over the previous settings.
When you are not using it, run `./run.sh --stop`. For all options, see `./run.sh --help`.
Pairing and calibrating base stations require Bluetooth: on the host, run `./run.sh --restart --ble` to recreate the container (if you passed options such as `--data` or `-e` when creating the container, include them as well); for the complete steps inside the container, see section 7 of [*08 Running in Containers*](../docs/08-running-in-containers.md) in the manual.
Once the allowlist is filled in, scanning, calibration and the self-check can also be done on the host with a single command, `calibrate_lighthouses.sh --docker` in the source bundle root (as in section 7.3).

To view the 3D visualization from the container, you can bring up the broadcast and the window from the host with a single command, `simulation/run_sim_docker.sh` (the window is shown on the host desktop, and closing the window also stops the broadcast):

```bash
cd ~/revohuman-release/revohuman-0.1.4/simulation
./run_sim_docker.sh                       # container options such as --image / --name / --ble / --data are passed to run.sh as is
```

See section 8 of [*08 Running in Containers*](../docs/08-running-in-containers.md) in the manual.
