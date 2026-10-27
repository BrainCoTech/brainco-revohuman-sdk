# 在容器里运行 RevoHuman

本目录提供一键构建脚本、构建 Docker 镜像的 Dockerfile 与启动脚本：构建出的镜像里装好了 Ubuntu 22.04、ROS 2 Humble、RevoHuman SDK、ROS 2 包与可视化环境，
主机上只需要 Docker，不必安装 ROS 2 与 Python 依赖。完整说明（前提条件、目录与标定的存放位置、设备与图形界面、常见问题）
见使用手册 [《08-容器运行》](../docs/08-容器运行.md)。

| 文件 | 用途 |
|---|---|
| `install.sh` | 一键构建镜像：检查并准备主机（Docker、`docker` 组、Tracker 设备规则），核对发布文件，构建镜像，构建后在镜像里跑一遍检查 |
| `Dockerfile` | 用发布文件（两个 wheel、源码包、`SHA256SUMS`）构建镜像，步骤与手册《01-环境安装与系统要求》相同，构建完成前逐项自检 |
| `Dockerfile.dockerignore` | 构建时只把发布文件发给 Docker |
| `run.sh` | 启动容器并进入：挂载手套、Tracker、图形界面与数据目录 |

## 构建

在存放发布文件的目录里（与手册《01》第 2 节的下载目录相同）解开源码包，再执行 `install.sh`：

```bash
cd ~/revohuman-release
tar xzf revohuman-0.1.1-bundle.tar.gz
cd revohuman-0.1.1/docker
./install.sh                # 国内网络：./install.sh --mirror cn
```

用普通用户执行，不要加 `sudo`；需要 `sudo` 的步骤（安装 Docker、加入 `docker` 组、安装 Tracker 设备规则）执行前逐项询问，
加 `--skip-host-setup` 则只检查、不改动主机。出错时会说明原因和处理办法，改好后重新执行即可：
镜像已经用同一批发布文件、同一个基础镜像为当前用户构建过时不重复构建，只重跑检查。全部选项见 `./install.sh --help`。

也可以不用脚本，直接在存放发布文件的目录里构建：

```bash
cd ~/revohuman-release
docker build -f revohuman-0.1.1/docker/Dockerfile \
    --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g) \
    -t revohuman:0.1.1 .
```

国内网络用的镜像源参数见 [《08-容器运行》](../docs/08-容器运行.md) 第 3 节。

## 运行

```bash
cd ~/revohuman-release/revohuman-0.1.1/docker
./run.sh                                  # 进入容器的 shell
./run.sh python3 -m revohuman doctor      # 或者直接执行一条命令
```

容器在后台常驻，再开一个终端执行 `./run.sh` 会进入同一个容器。录制数据写在主机的 `~/revohuman-data`（容器里的 `~/data`），
标定与运行状态存放在主机的 `~/.local/share/revohuman-docker`，删除容器不影响它们。
Docker 25 及以上，容器起来之后插拔手套不用重建容器；更早的版本插拔手套之后要执行 `./run.sh --restart`。
从之前的版本升级、重新构建镜像之后，之前创建的容器里仍是旧版本的程序，同样执行一次 `./run.sh --restart`。
不用时执行 `./run.sh --stop`。全部选项见 `./run.sh --help`。
配对、标定基站要用蓝牙：在主机上执行 `./run.sh --restart --ble` 重建容器，容器里的完整步骤见手册[《08-容器运行》](../docs/08-容器运行.md)第 7 节。

要在容器里看三维可视化，可以在主机上用 `simulation/run_sim_docker.sh` 一条命令拉起广播与窗口（窗口显示在主机桌面上，关窗口时一并停止广播）：

```bash
cd ~/revohuman-release/revohuman-0.1.1/simulation
./run_sim_docker.sh                       # --image / --name 与 run.sh 的同名选项相同
```

见手册 [《08-容器运行》](../docs/08-容器运行.md) 第 8 节。
