# 在容器里运行 RevoHuman

本目录提供构建 Docker 镜像的 Dockerfile 与启动脚本：构建出的镜像里装好了 Ubuntu 22.04、ROS 2 Humble、RevoHuman SDK、ROS 2 包与可视化环境，
主机上只需要 Docker，不必安装 ROS 2 与 Python 依赖。完整说明（前提条件、目录与标定的存放位置、设备与图形界面、常见问题）
见使用手册《08-容器运行》。

| 文件 | 用途 |
|---|---|
| `Dockerfile` | 用发布文件（两个 wheel、bundle、`SHA256SUMS`）构建镜像，步骤与手册《01-环境安装与系统要求》相同，构建完成前逐项自检 |
| `Dockerfile.dockerignore` | 构建时只把发布文件发给 Docker |
| `run.sh` | 启动容器并进入：挂载手套、Tracker、图形界面与数据目录 |

## 构建

在存放发布文件的目录里执行（与手册《01》第 2 节的下载目录相同）：

```bash
cd ~/revohuman-release
tar xzf revohuman-0.1.0-bundle.tar.gz
docker build -f revohuman-0.1.0/docker/Dockerfile \
    --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g) \
    -t revohuman:0.1.0 .
```

国内网络可以加上镜像源，例如：

```bash
docker build -f revohuman-0.1.0/docker/Dockerfile \
    --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g) \
    --build-arg UBUNTU_MIRROR=http://mirrors.aliyun.com \
    --build-arg ROS_APT_MIRROR=https://mirrors.aliyun.com/ros2/ubuntu \
    --build-arg PIP_INDEX_URL=https://mirrors.aliyun.com/pypi/simple/ \
    -t revohuman:0.1.0 .
```

## 运行

```bash
cd ~/revohuman-release/revohuman-0.1.0/docker
./run.sh                                  # 进入容器的 shell
./run.sh python3 -m revohuman doctor      # 或者直接执行一条命令
```

容器在后台常驻，再开一个终端执行 `./run.sh` 会进入同一个容器。录制数据写在主机的 `~/revohuman-data`（容器里的 `~/data`），
标定与运行状态存放在主机的 `~/.local/share/revohuman-docker`，删除容器不影响它们。
重新插拔手套之后执行 `./run.sh --restart`；不用时执行 `./run.sh --stop`。全部选项见 `./run.sh --help`。
