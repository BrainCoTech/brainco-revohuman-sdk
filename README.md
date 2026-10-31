# RevoHuman SDK

RevoHuman 数据手套的采集 SDK：读取双手关节角、Tracker 位姿、全指触觉与腕部相机，
按型号自动装配，提供 Python 与 ROS 2 两个入口，并能把整套数据录成 MCAP。

| 型号 | 组成 |
|---|---|
| **P1** | 编码器手套（每手 21 关节）+ 双手 Vive Tracker |
| **PTV1** | P1 + 全指触觉（241 点）+ 腕部相机 |

本文只讲安装与上手。全部接口、数据语义、ROS 话题与参数、错误码见 [《RevoHuman SDK 参考手册》](docs/RevoHuman%20SDK%20参考手册.md)（`docs/RevoHuman SDK 参考手册.md`）。
P1 的详细使用手册（环境安装、硬件连接、基站、录制、仿真、ROS 2、日志与错误码、容器运行、时间戳同步）从 [`docs/00-总览.md`](docs/00-总览.md) 读起。

---

## 支持的硬件

| 部件 | 要求 |
|---|---|
| 手套 | RevoHuman P1 或 PTV1，左右手各一只，USB 接主机 |
| Tracker | Vive Tracker，每只手一只，插在该手手套单元的 USB HUB 上 |
| 基站 | SteamVR 基站 2.0，1–4 台（背面标签上的序列号以 `FA` 开头，共 12 位） |
| 蓝牙 | 主机上一块可用的蓝牙适配器（用来读取基站的序列号与信道） |
| 腕部相机 | PTV1 自带，每只手一只 |

## 系统要求

| 项 | 要求 |
|---|---|
| 操作系统 | Ubuntu 22.04 |
| Python | CPython 3.10（Ubuntu 22.04 自带的 `python3`） |
| 架构 | x86_64 / aarch64 |
| ROS 2 | Humble，只有用 ROS 2 节点时需要（分步安装第 ⓪ 步） |

核心 SDK 以编译模块发布：wheel 只能装在文件名标明的 Python 版本与架构上，装错会被 pip 拒绝。

---

## 安装

发布文件在 [GitHub Release v0.1.2](https://github.com/BrainCoTech/brainco-revohuman-sdk/releases/tag/v0.1.2) 下载。源码包请下载 `revohuman-0.1.2-bundle.tar.gz`，不要用页面自动生成的 Source code 压缩包（目录名不同，本文命令按源码包解压出的 `revohuman-0.1.2/` 写）。

发布文件：

| 文件 | 内容 |
|---|---|
| `revohuman-<版本>-cp310-cp310-linux_x86_64.whl` | 核心 SDK，x86_64 |
| `revohuman-<版本>-cp310-cp310-linux_aarch64.whl` | 核心 SDK，aarch64 |
| `pysurvive-1.1.21+revohuman.2-cp310-cp310-linux_x86_64.whl` | Tracker 驱动 libsurvive，x86_64 |
| `pysurvive-1.1.21+revohuman.2-cp310-cp310-linux_aarch64.whl` | Tracker 驱动 libsurvive，aarch64 |
| `revohuman-<版本>-bundle.tar.gz` | 源码包：ROS 2 包、手部模型（URDF）、示例、文档、仿真、Docker 镜像构建文件、一键安装脚本 |
| `SHA256SUMS` | 校验和 |

以下以把发布文件放在 `~/revohuman-release` 为例。按用途选一种装法：

| 装法 | 适合 | 装上的内容 |
|---|---|---|
| [一键安装到虚拟环境](#一键安装到虚拟环境不含-ros-2) | 用 Python 接口、命令行、录制与三维可视化，**不用 ROS 2** | 除 ROS 2 以外的全部依赖，装进独立的虚拟环境 `revohuman_sdk` |
| [分步安装](#分步安装含-ros-2) | 要用 ROS 2 节点 | ROS 2 Humble、SDK（装在用户目录）、ROS 2 包、三维可视化 |
| [容器](#容器) | 主机不是 Ubuntu 22.04，或不想在主机上装任何依赖 | 全部装在 Docker 镜像里 |

前两种可以装在同一台主机上。基站白名单、基站几何、零位等配置与标定存放在家目录
（`~/.config/revohuman`、`~/.config/libsurvive`、`~/.local/state/revohuman`），两种装法共用一份，换用另一种不必重新配对、标定。

### 一键安装到虚拟环境（不含 ROS 2）

解开源码包，以普通用户执行根目录下的 `install_venv.sh`（不要加 `sudo`，需要 `sudo` 的步骤脚本会单独询问）：

```bash
cd ~/revohuman-release
tar xzf revohuman-0.1.2-bundle.tar.gz              # → revohuman-0.1.2/
revohuman-0.1.2/install_venv.sh
```

脚本按需安装系统依赖与 Tracker 设备规则、把当前用户加入 `dialout` 组（需要 `sudo` 的步骤执行前逐项询问），
再在 `~/.venvs/revohuman_sdk` 建虚拟环境，装好随附的 pysurvive、SDK（含录制、相机图像解码、基站蓝牙、
三维可视化四个 extra）与三维可视化的依赖，最后自检。
虚拟环境不改动系统 Python 和用户目录下的 Python 包；重新执行时已装好的部分自动跳过。国内网络加 `--mirror cn`。
脚本要用 CPython 3.10，Ubuntu 22.04 系统自带的就是。每一步的说明与全部选项见
[`docs/01-环境安装与系统要求.md`](docs/01-环境安装与系统要求.md) 第 11 节，或执行 `install_venv.sh --help`。

使用前激活虚拟环境，之后 `python3 -m revohuman`、`revohuman-live`、`revohuman-lighthouses` 都可直接使用：

```bash
source ~/.venvs/revohuman_sdk/bin/activate
python3 -m revohuman --help
```

每个新终端都要先激活一次；`deactivate` 退出虚拟环境。第一次被加入 `dialout` 组后，要**注销并重新登录一次**才能打开手套串口。
之后从[快速上手](#快速上手)接着做；虚拟环境里没有 ROS 2，要用 ROS 2 节点时按下面的分步安装再装一遍。

### 分步安装（含 ROS 2）

**⓪ ROS 2 Humble**

添加 ROS 2 软件源：

```bash
sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

> 国内网络访问 `packages.ros.org` 较慢时，可把上面的 `http://packages.ros.org/ros2/ubuntu`
> 换成 `https://mirrors.aliyun.com/ros2/ubuntu`，其余不变。

安装 ROS 2 与构建工具，并让每个新终端自动加载 ROS 环境：

```bash
sudo apt update
sudo apt install -y ros-humble-ros-base ros-dev-tools
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash
```

**① 系统依赖与串口权限**

```bash
sudo apt update
sudo apt install -y python3-pip python3-venv bluez libgl1 libegl1
sudo usermod -aG dialout $USER
```

`python3-venv`、`libgl1`、`libegl1` 供三维可视化使用，三个都要装。

**② Tracker 驱动 libsurvive（随附的 pysurvive 1.1.21+revohuman.2）**

```bash
cd ~/revohuman-release
sha256sum -c SHA256SUMS
python3 -m pip install --user --no-deps pysurvive-1.1.21+revohuman.2-cp310-cp310-linux_$(uname -m).whl
sudo tee /etc/udev/rules.d/81-revohuman-tracker.rules > /dev/null <<'RULES'
# Vive Tracker（USB 有线，28de:2300）
SUBSYSTEM=="usb", ATTRS{idVendor}=="28de", ATTRS{idProduct}=="2300", MODE="0666"
KERNEL=="hidraw*", SUBSYSTEM=="hidraw", ATTRS{idVendor}=="28de", ATTRS{idProduct}=="2300", MODE="0666"
RULES
```

附带的 pysurvive 基于上游 1.1.21（libsurvive `v1.01-21-gd89876a`），含本 SDK 所需的驱动修正。
请装这一份，不要装 PyPI 上的 `pysurvive==1.1.21` 或其他版本；已经装过的机器照样执行上面的命令即可替换。

`--no-deps` 不能省：pysurvive 声明了一个 SDK 用不到的图形界面依赖（wxPython），它在 Linux 上装不上。

使规则生效（已插着的 Tracker 需要重新插拔一次）：

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**③ SDK、源码包与三维可视化环境**

```bash
cd ~/revohuman-release
WHL=revohuman-0.1.2-cp310-cp310-linux_$(uname -m).whl
python3 -m pip install --user "$WHL[recording,camera,ble,viewer]"
tar xzf revohuman-0.1.2-bundle.tar.gz              # → revohuman-0.1.2/
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
export PATH="$HOME/.local/bin:$PATH"
revohuman-0.1.2/simulation/run_viewer.sh --help    # 建好可视化自带的 Python 环境，首次约 1–2 分钟
```

四个 extra 依次是：MCAP 录制、相机图像解码、基站蓝牙、三维可视化。三维可视化另在
`simulation/.venv` 里用自己的 Python 环境，与 SDK 互不影响；看到参数说明即表示已就绪。

**④ 注销并重新登录一次**，让 `dialout` 组、ROS 2 环境与 `~/.local/bin`（`revohuman-live`、`revohuman-lighthouses`
两个命令所在的目录）生效。

### 容器

**不想在主机上装 ROS 2 与 Python 依赖**时，可以用源码包 `docker/` 目录里的 `install.sh` 一键构建 Docker 镜像来运行，
主机上只需要 Docker（没装时脚本会询问后代为安装）：构建与运行见 [`docker/README.md`](docker/README.md)。

---

## 快速上手

以下命令在源码包解压出的 `~/revohuman-release/revohuman-0.1.2/` 目录里执行；手套与 Tracker 已插好，基站已通电。
用虚拟环境安装的，先 `source ~/.venvs/revohuman_sdk/bin/activate`。
两只 Tracker 要先开机：**短按** Tracker 上的电源键，指示灯变**绿色**即可（蓝色闪烁说明按久了、进了无线配对模式，本套件用不到，按住约 5 秒关机再短按开机；红色闪烁是电量低，连着 USB 可边充边用）。Tracker 经手套单元的 USB 线直接传数据，不需要 SteamVR 无线配对，详见 [`docs/02-硬件连接.md`](docs/02-硬件连接.md) 第 2.1 节。

### 1. 体检

```bash
cd ~/revohuman-release/revohuman-0.1.2
python3 -m revohuman doctor              # 只读：USB 拓扑、串口、手别、Tracker、相机
python3 -m revohuman doctor --probe      # 另外短开串口读固件信息（正在采集时不要加）
```

### 2. 确认型号与左右手

```bash
python3 -m revohuman product identify
```

SDK 从手套 USB 序列号与在位硬件识别左右手和型号。认不出时拒绝启动并说明原因，不按插口顺序猜；
早期批次两只手套序列号相同，这时改按串口指定（参考手册 2.3）。

### 3. 准备基站（每个场地一次）

**摆放**：基站装在约 2 m 高处（高于头顶），向下倾斜 30°–45°，对准双手的工作区。
一台基站放在人的前方或侧前方；两台放在前方左右两侧、视野交叉，需要四周都照得到时改为对角摆放（例如左前和右后）。
摆好后固定牢：标定之后再挪动或碰歪，就要重新标定。

**环境**：基站和工作区避开阳光直射与强光，基站视野里不要有镜子、玻璃、亮面金属等反光面，基站与 Tracker 之间不要有遮挡。
标定一直停在「几何置信度偏低」、或 Tracker 静止时位姿漂移，多半是环境干扰。
完整注意事项（参考 VIVE 官方说明）见 [`docs/03-基站使用与配对.md`](docs/03-基站使用与配对.md) 第 1.1 节「摆放环境」。

![基站摆放建议：一台基站与两台基站](docs/pic/lighthouse_placement.png)

**配对与标定**：基站的序列号印在背面标签上，`FA` 开头、共 12 位（下图橙框处），白名单只认这个编号。
先扫描确认 SDK 能读到它，把它写进白名单，再扫一次确认信道无冲突，最后标定基站位置：

<img src="docs/pic/lighthouse.png" alt="基站背面标签：橙框内是序列号" width="300">

```bash
revohuman-lighthouses scan                  # 扫描附近基站：序列号 / 信道 / 信号
revohuman-lighthouses init                  # 写出白名单配置文件，并打印它的路径
${EDITOR:-nano} ~/.config/revohuman/lighthouses.yaml   # 把占位符换成自家基站背面标签上的序列号
revohuman-lighthouses scan                  # 白名单里每台都打 ✓、且没有同信道冲突才算通过
python3 -m revohuman tracker calibrate      # 标定基站几何
python3 -m revohuman tracker check --live   # 起一次采集，确认基站与 tracker 都已就绪
```

`${EDITOR:-nano}` 会打开环境变量 `EDITOR` 指定的编辑器，没设时用 nano；两样都没有时，换成本机任意文本编辑器（例如 `gedit`、`vi`）打开这个文件，或先执行 `sudo apt install nano`。

白名单文件的写法：

```yaml
lighthouses:
  - FAXXXXXXXXXX          # 第一台基站背面标签上的序列号
  - serial: FAYYYYYYYYYY  # 第二台
    place: 右后上方        # 位置备注，报错时用来指明是哪一台
```

- 第二次 `scan` 报同信道冲突时：`revohuman-lighthouses plan` 看改信道方案，`revohuman-lighthouses apply --yes` 执行，再回到 `scan`。
- 标定与 `check --live` 之前先看两只 Tracker 的指示灯是否为绿色；不亮时短按电源键开机。
- 标定时拿着 Tracker 在所有基站都看得到的区域里**换一个位置和朝向 → 停住约 1 秒 → 再换**，直到命令报告完成。
- 几何会落盘，之后每次启动自动沿用。基站被移动、更换、增减或改过信道后，重新 `scan` 并 `tracker calibrate`。
- 白名单填好之后，后面的扫描、标定与 `check --live` 也可以用源码包根目录的 `./calibrate_lighthouses.sh` 一条命令做完（只读基站，有冲突时停下）。
- 逐步说明见 [`docs/03-基站使用与配对.md`](docs/03-基站使用与配对.md)。

### 4. 采集

启动前把双手摆在零位：SDK 启动时按当时的位姿自动记零位，Tracker 的 `delta` 都相对它。

```python
from revohuman import open_kit

with open_kit() as kit:                        # 识别型号并装配全部传感器
    print(kit)                                 # RevoHumanKit(PTV1, hands=['left', 'right'], sensors=[...])

    sub = kit.synchronizer().subscribe()       # 双手配对帧
    pair = sub.recv(timeout=1.0)
    print(pair.skew_ns, pair.left.joints_deg[:3])

    print(kit.tracker.delta("right"))          # 右手 tracker 相对零位的位姿

    rec = kit.recorder()                       # 已接好本套全部数据流
    handle = rec.start("episode_001.mcap")
    ...
    print(handle.stop())                       # {"frames": ..., "per_channel": {...}}
    rec.close()
```

可直接运行的版本：`python3 examples/kit_quickstart.py`。

### 5. 实时查看

一条命令拉起广播与可视化窗口：

```bash
simulation/run_sim_host.sh       # 后台启动 revohuman-live，前台打开 MuJoCo 窗口显示双手关节与 Tracker 位姿
```

关闭窗口时一并停止脚本启动的广播。不接硬件时用 `simulation/run_sim_host.sh --demo P1` 看演示；
用容器运行时改在主机上执行 `simulation/run_sim_docker.sh`。
用法、选项与常见问题见 [`docs/05-仿真与可视化.md`](docs/05-仿真与可视化.md) 第 4 节。

---

## ROS 2

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src && ln -s ~/revohuman-release/revohuman-0.1.2/ros2/* ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --symlink-install
```

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch revohuman_bringup revohuman.launch.py      # 双手，自动识别型号
```

`--show-args` 列出全部 launch 参数；话题、服务与参数见参考手册第 7 节。
ROS 节点与 Python 入口打开的是同一套手套和 Tracker，两者不要同时运行。

---

## 命令行

`python3 -m revohuman [-v] <命令>`，除 `live` 外所有命令都支持 `--json`：

| 命令 | 说明 |
|---|---|
| `doctor [--probe]` | 体检 |
| `product identify` | 识别型号与左右手 |
| `tracker calibrate` | 标定基站几何 |
| `tracker check [--live]` | 启动前自查 |
| `tracker status` | 离线查看白名单、几何、零位与上次启动检查的结论 |
| `tracker set-reference` | 重记零位 |
| `tracker preflight` | 只做信道预检的快速版（被动扫描，不连接基站） |
| `tracker lighthouses <子命令>` | 同 `revohuman-lighthouses <子命令>` |
| `live` | 本机实时广播，同 `revohuman-live` |
| `firmware info` / `firmware upgrade` | 手套固件版本 / 升级（参考手册 2.4） |
| `glove calibrate` | 手套编码器零位标定，写入手套、不可撤销（参考手册 2.5） |

基站管理用 `revohuman-lighthouses <子命令>`：`scan`、`show`、`init`、`plan`、`apply`、`set-channel`、`identify`。

源码包里的脚本（均支持 `--help`）：

| 脚本 | 说明 |
|---|---|
| `install_venv.sh` | 一键安装到虚拟环境（不含 ROS 2） |
| `calibrate_lighthouses.sh` | 一键标定基站：扫描 → 标定 → 启动前自查（加 `--docker` 在容器里执行） |
| `simulation/run_sim_host.sh` | 主机上一键拉起广播与三维可视化 |
| `simulation/run_sim_docker.sh` | 在容器里一键拉起广播与三维可视化 |
| `simulation/run_viewer.sh` | 只启动三维可视化 |
| `docker/install.sh` / `docker/run.sh` | 一键构建 Docker 镜像 / 进入容器 |

退出码：`0` 成功；`1` 失败或未就绪；`2` 用法错误；`3` 被拒绝或型号不符；`calibrate_lighthouses.sh` 另有 `4`（白名单没填好或扫描没通过，要先处理）。完整参数见参考手册第 8 节。

## 出错时

SDK 的异常都继承 `revohuman.RevoHumanError`，`revohuman.format_error(exc)` 给出带错误码的可读文本；
Tracker 启动检查不通过时抛 `TrackerNotReady`，异常里附有可直接复制执行的修复命令。错误码表见参考手册 6.5。

## 已知限制

- **重新见光时的跳变**：重新见光的那一刻，原始位姿会有一步跳变（实测约 2–3.5 cm、姿态最大约 30°）。
  `rt.latest()` / `delta()` 默认做遮挡恢复平滑，把这一跳摊成约 0.3–1.6 秒的滑移；录制文件与 ROS 话题里是原始位姿，
  保留这一跳。运行期保护只统计相邻两帧位移超过 5 cm 或转角超过 17.2° 的跳变并告警，更小的跳变不计。
- **基站挪动盲区**：运行时 SDK 不重新解算基站位置，基站被平移、或绕竖直轴转动时检测不到，单台、多台基站都是如此；
  倾斜达到 2° 会被发现并停止采集，不到 2° 的倾斜检测不到。
  动过基站之后务必重新 `tracker calibrate`。
- **重力读数抖动引起的停采**：驱动按基站广播的重力方向判断基站有没有被挪动，变化达到 2° 就清掉这台基站的位置
  并改写基站配置，SDK 随即停采，提示基站几何失效或被挪动过。没人碰过基站、但读数偶尔抖动超过 2° 时也会这样。
  此时重启无效，按提示运行 `revohuman-lighthouses scan`，再运行 `python3 -m revohuman tracker calibrate` 重新标定即可。

## 使用须知

- 一只手套、一套 libsurvive 同一时刻只能被一个进程打开；第二个进程会拿到 `PortBusyError` / `TrackerBusyError`（附持有者 pid）。
- 多台主机的数据要对齐时，让它们对同一台 NTP 服务器对时（参考手册第 10 节）。
- 手套序列号为出厂写入，SDK 只读不写。
