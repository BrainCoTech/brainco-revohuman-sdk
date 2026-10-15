# 手套可视化（simulation/）

一个 MuJoCo 窗口，左右两个视图同时看一副手套：

| 视图 | 内容 |
|---|---|
| 左：**笛卡尔视图** | 双手按 Tracker 给出的 **6D 位姿**摆在同一个 z 朝上的世界里；手指关节、触觉热力图随手一起动。每只手画掌根坐标轴与到地面的垂线，地面网格 0.1 m。 |
| 右：**固定基座视图** | 两掌固定摆放（间距 `--spacing`，默认 0.30 m），只看手指关节与**触觉热力图**。 |

- **型号跟着 SDK 走**：P1 / PTV1 都支持。型号、手、关节表都由 SDK 在连接时告诉可视化，两种型号走同一条路。
- **触觉按外设识别决定开不开**：型号带触觉、且这只手的固件外设探测报了触觉模块、且点数与布局一致（241 点）
  才画热力图；否则这只手只画关节，启动时终端写明原因。不看数据里有没有非零值。
- **笛卡尔视图跟着 SDK 端的 Tracker 走**：SDK 那边 Tracker 起来了才启用；否则左侧视图显示原因，
  右侧视图照常工作。

## 与 SDK 的关系：SDK 广播，可视化只订阅

可视化**不 import SDK**，也不碰串口和 Tracker。数据流只有一条本机 Unix socket：

```
终端 1（SDK 的环境）                         终端 2（可视化自己的环境）
revohuman-live             ──本机 socket──▶   simulation/run_viewer.sh
打开手套 + tracker，开始广播                 只订阅：收到就更新
```

- **先起哪个都行**。可视化先起时窗口显示「等待 SDK 广播」并给出终端 1 的命令；
  SDK 就绪（socket 出现）后自动连上，收到型号与关节表后建场景并开始更新。
- **SDK 退出时窗口不退**：保持最后一帧并标出断开原因，SDK 再起来自动接上。
  SDK 配置没变（同一型号、同一双手）时直接接着显示，变了就按新配置重建场景。
- 两边装在各自的 Python 环境里：可视化这边**不需要安装 SDK**，SDK 那边**不需要 mujoco**。
  源码包根目录的 `install_venv.sh` 建的 SDK 虚拟环境（默认 `~/.venvs/revohuman_sdk`）把可视化依赖也一起装了，两边可以共用它。
- 不想开两个终端时，用 `run_sim_host.sh` 一条命令把两边一起拉起（见下一节）。

广播端是 SDK 自带的命令行入口 `revohuman-live`（装 SDK 时一起装上）；
与 `python3 -m revohuman live` 完全等价，命令找不到时用后者
（命令入口不在 `PATH` 上：`pip install --user` 装的 SDK 要把 `~/.local/bin` 加进 `PATH`，装在 venv 里的要先激活）：

```bash
revohuman-live                               # 整套手套 + tracker
revohuman-live --hands left                  # 只开左手
revohuman-live --no-tracker                  # 只广播手套（笛卡尔视图不启用）
revohuman-live --help                        # 全部参数
python3 -m revohuman live --hands left       # 同 revohuman-live --hands left
```

socket 位置两边按同一套规则找，一般不用管：`$REVOHUMAN_LIVE_SOCKET` →
`$XDG_RUNTIME_DIR/revohuman/live.sock` → `/tmp/revohuman-<uid>/live.sock`。
目录 0700、socket 0600，只有当前用户能连。要换位置时两边设同一个 `REVOHUMAN_LIVE_SOCKET`
（或可视化加 `--socket <path>`）。协议定义见 SDK 的 `revohuman/live.py` 模块文档。

## 一条命令拉起广播与可视化

```bash
simulation/run_sim_host.sh                   # 后台起 revohuman-live，就绪后打开窗口；关窗口时一并停掉广播
simulation/run_sim_host.sh --hands left      # 只开左手
simulation/run_sim_host.sh --lite -- --tracker-gate error   # 可视化参数写在前面，-- 之后的交给 revohuman-live
simulation/run_sim_host.sh --demo P1         # 不起广播，只看演示（不需要 SDK）
simulation/run_sim_host.sh --help            # 全部参数
simulation/run_sim_docker.sh                 # 同上，广播与窗口都在容器里（先按 docker/README.md 构建镜像）
```

SDK 的查找顺序、等待与退出、已有广播时的处理、日志位置与全部选项见
[《05-仿真与可视化》](../docs/05-仿真与可视化.md) 第 4.1 节；`run_sim_docker.sh` 另见
[《08-容器运行》](../docs/08-容器运行.md) 第 8 节。

## 快速开始

```bash
simulation/run_viewer.sh --demo              # 不接手套：内置演示广播（PTV1，双手 + tracker）
simulation/run_viewer.sh --demo P1           # 演示 P1（无触觉）
simulation/run_viewer.sh                     # 接真手套：另一个终端先起 revohuman-live
simulation/run_viewer.sh --help              # 全部参数
simulation/run_viewer.sh --urdf-dir <目录>   # 手部模型不在默认位置（源码包根目录的 description/）时指定
```

`--demo` 在可视化进程里起一个私有的演示广播，走的是与真 SDK 完全相同的接收、建场景、渲染路径。

`run_viewer.sh` 第一次运行会在 `simulation/.venv` 建一个**隔离的 Python 环境**，
按 `simulation/requirements.txt` 装好依赖（`mujoco` / `glfw` / `numpy` / `PyYAML`），
不碰系统 Python，也不需要容器或 ROS；之后 `requirements.txt` 变了才会重装。
触觉手套的 241 个触觉点贴在手模型各指节上的位置取自随包的贴点表
`revohuman_sim/assets/tactile_placement_241.json`，建场景时直接载入，不需要现场拟合。
用 `--urdf-dir` 换成别的手部模型时，指节运动学与贴点表一致才用这份表，否则按模型网格现场拟合
（指节网格须带套环 / 套帽，十几秒），结果缓存在 `~/.cache/revohuman/`（设了 `XDG_CACHE_HOME` 时在
`$XDG_CACHE_HOME/revohuman/`）。`--refit` 忽略缓存与随包贴点表、按模型网格重新拟合，
同样要求网格带套环 / 套帽，随包的手部模型用不了它。

| 环境变量 | 作用 |
|---|---|
| `REVOHUMAN_SIM_VENV=<dir>` | venv 位置（默认 `simulation/.venv`） |
| `REVOHUMAN_VENV=<dir>` | `run_sim_host.sh` 默认找的 SDK 虚拟环境（默认 `~/.venvs/revohuman_sdk`；`install_venv.sh --venv` 的默认值也取它） |
| `REVOHUMAN_SIM_WAIT=<秒>` | `run_sim_host.sh` 最多等广播就绪多少秒，到时先打开窗口（默认 90） |
| `REVOHUMAN_SIM_PYTHON=<python>` | 建 venv 用的解释器（默认 `python3`，要求 ≥ 3.10） |
| `REVOHUMAN_SIM_PRIME=0` | 不设 NVIDIA PRIME 变量（默认在有 NVIDIA 驱动的机器上让窗口走独显） |
| `REVOHUMAN_LIVE_SOCKET=<path>` | 广播 socket（两边设同一个值） |
| `REVOHUMAN_LOG_COLOR=auto\|always\|never` | 终端日志按级别上色（默认 `auto`：输出是终端、没设 `NO_COLOR`、`TERM` 不是 `dumb` 时上色；重定向到文件时不带颜色）。与 SDK 命令行认同一个变量 |
| `PIP_INDEX_URL=<url>` | pip 镜像 |

不想用脚本，也可以装进自己的环境后直接跑：

```bash
pip install -r simulation/requirements.txt
python3 simulation/viewer.py --demo
```

## 运行前提

以下都是 **SDK 那一端**（`revohuman-live`）的前提，可视化这边只需要 OpenGL：

- **串口权限**：当前用户要能读写手套串口。没有权限时广播端直接给出处理办法，二选一：
  ```bash
  sudo usermod -aG dialout $USER        # 然后重新登录
  # 或装 udev 规则
  echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5710", MODE="0666"' \
    | sudo tee /etc/udev/rules.d/99-revohuman.rules
  sudo udevadm control --reload && sudo udevadm trigger
  ```
- **Tracker（笛卡尔视图）**：libsurvive 的 Python 绑定 pysurvive 装在 **SDK 的环境**里，可视化这边不需要。
  装 SDK 随附的 `pysurvive 1.1.21+revohuman.1`，命令见主 README 安装第 ② 步；不要装 PyPI 上的原版
  `pysurvive==1.1.21`，它少了随附版的两处修正。SDK 随包附带并在这一份上测试发布，
  换用其他版本时 Tracker 的基站检查会自检失败并降级。
  基站几何要先按主 README 快速上手第 3 步标定好（详见 [《RevoHuman SDK 参考手册》](../docs/RevoHuman%20SDK%20参考手册.md) 第 4 节）；
  广播端只读位姿：不做标定、不采纳基站几何，也不记零位（笛卡尔视图画的是绝对位姿，用不到零位）。
  没装 pysurvive、或 Tracker 被别的进程占着时，广播端只广播手套（不抢占），笛卡尔视图显示原因。
- **基站检查档位**（广播端 `--tracker-gate`）：默认 `flag`，基站检查没过也照常广播位姿，
  笛卡尔视图左下角以 `!!` 标出哪一项没过（如 `startup gate: S0 whitelist_missing`、`geometry: 2 issue(s)`），
  具体原因看广播端终端里的 Tracker 日志（这时两手的相对位姿不一定可信）；
  `error` 则检查没过就不广播位姿。
- **单进程独占**：手套串口与 Tracker 同时只能被一个进程打开。录制程序开着时广播端会提示被占用。
  同一个 socket 上只能有一个广播；**订阅方可以有多个**（多开几个可视化窗口都行）。
- **显卡**：需要 OpenGL。没有独显的机器加 `--lite`（关阴影、反射）。

## 操作

| 按键 | 作用 |
|---|---|
| `V` | 布局切换：双视图 → 只看笛卡尔 → 只看固定基座 |
| `F` | 笛卡尔视图相机跟随两掌中点（开 / 关；拖动平移时自动关） |
| `C` | 两个视图的相机复位，重新打开跟随 |
| `J` | 固定基座视图左上角的表格：关节角 ↔ 触觉统计（有手开着触觉时） |
| `Tab` | 触觉统计显示哪只手（双手时） |
| `←` / `→` | 选中上一个 / 下一个触觉点 |
| `PgUp` / `PgDn` | 跳到上一根 / 下一根手指 |
| `↑` / `↓` | 色标量程放大 / 缩小（改为固定量程） |
| `Home` | 以当前读数作为触觉基线（之后显示减去基线的值） |
| `Delete` | 清除基线 |
| `Insert` | 把当前手的触觉点表打印到终端 |
| 鼠标 | 左键拖动旋转、右键拖动平移、滚轮缩放（作用在鼠标所在的视图） |

触觉相关的按键只在固定基座视图显示触觉统计时列出。

## 覆盖层

只显示五类数据：数据帧率、实时关节角、位姿四元数、采样时间戳、到达延迟。行首 `L` / `R` 是左手 / 右手。

- **笛卡尔视图**：左上角每只手一行 Tracker 位姿四元数（`w x y z`，即 SDK 广播的 `TrackerPose.rotation`）；
  左下角每只手一行位姿流 `pose  <Hz>  sample <采样时刻>  latency <到达延迟>`；
  行尾标 `(excl. pack delay)` 时，这个到达延迟不含 Tracker 内部的打包延迟，比真实的到达延迟小。
  超过 0.5 s 没有新位姿的手保持最后位姿、半透明显示，四元数后面标 `LOST <秒>`。
- **固定基座视图**：左上角关节角表（`L deg` / `R deg`，手套上报的读数；`J` 换成当前手的触觉统计与选中点读数）；
  左下角每只手一行手套流 `glove  <Hz>  sample <采样时刻>  latency <到达延迟>`。
- **时间戳**：`sample` 是 SDK 给的采样时刻（Unix 秒，微秒位）；`latency` 是 SDK 收到该帧的时刻减采样时刻，
  取最近 1 s 的中位数；拿不到采样时刻时显示 `--`。
- **只在出问题时多一行 `!!`**：Tracker 停止输出位姿（`tracker stopped: <原因>`）、基站几何检查没过
  （`startup gate: …` / `geometry: … issue(s)`）、手套数据流中断。广播端没开 Tracker 时位姿行写 `none: <原因>`。
- **按键提示**在右上角；触觉相关的按键只在显示触觉统计时列出。
- **与 SDK 断开时**：两个视图变暗，顶部横幅写明断开原因，画面保持最后一帧。

## 坐标约定

- 世界系是 libsurvive 的世界系（z 朝上），**原点与水平朝向随基站标定而定，没有物理意义**；
  两只手之间的相对位姿是真实的。笛卡尔视图的相机默认跟随两掌中点。
- 掌根位姿由 Tracker 位姿经固定装配关系换算：`R_palm = R_tracker · X · R_ptᵀ`、
  `p_palm = p_tracker − R_palm · t_pt`，其中 `R_pt / t_pt` 是模型里 Tracker link
  （`left_tracker_Link` / `right_tracker_Link`，离掌根约 78 mm）相对掌根的装配位姿，
  `X` 是 Tracker 本体系到这个 link 的固定轴向换算（见 `revohuman_sim/poses.py`）。

## 无窗口模式

远程机器或自动检查可以不开窗口，跑几秒后打印统计，并可把两个视图存成图片：

```bash
simulation/run_viewer.sh --demo --headless --seconds 3 --snapshot out.png
simulation/run_viewer.sh --headless --wait 20 --seconds 5 --dump-layout points.json   # 触觉点位置与读数
```

无窗口模式默认用 EGL 离屏渲染（`MUJOCO_GL=egl`），不需要显示器；最多等 `--wait` 秒连上 SDK 广播。
退出码：`0` 正常；`2` 启动失败、没连上广播或期间没收到手套数据；`3` 触觉开着但没收到触觉帧。

## 排查

| 现象 | 原因 / 处理 |
|---|---|
| 启动时报「找不到手部模型目录」 | 源码包根目录下的 `description/`（手部模型）被移走或删除了：重新解压源码包，或用 `--urdf-dir` 指定 |
| 启动时报「手部模型目录不完整」 | 模型目录要有 `urdf/`（恰好一份 `.urdf`）与 `meshes/` |
| 直接跑 `viewer.py` 报「缺依赖 …」 | 自己的环境里没装可视化依赖：改用 `run_viewer.sh`，或 `pip install -r simulation/requirements.txt` |
| 窗口一直「waiting for the SDK broadcast」 | 广播没起，或两边 socket 不一致：看窗口里列出的路径与广播端启动时打印的 `广播中：<path>` 是否一致；不一致就两边设同一个 `REVOHUMAN_LIVE_SOCKET` |
| `run_sim_host.sh` 打印广播日志的最后几行后退出 | 广播端没起来：按日志里的报错查本表其余各行；完整日志在 `~/.local/state/revohuman/sim_live.log` |
| `run_sim_host.sh` 提示先跑 `install_venv.sh` | 没找到 SDK：运行源码包根目录的 `install_venv.sh`，或 `--venv` 指向装了 SDK 的虚拟环境 |
| 广播端报「已经有一个广播在运行」 | 另一个 `revohuman-live` 还开着；关掉它，或给新的换一个 socket |
| `revohuman-live: command not found` | 命令入口不在 `PATH` 上：用 `pip install --user` 装的 SDK，把 `~/.local/bin` 加进 `PATH`（`export PATH="$HOME/.local/bin:$PATH"`）；装在 venv 里的，先激活 venv；SDK 不是用 pip 装的则没有这个命令。三种情况都可以改用 `python3 -m revohuman live` |
| `unsupported broadcast schema` | 可视化与 SDK 的版本不一致：SDK 与源码包换成同一版本的发布文件 |
| 笛卡尔视图显示 `pysurvive not installed` / `tracker in use by another process` | 看广播端「运行前提」里 Tracker 那一条；可视化这边不用改 |
| 两手都标 `LOST`，左下角 `!! tracker stopped: …` | SDK 端 Tracker 已停止输出位姿，不会自动恢复；广播端终端打印 `tracker 已停：…`。按广播端 Tracker 日志里的提示处理（基站被挪动要重新标定）后重开 `revohuman-live` |
| 某只手 `off (firmware reports no tactile module)` | 那只手的固件外设探测没报触觉模块：这是硬件识别结论，可视化照实显示 |

## 目录

```
simulation/
  run_viewer.sh          启动器（隔离 venv + 依赖 + 启动）
  run_sim_host.sh        一条命令拉起广播 + 可视化（主机）
  run_sim_docker.sh      一条命令拉起广播 + 可视化（容器）
  requirements.txt       可视化的全部依赖（不含 SDK）
  viewer.py              免安装入口
  revohuman_sim/
    app.py               参数与启动流程
    paths.py             固定路径与本机缓存目录
    logcolor.py          终端日志按级别上色（REVOHUMAN_LOG_COLOR）
    live_client.py       订阅 SDK 广播（重连、解帧、最新状态）
    session.py           按 SDK 的 hello 建场景；触觉启用判定
    demo.py              内置演示广播（--demo）
    hand_model.py        双手 URDF 组装与正运动学（模型取源码包根目录的 description/）
    poses.py             tracker 位姿 → 掌根位姿
    tactile_layout.py    触觉点布局与贴合
    heatmap.py           触觉热力图
    view.py              双视图渲染与交互
    assets/              触觉点布局与随包贴点表（241 点）
```
