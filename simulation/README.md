# 手套可视化（simulation/）

> **URDF is coming soon.** 可视化要用手部模型（URDF 与网格）把关节角画成手，这份模型即将提供，
> 暂不在本发布包内。**仿真与可视化需要等 URDF 就位后才能使用**：在此之前 `run_viewer.sh`（包括 `--demo`）
> 启动时提示缺少手部模型并以退出码 2 退出。URDF 就位后放在发布包根目录的 `description/`
> （`urdf/` 下一份 `.urdf` + `meshes/`），或用 `--urdf-dir <目录>` 指定。

一个 MuJoCo 窗口，左右两个视图同时看一副手套：

| 视图 | 内容 |
|---|---|
| 左：**笛卡尔视图**（CARTESIAN） | 双手按 tracker 给出的 **6D 位姿**摆在同一个 z 朝上的世界里；手指关节、触觉热力图随手一起动。每只手画掌根坐标轴与到地面的垂线，地面网格 0.1 m。 |
| 右：**固定基座视图**（FIXED BASE） | 两掌固定摆放（间距 `--spacing`，默认 0.27 m），只看手指关节与**触觉热力图**。 |

- **型号跟着 SDK 走**：P1 / PTV1 都支持。型号、手、关节表都由 SDK 在连接时告诉可视化，两种型号走同一条路。
- **触觉按外设识别决定开不开**：型号带触觉、且这只手的固件外设探测报了触觉模块、且点数与布局一致（241 点）
  才画热力图；否则这只手只画关节，覆盖层写明原因。不看数据里有没有非零值。
- **笛卡尔视图跟着 SDK 端的 tracker 走**：SDK 那边 tracker 起来了才启用；否则左侧视图显示原因，
  右侧视图照常工作。

## 与 SDK 的关系：SDK 广播，可视化只订阅

可视化**不 import SDK**，也不碰串口和 tracker。数据流只有一条本机 Unix socket：

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

## 快速开始

```bash
simulation/run_viewer.sh --demo              # 不接手套：内置演示广播（PTV1，双手 + tracker）
simulation/run_viewer.sh --demo P1           # 演示 P1（无触觉）
simulation/run_viewer.sh                     # 接真手套：另一个终端先起 revohuman-live
simulation/run_viewer.sh --help              # 全部参数
simulation/run_viewer.sh --urdf-dir <目录>   # 手部模型不在默认位置（发布包根目录的 description/）时指定
```

以上除 `--help` 外都要先有手部模型（见开头 URDF is coming soon）。

`--demo` 在可视化进程里起一个私有的演示广播，走的是与真 SDK 完全相同的接收、建场景、渲染路径。

`run_viewer.sh` 第一次运行会在 `simulation/.venv` 建一个**隔离的 Python 环境**，
按 `simulation/requirements.txt` 装好依赖（`mujoco` / `glfw` / `numpy` / `PyYAML`），
不碰系统 Python，也不需要容器或 ROS；之后 `requirements.txt` 变了才会重装。
第一次建触觉手套的场景时要把触觉布局贴到手模型上（十几秒，窗口里显示进度），
结果缓存在 `~/.cache/revohuman/`（设了 `XDG_CACHE_HOME` 时在 `$XDG_CACHE_HOME/revohuman/`），之后秒开。

| 环境变量 | 作用 |
|---|---|
| `REVOHUMAN_SIM_VENV=<dir>` | venv 位置（默认 `simulation/.venv`） |
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
  装发布物随附的 `pysurvive 1.1.21+revohuman.1`，命令见主 README 安装第 ② 步；不要装 PyPI 上的原版
  `pysurvive==1.1.21`，它少了随附版的两处修正。SDK 随包附带并在这一份上测试发布，
  换用其他版本时 tracker 的基站检查会自检失败并降级。
  基站几何要先按主 README 快速上手第 3 步标定好（详见 [《RevoHuman SDK 参考手册》](../docs/RevoHuman%20SDK%20参考手册.md) 第 4 节）；
  广播端只读位姿：不做标定、不采纳基站几何，也不记零位（笛卡尔视图画的是绝对位姿，用不到零位）。
  没装 pysurvive、或 tracker 被别的进程占着时，广播端只广播手套（不抢占），笛卡尔视图显示原因。
- **基站检查档位**（广播端 `--tracker-gate`）：默认 `flag`，基站检查没过也照常广播位姿，
  笛卡尔视图底部标出哪一项没过（如 `startup gate: S0 whitelist_missing`、`geometry: 2 issue(s)`），
  具体原因看广播端终端里的 tracker 日志（这时两手的相对位姿不一定可信）；
  `error` 则检查没过就不广播位姿。
- **单进程独占**：手套串口与 tracker 同时只能被一个进程打开。录制程序开着时广播端会提示被占用。
  同一个 socket 上只能有一个广播；**订阅方可以有多个**（多开几个可视化窗口都行）。
- **显卡**：需要 OpenGL。没有独显的机器加 `--lite`（关阴影、反射）。

## 操作

| 按键 | 作用 |
|---|---|
| `V` | 布局切换：双视图 → 只看笛卡尔 → 只看固定基座 |
| `F` | 笛卡尔视图相机跟随两掌中点（开 / 关；拖动平移时自动关） |
| `C` | 两个视图的相机复位，重新打开跟随 |
| `Tab` | 切换当前手（双手时） |
| `J` | 固定基座视图的表格：触觉统计 ↔ 关节角 |
| `←` / `→` | 选中上一个 / 下一个触觉点 |
| `PgUp` / `PgDn` | 跳到上一根 / 下一根手指 |
| `↑` / `↓` | 色标量程放大 / 缩小（改为固定量程） |
| `Home` | 以当前读数作为触觉基线（之后显示减去基线的值） |
| `Delete` | 清除基线 |
| `Insert` | 把当前手的触觉点表打印到终端 |
| 鼠标 | 左键拖动旋转、右键拖动平移、滚轮缩放（作用在鼠标所在的视图） |

触觉相关的按键只在有手开着触觉时显示。

## 覆盖层

- **笛卡尔视图**：每只手的状态（`live` / `LOST` / 无位姿原因）、位姿年龄、掌根位置（米）与
  姿态（roll / pitch / yaw，度）；底部是 tracker 序列号、基站几何检查结果、跟随状态。
  SDK 端 tracker 停止输出位姿（基站被挪动、tracker 全部掉线等）时，检查结果换成 `tracker stopped: <原因>`。
  超过 0.5 s 没有新位姿的手保持最后位姿、半透明显示并标 `LOST`。
- **固定基座视图**：各手指触觉统计（或关节角表）、选中点的读数、每只手的帧率 / 年龄 / 无效通道、
  触觉启用原因、色标量程。
- **与 SDK 断开时**：两个视图变暗，顶部横幅写明断开原因，画面保持最后一帧。

## 坐标约定

- 世界系是 libsurvive 的世界系（z 朝上），**原点与水平朝向随基站标定而定，没有物理意义**；
  两只手之间的相对位姿是真实的。笛卡尔视图的相机默认跟随两掌中点。
- 掌根位姿由 tracker 位姿经固定装配关系换算：`R_palm = R_tracker · X · R_ptᵀ`、
  `p_palm = p_tracker − R_palm · t_pt`，其中 `R_pt / t_pt` 是模型里 tracker 相对掌根的装配位姿，
  `X` 是 tracker 本体系到模型 tracker link 的固定轴向换算（见 `revohuman_sim/poses.py`）。

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
| 启动时报「手部模型（URDF）尚未随本发布包提供（URDF is coming soon）」 | 手部模型即将提供；就位后放到发布包根目录的 `description/`，或用 `--urdf-dir` 指定 |
| 启动时报「手部模型目录不完整」 | 模型目录要有 `urdf/`（恰好一份 `.urdf`）与 `meshes/` |
| 直接跑 `viewer.py` 报「缺依赖 …」 | 自己的环境里没装可视化依赖：改用 `run_viewer.sh`，或 `pip install -r simulation/requirements.txt` |
| 窗口一直「waiting for the SDK broadcast」 | 广播没起，或两边 socket 不一致：看窗口里列出的路径与广播端启动时打印的 `广播中：<path>` 是否一致；不一致就两边设同一个 `REVOHUMAN_LIVE_SOCKET` |
| 广播端报「已经有一个广播在运行」 | 另一个 `revohuman-live` 还开着；关掉它，或给新的换一个 socket |
| `revohuman-live: command not found` | 命令入口不在 `PATH` 上：用 `pip install --user` 装的 SDK，把 `~/.local/bin` 加进 `PATH`（`export PATH="$HOME/.local/bin:$PATH"`）；装在 venv 里的，先激活 venv；SDK 不是用 pip 装的则没有这个命令。三种情况都可以改用 `python3 -m revohuman live` |
| `unsupported broadcast schema` | 可视化与 SDK 版本不配套，两边换成同一个发布包 |
| 笛卡尔视图显示 `pysurvive not installed` / `tracker in use by another process` | 看广播端「运行前提」里 tracker 那一条；可视化这边不用改 |
| 两手都标 `LOST`，底部 `tracker stopped: …` | SDK 端 tracker 已停止输出位姿，不会自动恢复；广播端终端打印 `tracker 已停：…`。按广播端 tracker 日志里的提示处理（基站被挪动要重新标定）后重开 `revohuman-live` |
| 某只手 `off (firmware reports no tactile module)` | 那只手的固件外设探测没报触觉模块：这是硬件识别结论，可视化照实显示 |

## 目录

```
simulation/
  run_viewer.sh          启动器（隔离 venv + 依赖 + 启动）
  requirements.txt       可视化的全部依赖（不含 SDK）
  viewer.py              免安装入口
  revohuman_sim/
    app.py               参数与启动流程
    paths.py             固定路径与本机缓存目录
    logcolor.py          终端日志按级别上色（REVOHUMAN_LOG_COLOR）
    live_client.py       订阅 SDK 广播（重连、解帧、最新状态）
    session.py           按 SDK 的 hello 建场景；触觉启用判定
    demo.py              内置演示广播（--demo）
    hand_model.py        双手 URDF 组装与正运动学（模型取发布包根目录的 description/，URDF 即将提供）
    poses.py             tracker 位姿 → 掌根位姿
    tactile_layout.py    触觉点布局与贴合
    heatmap.py           触觉热力图
    view.py              双视图渲染与交互
    assets/              触觉点布局（241 点）
```
