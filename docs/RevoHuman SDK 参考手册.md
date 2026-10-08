# RevoHuman SDK 参考手册

> 安装与上手以源码包根目录的 [README](../README.md) 为准；本手册是完整参考。
> 文中的路径（`examples/`、`ros2/`、`simulation/` 等）均相对源码包根目录 `revohuman-<版本>/`；
> SDK 代码注释与报错里提到的「README 第 N 节」，指本手册的同号章节。

RevoHuman 数据手套的采集 SDK。支持两种产品型号：

| 型号 | 组成 |
|---|---|
| **P1** | 编码器手套（每手 21 关节）+ 双手 Vive Tracker |
| **PTV1** | P1 + 全指触觉（241 点）+ 腕部相机 |

SDK 从 USB 序列号与在位硬件自动识别左右手和型号，再按型号装配出对应的传感器组合，
对外只有一套接口。提供两个入口，可以只用其中一个：

| 入口 | 用法 | 依赖 |
|---|---|---|
| 原生 Python | `from revohuman import open_kit` | `pyserial` + `PyYAML`（其余按需装 extra） |
| ROS 2 | `ros2 launch revohuman_bringup revohuman.launch.py` | 另需 ROS 2 Humble |

`import revohuman` 不会加载 numpy、rclpy、opencv 或 libsurvive，
tracker、相机、录制等子模块在用到时才导入。

---

## 1. 安装

核心 SDK 与 Tracker 驱动按平台出 wheel，其余内容是一个与平台无关的源码包。

| 文件 | 内容 |
|---|---|
| `revohuman-<版本>-cp310-cp310-linux_x86_64.whl` | 核心 SDK，x86_64 |
| `revohuman-<版本>-cp310-cp310-linux_aarch64.whl` | 核心 SDK，aarch64（即 Linux 上的 arm64） |
| `pysurvive-1.1.21+revohuman.1-cp310-cp310-linux_x86_64.whl` | Tracker 驱动 libsurvive，x86_64 |
| `pysurvive-1.1.21+revohuman.1-cp310-cp310-linux_aarch64.whl` | Tracker 驱动 libsurvive，aarch64 |
| `revohuman-<版本>-bundle.tar.gz` | ROS 2 包、示例、文档、仿真、Docker 镜像构建文件 |
| `SHA256SUMS` | 校验和 |

运行环境：Linux，CPython 3.10（Ubuntu 22.04 自带版本）。核心 SDK 以编译模块提供，
wheel 只能装在文件名标明的 Python 版本与架构上，装错会被 pip 拒绝。

```bash
sha256sum -c SHA256SUMS --ignore-missing
WHL=revohuman-0.1.0-cp310-cp310-linux_$(uname -m).whl
pip install "$WHL"                   # 核心：手套采集、识别、命令行
pip install "$WHL[recording]"        # MCAP 录制与回放（mcap、lz4）
pip install "$WHL[camera]"           # 腕部相机图像解码（opencv；采集与打戳本身不需要）
pip install "$WHL[ble]"              # 基站扫描 / 配对 / 改信道（bleak；使用 tracker 时必需）
pip install "$WHL[viewer]"           # MuJoCo 三维可视化工具
tar xzf revohuman-0.1.0-bundle.tar.gz   # → revohuman-0.1.0/：ros2/ examples/ docs/ simulation/ docker/ README.md
```

> **URDF is coming soon.** 仿真与可视化（`simulation/`）要用的手部模型（URDF）即将提供，暂不在本发布包内；
> 仿真功能需要等 URDF 就位后再使用（放在源码包根目录的 `description/`，见 `docs/05-仿真与可视化.md`）。

- **Tracker** 另需随发布物附带的 libsurvive Python 绑定 pysurvive 1.1.21+revohuman.1：`python3 -m pip install --user --no-deps pysurvive-1.1.21+revohuman.1-cp310-cp310-linux_$(uname -m).whl`（随附版本含本 SDK 所需的驱动修正，不要装 PyPI 上的版本；`--no-deps` 不能省，见 [README](../README.md) 安装第 ② 步）。
- **串口权限**：当前用户要能读写 `/dev/ttyACM*`（一般加入 `dialout` 组）。
- **单进程独占**：一只手套、一套 libsurvive 同时只能被一个进程打开。
  第二个进程会拿到 `PortBusyError`（附持有者 pid）或 `TrackerBusyError`。

装好后先做一次体检：

```bash
python3 -m revohuman doctor            # 只读体检：USB 拓扑、串口、注册表、tracker、相机
python3 -m revohuman doctor --probe    # 另外短开串口读固件信息（正在采集的手套不要加）
```

---

## 2. 硬件识别

### 2.1 左右手

手套 MCU 的 USB 序列号标明手别（L = 左手，R = 右手）：

| 序列号 | 型号 | 手别 |
|---|---|---|
| `BCRH1PL…` / `BCRH1PR…` | P1 | 第 7 个字符 |
| `BCRH1TL…` / `BCRH1TR…` | PTV1 | 第 7 个字符 |
| `BCRH1L…` / `BCRH1R…`（早期批次） | 不标明 | 第 6 个字符 |

SDK 按下面的顺序认手，全程只读：

1. **序列号**：全机唯一且带 L/R 时直接采信；
2. **注册表**：序列号认不出时（例如早期批次两只手套序列号相同），用人工声明过的手套组，见 2.3；
3. 都没有：拒绝连接并提示先声明一次，**不按枚举顺序猜**。

注册表与序列号同时存在时互为校验：结论不一致、声明的 tracker 与实插不符、或声明了 `camera_port` 而同 HUB 相机的口位不符，都拒绝连接（相机不在位时不核对口位）。

每只手的 Tracker 序列号：优先取同一 USB HUB 上实际插着的 tracker，其次取注册表里的 `tracker` 字段。

### 2.2 型号（P1 / PTV1）

每只手收集三条证据，结论必须一致：

| 证据 | 判为 PTV1 | 判为 P1 |
|---|---|---|
| 序列号（`BCRH1T…` / `BCRH1P…`，或 `products.yaml` 规则） | 指向 PTV1 | 指向 P1 |
| 固件上报的触觉模块 | 有触觉 | 无触觉 |
| 同一 USB HUB 上的腕部相机 | 在位 | 不在位 |

- 证据互相矛盾、或左右手结论不同：抛 `ProductUnidentified`，列出每条冲突。
- 显式声明了型号（`product="PTV1"`）而证据与之相反：抛 `ProductMismatch`。
- 没有任何可用证据时才采用声明值（`source="declared"`）。

SDK 内置两条序列号规则：`BCRH1P[LR]` → P1、`BCRH1T[LR]` → PTV1；早期批次的序列号不标明型号，
只按硬件证据判定。需要补充规则时写在 `~/.config/revohuman/products.yaml`（或用环境变量
`REVOHUMAN_PRODUCTS` 指定），正则从序列号开头匹配，排在内置规则之前：

```yaml
rules:
  - match: "BCRH2"     # 正则，re.match
    variant: PTV1
    note: 可选说明
```

查看识别结果：

```bash
python3 -m revohuman product identify            # 自动识别（会短开串口读触觉型号）
python3 -m revohuman product identify --no-probe # 只看 USB 拓扑与序列号
python3 -m revohuman product identify --product PTV1 --json
```

### 2.3 手套组注册表（可选）

序列号能认出手别时不需要注册表。以下情况才要写：序列号相同的早期批次手套、tracker 不插在手套 HUB 上（无线接收器），
或一台机器上接了多组手套。

```yaml
# ~/.config/revohuman/glove_sets.yaml（或 $REVOHUMAN_REGISTRY）
sets:
  bench:                    # 组名
    left:
      hub: "3-2"            # 手套单元的 USB 拓扑（usb_topology_id），不是 ttyACM 编号
      tracker: "LHR-XXXXXXXX"   # 可选：tracker 不在手套 HUB 上时填写
      camera_port: 2        # 可选校验位：腕部相机在本单元 HUB 的第几口
    right:
      hub: "3-3"
```

`hub` 的取值来自扫描：

```python
from revohuman import GloveManager
for d in GloveManager().scan():
    print(d.usb_topology_id, d.port, d.usb_serial, d.handedness, d.hand_source, d.tracker_serial)
```

也可以用代码声明（单元必须在位）：`GloveManager().register("3-2", Handedness.LEFT, set_name="bench")`。
同一只手在多组里都有声明时，连接必须显式给 `set_name`，否则抛 `ValueError` 并列出组名。
完整样例见 [`examples/glove_sets.example.yaml`](../examples/glove_sets.example.yaml)。

### 2.4 固件升级

只经命令行、一次升一只。开始写入前先校验固件文件和设备状态，并要求输入 `yes` 确认：

```bash
python3 -m revohuman firmware info                                      # 只读：在位手套的固件版本与升级状态
python3 -m revohuman firmware upgrade app.bin --version 1.3.0 --hand left
python3 -m revohuman firmware upgrade app.bin --version 1.3.0 --port /dev/ttyACM0 --yes   # 非交互环境必须带 --yes
```

- 只接受原始 `.bin`。`.bin` 里没有可靠的版本信息，`--version` 必须照固件发布方给的写；写错会在最后核对版本时报失败。
- 串口被占用（多半在采集）时拒绝，先停采集；降级默认拒绝，确要降级加 `--allow-downgrade`；
  该选项只放开 SDK 这一侧的检查，手套固件本身仍可能拒绝降级（报「设备不接受这份固件」）。
- `--version` 与手套当前版本相同时照常重刷（传输、校验、激活都走一遍），手套固件本身仍可能拒绝。
  版本号相同，最后核对版本就分不出新旧固件：结果只说明设备已校验并激活，JSON 里带 `"same_version": true`，
  `"rebooted"` 表示是否观察到手套重启（掉线后重新出现，或串口编号变化）。
- 传输中途失败或按 Ctrl+C，本次传输会被取消，手套继续运行旧固件。
- 激活后手套自动重启，命令按 USB 物理位置找回同一只手套并核对版本（重启后 ttyACM 编号可能变化），
  默认等 45 秒（`--reconnect-timeout`）。**命令报告完成前不要拔线、不要断电**；超时未确认时用 `firmware info` 查看。

### 2.5 编码器零位标定

手套出厂时已标定编码器零位，日常使用不需要重标。标定把手套**当前姿态**记为各路编码器的新零位并写入手套，
**替换当前零位（包括出厂零位），无法撤销**。与 Tracker 的零位（4.3）无关。

```bash
python3 -m revohuman glove calibrate --hand left                          # 全部在线的编码器；要输入 yes 确认
python3 -m revohuman glove calibrate --hand right --index 3 --index 4     # 只标第 3、4 路（编号 0–20）
python3 -m revohuman glove calibrate --port /dev/ttyACM0 --yes            # 非交互环境必须带 --yes
```

- 零位姿态是五指伸直、手掌平放；标好后在这个姿态下 `joints_deg` 各路约为 180°。
- 先把手套摆在零位姿态并保持不动，再执行；标定完成前不要动。一次标一只。
- 串口被占用（多半在采集）时拒绝，先停采集；目标手套不在位时拒绝，不按插口顺序猜。
- 成功时打印各路编码器的新零位原始值（`--json` 里是 `zero_raw`），退出码 `0`。
- 有编码器没标成功、或没有确认新零位已保存到手套时退出码为 `1`，并列出成功与失败的编码器；
  不要把部分成功当作完成，摆好姿态重标一次。
- 同样的操作也可以从 Python（`glove.calibrate()`，见 5.1）或 ROS 2（`calibrate` 服务，默认不注册，见 7.4）发起。

---

## 3. 快速上手

```python
import queue
from revohuman import open_kit

with open_kit() as kit:                        # 识别型号并装配全部传感器
    print(kit)                                 # RevoHumanKit(PTV1, hands=['left', 'right'], sensors=[...])

    # 零位已在启动时自动记下（启动前双手摆在零位，见 4.3）；要重记调 kit.set_reference()
    sub = kit.synchronizer().subscribe()       # 双手配对帧
    pair = sub.recv(timeout=1.0)
    print(pair.skew_ns, pair.left.joints_deg[:3])

    delta = kit.tracker.delta("right")         # 右手 tracker 相对零位的位姿
    if kit.has("wrist_camera"):
        frame = kit.camera("right").latest()   # PTV1：最新一帧相机图像

    rec = kit.recorder()                       # 已接好本套全部数据流
    handle = rec.start("episode_001.mcap")
    ...
    print(handle.stop())                       # {"frames": ..., "per_channel": {...}}
    rec.close()
```

可直接运行的版本：[`examples/kit_quickstart.py`](../examples/kit_quickstart.py)。
使用 Tracker 前，基站要先写进白名单并标定一次几何，见第 4 节。

### `open_kit()` 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `product` | `"auto"` | `"auto"` 按证据识别；`"P1"` / `"PTV1"` 为声明，与硬件相反即拒绝 |
| `hands` | `"both"` | `"both"`（双手硬件同步）/ `"left"` / `"right"` |
| `sensors` | 型号全部 | 只启用其中几类，如 `{"encoder", "tactile"}`；必须含 `encoder`，不能超出型号 |
| `strict` | `True` | 型号应有但在位硬件缺失的传感器：`True` 抛 `ProductMismatch`；`False` 跳过并记入 `kit.missing` |
| `set_name` | `None` | 注册表组名（2.3） |
| `rate_hz` | `200` | 手套数据速率 |
| `sync` | `True` | 双手硬件同步：`True`（失败即抛）/ `"best-effort"` / `False` |
| `tracker` | `None` | 传给 `TrackerRuntime` 的参数，如 `{"gate_mode": "flag"}` |
| `camera` | `None` | 传给 `CameraConfig` 的参数（分辨率、帧率、曝光等）；设备路径由 kit 按手套单元填写 |
| `rules_path` / `registry_path` | 默认路径 | `products.yaml` / 注册表路径 |
| `on_fault` | `None` | 运行期故障回调 `cb(exc)`（4.4） |

传感器名：`encoder`、`tracker`、`tactile`、`wrist_camera`。

### `RevoHumanKit`

| 成员 | 说明 |
|---|---|
| `product` | 识别结果：`variant`、`source`（`sn` / `hardware` / `declared`）、每只手的在位硬件 |
| `variant` / `sensors` / `hands` / `missing` | 型号 / 本次启用的传感器 / 打开的手 / 非严格模式下缺失的传感器 |
| `has(sensor)` | 是否启用了该传感器 |
| `glove(hand)` / `camera(hand)` | 单只手套（`Glove`）/ 腕部相机（`WristCamera`） |
| `synchronizer()` | 双手配对器（`BimanualSynchronizer`），重复调用返回同一个 |
| `tracker` | `TrackerRuntime`（4.6），未启用 tracker 时为 `None` |
| `set_reference()` | 重记零位（启动时已自动记过），等同 `kit.tracker.set_reference()` |
| `geometry()` | 基站几何门判定；`ok` 为假时不要采集 |
| `calibrate_geometry(timeout_s=None)` | 在子进程中标定基站几何，结束后自动重开运行会话 |
| `recorder(...)` | 返回接好全部数据流的 `TopicRecorder`（5.5） |
| `status()` | 各部件状态字典，含 `fatal` |
| `fatal` / `raise_if_fatal()` | 锁存的不可恢复故障（4.4），没有时为 `None` / 不抛 |
| `close()` | 关闭全部部件；支持 `with` 语句 |

装配顺序为 Tracker 静态检查（4.2 的 S0，只读文件；被拦时手套不会被打开）→ 手套 → 型号识别 → Tracker → 相机，任一步失败会关掉已打开的部分再抛出异常。

---

## 4. Tracker 与基站

Tracker 位姿以基站为参照。一套场地要先准备：把自家基站填进白名单 → 扫描确认、排除同信道冲突 → 标定基站几何。
几何会落盘，之后开机自动沿用；基站被移动、更换、增减或改过信道后才需要重标。零位每次启动时自动记录（见 4.3）。
标定同时确定了 tracker 位姿所在的坐标系，定义见 4.7。

**需要蓝牙。** 基站的序列号和信道只能通过蓝牙读到。扫描、改信道、闪灯都要求本机有可用的蓝牙适配器，并安装 `ble` extra。
蓝牙只在 `revohuman-lighthouses` 命令里用，SDK 的启动与运行不碰蓝牙。

### 4.1 部署流程

```bash
# ① 扫描附近基站：序列号 / 广播名 / 信道 / 信号（第一次见到的基站会连接一次，只读序列号）
revohuman-lighthouses scan

# ② 按示例写出配置文件（已存在则不动，会打印路径），再对照基站背面标签，
#    把占位符换成自家基站 FA 开头的 12 位序列号
revohuman-lighthouses init
${EDITOR:-nano} <上一步打印的路径>

# ③ 再扫一次：白名单里每台都要打 ✓，且没有同信道冲突（通过时顺带刷新信道预检，5 分钟内有效）
revohuman-lighthouses scan

# ③′ 只在 ③ 报同信道冲突时：看自动方案（只算不写）→ 确认后执行 → 回到 ③
revohuman-lighthouses plan
revohuman-lighthouses apply --yes

# ④ 标定基站几何（首次、基站动过、或改过信道之后）
python3 -m revohuman tracker calibrate

# ⑤ 启动前自查（可选）：默认只读文件；--live 会真起一次采集，把 4.2 的两段检查都跑一遍
python3 -m revohuman tracker check --live

# 随时离线查看：白名单、预检、几何、零位、上次启动检查的结论
python3 -m revohuman tracker status
```

`revohuman-lighthouses` 与 `python3 -m revohuman tracker lighthouses` 是同一个命令。

**配置文件**：基站白名单与标定门限写在同一份 YAML 里，人手写，SDK 只读。按下面的顺序只取第一份，
不跨文件合并；每条命令启动时第一行打印本次用的是哪一份、来自哪一层：

1. `--whitelist <路径>`
2. 环境变量 `$REVOHUMAN_LIGHTHOUSES`
3. 源码树根目录的 `revohuman_tracker.yaml`（从源码运行时才有这一层；pip 安装后没有，启动时会说明）
4. `~/.config/revohuman/lighthouses.yaml`

存在却没被读到的文件（比如设了环境变量后源码树里那份，或有了新配置后 4 那份旧文件）启动时会点名。
`init` 写到之后会被读的那一份（从源码运行、两处都还没有文件时写 3），已存在则不覆盖。
完整示例见 `examples/revohuman_tracker.example.yaml`：

```yaml
lighthouses:
  - FAXXXXXXXXXX          # 占位符：换成第一台基站背面标签上的序列号
  - serial: FAYYYYYYYYYY  # 第二台
    place: 右后上方        # 可选，报错时用来指明是哪一台

calibration:              # 可选：标定的视角覆盖门限，命令行参数优先
  coverage_cells: 2       # 至少覆盖几个方向格（0 关闭这条判据）
  coverage_span_deg: 5    # 基站看到的张角至少几度（0–120）
```

- 填了真实序列号的文件不要提交进 git（源码树根目录那份已在 `.gitignore` 里）。
- 缺文件或白名单为空时，启动检查直接拒绝（`whitelist_missing`），并提示先 `init`。
- `calibration:` 段只认 `coverage_cells` 与 `coverage_span_deg`；写错键名、写成非数字或张角超过 120° 都会拒绝并指出行号。
  这一段不写时用代码默认（2 格 / 5°），示例里写的就是默认值；加基站后按需要调高。
  一个位置就会同时占 1 个水平方向格和 1 个竖直方向格，所以格子数不超过 2 时这条判据一开始就满足，
  实际起作用的只剩张角。张角按所有 tracker 合起来算，两只手左右分开放时，一开始就可能够。

- 不要填 `LHB-xxxxxxxx` 广播名，也不要填信道号；填错时会指出是哪一行。最多 4 台，不许重复。
- 序列号与广播名的对应由 `scan` 读出，记在状态目录的身份表里；每台基站在每台机器上只需读一次。
  不想连接邻居的基站时用 `scan --no-connect`：只被动扫描，没读过序列号的站显示为 `?`。

**同信道冲突**：同一信道上的两台基站分不开，会反复丢失基站位姿。`plan` / `apply` 只挪白名单里的站：

- 本轮扫到的其它基站（含序列号没读出的），以及最近一次启动时光学上看到过的基站，它们占用的信道都会避开；
  光学记录不按时间过期（输出里会注明是多久前见到的），只有当它与当前白名单毫无交集（像是换了一整套基站）时
  才不采信，并会打印一行说明；
- 确认已经换了场地、上次光学上见过的基站已不在时，加 `--ignore-optical` 只按本轮蓝牙扫描判；
  蓝牙本轮扫到的冲突不受这个开关影响，照样拒绝；
- 能不动就不动，只挪撞车的；`--pin FA…=CH` 指定某台去某信道，`--avoid CH` 额外避开某些信道；
- 默认只打印方案，`apply --yes` 才写；`apply` 会自己重扫、重算，不沿用之前 `plan` 的结果；
- 写之前在同一次连接里核对序列号与当前信道，写后读回，全部写完再重扫复验；
- 挪动成功后信道预检作废：要重新 `scan`，并重新 `calibrate`。

其它操作：

- `set-channel FA… CH [--yes] [--ignore-optical]`：只改指定的一台，目标信道必须空闲；被拒时会说明占着它的是哪台、
  是本轮蓝牙扫到的还是多久前光学上见过的。
- `identify FA…|LHB-… [--seconds 5]`：让一台基站闪灯几秒，方便到现场认出是哪一台。
- 不在白名单里的基站：信道永远不改；闪灯每次都要加 `--foreign`，并记入审计日志。
- 集中部署可设 `REVOHUMAN_LIGHTHOUSE_WRITE=0` 关掉全部写操作（改信道与闪灯），此时只出方案。
- `python3 -m revohuman tracker preflight` 是只做信道预检的快速版：只被动扫描，不连接任何基站。

`revohuman-lighthouses` 退出码：`0` 白名单完整、无冲突，或操作成功；`1` 有缺口、有冲突、方案待执行或写入失败；
`2` 蓝牙不可用、另一个蓝牙操作正在进行，或用法错。

**标定操作**：把 tracker 放在所有基站都能看到的区域，**换一个位置和朝向 → 停住约 1 秒 → 再换**，
重复到命令报告完成。匀速晃动或一直静止都采不到有效样本。

**标定完成的判据**（持续满足 `--stable-sec` 秒）：

- 白名单里每台基站都已收到数据、已解出位姿、位姿非零、方差在限值内（允许一台作为参考站方差为零）；
- 视角覆盖达标：至少 `--coverage-cells` 个方向格、张角至少 `--coverage-span-deg`；两者依次取命令行参数、
  配置文件 `calibration:` 段、代码默认（2 格 / 5°），标定开始时打印本次取值与来源；
- 多站时，各基站解出的重力方向两两夹角不超过 10°（超过是坏解，判失败）。

通过后写入「已接受几何」记录，启动检查以它为准。超时（默认 300 秒）、失败或中止时，自动回滚到标定前的几何配置。

**外来基站围栏**：默认只使用白名单里的基站，屏蔽其它基站。`--no-fence` 关闭围栏，
`--no-preflight` 同时跳过信道预检并关闭围栏，仅用于排查问题。

### 4.2 启动检查

`open_kit()`（启用 tracker 时）与 `TrackerRuntime.start()` 在交出句柄之前分两段检查，不满足就抛
`TrackerNotReady`（码 1905）。异常带 `kind`、`reasons`（逐条列出，基站按序列号称呼）和 `remedy`
（按 `kind` 给出、可直接复制执行的修复命令），抛出前会先写一条 ERROR 日志。

**第一段：静态检查。** 只读文件，毫秒级。`open_kit()` 在连接手套之前就做，被拦时手套一次都不启动。

| `kind` | 含义 | 修复 |
|---|---|---|
| `whitelist_missing` | 白名单不存在或为空 | `scan` → 填白名单 → `calibrate` |
| `whitelist_invalid` | 白名单格式错、重复或超过台数 | 按提示改那一行 |
| `lighthouse_unresolved` | 白名单里的某台本机从没扫到过 | 那台通电、人在附近时跑 `scan` |
| `config_missing` | 还没标定过基站几何（首次运行落在这里） | `scan` → `calibrate` |
| `config_unreadable` | 几何文件读不了或已损坏 | 检查权限；损坏就重标 |
| `geometry_unsolved` | 白名单里有基站没有解出位姿 | `calibrate` |
| `geometry_unaccepted` | 当前几何不是 `calibrate` 验收过的那份（标定中断、被手改或拷贝过） | `calibrate` |
| `floor_offset` | 地面偏移超过 1 m | `calibrate` |
| `geometry_inconsistent` | 多站时基站重力方向夹角超过 10°（坏解） | `calibrate` |

- 某台基站当前信道与标定时不同（`channel_changed`）只告警、不拦，提示重新 `scan` 并 `calibrate`。
- 已有几何但没有「已接受几何」记录时，若其余检查全部通过，首次启动自动采纳一次并告警。

**第二段：起流后检查。** 采集起来之后、交出句柄之前：

| `kind` | 含义 |
|---|---|
| `geometry_unavailable` | `ootx_wait_s`（默认 25 秒）内有白名单基站没有数据（没通电、被挡住），或读不到基站状态 |
| `lighthouse_wiped` / `lighthouse_moved` | 基站位姿在启动时被清除：驱动在基站广播的重力方向变化达到 2° 时清掉该基站的位置并改写基站配置。可能是基站被碰歪或挪动过，也可能只是读数抖动（没人碰也会出现），两者的角度有重叠、分不开。随包驱动下一般报 `lighthouse_moved`，不代表一定有人碰过；两者处置相同：重启无效，重新 `scan` 并 `calibrate` |
| `tracker_no_pose` | 某只 tracker 5 秒内没有出位姿，附带它看不到哪台基站 |
| `reference_failed` | 自动零位没有记上 |

每次启动都在日志里打一行 `[tracker-gate] mode=... registered=... config=...`，结论落盘到状态目录的 `last_gate.json`，
`tracker status` 会显示「上次启动检查」的结果。

`gate_mode`：`error`（默认，拦）/ `flag`（只写日志与 `status()`，照常返回）/ `off`（不检查）。

修复命令不受这些检查约束：`tracker calibrate` 不做启动检查，否则基站动过之后就再也标不回来。

### 4.3 零位

零位是 tracker 位姿的参照：`delta` 表示当前位姿相对零位时刻的变化（定义见 4.6，坐标系见 4.7）。

- 通过启动检查后自动按当时的位姿记一次零位，所以**启动前把双手摆在零位**。
- 需要时调 `kit.set_reference()` / `rt.set_reference()` 或 `tracker set-reference` 重记。录制中重记会作废当段。
- 默认不沿用上次落盘的零位：基站被整体挪动后，旧零位会让数据整体偏掉，且无法从数据中察觉。
  确需沿用时传 `restore=True`（仍要求基站几何指纹与 tracker 序列号一致、且不超过 8 小时）；`auto_reference=False` 关闭自动记录。
- 基站几何异常时拒绝零位标定（`CalibrationRejected`，`stage="geometry"`）；`tracker set-reference` 被启动检查拦下时退出码为 3。

### 4.4 运行期保护

句柄交出后，SDK 每秒自查一次（`guard_interval_s`）：

| 编号 | 情况 | 级别 | 异常 |
|---|---|---|---|
| R1 | 白名单基站的位姿被清除（该站广播的重力方向变化达到 2°：被碰歪，或读数抖动，见 4.2） | 不可恢复 | `GeometryLost`（1906） |
| R1 | 任一白名单基站的完整位姿与标定验收记录相差超过 1 mm 或 0.1°（逐台比，单站同样适用；当拍即停，不等 3 秒；`gate_mode="flag"` 下同样停止） | 不可恢复 | `GeometryLost`（1906，`kind="changed"`） |
| R1 | 可用基站不够（遮挡），或基站间距相对标定变化超过 ±10 mm，持续 3 秒 | 可恢复 | `GeometryLost`（1906） |
| R2 | 基站在采集中被移动（检测范围见本节末） | 不可恢复 | `WorldFrameChanged`（1902） |
| R3 | 位姿帧间跳变 | 只计数并告警 | — |
| R4 | 全部 tracker 停流 10 秒 | 不可恢复 | `TrackerStreamDead`（1901） |
| R4' | 单只 tracker 断流 | 只标记（遮挡是常态） | — |

**可恢复**：作废当段录制；条件消失前 `start()` 抛该异常，恢复后可以继续录，不用重开进程。

**不可恢复**：整套中断。

1. tracker 采集停止，订阅上的 `recv` / `async for` 抛出该异常；
2. 录制器作废当段 episode，之后 `start()` 一律抛该异常（`kit.recorder()` 已自动接上；自己建的录制器调 `rec.guard_tracker(kit.tracker)`）；
3. kit 锁存故障：`kit.fatal`、`kit.status()["fatal"]`、`kit.raise_if_fatal()`；此后 `set_reference()` / `geometry()` / `recorder()` 都抛该异常；
4. 手套与相机的硬件流不主动停，等 `close()`；**只能重开进程**。

两级故障都会回调 `open_kit(on_fault=cb)` 的 `cb(exc)`。`gate_mode="flag"` 下只记录、不中断，`"off"` 不检查。
例外是「基站位姿与验收记录不符」：它一旦成立，之后的每一帧都在错误的坐标系里，所以 `"flag"` 下同样停止采集、拒绝零位标定；只有 `"off"` 不检查。

**几何门**也可以主动查询：`kit.geometry()` / `TrackerRuntime.geometry()` 返回含 `ok`、`errors`、`flags`、`pending` 的字典，
判据 `wipe:<id>`（该基站位姿被清除）、`changed:<id>`（该基站位姿与验收记录相差超过 1 mm / 0.1°）、
`unavailable`（读不到几何）、`drift`（间距变化超过 ±10 mm，持续 3 秒）。
`accepted` 字段给出每台基站与验收记录的现测差值（`stations[<id>] = {dpos_mm, drot_deg}`）；没有验收记录时为 `None`，不做这项比对。

**世界系门**（`SurviveTrackerSource` 的 `world_gate_mode`，默认 `error`）：检测到基站在采集中被移动时停止采集，
`stats()["world"]` 记录这次事件。`stats()["world"]["pose_watch"]` 不为 `"ok"` 表示检测器没有在工作，
此时"没有报警"不代表基站没动；为 `"ok"` 时也只在基站位姿被重新解算时才看得到变化，见下条。

**检测不到的挪动**：运行时 SDK 不重新解算基站位置，靠基站自己广播的重力方向判断它有没有被动过（变化达到 2° 时驱动清除该站位姿，触发 R1）。
因此基站被平移、或绕竖直轴转动（重力方向不变）时，R1、R2 都不会触发，单台、多台基站都是如此；不到 2° 的倾斜同样检测不到。
基站动过之后必须重新标定，不要等报错。

### 4.5 状态文件

状态目录：`$REVOHUMAN_TRACKER_STATE_DIR` > `$XDG_STATE_HOME/revohuman/tracker` > `~/.local/state/revohuman/tracker`。

| 文件 | 内容 |
|---|---|
| `lighthouse_identities.json` | 基站身份表：序列号 ↔ 广播名、最近一次扫到的信道（`scan` 写，不要手改） |
| `lighthouse_channel_preflight.json` | 最近一次信道预检 |
| `geometry_accepted.json` | 验收过的基站几何，也是运行期间距漂移与逐台位姿比对的基线 |
| `last_gate.json` | 最近一次启动检查的结论 |
| `lighthouse_write_log.jsonl` | 改信道与闪灯的审计记录 |
| `calibration.json` | 零位 |
| `coverage.json` | 最近一次标定的视角覆盖 |
| `survive_mode` | 当前模式（运行 / 标定） |
| `last_exit.json` / `alive.json` | 进程退出原因 / 心跳 |

基站几何由 libsurvive 保存在 `~/.config/libsurvive/config.json`，重新标定请用 `tracker calibrate`，不要手动删除或拷贝该文件。

### 4.6 Python 接口：`TrackerRuntime`

`open_kit()` 已自动创建；单独使用时：

```python
from revohuman.tracker.calibration import TrackerRuntime

with TrackerRuntime({"left": "LHR-AAAAAAAA", "right": "LHR-BBBBBBBB"}).start() as rt:
    print(rt.geometry())              # 几何门
    frame, age_s = rt.latest("left")  # 最新 TrackerFrame 与其年龄（秒），遮挡恢复平滑后
    raw, _ = rt.latest("left", raw=True)  # 原始帧（不经平滑）
    d = rt.delta("left")              # TrackerPose：相对零位的增量（start() 已自动记过零位）
    print(d.position, d.rotation)     # 米；四元数 (w, x, y, z)
    rt.set_reference()                # 需要时重新记零位
    print(rt.status())                # 含 last_gate、fatal
```

增量定义：`delta.position = R_refᵀ (p − p_ref)`，`delta.rotation = R_refᵀ R`，即在零位时刻 tracker 自身坐标系中表达，与基站坐标系（4.7）的原点、朝向无关。

**遮挡恢复平滑**：tracker 被整只遮住（或背对全部基站）再露出来时，恢复后的头几帧位姿可能一步跳几厘米。
`latest()` / `delta()` 默认在「刚恢复的那 1 秒」里把这一跳摊成几百毫秒的平滑过渡，其余时间原样输出；
`status()["roles"][手]["recovery"]` 记录当前偏移与发生次数。零位（`set_reference`）按原始位姿记；
直接订阅采集源（录制、ROS 节点）拿到的始终是原始帧。不需要时传 `recovery_smoothing=False`。

主要参数：

- 检查：`gate_mode`（`error`/`flag`/`off`）、`whitelist`、`registered`（默认由白名单解析）、`ootx_wait_s`（25；基站就绪即提前放行，正常启动不会等满）、`pose_wait_s`（5）；
- 零位：`auto_reference`（True）、`restore`（False）、`restore_max_age_s`（8 小时）；
- 运行期：`guard_interval_s`（1）、`stall_s`（10）、`drift_tol_m`（0.01）、`hold_sec`（3）、`fence`（True）；
- 遮挡恢复平滑：`recovery_smoothing`（True）、`recovery_params`（`RecoveryParams` 或同名字段的 dict：`dark_s` 0.2、`window_s` 1.0、`jump_mps` 1.0、`jump_dps` 180、`tau_s` 0.15、`glide_mps` 0.5、`glide_dps` 90）；
- 其它：`adopt_geometry`（True；只读自查时传 False，不采纳旧几何）、`state_dir`、`config_path`。

标定与运行不能在同一进程中切换（libsurvive 限制），标定请用命令行 `tracker calibrate`。

### 4.7 坐标系

`TrackerFrame.pose`、录制文件中的 `/tracker/<hand>`、ROS 2 的 `tracker_pose` 都是 tracker 在**基站坐标系**中的绝对位姿。
基站坐标系由 `tracker calibrate` 确定：

| 项目 | 定义 |
|---|---|
| 类型 | 右手直角坐标系 |
| 单位 | 位置为米；姿态为单位四元数，SDK 与录制文件中按 (w, x, y, z)，ROS 2 消息中按 (x, y, z, w) |
| +Z | 竖直向上。由基站与 tracker 内置的加速度计约束，接近但不严格等于铅垂方向 |
| +Y | 水平，从原点指向参考基站（参考基站的 x ≈ 0、y > 0） |
| +X | X = Y × Z：站在原点、面朝参考基站时指向右手边 |
| 水平原点 | 标定过程中 tracker 静止过的某一处位置，由解算自动选取；不是基站的位置 |
| 高度零点 | 约为标定过程中 tracker 静止过的最低高度；z 不是离地高度 |
| 位姿描述的对象 | tracker 自身的坐标系（由 tracker 出厂标定数据定义）。SDK 不叠加 tracker 在手套上的安装偏移，需要手背或手腕的位姿时请自行测量安装关系后换算 |

- **参考基站**：只有一台时就是它；有多台时，以 `tracker calibrate` 输出中最后一行
  `Using LH … (xxxxxxxx) as reference lighthouse` 为准，括号里是参考基站的 OOTX id。
- **每次标定都重建坐标系**：原点、+Y 朝向、高度零点取决于这次标定时 tracker 走过的位置，
  基站没动、只是重新标定，坐标系也会变（实测差过近 20 cm、约 5°）。不同次标定下的绝对位姿不能直接比较；
  落盘的零位带有基站几何指纹，重新标定后不再沿用（4.3）。
- **运行期固定**：运行时不重新解算基站位置，同一次标定下的绝对位姿可以直接比较。
  基站被挪动后坐标系失效；能检测到与检测不到的情况见 4.4。
- **增量**：`delta` 表达在零位时刻 tracker 自身的坐标系中（4.6），不随基站坐标系的原点、朝向变化。
- ROS 2 节点只在消息 header 里写 `frame_id`，不发布 tf（7.3）。

---

## 5. 按模块使用

`open_kit()` 覆盖常见用法。需要更细的控制时可以直接使用各模块。

### 5.1 单只手套

```python
import queue
from revohuman import GloveManager

with GloveManager().connect(handedness="left") as glove:
    sub = glove.subscribe()                  # 默认 200 Hz
    for _ in range(200):
        try:
            f = sub.recv(timeout=1.0)
        except queue.Empty:
            continue                         # 预热、时间戳门关或重连中（6.3）
        print(f.seq, f.sample_wall_ns, f.joints_deg[:3])
    print(glove.stats().to_dict())
```

`connect()` 还接受 `port=`、`unit_id=`、`set_name=`、`rate_hz=`、`require_tactile=`。
`glove.selftest()` 做只读自检。自检要先停这只手的数据流、读完再重启：这只手约 50 ms 没有新帧，双手配对流同样空约 50 ms（硬件同步保持），结束后 `f.generation` 加 1。**录制中不要调用。**
`glove.calibrate(target=255, timeout=15.0)` 做编码器零位标定（写入手套、不可撤销；标定前五指伸直、手掌平放，见 2.5）：`target=255` 标全部在线的编码器，
`0`–`20` 只标一路；返回结果带 `succeeded_indices`、`failed_indices`、`zero_raw`、`persisted`，
有编码器失败、未保存或超时时抛 `CalibrationError`（错误码 1300），前两种情况下异常的 `result` 属性是同样的部分结果。
`glove.clear_calibration()` 清除编码器标定（写入手套）。两者和自检一样要先停这只手的数据流，**录制中不要调用**。

### 5.2 双手配对

```python
from revohuman import GloveManager

pair = GloveManager().connect_pair()         # 起流前把两只手套对齐到同一 USB 帧时钟
sync = pair.synchronizer()
pf = sync.subscribe().recv(timeout=1.0)      # PairedFrame
print(pf.skew_ns)                            # 右 − 左 采样刻；硬件同步下恒为 0
```

配对精度预算 `pair.synchronizer(pair_accuracy_ms=1.0)`（默认 1.0 ms），达不到的对子不输出。
硬件同步丢失时 SDK 改用软件配对并在后台重建，不会自动停录；需要严格同步的录制可用 `sync.on_sync_lost(callback)` 注册回调（如 `rec.abort`：停下当前段并闩上录制器），之后再调用 `rec.start()` 抛 `StrictSyncLost`。

### 5.3 Tracker 底层接口

```python
from revohuman import GloveManager, Handedness
from revohuman.tracker import SurviveTrackerSource, BimanualTrackerSynchronizer

reg = GloveManager().tracker_registry()                  # {"LHR-...": Handedness.LEFT, ...}
with SurviveTrackerSource(registry=reg, lighthouses=["xxxxxxxx", "yyyyyyyy"]) as src:
    tsync = BimanualTrackerSynchronizer(src.hand(Handedness.LEFT), src.hand(Handedness.RIGHT))
    pair = tsync.subscribe().recv(timeout=1.0)           # PairedTrackerFrame
    print(pair.left.sample_wall_ns, pair.left.pose.position)
```

`lighthouses` 填 OOTX id。任一只 tracker 关闭都会结束配对流；世界系变化时抛 `WorldFrameChanged`（4.4）。

### 5.4 腕部相机（PTV1）

```python
from revohuman.camera import WristCamera, CameraConfig

cfg = CameraConfig(usb_hub="3-3", handedness="right",      # usb_hub = 该手手套单元的 usb_topology_id
                   controls={"auto_exposure": 1, "exposure_time_absolute": 333})
with WristCamera(cfg) as cam:                # with 即 start()
    f = cam.subscribe(queue_size=8).recv(timeout=1.0)
    print(f.sample_wall_ns - glove_frame.sample_wall_ns)   # 与手套同一时间轴，可直接相减
    img = f.decode()                         # 需要 camera extra
```

- 设备：给 `device="/dev/videoN"`，或给 `usb_hub=` 在该 HUB 下自动发现；两者都不给时取发现的第一台（多台时告警）。
  `open_kit()` 会按手套单元自动填写。
- 默认 640×480@30 MJPG。采集、打戳、录制只用标准库（V4L2），只有 `decode()` 需要 opencv。
- **时间戳**：由设备 PTS（帧捕获起点）经 USB 帧时钟映射到主机时钟。映射本身是微秒级的
  （抖动 p95 3 µs / max 5 µs，另有不超过 125 µs 的固定偏置），同一台相机前后帧之间的时刻很准。
  PTS 到真实曝光时刻之间有一个设备常数相位。默认按「读出中点」建模：采样刻 = PTS + 帧周期/2；
  `cam.stats().stream["sample_phase_bound_ms"]` 给出该模型下的相位界 `(T − t_exp)/2`（曝光 166 时 8.35 ms、曝光 333 时约 0）。
  需要与手套做亚帧级对齐时，测出这个常数后填入 `CameraConfig(pts_offset_ns=...)`（测量工具不随发布包提供）。
  建议固定曝光（如 333）：曝光固定时这个相位是常数，测一次即可；曝光随场景变化时它也跟着变。
- 映射拟合在主机的 CLOCK_MONOTONIC_RAW 上，**不受主机 NTP 调频影响**（见 6.1）。映射状态看 `cam.stats().clock`：
  `converged`、`model_domain`（`"raw"`）、`usb_ppm_vs_raw`（USB 帧时钟相对主机时钟的频偏）、
  `dev_ppm_vs_raw`（相机时钟相对主机时钟的频偏）。两者都是硬件时钟之间的频偏，正常时基本恒定。
- 卷帘快门逐行时刻：`f.row_sample_wall_ns(row)`。
- **零拷贝**：`cam.subscribe(loaned=True)` 得到驱动缓冲上的只读视图，必须用 `with frame:` 或 `frame.release()` 归还；
  额度用完时自动退回拷贝（`stats().stream["loan"]["starved"]` 计数），数据不丢。零拷贝与普通订阅可以并存。
- 时间映射尚未收敛的帧默认不派发（`deliver_unmapped=True` 可放行，其 `stamp_source` 为 `first_payload`，精度 ±2 ms）。

`CameraFrame` 字段：`sample_wall_ns`、`sample_mono_ns`、`recv_wall_ns`（到达刻，比采样晚约 67 ms，不要用于对齐）、
`pts`、`buffer_mono_ns`、`pts_offset_ns`、`exposure_us`、`readout_ns`、`stamp_source`、`flags`、`data`、`nbytes`。

示例：[`examples/wrist_camera_async.py`](../examples/wrist_camera_async.py)、
[`examples/wrist_camera_record.py`](../examples/wrist_camera_record.py)。

### 5.5 录制与回放（MCAP）

`kit.recorder()` 已接好全部数据流。话题：

| 话题 | 内容 |
|---|---|
| `/glove/<hand>` | 单手原始帧（`GloveFrame`） |
| `/paired` | 双手配对帧（`PairedFrame`） |
| `/tracker/<hand>` | Tracker 帧（`TrackerFrame`） |
| `/camera/<hand>` | 相机帧（原始 JPEG 字节，不转码） |

手动组装：

```python
from revohuman import TopicRecorder, recording

rec = TopicRecorder(compression="lz4")
rec.record(pair.left.subscribe())
rec.record(pair.right.subscribe())
rec.record(pair.synchronizer().subscribe())
rec.guard_capture(pair)                      # 手套时间戳门关闭 → 当段 episode 作废
rec.guard_world_frame(src)                   # 基站被移动 → 当段作废并锁定

h = rec.start("ep_001.mcap")                 # 再次 start 会先收尾上一段
...
print(h.stop())                              # 阻塞到文件写完；不可取消
rec.close()

with recording.open("ep_001.mcap") as reader:
    for msg in reader:
        print(msg.topic, msg.log_time_ns, msg.frame)   # 读回即原类型的帧
```

- MCAP 的 `log_time` 为到达刻（`recv_wall_ns`，同一话题内随写入顺序单调）；跨模态对齐请用帧内的 `sample_wall_ns`（双手配对帧用 `left.sample_wall_ns`）。
- 录制器不接受免门订阅（`gated=False`，见 6.3），传入即抛 `ValueError`。
- 时间戳门关闭时 `start()` 抛 `CaptureGateClosed`。
- 数据量参考：双手 + 双相机约 11 GB/小时（随画面细节变化）。

示例：[`examples/pair_and_record.py`](../examples/pair_and_record.py)。

### 5.6 手套按键

手套上的按键在「开始」与「结束」之间切换：每次有效按压切换一次，长按不连发，上电时为「结束」。
按键状态和关节数据在同一帧里上报，不另开连接，也不改变采样率。按键默认不开启，需要时在连接时显式传 `button=True`：

```python
import queue
from revohuman import ButtonMode, GloveManager

with GloveManager().connect(handedness="right", button=True) as glove:
    presses = glove.button.subscribe()
    while True:
        try:
            ev = presses.recv(timeout=1.0)   # ButtonEvent
        except queue.Empty:
            continue
        print(ev.mode, ev.press_count, ev.sample_wall_ns)
        if ev.mode is ButtonMode.END:
            break
    presses.close()
```

双手用 `connect_pair(..., button=True)`，两只手套各自的按键在 `pair.left.button` / `pair.right.button`。
按键不参与双手配对。`open_kit()` 默认不开启按键，需要时传 `glove_kwargs={"button": True}`，按键在 `kit.glove(h).button`；ROS 2 节点不开启按键。

- **是否可用**：`glove.button.supported` 在第一次起流后才确定。起流是异步的，所以要在拿到第一帧之后再看。
  - `None`：还没开过流。
  - `True`：手套固件支持在采集期间上报按键。
  - `False`：固件不支持。此时照常连接和采集，已打开的按键订阅会关闭（`recv()` 抛 `UnsupportedFeatureError`），并打一条 `N2406`。
- **不开启时**：不传 `button=True`，`glove.button.enabled` 为 `False`。`subscribe()` / `read()` 抛 `UnsupportedFeatureError`，数据流与不带按键时完全相同。
- **当前状态**：`glove.button.read()` 返回最近一帧的 `ButtonState`，含 `mode`、`pressed`（这一帧是否正被按住）和 `press_sequence`（手套累计有效按压次数）。还没有有效状态时为 `None`。
- **事件判定**：
  - 每次起流后的第一个有效状态只作基线，不产生事件。连接前按过键的话，第一次按压可能报「结束」，再按一下即「开始」。
  - 之后累计按压次数每增加 n，报一个 `press_count=n` 的 `ButtonEvent`。
  - `press_count > 1`（即 `ev.missed_presses > 0`）表示中间有按压没有单独上报，此时以 `ev.mode` 为准。
- **重连之后**：重连或自检 / 标定重启数据流后，按键状态从头初始化，`ev.generation` 随之变化。断线前的状态不沿用，断线期间的按压也不补报。
- **时刻**：事件的 `recv_wall_ns` / `sample_wall_ns` / `seq` 取第一次看到这次按压的那一帧。
  - 按压时刻只精确到帧，适合用作 episode 的起止，不适合做精确对时。
  - 时间戳门关闭时事件照常上报，`sample_wall_ns` 为 `None`。
- **录制**：按键事件不写入 MCAP。需要留痕时，自行保存事件字段。
- **计数**：`glove.button.events_total` 是事件数，`glove.button.missed_presses_total` 是没有单独上报的按压数。
- **需要支持采集期间上报按键的手套固件。** 同一只手套同时只能被一个进程打开，不要另起程序读取按键。

示例：[`examples/button_record.py`](../examples/button_record.py)（按一下开始录一段、再按一下结束）。

---

## 6. 数据语义

### 6.1 时间戳

所有 `*_wall_ns` 均为 CLOCK_REALTIME 纳秒，共用同一个进程级时间基准，跨模态可直接相减。每种帧都带下面两个时刻：

| 字段 | 含义 |
|---|---|
| `sample_wall_ns` | **采样刻**：传感器实际采样的时刻。跨模态对齐用它 |
| `recv_wall_ns` | **到达刻**：SDK 收到的时刻，含链路延迟，不要用于对齐 |

P1 的对齐方法、同步指标与时间相关的异常处理见使用手册《09-时间戳同步》。

各模态另有自己的时间字段：

| 帧 | 字段 | 含义 |
|---|---|---|
| `GloveFrame` | `device_tick` | 编码器采样刻的设备原始值（1 ms 计数），`sample_wall_ns` 由它换算 |
| `TactileData` | `sample_wall_ns` / `device_tick` | 触觉自己的采样刻 / 其设备原始值（与编码器同一个 1 ms 时基），见下 |
| `TactileData` | `age_ms` | 设备自报的触觉数据龄，诊断用，不参与打戳 |
| `TrackerFrame` | `timecode` / `tc_unwrapped` | 设备 48 MHz 计数（约 89.5 秒回绕）/ 展开值，跨帧比较用后者 |
| `CameraFrame` | `pts` | 设备「帧捕获起点」计数，`sample_wall_ns` 由它换算，见 5.4 |
| `CameraFrame` | `sample_mono_ns` | 与 `sample_wall_ns` 同一时刻，CLOCK_MONOTONIC 表示 |
| `CameraFrame` | `buffer_mono_ns` | 驱动缓冲区时刻（CLOCK_MONOTONIC），仅供排障 |
| `CameraFrame` | `stamp_source` | `"pts"`（正常）或 `"first_payload"`（兜底，由缓冲区时刻反推，±2 ms） |
| `PairedFrame` / `PairedTrackerFrame` | `skew_ns` | 左右两只的采样刻之差 |

- **帧上没有质量字段**：时间映射不可信的帧不派发，配对精度不达标的对子不输出。拿到的每一帧都在精度预算内。
- **触觉有自己的采样刻**：触觉按自己的节拍采样，与编码器不同拍。`frame.tactile.sample_wall_ns` 比同帧编码器晚约 3–9 ms，
  这个差随手、随会话变化，**不是常数**；毫秒级分析请直接用触觉刻，不要用编码器刻加一个常数去估。
- **`skew_ns`**：手套在硬件同步下恒为 0；两只 tracker 各自采样，即使在最好的档位也不为 0（最多约 ±2 ms）。
- **绝对相位**：tracker 与腕部相机的采样刻都由设备计数映射而来，映射本身很准；「设备打戳 → 真实采样」之间
  另有一个设备常数：tracker 是设备内部的打包延迟，按序列号配置 `SurviveTrackerSource(phi_pack_ms={"LHR-…": 毫秒})`
  （ROS 参数 `tracker_phi_pack_ms:=LHR-…=毫秒,LHR-…=毫秒`，测量工具不随发布包提供）；
  相机是 PTS 到曝光时刻的相位，配置 `CameraConfig(pts_offset_ns=...)`（ROS 参数 `camera_pts_offset_ms`，见 5.4）。该常数在同一模态内前后一致，不影响该模态自身的时序；
  跨模态做绝对对齐时配置它，不配置时表现为一个固定偏置。
- 精度预算用物理量给出：双手配对 `synchronizer(pair_accuracy_ms=...)`（默认 1.0 ms）；
  Tracker 采样刻 `SurviveTrackerSource(stamp_accuracy_ms=...)`（默认不要求）。达不到预算的帧或对子不派发。
- 运行质量看统计而不是帧：`glove.stats().stream` 中的 `frames_by_tier`、`capture_gate["transitions"]`、`frames_blocked`；
  配对器 `sync.stats()["pairs_by_tier"]`。
- **主机时钟**：手套、tracker、腕部相机三路的时间映射都拟合在主机的 CLOCK_MONOTONIC_RAW（硬件时钟，NTP 不调它的频率）上，
  输出再换回 CLOCK_MONOTONIC 与 CLOCK_REALTIME。因此主机 NTP 客户端**调频**（slew）不影响采样刻：
  仿真中主机时钟突加 0.07% 的调频，三路采样刻与不调频时相差不超过 2 µs。
  主机墙钟**跳变**（step：NTP 首次对时、手动改时间、休眠唤醒补时）由手套侧检测：进程级时间基准随之刷新，
  三路的 `*_wall_ns` 同时平移同一个量、仍可互相比较；手套暂停派发帧，自动重新对时后恢复
  （计入 `capture_gate["transitions"]` 与 `frames_blocked`）。只开 tracker 或相机、不开手套时时间基准不刷新：
  各路 `*_wall_ns` 之间仍一致，但与主机当前墙钟差一个跳变量。
  录制中不希望出现这种停顿，建议主机只在开机时步进、运行中只调频（见 10）。

### 6.2 帧结构

**`GloveFrame`**

| 字段 | 说明 |
|---|---|
| `seq` / `generation` | 序号；`generation` 变化表示中间重连过 |
| `sample_wall_ns` / `recv_wall_ns` | 见 6.1 |
| `device_tick` | 设备侧 1 ms 计数 |
| `joints_deg` | 21 个关节角（度）；名称见 `revohuman.joint_map.joint_names()` |
| `joints_valid_mask` / `all_joints_valid` | 每路有效位 |
| `tactile` | `TactileData`；无触觉或本拍不可用时为 `None` |
| `handedness` | `Handedness.LEFT` / `RIGHT` |
| `sync_key` | 双手精确配对键 |
| `gate_open` | 时间戳门是否开着；只有免门订阅（6.3）会收到 `False` 的帧 |
| `skipped_snapshots` / `dropped_local` | 设备侧跳过 / 本地队列丢弃计数 |

**`TactileData`**：`values`、`point_count`、`valid`、`link_ok`、`age_ms`、`unit`、`calibrated`、`device_tick`、`sample_wall_ns`。
`unit` 随帧携带：`"mN"` 为已标定的力值（毫牛）；`"device_native"` 为原始计数，不是力。

**`PairedFrame`**：`left`、`right`、`skew_ns`。

**`TrackerFrame`**：`serial`、`seq`、`handedness`、`pose`（`TrackerPose`：基站坐标系中的位姿，`position` 米、`rotation` 四元数 w,x,y,z，见 4.7）、
`sample_wall_ns`、`recv_wall_ns`、`timecode`（设备 48 MHz 原始值，约 89.5 秒回绕）、`tc_unwrapped`。
**`PairedTrackerFrame`**：`left`、`right`、`skew_ns`。

所有帧都有 `to_dict()` / `from_dict()`（录制文件中的格式）。`from_dict()` 严格解析，缺字段即抛，
避免跨版本数据被静默读成缺省值；唯一例外是 `calibrated` / `unit`，缺失时按未标定的原始计数读回。

### 6.3 订阅的读法

SDK 的接口都是同步函数；采集在 SDK 自己的后台线程里进行，调用方不需要轮询设备。
取数据有两类接口：**订阅**一帧不漏地按顺序交付，**最新值**不阻塞、立即返回当前最新的一份。

**哪些调用会阻塞**

| 调用 | 阻塞 | 说明 |
|---|---|---|
| `open_kit()`、`connect()`、`connect_pair()` | 是 | 打开设备并做启动检查；`open_kit()` 等基站与 tracker 就绪，约 15 s。返回时不等第一帧 |
| `close()`、离开 `with` | 是 | 停止采集并等后台线程退出 |
| `sub.recv(timeout=t)` | 最多 `t` 秒 | 超时抛 `queue.Empty`；`timeout=0` 不等待；省略 `timeout` 一直等 |
| `await sub.recv_async(timeout=t)` | 否（挂起协程） | 由采集线程直接唤醒事件循环，不占线程池 |
| `rt.latest(role)` / `rt.delta(role)`（`kit.tracker`，4.6） | 否 | 没有位姿时返回 `(None, None)` / `None` |
| `cam.latest()`（5.4） | 否 | 没有帧时返回 `None` |
| `glove.button.read()`（5.6） | 否 | 返回最新按键状态 |
| `rec.start()` / `h.stop()`（5.5） | 是 | `stop()` 等文件完整写完才返回 |
| `stats()` / `status()` | 否 | 快照；`status()` 不抛异常 |
| `selftest()` / `calibrate()` | 是 | 期间这只手暂停出流（5.1） |

打开、关闭、录制起停没有 asyncio 版本，在 asyncio 程序里用 `await asyncio.to_thread(open_kit, "P1")` 这类写法调用。

**订阅的读法**（手套、双手配对、按键、tracker、相机的订阅相同）

| 读法 | 写法 |
|---|---|
| 同步取一帧 | `f = sub.recv(timeout=1.0)` |
| 同步循环 | `for f in sub:` |
| 回调 | `x.subscribe(callback=on_frame)` |
| 异步取一帧 | `f = await sub.recv_async(timeout=1.0)` |
| 异步循环 | `async for f in sub:` |

- **队列**：每个订阅是独立的有界队列，每一帧投给每个订阅。默认容量：手套与双手配对 64、按键 16、tracker（`src.tracker(serial).subscribe()`）128、
  tracker 配对 64、相机 8，用 `subscribe(queue_size=...)` 调整。队列满时**丢最旧的帧**，不影响采集与其它订阅，丢弃数见 `sub.dropped_count`。
- **一个订阅只给一个读者**：多个线程读同一订阅时，每帧只交给其中一个。需要多路消费时各自 `subscribe()`。
- **回调**：每个回调订阅有自己的后台线程，按顺序逐帧调用；回调慢同样按上条丢最旧的帧。回调抛出的异常不中断采集、**不打印**，
  只把最近一次记在 `sub.last_error`。回调订阅不要再 `recv()`，回调里不要关闭它自己的订阅。
- **最新值**：手套没有「最新一帧」接口；只要最新值时，用 `recv(timeout=0)` 把积压读空、留最后一帧。
  `rt.latest()` 只缓存带位姿的帧：遮挡、故障或关闭后仍返回最后一次的位姿，`age_s`（收到至今的秒数，主机单调时钟）随之增长，要用它判断新鲜度。
- **手套的时间戳门**：默认订阅只派发时间戳合格的帧。启动预热、时间戳质量下降（`N2102`）、断线重连期间不出帧，读的一方只看到 `queue.Empty`；
  门的状态见 `glove.capture_ok` / `glove.capture_cause`（6.6）。重连后帧的 `generation` 加 1。

**关闭与异常**

- 订阅、设备或 kit 关闭后，`recv()` / `recv_async()` 抛 `Disconnected`（1101）。`async for` 随之结束；
  同步 `for f in sub:` 若关闭时正阻塞在等待中，会抛 `Disconnected`，需要 `try` 包住，或改用 `recv(timeout=...)` 循环并检查 `sub.closed`。
- 关闭的通知只送达一次。拿到 `Disconnected` 后不要再对同一订阅 `recv()`：不带 `timeout` 会一直等，带 `timeout` 抛 `queue.Empty`。
- 因不可恢复故障关闭的订阅，`recv()` 抛对应的异常，`async for` 不会吞掉：tracker 订阅的 `WorldFrameChanged`（1902）、`GeometryLost`（1906）、
  `TrackerStreamDead`（1901），手套订阅的 `FirmwareIncompatible`（1205）等。统一捕获 `RevoHumanError`（6.5）。
- `rt.latest()` / `rt.delta()` / `status()` 不抛异常。不可恢复的 tracker 故障用 `kit.fatal`（为 `None` 表示正常）或 `kit.raise_if_fatal()` 判断；
  此后 tracker 停止，手套数据照常流动，`kit.glove()` / `kit.camera()` / `kit.synchronizer()` 改为抛出该故障。

**免门订阅**：`glove.subscribe(gated=False)` 在时间戳门关闭期间照常出帧（`frame.gate_open == False`，这些帧的 `sample_wall_ns` 不保证精度），适合只看画面、不做跨源对齐的场景；录制器不接受这种订阅。

完整示例（阻塞读、回调、asyncio 三种写法）见《04-数据录制》第 9 节。

### 6.4 触觉

- 型号不带触觉时 `frame.tactile` 为 `None`。
- 要求必须有触觉：`connect(..., require_tactile=True)`，型号不带抛 `TactileUnsupported`，判不出抛 `TactileUndetermined`。
- 双手触觉型号不一致时抛 `VariantMismatch`。

### 6.5 异常与错误码

全部异常继承 `RevoHumanError`（**不是** `RuntimeError`，捕获时请用 `RevoHumanError` 或具体类型）。
`revohuman.error_code(exc)` 给出机器可判的错误码，`format_error(exc)` 给出带码的可读文本。

| 码 | 异常 | 场景 |
|---:|---|---|
| 1000 | `RevoHumanError` | 通用 |
| 1101 | `Disconnected` | 连接断开 / 订阅已关闭 |
| 1102 | `PortBusyError` | 串口被其它进程占用（附持有者 pid） |
| 1201 | — | 设备应答异常 |
| 1202 | `UnsupportedFeatureError` | 固件不支持该功能 |
| 1203 / 1204 | `TimeProbeUnsupported` / `TimeProbeIncompatible` | 固件不支持 / 不兼容时间同步功能 |
| 1205 | `FirmwareIncompatible` | 固件版本不兼容（不可恢复） |
| 1300 | `CalibrationError` | 编码器标定失败 |
| 1400 | `UnregisteredGloveError` | 认不出手别，需要在注册表中声明 |
| 1500 | `StreamRateConflict` | 同一连接上以不同速率重复订阅 |
| 1600 | `CaptureGateClosed` | 时间戳门关闭时开始录制 |
| 1700 | `StrictSyncLost` | 双手硬件同步丢失 |
| 1800 / 1801 | `TactileUnsupported` / `TactileUndetermined` | 要求触觉但型号不带 / 判不出 |
| 1802 | `VariantMismatch` | 左右手触觉型号不一致 |
| 1803 | `ProductUnidentified` | 判不出型号（证据冲突或不足） |
| 1804 | `ProductMismatch` | 声明的型号与硬件不符，或严格模式下缺传感器 |
| 1900 | `TrackerBusyError` | libsurvive 已被其它进程占用 |
| 1901 | `TrackerStreamDead` | 启动后等不到 tracker 数据 |
| 1902 | `WorldFrameChanged` | 采集中检测到基站被移动（检测范围见 4.4） |
| 1903 | `LighthouseError` | 基站白名单、信道预检或基站写操作不通过（`reasons` 逐条列出） |
| 1904 | `CalibrationRejected` | 标定被拒绝或未完成（`stage`：`preflight` / `geometry` / `timeout` / `reference` / `state`） |
| 1905 | `TrackerNotReady` | tracker 启动检查不通过（`kind` / `reasons` / `remedy`，见 4.2） |
| 1906 | `GeometryLost` | 采集中基站几何丢失（`recoverable` 区分两级，见 4.4） |

### 6.6 运行期提示（N 码）与日志

错误码（6.5）回答「抛了哪个异常」；运行期提示回答「运行中发生了什么」。SDK 的提示日志一律以码开头：

```
N2102 [/dev/ttyACM0] 录制门关：时间基准质量下降，帧暂停派发（门开后自动恢复）
```

提示与报错里的「录制门」即本手册所说的时间戳门。

- 按前 5 个字符判码，不必解析中文。结构化字段挂在 `LogRecord` 上：`record.revohuman_code`（如 `"N2102"`）、
  `record.revohuman_fields`（端口、手别、次数、时长等 dict），日志处理器可直接转 JSON。
- 全表见 `revohuman.notices.Notice`；`revohuman.notices.notice_spec(code)` 给出默认级别与文本模板。
- 命令行（`python3 -m revohuman`、`revohuman-live`、`revohuman-lighthouses`）与三维可视化在终端里按级别给日志上色：
  DEBUG 暗淡、INFO 不上色、WARNING 黄、ERROR 红、CRITICAL 红色加粗。由环境变量 `REVOHUMAN_LOG_COLOR` 控制：
  `auto`（默认，输出是终端、没设 `NO_COLOR`、`TERM` 不是 `dumb` 时上色）/ `always` / `never`。
  重定向到文件或管道时不带颜色，内容与不上色时逐字节相同。SDK 作为库使用时不配置日志处理器，也就不上色。

| 码段 | 类别 | 例 |
|---|---|---|
| 21xx | 单手时间与录制门 | 2101 录制门开 · 2102 录制门关 · 2103 录制门重开 · 2104 / 2105 时间戳质量降级 / 恢复 · 2107 传输延迟异常 |
| 22xx | 双手同步与配对 | 2201 退回软件配对 · 2202 硬件同步丢失 · 2203 已重建 · 2204 重建失败 · 2205 / 2206 对齐偏差超预算 / 恢复 · 2207 / 2208 触觉配对暂停 / 恢复 |
| 23xx | USB 链路 | 2301 链路不稳 · 2302 复位中 · 2303 恢复失败 · 2304 已恢复 |
| 24xx | 流 | 2401 重连中 · 2402 已恢复 · 2403 串口换号 · 2404 流中断 · 2405 固件不兼容 · 2406 固件不支持采集期间上报按键 |
| 25xx | 录制 | 2501 整条 episode 作废 · 2502 录制因双手同步丢失而中止 |

时间质量问题只报**原因大类**（`revohuman.notices.SyncCause`），够决定怎么处置：

| `cause` | 含义 | 处置 |
|---|---|---|
| `clock_not_ready` | 时间基准尚未建立或已过期，正在重建 | 通常数秒内自愈，无需操作 |
| `clock_degraded` | 时间基准质量下降 | 持续出现时检查 USB 线缆、HUB 与主机负载 |
| `host_clock_event` | 主机时钟跳变（休眠唤醒 / 手动改时 / 对时步进） | 自动恢复；录制中避免，见 10 |
| `device_reset` | 设备复位 | 自动恢复；频繁出现检查供电 |
| `link_unstable` | USB 链路不稳 | 检查线缆、HUB 与供电 |

读取位置：`glove.capture_cause`（门开时为 `None`）、`glove.stats().stream["capture_gate"]["cause"]`、
`glove.stats().mapper_quality["cause"]`、`glove.on_capture_gate` 回调（`(glove, ok, detail)`）的 `detail["cause"]`，
以及 ROS `status` 里同名字段。

---

## 7. ROS 2

### 7.1 构建与启动

```bash
mkdir -p ~/ros2_ws/src && ln -s $PWD/ros2/* ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash

ros2 launch revohuman_bringup revohuman.launch.py                 # 双手，自动识别型号
ros2 launch revohuman_bringup revohuman.launch.py mode:=left      # 单手
ros2 launch revohuman_bringup revohuman.launch.py product:=PTV1   # 声明型号
ros2 launch revohuman_bringup revohuman.launch.py --show-args     # 全部可调参数
```

三个包：`revohuman_msgs`（`TactileArray`、`Calibrate`）、`revohuman_ros`（节点）、`revohuman_bringup`（launch 与参数）。
以上命令在源码包解压出的 `revohuman-<版本>/` 目录里执行。ROS 节点需要能 import `revohuman`：
先在 ROS 所用的 Python 3.10 环境里装好 §1 的 wheel。

| `mode` | 节点 | 命名空间 |
|---|---|---|
| `pair`（默认） | `bimanual_node`，42 关节 | `/revohuman/pair` |
| `left` / `right` | `glove_node`，21 关节 | `/revohuman/left` / `/revohuman/right` |

参数优先级：命令行 > `params_file`（默认 `config/revohuman.yaml`）> 节点默认值。
`mode:=pair` 与 `mode:=left|right` 不要同时启动（同一只手套只能被一个进程打开）。

节点以退出码 5 退出表示切换 Tracker 模式（运行 ↔ 基站标定），launch 会自动重启它；60 秒内超过 10 次则停止重启。

### 7.2 型号与传感器

节点启动时按 2.2 识别型号，并据此决定启用哪些传感器：

- `product`：`auto`（默认）/ `P1` / `PTV1` / `off`（不识别，完全按下面的参数）。
- 触觉：型号带触觉而某只手读不到 → 拒绝启动（`product_strict:=false` 时只告警）。`publish_tactile:=off` 可关闭。
- 相机：`camera_mode` 留空时 PTV1 为 `required`、P1 为 `off`；显式给值优先。
- Tracker：两个型号都带，由 `tracker_mode`（默认 `required`）控制。

### 7.3 话题

单手（`<ns>` = `/revohuman/left` 或 `/revohuman/right`）：

| 话题 | 类型 | 说明 |
|---|---|---|
| `joint_states` | `sensor_msgs/JointState` | 21 关节，**弧度** |
| `tactile` | `revohuman_msgs/TactileArray` | 仅带触觉时存在 |
| `tracker_pose` | `geometry_msgs/PoseStamped` | 基站坐标系中的绝对位姿（4.7），`frame_id` 由 `tracker_frame_id` 设定（默认 `world`） |
| `tracker_delta` | `geometry_msgs/PoseStamped` | 相对零位的增量，表达在零位时刻 tracker 自身坐标系中（4.6），`frame_id` = `revohuman_<hand>_tracker_zero`；零位标定后才发布 |
| `wrist_camera/image/compressed` | `sensor_msgs/CompressedImage` | 仅启用相机时存在；MJPG 原始字节 |
| `status` / `tracker_status` / `wrist_camera/status` | `std_msgs/String`（JSON） | 1 Hz 状态 |

双手（`/revohuman/pair`）：`joint_states`（左 21 + 右 21）、`tactile_left|right`、`tracker_pose_left|right`、
`tracker_delta_left|right`、`wrist_camera_left|right/image/compressed`、`status`、`tracker_status`、`wrist_camera_status`。

- 数据话题 QoS：BEST_EFFORT，depth 10。
- **状态话题 QoS：RELIABLE + TRANSIENT_LOCAL，depth 1**。订阅方 QoS 不匹配时一条也收不到。
- 所有 `header.stamp` 均为采样刻（与原生 SDK 相同）。到达刻不发布。
- `tracker_pose` / `tracker_delta` 是原始位姿（不经 4.6 的遮挡恢复平滑），四元数按 ROS 惯例为 (x, y, z, w)。
  节点不发布 tf，只在 header 里写 `frame_id`；需要接入 tf 树时由使用方自行广播。
- 双手 `joint_states` 的 `header.stamp` 取左手采样刻；硬件同步下两手同刻，否则右手相差不超过 `pair_accuracy_ms`。
- `TactileArray`：`header.stamp` 为触觉自身采样刻，`frame_stamp` 等于同周期 `joint_states` 的 `header.stamp`（按它等值配对），
  `calibrated` 为假时 `data` 是原始计数。

### 7.4 服务

手套服务（单手在 `<ns>/` 下，双手在 `<ns>/left/` 与 `<ns>/right/` 下）：

| 服务 | 类型 | 说明 |
|---|---|---|
| `selftest` | `std_srvs/Trigger` | 只读自检；该手停流约 50 ms，录制中不要调用 |
| `get_sn` | `std_srvs/Trigger` | 读序列号 |
| `calibrate` | `revohuman_msgs/Calibrate` | 编码器标定（写设备），仅 `allow_calibration:=true` 时注册 |
| `clear_calibration` | `std_srvs/Trigger` | 清除编码器标定（写设备），同上 |

ROS 接口不提供序列号写入。

Tracker 服务（在 `<ns>/` 下）：

| 服务 | 类型 | 说明 |
|---|---|---|
| `tracker/set_reference` | `std_srvs/Trigger` | 零位标定；成功时 `message` 为结果 JSON |
| `tracker/get_status` | `std_srvs/Trigger` | 返回 tracker 状态 JSON |
| `tracker/set_calibration_mode` | `std_srvs/SetBool` | `true`：进入基站标定；标定中 `false`：中止并回滚 |

**在 ROS 中标定基站**：先用命令行完成 4.1 的 ①–③（白名单与扫描），然后

```bash
ros2 service call /revohuman/pair/tracker/set_calibration_mode std_srvs/srv/SetBool "{data: true}"
```

白名单或信道预检不通过时直接拒绝。通过后节点退出并以标定模式重启（此时不打开手套），
在 `tracker_status` 上发布 `{"mode": "calibrate", "state", "coverage", "hint", "trackers"}` 供界面引导操作；
完成、失败或中止后自动回到运行模式。之后再调 `tracker/set_reference` 做零位标定。

ROS 节点不执行 4.2 的启动检查和 4.4 的运行期保护（世界系门除外：它由 `tracker_world_gate_mode` 控制，照常生效）：
启动时不拦截、不自动记零位，而是沿用上次落盘的零位
（条件同 4.3 的 `restore=True`），没有可沿用的零位时调 `tracker/set_reference`。
基站几何是否达标看 `tracker_status` 的 `geometry` 字段；`tracker_geometry_gate_mode` 为 `error` 时几何不达标拒绝零位标定。

### 7.5 参数

完整列表与注释见 [`ros2/revohuman_bringup/config/revohuman.yaml`](../ros2/revohuman_bringup/config/revohuman.yaml)。常用项：

| 参数 | 默认 | 说明 |
|---|---|---|
| `product` / `product_strict` / `product_rules_path` | `auto` / `true` / 空 | 型号识别（7.2） |
| `set_name` / `registry_path` | 空 | 注册表（2.3） |
| `rate_hz` | `200.0` | 手套速率 |
| `publish_tactile` / `require_tactile` | `auto` / `false` | 触觉发布 |
| `allow_calibration` | `false` | 是否注册编码器标定服务 |
| `pair_mode` | `strict` | `strict`：只输出同一硬件同步周期的对子；`best_effort`：允许 1 ms 内的对子 |
| `tracker_mode` | `required` | `required` / `auto` / `off` |
| `tracker_left_serial` / `tracker_right_serial` / `tracker_serial` | 空 | 覆盖自动解析的 tracker 序列号 |
| `tracker_lighthouses` | 空 | 在册基站 OOTX id；留空由基站白名单解析（4.1） |
| `tracker_world_gate_mode` / `tracker_geometry_gate_mode` | `error` / `error` | 世界系门 / 几何门（7.4） |
| `tracker_fence` / `tracker_require_preflight` | `true` / `true` | 外来基站围栏 / 标定前要求预检 |
| `tracker_calibration_timeout_s` | `300.0` | 基站标定超时 |
| `tracker_state_dir` / `tracker_config_path` | 空 | 状态目录 / libsurvive 配置路径 |
| `tracker_frame_id` | `world` | `tracker_pose` 的 `frame_id`，即基站坐标系（4.7）的名字 |
| `camera_mode` | 空（随型号） | `required` / `auto` / `off` |
| `camera_width` / `camera_height` / `camera_fps` / `camera_fourcc` | `640` / `480` / `30` / `MJPG` | 相机格式 |
| `camera_exposure` | `166` | 曝光（100 µs 一格，≤0 为自动）；建议 `333`（5.4） |
| `camera_zero_copy` | `true` | 零拷贝发布 |

经 `ros2 launch` 传参时类型由 launch 自动转换，列表参数用逗号分隔（`tracker_lighthouses:=xxxxxxxx,yyyyyyyy`）。
直接 `ros2 run` 或写 yaml 时类型必须与默认值一致：浮点写 `200.0` 而不是 `200`；
字符串 `off` 要加引号 `-p "tracker_mode:='off'"`（否则被解析成布尔）。

### 7.6 状态 JSON

- `status`：`handedness`、`effective_rate_hz`、`frames_by_tier`、`capture_gate`、`frames_blocked`、`mapper_quality`、
  `pairing`（双手）、`product`（型号识别结果）、`node`。
- `tracker_status`（运行模式）：`mode`（`runtime`）、`running`、`dead_reason`、`world`（世界系门状态）、`lighthouses`、`trackers`、`pairing`、
  `session`（`registered_lighthouses`、各手 `online` / `pose_age_s` / `referenced`、`reference`、`geometry`、`startup`）。
- `wrist_camera/status`：`running`、`resolved`（设备路径）、`stream`（帧计数、`effective_fps`、`sample_phase_bound_ms`、
  `loan`、`stalled`、`fatal_error`）、`clock`、`config`、`node`。判活看 `stalled` / `fatal_error` 与帧计数是否递增。

---

## 8. 命令行参考

`python3 -m revohuman [-v] <命令>`，所有命令支持 `--json`。

| 命令 | 说明 |
|---|---|
| `doctor [--probe]` | 体检 |
| `firmware info [--hands both\|left\|right] [--port P]` | 在位手套的固件版本与升级状态（只读） |
| `firmware upgrade IMAGE.bin --version X.Y.Z (--hand left\|right \| --port P) [--allow-downgrade] [--yes] [--reconnect-timeout 45]` | 升级一只手套的固件（2.4） |
| `glove calibrate (--hand left\|right \| --port P) [--index N ...] [--yes] [--timeout 15]` | 编码器零位标定，写入手套、不可撤销（2.5） |
| `product identify [--product auto\|P1\|PTV1] [--hands both\|left\|right] [--set S] [--no-probe] [--rules P]` | 识别型号 |
| `tracker lighthouses <子命令>` | 同 `revohuman-lighthouses <子命令>`（见下表） |
| `tracker preflight [--seconds 10]` | 只做信道预检的快速版：被动扫描，不连接基站 |
| `tracker check [--live]` | 启动前自查：默认只读文件；`--live` 起一次采集，连基站数据与 tracker 位姿一起查 |
| `tracker calibrate [--timeout 300] [--min-sec 3] [--stable-sec 3] [--coverage-cells N] [--coverage-span-deg DEG] [--no-fence] [--no-preflight] [--quiet]` | 基站几何标定 |
| `tracker set-reference [--wait 5]` | 零位标定 |
| `tracker status` | 离线查看白名单、预检、几何、零位、上次启动检查 |

`check`、`calibrate` 与 `set-reference` 另接受 `--tracker left=LHR-... --tracker right=LHR-...`、`--hands`、`--set`、
`--registry`（手套注册表）、`--whitelist`、`--state-dir`、`--config`；不给时按手套单元与基站白名单自动解析。

退出码：`0` 成功；`1` 失败、未就绪或被启动检查拦下；`2` 用法错误；`3` 被拒绝或型号不符（`set-reference` 被启动检查拦下也是 `3`）。

基站命令 `revohuman-lighthouses [-v] <子命令>`（需 `ble` extra），均接受 `--whitelist`、`--state-dir`、`--json`：

| 子命令 | 说明 |
|---|---|
| `scan [--seconds 10] [--no-connect]` | 扫描附近基站，对照白名单，查同信道冲突；通过时刷新信道预检 |
| `show` | 离线查看白名单解析结果（不用蓝牙） |
| `init` | 按示例写出配置文件（已存在则不动） |
| `plan [--pin FA…=CH] [--avoid CH]` | 算改信道方案并打印，不写 |
| `apply --yes [--pin FA…=CH] [--avoid CH]` | 重扫、重算并执行方案（不带 `--yes` 等于 `plan`） |
| `set-channel FA… CH [--yes]` | 把白名单里的一台改到指定的空闲信道（不带 `--yes` 只打印） |
| `identify FA…\|LHB-… [--seconds 5] [--foreign]` | 让一台基站闪灯；外来站必须加 `--foreign`，并记审计 |

退出码见 4.1。

---

## 9. 仓库结构与测试

发布物里的布局：核心 SDK 在 wheel 里（`pip install` 后位于 site-packages），源码包解压为：

```
revohuman-<版本>/
  ├ README.md           安装与上手
  ├ ros2/               ROS 2 包（见下）
  ├ examples/           示例
  ├ docs/               使用手册（00-总览.md 起）与本参考手册（RevoHuman SDK 参考手册.md）
  ├ simulation/         仿真与可视化（手部模型 URDF 即将提供，就位后使用）
  └ docker/             Docker 镜像构建文件（Dockerfile 与启动脚本 run.sh）
```

模块划分：

```
revohuman/              SDK（核心，wheel 内为编译模块）
  ├ kit.py              open_kit / RevoHumanKit
  ├ product.py          型号识别
  ├ manager.py          GloveManager：扫描、手别解析、连接
  ├ glove.py            单只手套
  ├ pairing.py          双手配对
  ├ frames.py           帧类型
  ├ recording.py        MCAP 录制与回放
  ├ doctor.py           体检
  ├ tracker/            Vive Tracker：采集、时间映射、配对、世界系检测
  │  └ calibration/     基站白名单、信道预检、几何标定、启动检查、零位、运行会话
  └ camera/             腕部相机
ros2/
  ├ revohuman_msgs/     消息与服务定义
  ├ revohuman_ros/      glove_node / bimanual_node
  └ revohuman_bringup/  launch 与参数
docs/                   使用手册（00–09）与参考手册
examples/               示例
simulation/             手套可视化：订阅 revohuman-live 的本机广播（见 simulation/README.md）
```

示例（`examples/`）：`kit_quickstart.py`（整套）、`read_frames.py` / `read_frames_async.py`（单手）、
`pair_and_record.py`（双手录制）、`button_record.py`（按键控制录制）、`tracker_bimanual.py`、`wrist_camera_record.py`、`wrist_camera_async.py`。

---

## 10. 使用须知

- **手别与型号不靠猜**：认不出时 SDK 拒绝启动并说明原因。按提示声明一次注册表或型号即可。
- **一只手套、一套 libsurvive 同时只能被一个进程打开。**
- **基站被移动、更换或增减后**，更新白名单、重新 `revohuman-lighthouses scan`，再 `tracker calibrate`。
  基站被平移或水平转动时运行期检测不到（4.4），动过就重标，不要等报错。
- **每次标定都会重建基站坐标系**（4.7），不同次标定下录的 tracker 绝对位姿不能直接比较。
- **更换场地或灯光后**，重新确认相机曝光。
- **主机对时**：多台主机的数据要对齐时，让它们对同一台 NTP 服务器对时，并配置成只在开机时步进、运行中只调频
  （如 chrony 的 `makestep 1 3`）。运行中步进会让手套短暂暂停派发（见 6.1）。
- 序列号为出厂写入，SDK 只读不写。
