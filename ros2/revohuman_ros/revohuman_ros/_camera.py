"""两节点共用的腕部相机嵌入：起 ``WristCamera``、帧 → CompressedImage、1 Hz 状态 JSON。

``camera_mode``：留空（默认）= 按产品型号定（PTV1 = ``required``、P1 / ``product=off`` = ``off``）；
``off`` = 不 import ``revohuman.camera``、不建话题、不碰 sysfs；``auto`` = 起得来就发、起不来只告警
（手套照发）；``required`` = 起不来整个节点启动失败。

时基：``header.stamp`` = 帧的采样时刻（曝光中点的 CLOCK_REALTIME），与 joint_states / tracker_pose
同一条时间轴。

零拷贝（``camera_zero_copy``，默认 true）：``WristCamera.subscribe(loaned=True)`` 拿驱动 mmap 缓冲上的
只读视图，进 ROS 消息时只拷一次（``array.array("B").frombytes``），拷完立即归还驱动。

定位：相机跟着手套单元走——``camera_usb_hub`` 留空时取产品识别给出的该手 HUB（MCU 序列号定手别），
只在那个 HUB 下找相机；显式 ``camera_device``（+ ``camera_meta_device``）则不查 sysfs。
"""

from __future__ import annotations

import array
import json
from typing import Callable, Optional

from sensor_msgs.msg import CompressedImage

from revohuman.codes import format_error

from . import _common as C

CAMERA_MODES = ("", "off", "auto", "required")
DEFAULT_CAMERA_MODE = ""         # 空 = 按产品型号定（见 _product.resolve_camera_mode）
# 曝光：UVC bAutoExposureMode 枚举——1 = 手动，3 = 光圈优先自动。`camera_exposure<=0` 走自动档。
_MANUAL_EXPOSURE = 1
_AUTO_EXPOSURE = 3
# sensor_msgs/CompressedImage.format：MJPG 帧就是完整 JPEG；其它 fourcc 原样小写透传（非标准，
# 只是为了不丢信息——腕部相机的定型配置是 MJPG）。
_FORMATS = {"MJPG": "jpeg", "JPEG": "jpeg"}


def declare_camera_params(node) -> dict:
    """两节点共用的相机参数（设备/HUB 这几项按手各自声明，见 declare_camera_hand_params）。"""
    mode = C.check_choice(
        "camera_mode",
        str(node.declare_parameter("camera_mode", DEFAULT_CAMERA_MODE).value).strip().lower(),
        CAMERA_MODES)
    width = int(node.declare_parameter("camera_width", 640).value)
    height = int(node.declare_parameter("camera_height", 480).value)
    fps = int(node.declare_parameter("camera_fps", 30).value)
    fourcc = str(node.declare_parameter("camera_fourcc", "MJPG").value).strip().upper() or "MJPG"
    # 100µs 单位（v4l2 exposure_time_absolute），166 = 设备出厂默认 16.6ms；<=0 = 自动曝光
    exposure = int(node.declare_parameter("camera_exposure", 166).value)
    buffers = int(node.declare_parameter("camera_buffers", 4).value)
    use_metadata = bool(node.declare_parameter("camera_use_metadata", True).value)
    zero_copy = bool(node.declare_parameter("camera_zero_copy", True).value)
    frame_id = str(node.declare_parameter("camera_frame_id", "").value).strip()
    # 标定量回填：**空串 = 未标定**，不用数值哨兵。pts_offset 标定值 0 合法（PTS 正好落在采样刻），
    # 负值也合法（PTS 晚于采样刻），所以 -1 / 0 都不能当「没设」；readout 为了一致也用空串。
    pts_offset_ms = _optional_float(node, "camera_pts_offset_ms")
    readout_ms = _optional_float(node, "camera_readout_ms")
    if readout_ms is not None and readout_ms <= 0:
        raise ValueError(f"参数 camera_readout_ms 必须 > 0，收到 {readout_ms}")
    overlay = _parse_control_overlay(
        # 默认值必须是 [""] 不能是 []：rclpy 从默认值推类型，空列表推出来是
        # BYTE_ARRAY，之后传字符串数组会被 InvalidParameterTypeException 挡在
        # 解析之前。tracker_lighthouses / tracker_survive_args 也是这个写法。
        node.declare_parameter("camera_controls", [""]).value or [])

    from revohuman.camera.frames import DEFAULT_CONTROLS
    controls = dict(DEFAULT_CONTROLS)          # 托管全部 12 项，不是只管曝光那两项
    if exposure > 0:
        controls["auto_exposure"] = _MANUAL_EXPOSURE
        controls["exposure_time_absolute"] = exposure
    else:
        controls["auto_exposure"] = _AUTO_EXPOSURE
        # 自动曝光下 exposure_time_absolute 是 INACTIVE：留着必然回读不符、每次起节点都告警一条
        controls.pop("exposure_time_absolute", None)
    controls.update(overlay)
    params = {
        "mode": mode,
        "width": width, "height": height, "fps": fps, "fourcc": fourcc,
        "exposure": exposure, "controls": controls,
        "buffers": buffers, "use_metadata": use_metadata,
        "zero_copy": zero_copy,
        "frame_id": frame_id or None,
        "pts_offset_ms": pts_offset_ms, "readout_ms": readout_ms,
        "strict_controls": bool(node.declare_parameter("camera_strict_controls", False).value),
    }
    if mode:                                   # 显式给了档位：连手套之前就把错参数拦下
        check_camera_params(params)
    return params


def check_camera_params(params: dict) -> None:
    """成像参数合法性。``off`` 档不校验：没装相机的机器不该被相机参数拦住。
    ``camera_mode`` 留空时由节点在型号识别、定下档位之后再调一次。"""
    if params["mode"] in ("", "off"):
        return
    for name in ("width", "height", "fps", "buffers"):
        if params[name] <= 0:
            raise ValueError(f"参数 camera_{name} 必须 > 0，收到 {params[name]}")
    if len(params["fourcc"]) != 4:
        raise ValueError(f"参数 camera_fourcc 要 4 个字符的 fourcc（MJPG / YUYV），收到 {params['fourcc']!r}")


def _optional_float(node, name: str):
    """空串 = 没配。ROS 的 double 参数没有「未设置」这个状态，而 0 与负数都是合法标定值。"""
    raw = str(node.declare_parameter(name, "").value).strip()
    if not raw:
        return None
    try:
        return float(raw)
    except ValueError:
        raise ValueError(f"参数 {name} 要是数字或空串（空 = 未标定），收到 {raw!r}") from None


def _parse_control_overlay(items) -> dict:
    """`["gamma=120", "sharpness=3"]` → `{"gamma": 120, "sharpness": 3}`，叠在 DEFAULT_CONTROLS 上。

    名字在这里就校验：写错的控制项名如果放到 `_apply_controls` 才发现，只会变成 stats 里
    一行 `设备不支持`，而节点照常起来——配置写错和设备真没这一项从外面看一模一样。"""
    from revohuman.camera._v4l2 import CONTROL_IDS
    out = {}
    for raw in items:
        txt = str(raw).strip()
        if not txt:
            continue
        if "=" not in txt:
            raise ValueError(f"参数 camera_controls 的条目要写成 name=value，收到 {txt!r}")
        name, _, val = txt.partition("=")
        name = name.strip()
        if name not in CONTROL_IDS:
            raise ValueError(f"参数 camera_controls 里的 {name!r} 不是已知控制项；"
                             f"可用：{sorted(CONTROL_IDS)}")
        try:
            out[name] = int(val.strip())
        except ValueError:
            raise ValueError(f"参数 camera_controls 的 {name} 取值要是整数，收到 {val.strip()!r}") from None
    return out


def declare_camera_hand_params(node, prefix: str = "camera_") -> dict:
    """某一只手的相机定位参数：`<prefix>device` / `<prefix>meta_device` / `<prefix>usb_hub`，
    空串 = 自动（device 自动 = sysfs 按 HUB 找；usb_hub 自动 = 该手 MCU 所在的 HUB，见 resolve_hand_hub）。"""
    out = {}
    for key in ("device", "meta_device", "usb_hub"):
        val = str(node.declare_parameter(prefix + key, "").value).strip()
        out[key] = val or None
    return out


def resolve_hand_hub(handedness, sysfs_root=None,
                     warn: Optional[Callable[[str], None]] = None) -> Optional[str]:
    """该手手套单元的 HUB（unit_id，如 "3-1"）：相机跟着 MCU 走（`identity.hand_hub`，
    手别只认 MCU 序列号的 L/R 手别位）。None = 没有任何依据，相机按全机第一台找
    （单手套老用法）；该手认不出而另一只手认得出来 → 抛 `identity.HandLocateError`。"""
    from revohuman import Handedness, identity
    hub, note = identity.hand_hub(Handedness(handedness), sysfs_root=sysfs_root)
    if note and warn is not None:
        warn(f"{handedness} 手相机 HUB：{note}")
    return hub


def create_camera(node, params: dict, hand: dict, handedness: str,
                  hub_resolver: Optional[Callable[[], Optional[str]]] = None,
                  extra_kwargs: Optional[dict] = None):
    """构造 `WristCamera`（**不 start**）。失败时 auto 档返回 None（手套照发），required 档抛出。
    `extra_kwargs` 给测试塞 `capture_factory=`（仿真采集，不碰 /dev/video*）。

    构造与 start 分两步同 tracker：订阅必须挂在 start() 之前，起完再订只收得到尾巴。"""
    try:
        from revohuman.camera import CameraConfig, WristCamera   # 惰性：camera_mode=off 时根本不碰
        from revohuman.identity import HandLocateError
        usb_hub = hand["usb_hub"]
        if usb_hub is None and hand["device"] is None and hub_resolver is not None:
            try:
                usb_hub = hub_resolver()
            except HandLocateError:
                raise   # 该手认不出 / 不在位：退成「不限 HUB」会拿到另一只手的相机，不开（auto 档手套照发）
            except Exception as exc:  # noqa: BLE001 —— sysfs 读失败等：按「不限 HUB」处理并说明
                node.get_logger().warning(f"{handedness} 手相机 HUB 自动解析失败：{format_error(exc)}")
        extra_cfg = {"usb_ids": tuple(hand["usb_ids"])} if hand.get("usb_ids") else {}
        cfg = CameraConfig(
            **extra_cfg,
            device=hand["device"], meta_device=hand["meta_device"], use_metadata=params["use_metadata"],
            usb_hub=usb_hub, handedness=handedness,
            width=params["width"], height=params["height"], fps=params["fps"], fourcc=params["fourcc"],
            controls=params["controls"], buffers=params["buffers"],
            strict_controls=params.get("strict_controls", False),
            pts_offset_ns=(None if params.get("pts_offset_ms") is None
                           else int(params["pts_offset_ms"] * 1e6)),
            readout_ns=(None if params.get("readout_ms") is None
                        else int(params["readout_ms"] * 1e6)))
        kwargs = dict(config=cfg, name=f"wrist_camera_{handedness}")
        kwargs.update(extra_kwargs or {})
        return WristCamera(**kwargs)
    except Exception as exc:  # noqa: BLE001 —— ImportError / 参数不合法
        return _fail(node, params, handedness, exc)


def start_camera(node, cam, params: dict, handedness: str) -> bool:
    """起采集。True = 起来了；False = auto 档下没起来（调用方拆掉相机那半）；required 档抛出。"""
    try:
        cam.start()
        return True
    except Exception as exc:  # noqa: BLE001 —— FileNotFoundError（没找到相机）/ OSError（被占用）…
        _fail(node, params, handedness, exc)
        return False


def _fail(node, params: dict, handedness: str, exc: BaseException):
    msg = f"{handedness} 手腕部相机启动失败：{format_error(exc)}"
    if params["mode"] == "required":
        node.get_logger().error(msg)
        raise exc
    node.get_logger().warning(msg + "（camera_mode=auto：手套照发，相机话题不建）")
    return None


def image_format(fourcc: str) -> str:
    return _FORMATS.get(str(fourcc).upper(), str(fourcc).lower())


def make_compressed_image(frame, frame_id: str) -> CompressedImage:
    """CameraFrame / LoanedFrame → sensor_msgs/CompressedImage。唯一一次拷贝在 `frombytes`：
    memoryview（零拷贝帧）与 bytes（拷贝帧）都走 buffer 协议，一次 memcpy。"""
    msg = CompressedImage()
    msg.header.stamp = C.to_stamp(frame.sample_wall_ns)
    msg.header.frame_id = frame_id
    msg.format = image_format(frame.fourcc)
    buf = array.array("B")
    buf.frombytes(frame.data)
    msg.data = buf          # array.array('B') 走 setter 的快路径，不逐字节断言
    return msg


_STREAM_KEYS = (
    "frames_received", "frames_delivered", "frames_unmapped", "frames_lost_driver", "frames_error",
    "frames_bad_jpeg", "frames_meta_missing", "frames_dropped_local", "meta_without_pts", "read_timeouts",
    "effective_fps", "period_ms", "recv_minus_sample_ms", "bytes_per_frame_mean", "last_frame_age_s",
    "stalled", "uptime_s", "stamp_sources", "exposure_us", "sample_phase_bound_ms", "controls",
    "subscribers", "loaned_subscribers", "queue_len", "loans_delivered", "loan",
    "fatal_error", "device", "meta_device", "usb_path",
)


def status_dict(cam, counters: Optional[dict] = None) -> dict:
    """1Hz 状态：采集账目 + 时钟映射质量 + 零拷贝账（`stream.loan.outstanding/starved`）+ 节点计数。
    只报不判——`stream.stalled` / `stream.fatal_error` 是给上位机判活的原料。"""
    st = cam.stats()
    stream = st.stream or {}
    out = {
        "running": bool(cam.running),
        "handedness": st.handedness,
        "resolved": dict(getattr(cam, "resolved", {}) or {}),
        "stream": {k: stream.get(k) for k in _STREAM_KEYS},
        "clock": st.clock,
        "config": {k: (st.config or {}).get(k)
                   for k in ("width", "height", "fps", "fourcc", "buffers", "loan_reserve",
                             "use_metadata", "pts_offset_ns", "readout_ns")},
    }
    if counters:
        out["node"] = dict(counters)
    return out


def status_payload(cam, counters: Optional[dict] = None) -> str:
    return json.dumps(status_dict(cam, counters), ensure_ascii=False, default=str)
