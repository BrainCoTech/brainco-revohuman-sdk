"""Wrist camera embedding shared by both nodes: starts ``WristCamera``, frame → CompressedImage, 1 Hz status
JSON.

``camera_mode``: empty (default) = decided by product model (PTV1 = ``required``, P1 / ``product=off`` =
``off``); ``off`` = does not import ``revohuman.camera``, creates no topics, does not touch sysfs; ``auto`` =
publish if the camera starts, only a warning if it does not (the glove still publishes); ``required`` = if
the camera does not start, the whole node fails to start.

Time base: ``header.stamp`` = the frame's sampling time (CLOCK_REALTIME at mid-exposure), on the same time
axis as joint_states / tracker_pose.

Zero-copy (``camera_zero_copy``, default true): ``WristCamera.subscribe(loaned=True)`` gets a read-only view
on the driver's mmap buffer; it is copied exactly once into the ROS message (``array.array("B").frombytes``)
and returned to the driver immediately after the copy.

Locating: the camera follows the glove unit. When ``camera_usb_hub`` is empty, the HUB of this hand given by
product identification is used (the MCU firmware decides handedness), and only cameras of known models
(``revohuman.camera.models``) under that HUB are searched; an explicit ``camera_device``
(+ ``camera_meta_device``) skips sysfs.

Model: ``camera_model`` empty = auto-detect by the device's vid:pid, and the control base table is the one
listed for that model; if set, the model is pinned, and start() fails outright if the plugged-in device is
not that model. This module only produces **overrides relative to the base table** (exposure policy +
``camera_controls``).
"""

from __future__ import annotations

import array
import json
from typing import Callable, Optional

from sensor_msgs.msg import CompressedImage

from revohuman.codes import format_error

from . import _common as C

CAMERA_MODES = ("", "off", "auto", "required")
DEFAULT_CAMERA_MODE = ""         # empty = decided by product model (see _product.resolve_camera_mode)
# Exposure: UVC bAutoExposureMode enum: 1 = manual, 3 = aperture-priority auto. `camera_exposure<=0` = auto.
_MANUAL_EXPOSURE = 1
_AUTO_EXPOSURE = 3
# sensor_msgs/CompressedImage.format: an MJPG frame is a complete JPEG; other fourccs are passed through as-is,
# lowercased (non-standard, only so no information is lost; the wrist camera's fixed configuration is MJPG).
_FORMATS = {"MJPG": "jpeg", "JPEG": "jpeg"}


def declare_camera_params(node) -> dict:
    """Camera parameters shared by both nodes (the device/HUB ones are declared per hand, see
    declare_camera_hand_params)."""
    mode = C.check_choice(
        "camera_mode",
        str(node.declare_parameter("camera_mode", DEFAULT_CAMERA_MODE).value).strip().lower(),
        CAMERA_MODES)
    width = int(node.declare_parameter("camera_width", 640).value)
    height = int(node.declare_parameter("camera_height", 480).value)
    fps = int(node.declare_parameter("camera_fps", 30).value)
    fourcc = str(node.declare_parameter("camera_fourcc", "MJPG").value).strip().upper() or "MJPG"
    # In units of 100µs (v4l2 exposure_time_absolute); <=0 = auto exposure (default, same as the SDK model base
    # table). For a fixed exposure give a positive number, e.g. 333 = 33.3ms (30fps upper limit)
    exposure = int(node.declare_parameter("camera_exposure", -1).value)
    buffers = int(node.declare_parameter("camera_buffers", 4).value)
    use_metadata = bool(node.declare_parameter("camera_use_metadata", True).value)
    zero_copy = bool(node.declare_parameter("camera_zero_copy", True).value)
    frame_id = str(node.declare_parameter("camera_frame_id", "").value).strip()
    # Calibrated values: **empty string = not calibrated**, no numeric sentinel. A pts_offset calibration of 0
    # is legal (PTS falls exactly on the sampling time), and so are negative values (PTS later than the
    # sampling time), so neither -1 nor 0 can mean "not set"; readout also uses the empty string, for
    # consistency.
    pts_offset_ms = _optional_float(node, "camera_pts_offset_ms")
    readout_ms = _optional_float(node, "camera_readout_ms")
    if readout_ms is not None and readout_ms <= 0:
        raise ValueError(f"参数 camera_readout_ms 必须 > 0，收到 {readout_ms}")
    model = str(node.declare_parameter("camera_model", "").value).strip()
    overlay = _parse_control_overlay(
        # The default must be [""], not []: rclpy infers the type from the default, an empty list is inferred
        # as BYTE_ARRAY, and a string array passed later is then rejected by InvalidParameterTypeException
        # before parsing. tracker_lighthouses / tracker_survive_args are written the same way.
        node.declare_parameter("camera_controls", [""]).value or [])

    # The base table (all controls managed for this model) is fixed only after start() identifies the model;
    # only overrides are given here, None = this control is not managed
    overrides = {}
    if exposure > 0:
        overrides["auto_exposure"] = _MANUAL_EXPOSURE
        overrides["exposure_time_absolute"] = exposure
    else:
        overrides["auto_exposure"] = _AUTO_EXPOSURE
        # Under auto exposure exposure_time_absolute is INACTIVE: keeping it would always read back mismatched
        # and log a warning every time the node starts
        overrides["exposure_time_absolute"] = None
    overrides.update(overlay)
    params = {
        "mode": mode,
        "width": width, "height": height, "fps": fps, "fourcc": fourcc,
        "exposure": exposure, "model": model or None,
        "controls": None, "control_overrides": overrides,
        "buffers": buffers, "use_metadata": use_metadata,
        "zero_copy": zero_copy,
        "frame_id": frame_id or None,
        "pts_offset_ms": pts_offset_ms, "readout_ms": readout_ms,
        "strict_controls": bool(node.declare_parameter("camera_strict_controls", False).value),
    }
    if mode:                                   # explicit mode: reject bad parameters before the glove connects
        check_camera_params(params)
    return params


def check_camera_params(params: dict) -> None:
    """Validity of the imaging parameters. Not checked in ``off`` mode: a machine without a camera should not
    be blocked by camera parameters. When ``camera_mode`` is empty, the node calls this again after product
    model identification has decided the mode."""
    if params["mode"] in ("", "off"):
        return
    for name in ("width", "height", "fps", "buffers"):
        if params[name] <= 0:
            raise ValueError(f"参数 camera_{name} 必须 > 0，收到 {params[name]}")
    if len(params["fourcc"]) != 4:
        raise ValueError(f"参数 camera_fourcc 要 4 个字符的 fourcc（MJPG / YUYV），收到 {params['fourcc']!r}")
    model = params.get("model")
    if model:
        from revohuman.camera import models as _models
        if model not in _models.names():
            raise ValueError(f"参数 camera_model 不是已登记的相机型号：{model!r}；"
                             f"可用：{list(_models.names())}（留空 = 按设备自动认）")


def _optional_float(node, name: str):
    """Empty string = not configured. A ROS double parameter has no "unset" state, and 0 and negative numbers
    are both legal calibration values."""
    raw = str(node.declare_parameter(name, "").value).strip()
    if not raw:
        return None
    try:
        return float(raw)
    except ValueError:
        raise ValueError(f"参数 {name} 要是数字或空串（空 = 未标定），收到 {raw!r}") from None


def _parse_control_overlay(items) -> dict:
    """`["gamma=120", "sharpness=3"]` → `{"gamma": 120, "sharpness": 3}`, layered on the model's base table.

    Names are validated here: if a misspelled control name were only caught in `_apply_controls`, it would
    just become a `设备不支持` (not supported by the device) line in stats while the node starts normally; a
    configuration typo and a device that really lacks the control would look exactly the same from outside."""
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
    """Camera locating parameters for one hand: `<prefix>device` / `<prefix>meta_device` / `<prefix>usb_hub`;
    empty string = automatic (device automatic = search sysfs by HUB; usb_hub automatic = the HUB of this
    hand's MCU, see resolve_hand_hub)."""
    out = {}
    for key in ("device", "meta_device", "usb_hub"):
        val = str(node.declare_parameter(prefix + key, "").value).strip()
        out[key] = val or None
    return out


def resolve_hand_hub(handedness, sysfs_root=None,
                     warn: Optional[Callable[[str], None]] = None) -> Optional[str]:
    """HUB of this hand's glove unit (unit_id, e.g. "3-1"): the camera follows the MCU (`identity.hand_hub`;
    handedness comes only from the MCU firmware: recorded handedness > serial number). None = no evidence at
    all, the camera is the first one found on the machine (the older single-glove usage); this hand cannot be
    identified while the other hand can → raises `identity.HandLocateError`."""
    from revohuman import Handedness, identity
    hub, note = identity.hand_hub(Handedness(handedness), sysfs_root=sysfs_root)
    if note and warn is not None:
        warn(f"{handedness} 手相机 HUB：{note}")
    return hub


def create_camera(node, params: dict, hand: dict, handedness: str,
                  hub_resolver: Optional[Callable[[], Optional[str]]] = None,
                  extra_kwargs: Optional[dict] = None,
                  on_error: Optional[Callable[[BaseException], None]] = None):
    """Constructs `WristCamera` (**does not start it**). On failure, auto mode returns None (the glove still
    publishes) and required mode raises. `extra_kwargs` lets tests inject `capture_factory=` (simulated
    capture, does not touch /dev/video*). `on_error(exc)`: on failure the original exception is first handed
    to the caller (so it can report the real cause; `CameraNotFound.probe` carries the HUB inventory), then
    None is returned / the exception raised according to the mode.

    Construction and start are two steps, as for tracker: subscriptions must be attached before start();
    subscribing after it has started only receives the tail."""
    try:
        from revohuman.camera import CameraConfig, WristCamera   # lazy: not touched at all when camera_mode=off
        from revohuman.identity import HandLocateError
        usb_hub = hand["usb_hub"]
        if usb_hub is None and hand["device"] is None and hub_resolver is not None:
            try:
                usb_hub = hub_resolver()
            except HandLocateError:
                raise   # hand not identified / absent: falling back to "any HUB" would get the other hand's
                        # camera, so do not open (in auto mode the glove still publishes)
            except Exception as exc:  # noqa: BLE001 —— sysfs read failure etc.: treat as "any HUB" and say so
                node.get_logger().warning(f"{handedness} 手相机 HUB 自动解析失败：{format_error(exc)}")
        extra_cfg = {"usb_ids": tuple(hand["usb_ids"])} if hand.get("usb_ids") else {}
        cfg = CameraConfig(
            **extra_cfg,
            device=hand["device"], meta_device=hand["meta_device"], use_metadata=params["use_metadata"],
            usb_hub=usb_hub, handedness=handedness,
            width=params["width"], height=params["height"], fps=params["fps"], fourcc=params["fourcc"],
            model=params.get("model"),
            controls=params.get("controls"), control_overrides=params.get("control_overrides") or {},
            buffers=params["buffers"],
            strict_controls=params.get("strict_controls", False),
            pts_offset_ns=(None if params.get("pts_offset_ms") is None
                           else int(params["pts_offset_ms"] * 1e6)),
            readout_ns=(None if params.get("readout_ms") is None
                        else int(params["readout_ms"] * 1e6)))
        kwargs = dict(config=cfg, name=f"wrist_camera_{handedness}")
        kwargs.update(extra_kwargs or {})
        return WristCamera(**kwargs)
    except Exception as exc:  # noqa: BLE001 —— ImportError / invalid parameters
        return _fail(node, params, handedness, exc, on_error)


def start_camera(node, cam, params: dict, handedness: str,
                 on_error: Optional[Callable[[BaseException], None]] = None) -> bool:
    """Starts capture. True = started; False = did not start in auto mode (the caller tears down the camera
    half); required mode raises. `on_error` as in `create_camera`."""
    try:
        cam.start()
        return True
    except Exception as exc:  # noqa: BLE001 —— FileNotFoundError (camera not found) / OSError (busy) …
        _fail(node, params, handedness, exc, on_error)
        return False


def _fail(node, params: dict, handedness: str, exc: BaseException,
          on_error: Optional[Callable[[BaseException], None]] = None):
    if on_error is not None:
        try:
            on_error(exc)
        except Exception as cb_exc:  # noqa: BLE001 —— a callback error must not mask the original exception
            node.get_logger().warning(f"{handedness} 手腕部相机 on_error 回调失败：{format_error(cb_exc)}")
    msg = f"{handedness} 手腕部相机启动失败：{format_error(exc)}"
    if params["mode"] == "required":
        node.get_logger().error(msg)
        raise exc
    node.get_logger().warning(msg + "（camera_mode=auto：手套照发，相机话题不建）")
    return None


def image_format(fourcc: str) -> str:
    return _FORMATS.get(str(fourcc).upper(), str(fourcc).lower())


def make_compressed_image(frame, frame_id: str) -> CompressedImage:
    """CameraFrame / LoanedFrame → sensor_msgs/CompressedImage. The only copy is in `frombytes`: memoryview
    (zero-copy frame) and bytes (copied frame) both go through the buffer protocol, one memcpy."""
    msg = CompressedImage()
    msg.header.stamp = C.to_stamp(frame.sample_wall_ns)
    msg.header.frame_id = frame_id
    msg.format = image_format(frame.fourcc)
    buf = array.array("B")
    buf.frombytes(frame.data)
    msg.data = buf          # array.array('B') takes the setter's fast path, no per-byte assertion
    return msg


_STREAM_KEYS = (
    "frames_received", "frames_delivered", "frames_unmapped", "frames_lost_driver", "frames_error",
    "frames_bad_jpeg", "frames_meta_missing", "frames_dropped_local", "meta_without_pts", "read_timeouts",
    "effective_fps", "period_ms", "recv_minus_sample_ms", "bytes_per_frame_mean", "last_frame_age_s",
    "stalled", "uptime_s", "stamp_sources", "exposure_us", "exposure_mode", "sample_phase_bound_ms", "controls",
    "stamp_model", "pts_offset_ms", "readout_ms", "stamp_assumption", "stamp_fallback", "stamp_uncertainty_ms",
    "subscribers", "loaned_subscribers", "queue_len", "loans_delivered", "loan",
    "fatal_error", "device", "meta_device", "usb_path", "model", "vid_pid",
    # camera firmware version (raw USB bcdDevice string, e.g. "1008")
    "usb_bcd_device",
    # metadata missing: video frames are arriving but the clock has not converged for meta_missing_s in a
    # row; frames are still not delivered, and the host application reports "metadata missing" from this
    # instead of "never received an image"
    "meta_buffers", "meta_missing", "meta_missing_reason", "meta_missing_age_s",
)


def status_dict(cam, counters: Optional[dict] = None) -> dict:
    """1Hz status: capture counters + clock mapping quality + zero-copy accounting
    (`stream.loan.outstanding/starved`) + node counters. Reports only, does not judge: `stream.stalled` /
    `stream.fatal_error` are the raw input for the host application's liveness check."""
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
                             "use_metadata", "meta_missing_s", "pts_offset_ns", "readout_ns", "model_timing",
                             "stamp_model", "effective_pts_offset_ns", "effective_readout_ns")},
    }
    if counters:
        out["node"] = dict(counters)
    return out


def status_payload(cam, counters: Optional[dict] = None) -> str:
    return json.dumps(status_dict(cam, counters), ensure_ascii=False, default=str)
