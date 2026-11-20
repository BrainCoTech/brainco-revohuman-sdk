"""Product model identification shared by both nodes: decides P1 / PTV1 from serial-number rules plus the
hardware present, and from that which sensors to enable.

``product``: ``auto`` (default, decide from the evidence) / ``P1`` / ``PTV1`` (declared; refuse to start if
the evidence contradicts it) / ``off`` (no identification; follow the ``publish_tactile`` / ``tracker_mode`` /
``camera_mode`` parameters entirely).

The product model decides **which sensors are enabled by default**; explicit parameters always take precedence:

- Tactile: the product model has tactile but a hand reads no tactile data → refuse to start (only a warning
  when ``product_strict=false``); ``publish_tactile=off`` means no tactile is wanted and skips this check;
- Wrist camera: when ``camera_mode`` is empty, PTV1 = ``required`` and P1 = ``off``; an explicit value is
  followed as given;
- tracker: both product models have it; gated by ``tracker_mode`` (default ``required``) and serial-number
  resolution.

Identification is read-only: it looks at the USB topology, the MCU serial number and the firmware tactile
model of the connected gloves, and never writes to the device.
"""

from __future__ import annotations

import os
from typing import Optional

from revohuman.codes import format_error

from . import _common as C

PRODUCT_CHOICES = ("auto", "P1", "PTV1", "off")


def declare_product_params(node) -> dict:
    raw = str(node.declare_parameter("product", "auto").value).strip()
    value = raw.upper() if raw.lower() in ("p1", "ptv1") else raw.lower()
    C.check_choice("product", value, PRODUCT_CHOICES)
    rules = str(node.declare_parameter("product_rules_path", "").value).strip()
    strict = bool(node.declare_parameter("product_strict", True).value)
    return {"product": value, "rules_path": rules or None, "strict": strict}


def identify_connected(node, params: dict, gloves: dict, *, sysfs_root=None):
    """Connected gloves → ``ProductIdentity``; returns None when ``product=off``.

    ``gloves`` = ``{Handedness: Glove}``. The tactile model is taken from the connected gloves; the serial
    port is not opened a second time. If the identified serial port differs from the connected one (the
    device re-enumerated during identification) → ``RevoHumanError``."""
    if params["product"] == "off":
        return None
    from revohuman import product as P
    from revohuman.errors import RevoHumanError

    ident = P.identify(
        params["product"], hands=list(gloves), sysfs_root=sysfs_root, rules_path=params["rules_path"], probe=False,
        tactile={h: g.tactile_state for h, g in gloves.items()})
    for hand, glove in gloves.items():
        rep = ident.hands[hand]
        if rep.port and os.path.realpath(rep.port) != os.path.realpath(glove.port):
            raise RevoHumanError(f"{hand.value} 手：连上的串口 {glove.port} 与身份识别到的 "
                                 f"{rep.port} 不一致（识别期间设备重枚举？），请重启节点")
    node.get_logger().info(
        f"产品型号 {ident.variant.code}（{ident.variant.name}，依据 {ident.source}），"
        f"传感器：{', '.join(sorted(s.value for s in ident.sensors))}")
    for note in ident.notes:
        node.get_logger().warning(f"型号识别：{note}")
    return ident


def resolve_camera_mode(camera_mode: str, ident) -> str:
    """When ``camera_mode`` is empty, decide by product model: a model with a wrist camera = ``required``,
    all others = ``off``."""
    if camera_mode:
        return camera_mode
    if ident is None:
        return "off"
    from revohuman.product import Sensor
    return "required" if Sensor.WRIST_CAMERA in ident.sensors else "off"


def check_sensors(node, params: dict, ident, *, tactile: bool, camera_mode: str) -> None:
    """Sensors that the product model should have and that are enabled this run, but are missing from the
    hardware present: strict mode raises ``ProductMismatch``, otherwise a warning is logged.

    Only tactile and camera are checked; tracker is gated by ``tracker_mode`` and serial-number resolution."""
    if ident is None:
        return
    from revohuman.errors import ProductMismatch
    from revohuman.product import Sensor

    want = set()
    if tactile:
        want.add(Sensor.TACTILE)
    if camera_mode == "required":
        want.add(Sensor.WRIST_CAMERA)
    want &= set(ident.sensors)
    reasons = ident.missing_reasons(want) if want else []
    if not reasons:
        return
    if params["strict"]:
        raise ProductMismatch(ident.variant.code, "传感器不全", reasons)
    for reason in reasons:
        node.get_logger().warning(f"product_strict=false，跳过：{reason}")


def camera_hint(ident, handedness) -> dict:
    """Camera location for this hand found by identification: ``{"usb_hub": ..., "usb_ids": (...)}``;
    empty if there is none."""
    if ident is None:
        return {}
    from revohuman import Handedness
    rep = ident.hands.get(Handedness(handedness))
    if rep is None or not rep.unit_id:
        return {}
    out = {"usb_hub": rep.unit_id}
    if rep.camera_usb_id:
        out["usb_ids"] = (rep.camera_usb_id,)
    return out


def status_extra(ident) -> Optional[dict]:
    """The ``product`` section in ``~/status``."""
    if ident is None:
        return None
    try:
        return {"product": ident.to_dict()}
    except Exception as exc:  # noqa: BLE001
        return {"product": {"error": format_error(exc)}}
