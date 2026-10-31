"""两节点共用的产品型号识别：按序列号规则 + 在位硬件判 P1 / PTV1，决定启用哪些传感器。

``product``：``auto``（默认，按证据判）/ ``P1`` / ``PTV1``（声明，与证据相反拒绝启动）/
``off``（不识别，完全按 ``publish_tactile`` / ``tracker_mode`` / ``camera_mode`` 参数走）。

型号决定的是**默认启用哪些传感器**，显式参数永远优先：

- 触觉：型号带触觉而某只手读不到触觉 → 拒绝启动（``product_strict=false`` 时只告警）；
  ``publish_tactile=off`` 表示不要触觉，不做这条检查；
- 腕部相机：``camera_mode`` 留空时 PTV1 = ``required``、P1 = ``off``；显式给值则按给的走；
- tracker：两个型号都带，由 ``tracker_mode``（默认 ``required``）与序列号解析把关。

识别只读：看 USB 拓扑、MCU 序列号与已连手套的固件触觉型号，从不写设备。
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
    """已连上的手套 → ``ProductIdentity``；``product=off`` 时返回 None。

    ``gloves`` = ``{Handedness: Glove}``。触觉型号取自已连手套，不再二次打开串口。
    识别到的串口与已连串口不一致（识别期间设备重枚举）→ ``RevoHumanError``。"""
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
    """``camera_mode`` 留空时按型号定：带腕部相机的型号 = ``required``，其余 = ``off``。"""
    if camera_mode:
        return camera_mode
    if ident is None:
        return "off"
    from revohuman.product import Sensor
    return "required" if Sensor.WRIST_CAMERA in ident.sensors else "off"


def check_sensors(node, params: dict, ident, *, tactile: bool, camera_mode: str) -> None:
    """型号应有、本次启用、但在位硬件缺的传感器：严格档抛 ``ProductMismatch``，否则告警。

    只查触觉与相机；tracker 由 ``tracker_mode`` 与序列号解析把关。"""
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
    """身份识别找到的该手相机位置：``{"usb_hub": ..., "usb_ids": (...)}``，没有就空。"""
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
    """``~/status`` 里的 ``product`` 段。"""
    if ident is None:
        return None
    try:
        return {"product": ident.to_dict()}
    except Exception as exc:  # noqa: BLE001
        return {"product": {"error": format_error(exc)}}
