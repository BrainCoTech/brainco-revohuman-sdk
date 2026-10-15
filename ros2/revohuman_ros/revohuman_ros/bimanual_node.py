"""bimanual_node — 双手硬件同步（ns=/revohuman/pair）。

话题（均在节点命名空间下）：

- ``joint_states``（左 21 + 右 21 = 42 关节；``header.stamp`` 取左手帧的采样时刻）
- ``tactile_left`` / ``tactile_right``：两手都带触觉才创建；任一侧本拍触觉缺失时整拍
  （含 ``joint_states``）不发，恢复后自动续发
- ``status``（String+JSON，1 Hz）：两手各自的采集计数与录制门边沿、配对统计、产品型号
- ``tracker_pose_left`` / ``tracker_pose_right`` / ``tracker_delta_left`` / ``tracker_delta_right`` /
  ``tracker_status``：tracker 在时（tracker 与手套各自配对，下游按 ``header.stamp`` 对齐）
- ``wrist_camera_{left,right}/image/compressed`` / ``wrist_camera_status``：腕部相机在时

两手型号不一致 → 启动失败。配对精度达不到 ``pair_accuracy_ms`` 的对子不产生；
预算不可满足在启动时抛 ``ValueError``。服务 ``left/…`` 与 ``right/…`` 各一套。
"""

from __future__ import annotations

import json
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import String

from revohuman import GloveManager, Handedness, TactileUndetermined, identity
from revohuman_msgs.msg import TactileArray

from . import _camera as K
from . import _common as C
from . import _product as P
from . import _tracker as T
from .glove_node import shutdown_node, spin_until_exit


class BimanualNode(Node):
    def __init__(self, *, connect_kwargs: dict | None = None, tracker_kwargs: dict | None = None,
                 camera_kwargs: dict | None = None, calibration_kwargs: dict | None = None,
                 node_name: str = "bimanual_node", namespace: str | None = "/revohuman/pair",
                 **node_kwargs):
        super().__init__(node_name, namespace=namespace, **node_kwargs)
        # 非 None 时 main() 退出主循环并以此为进程退出码（5 = 切换 tracker 模式，由 launch 重拉）
        self.exit_code = None
        self.tracker_rt = None
        self.tracker_calibration = None
        self.tracker_services = None
        self.product_identity = None
        C.install_sdk_log_bridge(self)
        try:
            self._init(connect_kwargs, tracker_kwargs, camera_kwargs, calibration_kwargs)
        except Exception:
            self.destroy_node()
            raise

    def _init(self, connect_kwargs: dict | None, tracker_kwargs: dict | None,
              camera_kwargs: dict | None = None, calibration_kwargs: dict | None = None) -> None:
        self.set_name = self.declare_parameter("set_name", "").value or None
        self.registry_path = self.declare_parameter("registry_path", "").value or None
        self.rate_hz = float(self.declare_parameter("rate_hz", 200.0).value)
        self.pair_mode = C.check_choice(
            "pair_mode", self.declare_parameter("pair_mode", "strict").value, C.PAIR_MODES)
        # 配对精度预算（毫秒）。负数 = 跟 pair_mode：strict → 0.0（只收硬件同步下同一采样刻的对）；
        # best_effort → 1.0（允许两侧各自的绝对精度叠加）
        pa = float(self.declare_parameter("pair_accuracy_ms", -1.0).value)
        self.pair_accuracy_ms = pa if pa >= 0 else (0.0 if self.pair_mode == "strict" else 1.0)
        # tracker 是两个自由运行的 IMU，不跟 pair_mode 走；负数 = 不设预算
        tpa = float(self.declare_parameter("tracker_pair_accuracy_ms", 1.0).value)
        self.tracker_pair_accuracy_ms = tpa if tpa >= 0 else None
        self.publish_tactile = C.check_choice(
            "publish_tactile", self.declare_parameter("publish_tactile", "auto").value, C.PUBLISH_TACTILE)
        self.require_tactile = bool(self.declare_parameter("require_tactile", False).value)
        self.allow_calibration = bool(self.declare_parameter("allow_calibration", False).value)
        left_port = self.declare_parameter("left_port", "").value or None
        right_port = self.declare_parameter("right_port", "").value or None
        sysfs_root = self.declare_parameter("sysfs_root", "").value or None
        self.frame_id = self.declare_parameter("frame_id", "revohuman_pair").value
        mgr = GloveManager(registry_path=self.registry_path, sysfs_root=sysfs_root)
        self.product_params = P.declare_product_params(self)
        self.tracker_params = T.declare_tracker_params(self)
        self.camera_params = K.declare_camera_params(self)
        self.camera_hands = {hand: K.declare_camera_hand_params(self, f"camera_{hand}_")
                             for hand in ("left", "right")}
        mode = self.tracker_params["mode"]

        def _resolver(hand):
            return lambda: identity.resolve_tracker_serial(
                hand, self.set_name, registry=mgr.registry, sysfs_root=sysfs_root)

        self.tracker_left_serial = T.require_serial(
            self, "tracker_left_serial", mode, resolver=_resolver(Handedness.LEFT))
        self.tracker_right_serial = T.require_serial(
            self, "tracker_right_serial", mode, resolver=_resolver(Handedness.RIGHT))
        if mode != "off" and self.tracker_left_serial == self.tracker_right_serial:
            raise ValueError("tracker_left_serial 与 tracker_right_serial 不能是同一只")

        # —— 基站几何标定代：本进程不连手套，标定结束后以退出码 5 退出 ——
        if mode != "off" and T.calibration_pending(self.tracker_params):
            self.tracker_calibration = T.CalibrationGeneration(
                self, {self.tracker_left_serial: Handedness.LEFT,
                       self.tracker_right_serial: Handedness.RIGHT},
                self.tracker_params, calibration_kwargs)
            return

        kwargs = dict(
            rate_hz=self.rate_hz, require_tactile=self.require_tactile,
            sync=True if self.pair_mode == "strict" else "best-effort",
        )
        kwargs.update(connect_kwargs or {})
        # VariantMismatch / TactileUnsupported / TactileUndetermined 在这里直接抛 → 启动失败
        self.pair = mgr.connect_pair(set_name=self.set_name, left_port=left_port,
                                     right_port=right_port, **kwargs)

        # —— 产品型号：决定默认启用哪些传感器 ——
        self.product_identity = P.identify_connected(
            self, self.product_params,
            {Handedness.LEFT: self.pair.left, Handedness.RIGHT: self.pair.right},
            set_name=self.set_name, registry=mgr.registry, sysfs_root=sysfs_root)
        self.camera_params["mode"] = P.resolve_camera_mode(self.camera_params["mode"],
                                                           self.product_identity)
        K.check_camera_params(self.camera_params)
        P.check_sensors(self, self.product_params, self.product_identity,
                        tactile=self.publish_tactile == "auto",
                        camera_mode=self.camera_params["mode"])
        hints = {hand: P.camera_hint(self.product_identity, hand) for hand in ("left", "right")}
        for hand, hint in hints.items():
            if hint.get("usb_ids") and not self.camera_hands[hand].get("usb_ids"):
                self.camera_hands[hand]["usb_ids"] = hint["usb_ids"]
        self._camera_hub_resolver = lambda hand: (
            lambda: hints[hand].get("usb_hub") or K.resolve_hand_hub(
                mgr.registry, hand, self.set_name, sysfs_root=sysfs_root,
                warn=self.get_logger().warning))

        self.names = C.joint_names("left") + C.joint_names("right")
        self.pub_joints = self.create_publisher(JointState, "joint_states", C.DATA_QOS)
        self.pub_tactile_left = self.pub_tactile_right = None
        both_tactile = False
        if self.publish_tactile == "auto":
            try:
                both_tactile = bool(self.pair.left.has_tactile) and bool(self.pair.right.has_tactile)
            except TactileUndetermined as exc:
                self.pair.close()
                raise RuntimeError(f"触觉型号未判定，拒绝启动：{exc}") from exc
            if both_tactile:
                self.pub_tactile_left = self.create_publisher(TactileArray, "tactile_left", C.DATA_QOS)
                self.pub_tactile_right = self.create_publisher(TactileArray, "tactile_right", C.DATA_QOS)
        # 触觉门 = 两手都带触觉且要发触觉；publish_tactile=off 时不开门（关节照发）
        self.syn = self.pair.synchronizer(
            mode="interpolate", tactile_gate=both_tactile,
            pair_accuracy_ms=self.pair_accuracy_ms,
            on_tactile_gate=self._on_gate)
        self.get_logger().info(
            f"left={self.pair.left.port} right={self.pair.right.port} "
            f"触觉型号={self.pair.left.tactile_state.value}/{self.pair.right.tactile_state.value} "
            f"tactile_topics={'on' if both_tactile else 'off'} pair_mode={self.pair_mode} "
            f"synchronized={self.pair.synchronized} pair_accuracy_ms={self.pair_accuracy_ms} "
            f"calibration_srv={'on' if self.allow_calibration else 'off'}")

        self.pub_status = self.create_publisher(String, "status", C.STATUS_QOS)
        self._status_timer = self.create_timer(1.0, self._publish_status)

        self.pairs_published = 0
        self.pairs_no_stamp = 0
        self.tactile_published = 0
        self._services = (
            C.register_glove_services(self, self.pair.left, "left/",
                                      allow_calibration=self.allow_calibration)
            + C.register_glove_services(self, self.pair.right, "right/",
                                        allow_calibration=self.allow_calibration))
        self._sub = self.syn.subscribe(callback=self._on_pair)

        # —— tracker：同进程、同一条时间轴 ——
        self.tracker_src = self.tracker_syn = None
        self._tracker_sub = self._tracker_timer = None
        self.pub_tracker_left = self.pub_tracker_right = self.pub_tracker_status = None
        self.pub_tracker_delta_left = self.pub_tracker_delta_right = None
        self.tracker_pairs_published = 0
        self.tracker_delta_pairs_published = 0
        self.tracker_pairs_no_stamp = 0
        if mode != "off":
            self._tracker_bringup(tracker_kwargs)

        # —— 腕部相机：每手一台，不做左右配对 ——
        self.cameras = {}
        self._camera_subs = {}
        self.pub_camera = {}
        self.pub_camera_status = None
        self._camera_timer = None
        self.camera_published = {"left": 0, "right": 0}
        self.camera_publish_errors = {"left": 0, "right": 0}
        fid = self.camera_params["frame_id"]
        self.camera_frame_ids = {hand: (f"{fid}_{hand}" if fid else f"revohuman_{hand}_wrist_camera")
                                 for hand in ("left", "right")}
        if self.camera_params["mode"] != "off":
            self._camera_bringup(camera_kwargs)

    # ------------------------------------------------------------ tracker

    def _tracker_bringup(self, tracker_kwargs) -> None:
        rt = T.create_tracker_runtime(
            self, {"left": self.tracker_left_serial, "right": self.tracker_right_serial},
            self.tracker_params, tracker_kwargs)
        if rt is None:
            return
        self.tracker_rt = rt
        self.pub_tracker_left = self.create_publisher(PoseStamped, "tracker_pose_left", C.DATA_QOS)
        self.pub_tracker_right = self.create_publisher(PoseStamped, "tracker_pose_right", C.DATA_QOS)
        self.pub_tracker_delta_left = self.create_publisher(PoseStamped, "tracker_delta_left", C.DATA_QOS)
        self.pub_tracker_delta_right = self.create_publisher(PoseStamped, "tracker_delta_right", C.DATA_QOS)
        self.pub_tracker_status = self.create_publisher(String, "tracker_status", T.STATUS_QOS)

        def attach(src):
            from revohuman.tracker import BimanualTrackerSynchronizer
            T.install_world_logger(self, src)
            self.tracker_syn = BimanualTrackerSynchronizer(
                src.hand(Handedness.LEFT), src.hand(Handedness.RIGHT), mode="interpolate",
                pair_accuracy_ms=self.tracker_pair_accuracy_ms)
            # 订阅先于采集开始：起完再订只收得到尾巴
            self._tracker_sub = self.tracker_syn.subscribe(callback=self._on_tracker_pair)

        if not T.start_tracker_runtime(self, rt, self.tracker_params, before_start=attach):
            self._tracker_teardown()
            return
        self.tracker_src = rt.source
        self.tracker_services = T.TrackerServices(self, rt, self.tracker_params)
        self._tracker_timer = self.create_timer(1.0, self._tracker_tick)
        self.get_logger().info(
            f"tracker left={self.tracker_left_serial} right={self.tracker_right_serial} "
            f"lighthouses={list(rt.registered)} frame_id={self.tracker_params['frame_id']}")

    def _tracker_teardown(self) -> None:
        for closer in (lambda: self.tracker_calibration.close(),
                       lambda: self.tracker_services.close(),
                       lambda: self._tracker_sub.close(), lambda: self.tracker_syn.close(),
                       lambda: self.tracker_rt.close(),
                       lambda: self.destroy_timer(self._tracker_timer),
                       lambda: self.destroy_publisher(self.pub_tracker_left),
                       lambda: self.destroy_publisher(self.pub_tracker_right),
                       lambda: self.destroy_publisher(self.pub_tracker_delta_left),
                       lambda: self.destroy_publisher(self.pub_tracker_delta_right),
                       lambda: self.destroy_publisher(self.pub_tracker_status)):
            try:
                closer()
            except Exception:  # noqa: BLE001
                pass
        self.tracker_calibration = self.tracker_services = None
        self.tracker_src = self.tracker_rt = self.tracker_syn = None
        self._tracker_sub = self._tracker_timer = None
        self.pub_tracker_left = self.pub_tracker_right = self.pub_tracker_status = None
        self.pub_tracker_delta_left = self.pub_tracker_delta_right = None

    # 在 tracker 配对器 Subscription 的回调线程里跑
    def _on_tracker_pair(self, pf) -> None:
        if pf.left.pose is None or pf.right.pose is None:
            return
        sl = T.frame_stamp_ns(pf.left)
        sr = T.frame_stamp_ns(pf.right)
        if sl is None or sr is None:
            self.tracker_pairs_no_stamp += 1
            return
        fid = self.tracker_params["frame_id"]
        self.pub_tracker_left.publish(T.make_pose_stamped(sl, fid, pf.left.pose))
        self.pub_tracker_right.publish(T.make_pose_stamped(sr, fid, pf.right.pose))
        self.tracker_pairs_published += 1
        rt = self.tracker_rt
        if rt is None or self.pub_tracker_delta_left is None:
            return
        # 零位相对位姿两手同发同不发：只标定了一只手（不可能出现）或位姿无效时整对不发
        ref = rt.reference
        dl = ref.delta("left", pf.left.pose)
        dr = ref.delta("right", pf.right.pose)
        if dl is None or dr is None:
            return
        self.pub_tracker_delta_left.publish(T.make_pose_stamped(sl, T.delta_frame_id("left"), dl))
        self.pub_tracker_delta_right.publish(T.make_pose_stamped(sr, T.delta_frame_id("right"), dr))
        self.tracker_delta_pairs_published += 1

    def _tracker_tick(self) -> None:
        if self.tracker_services is None:
            return
        self.tracker_services.evaluate_geometry()
        msg = String()
        msg.data = T.status_payload(
            self.tracker_src, sync=self.tracker_syn,
            counters={"tracker_pairs_published": self.tracker_pairs_published,
                      "tracker_delta_pairs_published": self.tracker_delta_pairs_published,
                      "tracker_pairs_no_stamp": self.tracker_pairs_no_stamp,
                      "pair_mode": self.pair_mode,
                      "pair_accuracy_ms": self.pair_accuracy_ms,
                      "tracker_pair_accuracy_ms": self.tracker_pair_accuracy_ms},
            session=self.tracker_services.status())
        self.pub_tracker_status.publish(msg)

    # ------------------------------------------------------------ 腕部相机

    def _camera_bringup(self, camera_kwargs) -> None:
        for hand in ("left", "right"):
            cam = K.create_camera(self, self.camera_params, self.camera_hands[hand], hand,
                                  hub_resolver=self._camera_hub_resolver(hand), extra_kwargs=camera_kwargs)
            if cam is None:
                continue        # auto 档：这只手没相机，另一只照起
            pub = self.create_publisher(CompressedImage, f"wrist_camera_{hand}/image/compressed", C.DATA_QOS)
            sub = cam.subscribe(callback=(lambda frame, h=hand: self._on_camera_frame(h, frame)),
                                loaned=self.camera_params["zero_copy"])
            if not K.start_camera(self, cam, self.camera_params, hand):
                for closer in (sub.close, cam.stop, lambda: self.destroy_publisher(pub)):
                    try:
                        closer()
                    except Exception:  # noqa: BLE001
                        pass
                continue
            self.cameras[hand] = cam
            self._camera_subs[hand] = sub
            self.pub_camera[hand] = pub
            self.get_logger().info(
                f"wrist_camera_{hand}={cam.resolved.get('device')} meta={cam.resolved.get('meta_device')} "
                f"hub={cam.resolved.get('hub')} {self.camera_params['width']}x{self.camera_params['height']} "
                f"{self.camera_params['fourcc']}@{self.camera_params['fps']} "
                f"zero_copy={'on' if self.camera_params['zero_copy'] else 'off'} "
                f"frame_id={self.camera_frame_ids[hand]}")
        if self.cameras:
            self.pub_camera_status = self.create_publisher(String, "wrist_camera_status", T.STATUS_QOS)
            self._camera_timer = self.create_timer(1.0, self._publish_camera_status)

    def _camera_teardown(self) -> None:
        subs = list(getattr(self, "_camera_subs", {}).values())
        cams = list(getattr(self, "cameras", {}).values())
        pubs = list(getattr(self, "pub_camera", {}).values())
        closers = ([s.close for s in subs] + [c.stop for c in cams]
                   + [lambda: self.destroy_timer(self._camera_timer)]
                   + [(lambda p=p: self.destroy_publisher(p)) for p in pubs]
                   + [lambda: self.destroy_publisher(self.pub_camera_status)])
        for closer in closers:
            try:
                closer()
            except Exception:  # noqa: BLE001
                pass
        self.cameras, self._camera_subs, self.pub_camera = {}, {}, {}
        self._camera_timer = self.pub_camera_status = None

    # 在该手相机订阅的派发线程里跑。零拷贝帧无论发没发成功都要归还
    def _on_camera_frame(self, hand: str, frame) -> None:
        try:
            self.pub_camera[hand].publish(K.make_compressed_image(frame, self.camera_frame_ids[hand]))
            self.camera_published[hand] += 1
        except Exception as exc:  # noqa: BLE001 —— 收尾期 publisher 已销毁等
            self.camera_publish_errors[hand] += 1
            self.get_logger().warning(f"{hand} 相机帧发布失败：{exc!r}", throttle_duration_sec=5.0)
        finally:
            release = getattr(frame, "release", None)
            if release is not None:
                release()

    def _publish_camera_status(self) -> None:
        body = {"zero_copy": self.camera_params["zero_copy"]}
        for hand, cam in self.cameras.items():
            body[hand] = K.status_dict(
                cam, counters={"camera_published": self.camera_published[hand],
                               "camera_publish_errors": self.camera_publish_errors[hand],
                               "frame_id": self.camera_frame_ids[hand]})
        msg = String()
        msg.data = json.dumps(body, ensure_ascii=False, default=str)
        self.pub_camera_status.publish(msg)

    # ------------------------------------------------------------ 手套

    def _on_gate(self, is_open: bool, reason) -> None:
        if is_open:
            self.get_logger().info("触觉门重开：合并话题恢复发布")
        else:
            self.get_logger().warning(f"触觉门关闭（{reason}）：joint_states 与两条触觉话题整拍停发")

    def _on_pair(self, pf) -> None:
        # 精度筛在 SDK 的配对出口（pair_accuracy_ms），这里拿到的对都是达标的
        stamp = C.frame_stamp_ns(pf.left)      # 合并帧的戳取左手的采样时刻
        if stamp is None:
            self.pairs_no_stamp += 1
            return
        if self.pub_tactile_left is not None:
            # 触觉门已保证两侧都在；这里再核一遍，任一侧不可用就整拍不发（不发半套）
            if not (C.tactile_usable(pf.left.tactile) and C.tactile_usable(pf.right.tactile)):
                return
        self.pub_joints.publish(C.make_joint_state(
            stamp, self.frame_id, self.names,
            C.joint_positions_rad(pf.left) + C.joint_positions_rad(pf.right)))
        self.pairs_published += 1
        if self.pub_tactile_left is not None:
            self.pub_tactile_left.publish(
                C.make_tactile(pf.left, pf.left.tactile, stamp, self.frame_id))
            self.pub_tactile_right.publish(
                C.make_tactile(pf.right, pf.right.tactile, stamp, self.frame_id))
            self.tactile_published += 1

    def _publish_status(self) -> None:
        msg = String()
        counters = {"pairs_published": self.pairs_published,
                    "pairs_no_stamp": self.pairs_no_stamp,
                    "tactile_published": self.tactile_published,
                    "pair_mode": self.pair_mode,
                    "pair_accuracy_ms": self.pair_accuracy_ms}
        body = json.loads(C.glove_status(self.pair.left, counters=counters,
                                         pairing=self.syn.stats(),
                                         extra=P.status_extra(self.product_identity)))
        # 两手各一份档位账：右手退档同样会让对子消失
        body["right"] = json.loads(C.glove_status(self.pair.right))
        msg.data = json.dumps(body, ensure_ascii=False, default=str)
        self.pub_status.publish(msg)

    def destroy_node(self):
        self._camera_teardown()
        self._tracker_teardown()
        for closer in (lambda: self.destroy_timer(self._status_timer),
                       lambda: self.destroy_publisher(self.pub_status),
                       lambda: self._sub.close(), lambda: self.syn.close(),
                       lambda: self.pair.close()):
            try:
                closer()
            except Exception:  # noqa: BLE001
                pass
        return super().destroy_node()


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = None
    code = 0
    try:
        node = BimanualNode()
        code = spin_until_exit(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:  # noqa: BLE001
        if rclpy.ok():
            print(f"bimanual_node 启动失败：{type(exc).__name__}: {exc}", file=sys.stderr)
            code = 1
    finally:
        shutdown_node(node)
    return code


if __name__ == "__main__":
    sys.exit(main())
