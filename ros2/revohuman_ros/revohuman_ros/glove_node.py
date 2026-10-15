"""glove_node — 单手（ns=/revohuman/{left|right}）。

话题（均在节点命名空间下）：

- ``joint_states``（sensor_msgs/JointState，rad，21 关节）
- ``tactile``（revohuman_msgs/TactileArray）：型号带触觉才创建；本拍触觉不可用时不发
- ``status``（std_msgs/String+JSON，1 Hz，RELIABLE+TRANSIENT_LOCAL）：采集计数、录制门边沿、产品型号
- ``tracker_pose`` / ``tracker_delta`` / ``tracker_status``：tracker 在时
- ``wrist_camera/image/compressed`` / ``wrist_camera/status``：腕部相机在时

``header.stamp`` 一律是采样时刻（CLOCK_REALTIME 纳秒），各模态同一条时间轴。

启动流程：解析 tracker 序列号 → （模式文件为 calibrate 时进入基站几何标定代，见 ``_tracker``）
→ 连手套 → 识别产品型号（``_product``）→ 起 tracker 运行档 → 起腕部相机。
"""

from __future__ import annotations

import signal
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import String

from revohuman import GloveManager, Handedness, TactileUndetermined, identity
from revohuman_msgs.msg import TactileArray

from . import _camera as K
from . import _common as C
from . import _product as P
from . import _tracker as T


class GloveNode(Node):
    def __init__(self, *, connect_kwargs: dict | None = None, tracker_kwargs: dict | None = None,
                 camera_kwargs: dict | None = None, calibration_kwargs: dict | None = None,
                 node_name: str = "glove_node", namespace: str | None = None, **node_kwargs):
        super().__init__(node_name, namespace=namespace, **node_kwargs)
        # 非 None 时 main() 退出主循环并以此为进程退出码（5 = 切换 tracker 模式，由 launch 重拉）
        self.exit_code = None
        self.tracker_rt = None
        self.tracker_calibration = None
        self.tracker_services = None
        self.product_identity = None
        # SDK 的 Python logging 接到本节点 logger 上：构造期的告警也能带上时间戳与级别
        C.install_sdk_log_bridge(self)
        # 构造期任何一步拒启都先把节点销毁再抛：进程内用法下不留活的 DDS participant
        try:
            self._init(connect_kwargs, tracker_kwargs, camera_kwargs, calibration_kwargs)
        except Exception:
            self.destroy_node()
            raise

    def _init(self, connect_kwargs: dict | None, tracker_kwargs: dict | None,
              camera_kwargs: dict | None = None, calibration_kwargs: dict | None = None) -> None:
        hand_param = self.declare_parameter("hand", "").value
        self.hand = C.check_choice("hand", str(hand_param), ("left", "right"))
        self.registry_path = self.declare_parameter("registry_path", "").value or None
        self.rate_hz = float(self.declare_parameter("rate_hz", 200.0).value)
        self.publish_tactile = C.check_choice(
            "publish_tactile", self.declare_parameter("publish_tactile", "auto").value, C.PUBLISH_TACTILE)
        self.require_tactile = bool(self.declare_parameter("require_tactile", False).value)
        # 写设备的标定服务：默认 false 时 calibrate / clear_calibration 根本不注册
        self.allow_calibration = bool(self.declare_parameter("allow_calibration", False).value)
        port = self.declare_parameter("port", "").value or None
        set_name = self.declare_parameter("set_name", "").value or None
        sysfs_root = self.declare_parameter("sysfs_root", "").value or None
        self.frame_id = self.declare_parameter("frame_id", f"revohuman_{self.hand}").value
        mgr = GloveManager(registry_path=self.registry_path, sysfs_root=sysfs_root)
        self.product_params = P.declare_product_params(self)
        self.tracker_params = T.declare_tracker_params(self)
        self.camera_params = K.declare_camera_params(self)
        self.camera_hand = K.declare_camera_hand_params(self, "camera_")
        # tracker 序列号：参数 > 该手手套单元 HUB 上实挂的 tracker > 注册表 tracker 字段
        self.tracker_serial = T.require_serial(
            self, "tracker_serial", self.tracker_params["mode"],
            resolver=lambda: identity.resolve_tracker_serial(
                self.hand, set_name, registry=mgr.registry, sysfs_root=sysfs_root))

        # —— 基站几何标定代：本进程不连手套，标定结束后以退出码 5 退出 ——
        if self.tracker_params["mode"] != "off" and T.calibration_pending(self.tracker_params):
            self.tracker_calibration = T.CalibrationGeneration(
                self, {self.tracker_serial: Handedness(self.hand)}, self.tracker_params,
                calibration_kwargs)
            return

        # —— 连接手套 ——
        kwargs = dict(rate_hz=self.rate_hz, require_tactile=self.require_tactile)
        kwargs.update(connect_kwargs or {})
        if port:
            self.glove = mgr.connect(port=port, **kwargs)
        else:
            self.glove = mgr.connect(Handedness(self.hand), set_name=set_name, **kwargs)

        # —— 产品型号：决定默认启用哪些传感器 ——
        self.product_identity = P.identify_connected(
            self, self.product_params, {Handedness(self.hand): self.glove}, set_name=set_name,
            registry=mgr.registry, sysfs_root=sysfs_root)
        self.camera_params["mode"] = P.resolve_camera_mode(self.camera_params["mode"],
                                                           self.product_identity)
        K.check_camera_params(self.camera_params)
        P.check_sensors(self, self.product_params, self.product_identity,
                        tactile=self.publish_tactile == "auto",
                        camera_mode=self.camera_params["mode"])
        hint = P.camera_hint(self.product_identity, self.hand)
        if hint.get("usb_ids") and not self.camera_hand.get("usb_ids"):
            self.camera_hand["usb_ids"] = hint["usb_ids"]
        self._camera_hub_resolver = lambda: hint.get("usb_hub") or K.resolve_hand_hub(
            mgr.registry, self.hand, set_name, sysfs_root=sysfs_root, warn=self.get_logger().warning)

        # —— 手套话题：型号带触觉才建 tactile ——
        self.names = C.joint_names(self.hand)
        self.pub_joints = self.create_publisher(JointState, "joint_states", C.DATA_QOS)
        self.pub_tactile = None
        if self.publish_tactile == "auto":
            try:
                has = self.glove.has_tactile
            except TactileUndetermined as exc:
                self.glove.close()
                raise RuntimeError(f"触觉型号未判定，拒绝启动：{exc}") from exc
            if has:
                self.pub_tactile = self.create_publisher(TactileArray, "tactile", C.DATA_QOS)
        self.get_logger().info(
            f"hand={self.hand} port={self.glove.port} 触觉型号={self.glove.tactile_state.value} "
            f"tactile_topic={'on' if self.pub_tactile else 'off'} "
            f"calibration_srv={'on' if self.allow_calibration else 'off'}")

        self.pub_status = self.create_publisher(String, "status", C.STATUS_QOS)
        self._status_timer = self.create_timer(1.0, self._publish_status)

        self.frames_published = 0
        self.frames_no_stamp = 0
        self.tactile_published = 0
        self.tactile_skipped = 0
        self._services = C.register_glove_services(
            self, self.glove, allow_calibration=self.allow_calibration)
        self._sub = self.glove.subscribe(callback=self._on_frame)

        # —— tracker：同进程、同一条时间轴 ——
        self.tracker_src = None
        self._tracker_sub = None
        self.pub_tracker = self.pub_tracker_delta = self.pub_tracker_status = None
        self._tracker_timer = None
        self.tracker_published = 0
        self.tracker_delta_published = 0
        self.tracker_no_stamp = 0
        if self.tracker_params["mode"] != "off":
            self._tracker_bringup(tracker_kwargs)

        # —— 腕部相机 ——
        self.camera = None
        self._camera_sub = None
        self.pub_camera = self.pub_camera_status = None
        self._camera_timer = None
        self.camera_published = 0
        self.camera_publish_errors = 0
        self.camera_frame_id = self.camera_params["frame_id"] or f"revohuman_{self.hand}_wrist_camera"
        if self.camera_params["mode"] != "off":
            self._camera_bringup(camera_kwargs)

    # ------------------------------------------------------------ tracker

    def _tracker_bringup(self, tracker_kwargs) -> None:
        rt = T.create_tracker_runtime(self, {self.hand: self.tracker_serial}, self.tracker_params,
                                      tracker_kwargs)
        if rt is None:
            return
        self.tracker_rt = rt
        self.pub_tracker = self.create_publisher(PoseStamped, "tracker_pose", C.DATA_QOS)
        self.pub_tracker_delta = self.create_publisher(PoseStamped, "tracker_delta", C.DATA_QOS)
        self.pub_tracker_status = self.create_publisher(String, "tracker_status", T.STATUS_QOS)

        def attach(src):
            # 订阅先于采集开始：起完再订只收得到尾巴
            T.install_world_logger(self, src)
            self._tracker_sub = src.hand(Handedness(self.hand)).subscribe(
                callback=self._on_tracker_frame)

        if not T.start_tracker_runtime(self, rt, self.tracker_params, before_start=attach):
            self._tracker_teardown()
            return
        self.tracker_src = rt.source
        self.tracker_services = T.TrackerServices(self, rt, self.tracker_params)
        self._tracker_timer = self.create_timer(1.0, self._tracker_tick)
        self.get_logger().info(
            f"tracker={self.tracker_serial} lighthouses={list(rt.registered)} "
            f"frame_id={self.tracker_params['frame_id']}")

    def _tracker_teardown(self) -> None:
        for closer in (lambda: self.tracker_calibration.close(),
                       lambda: self.tracker_services.close(),
                       lambda: self._tracker_sub.close(), lambda: self.tracker_rt.close(),
                       lambda: self.destroy_timer(self._tracker_timer),
                       lambda: self.destroy_publisher(self.pub_tracker),
                       lambda: self.destroy_publisher(self.pub_tracker_delta),
                       lambda: self.destroy_publisher(self.pub_tracker_status)):
            try:
                closer()
            except Exception:  # noqa: BLE001
                pass
        self.tracker_calibration = self.tracker_services = None
        self.tracker_src = self.tracker_rt = self._tracker_sub = self._tracker_timer = None
        self.pub_tracker = self.pub_tracker_delta = self.pub_tracker_status = None

    # 在 tracker Subscription 的回调线程里跑
    def _on_tracker_frame(self, frame) -> None:
        if frame.pose is None:
            return
        stamp = T.frame_stamp_ns(frame)
        if stamp is None:
            self.tracker_no_stamp += 1
            return
        self.pub_tracker.publish(T.make_pose_stamped(stamp, self.tracker_params["frame_id"], frame.pose))
        self.tracker_published += 1
        rt = self.tracker_rt
        if rt is None or self.pub_tracker_delta is None:
            return
        delta = rt.reference.delta(self.hand, frame.pose)
        if delta is not None:
            self.pub_tracker_delta.publish(
                T.make_pose_stamped(stamp, T.delta_frame_id(self.hand), delta))
            self.tracker_delta_published += 1

    def _tracker_tick(self) -> None:
        if self.tracker_services is None:
            return
        self.tracker_services.evaluate_geometry()
        msg = String()
        msg.data = T.status_payload(
            self.tracker_src,
            counters={"tracker_published": self.tracker_published,
                      "tracker_delta_published": self.tracker_delta_published,
                      "tracker_no_stamp": self.tracker_no_stamp},
            session=self.tracker_services.status())
        self.pub_tracker_status.publish(msg)

    # ------------------------------------------------------------ 腕部相机

    def _camera_bringup(self, camera_kwargs) -> None:
        cam = K.create_camera(self, self.camera_params, self.camera_hand, self.hand,
                              hub_resolver=self._camera_hub_resolver, extra_kwargs=camera_kwargs)
        if cam is None:
            return
        self.camera = cam
        self.pub_camera = self.create_publisher(CompressedImage, "wrist_camera/image/compressed", C.DATA_QOS)
        self.pub_camera_status = self.create_publisher(String, "wrist_camera/status", T.STATUS_QOS)
        # 订阅先于 start()；loaned=True 拿驱动缓冲上的视图，回调里发完即归还
        self._camera_sub = cam.subscribe(callback=self._on_camera_frame,
                                         loaned=self.camera_params["zero_copy"])
        if not K.start_camera(self, cam, self.camera_params, self.hand):
            self._camera_teardown()
            return
        self._camera_timer = self.create_timer(1.0, self._publish_camera_status)
        self.get_logger().info(
            f"wrist_camera={cam.resolved.get('device')} meta={cam.resolved.get('meta_device')} "
            f"hub={cam.resolved.get('hub')} {self.camera_params['width']}x{self.camera_params['height']} "
            f"{self.camera_params['fourcc']}@{self.camera_params['fps']} "
            f"zero_copy={'on' if self.camera_params['zero_copy'] else 'off'} frame_id={self.camera_frame_id}")

    def _camera_teardown(self) -> None:
        for closer in (lambda: self._camera_sub.close(), lambda: self.camera.stop(),
                       lambda: self.destroy_timer(self._camera_timer),
                       lambda: self.destroy_publisher(self.pub_camera),
                       lambda: self.destroy_publisher(self.pub_camera_status)):
            try:
                closer()
            except Exception:  # noqa: BLE001
                pass
        self.camera = self._camera_sub = self._camera_timer = None
        self.pub_camera = self.pub_camera_status = None

    # 在相机订阅的派发线程里跑。零拷贝帧无论发没发成功都要归还，否则驱动缓冲越借越少
    def _on_camera_frame(self, frame) -> None:
        try:
            self.pub_camera.publish(K.make_compressed_image(frame, self.camera_frame_id))
            self.camera_published += 1
        except Exception as exc:  # noqa: BLE001 —— 收尾期 publisher 已销毁等
            self.camera_publish_errors += 1
            self.get_logger().warning(f"相机帧发布失败：{exc!r}", throttle_duration_sec=5.0)
        finally:
            release = getattr(frame, "release", None)
            if release is not None:
                release()

    def _publish_camera_status(self) -> None:
        msg = String()
        msg.data = K.status_payload(
            self.camera,
            counters={"camera_published": self.camera_published,
                      "camera_publish_errors": self.camera_publish_errors,
                      "zero_copy": self.camera_params["zero_copy"],
                      "frame_id": self.camera_frame_id})
        self.pub_camera_status.publish(msg)

    # ------------------------------------------------------------ 手套

    # 在 Glove 的派发线程里跑；rclpy publisher 可跨线程调用
    def _on_frame(self, frame) -> None:
        stamp = C.frame_stamp_ns(frame)
        if stamp is None:
            self.frames_no_stamp += 1
            return
        self.pub_joints.publish(C.make_joint_state(stamp, self.frame_id, self.names,
                                                   C.joint_positions_rad(frame)))
        self.frames_published += 1
        if self.pub_tactile is not None:
            t = frame.tactile
            if C.tactile_usable(t):
                self.pub_tactile.publish(C.make_tactile(frame, t, stamp, self.frame_id))
                self.tactile_published += 1
            else:
                self.tactile_skipped += 1

    def _publish_status(self) -> None:
        msg = String()
        msg.data = C.glove_status(self.glove, counters={
            "frames_published": self.frames_published,
            "frames_no_stamp": self.frames_no_stamp,
            "tactile_published": self.tactile_published,
            "tactile_skipped": self.tactile_skipped,
        }, extra=P.status_extra(self.product_identity))
        self.pub_status.publish(msg)

    def destroy_node(self):
        self._camera_teardown()
        self._tracker_teardown()
        for closer in (lambda: self.destroy_timer(self._status_timer),
                       lambda: self.destroy_publisher(self.pub_status),
                       lambda: self._sub.close(), lambda: self.glove.close()):
            try:
                closer()
            except Exception:  # noqa: BLE001
                pass
        return super().destroy_node()


def spin_until_exit(node) -> int:
    """跑到 Ctrl-C 或 ``node.exit_code`` 被置位（tracker 模式切换），返回进程退出码。"""
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and node.exit_code is None:
            executor.spin_once(timeout_sec=0.2)
    finally:
        executor.remove_node(node)
        executor.shutdown()
    return int(node.exit_code or 0)


def shutdown_node(node) -> None:
    """进程收尾：屏蔽 SIGINT → destroy_node → rclpy.shutdown。

    终端 Ctrl+C 发给整个进程组，launch 还会再转发一次。第二下若落在 glove.close() 等读线程
    join 时，KeyboardInterrupt 会打断收尾——流不停、后面的手套没关，手套继续推流，
    下一个打开它的进程读不到 device_info（实测单手节点因此拒绝启动）。
    收尾卡住时 launch 仍会按 SIGTERM → SIGKILL 升级，不会挂死。"""
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    if node is not None:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = None
    code = 0
    try:
        node = GloveNode()
        code = spin_until_exit(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass        # Ctrl-C / SIGINT 走 rclpy 的 shutdown，不是启动失败
    except Exception as exc:  # noqa: BLE001 —— 启动失败要能看见原因（型号不符 / 未注册 / 串口忙）
        if rclpy.ok():
            print(f"glove_node 启动失败：{type(exc).__name__}: {exc}", file=sys.stderr)
            code = 1
        # 否则是 SIGINT 让 context 失效后回调里 publish 抛的异常：收尾，不是启动失败
    finally:
        shutdown_node(node)
    return code


if __name__ == "__main__":
    sys.exit(main())
