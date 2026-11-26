"""glove_node — single hand (ns=/revohuman/{left|right}).

Topics (all under the node namespace):

- ``joint_states`` (sensor_msgs/JointState, rad, 21 joints)
- ``tactile`` (revohuman_msgs/TactileArray): created only if the product model has tactile; not published
  for a cycle in which tactile is unavailable
- ``status`` (std_msgs/String+JSON, 1 Hz, RELIABLE+TRANSIENT_LOCAL): capture counters, capture gate edges,
  product model
- ``tracker_pose`` / ``tracker_delta`` / ``tracker_status``: when a tracker is present
- ``wrist_camera/image/compressed`` / ``wrist_camera/status``: when a wrist camera is present

``header.stamp`` is always the sampling time (CLOCK_REALTIME nanoseconds); all modalities share one time axis.

Startup sequence: resolve the tracker serial number → (if the mode file is calibrate, enter the base station
geometry calibration generation, see ``_tracker``) → connect the glove → identify the product model
(``_product``) → start the tracker in runtime mode → start the wrist camera.
"""

from __future__ import annotations

import signal
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import String

from revohuman import GloveManager, Handedness, TactileUndetermined
from revohuman_msgs.msg import TactileArray

from . import _camera as K
from . import _common as C
from . import _product as P
from . import _tracker as T


class GloveNode(Node):
    def __init__(self, *, connect_kwargs: dict | None = None, tracker_kwargs: dict | None = None,
                 camera_kwargs: dict | None = None, calibration_kwargs: dict | None = None,
                 manager_kwargs: dict | None = None,
                 node_name: str = "glove_node", namespace: str | None = None, **node_kwargs):
        super().__init__(node_name, namespace=namespace, **node_kwargs)
        # when not None, main() leaves the main loop and uses it as the process exit code (5 = switch tracker
        # mode, relaunched by launch)
        self.exit_code = None
        self.tracker_rt = None
        self.tracker_calibration = None
        self.tracker_services = None
        self.product_identity = None
        # route the SDK's Python logging to this node's logger, so warnings during construction also carry
        # timestamp and level
        C.install_sdk_log_bridge(self)
        # if any construction step refuses to start, destroy the node first and then raise: in-process use must
        # not leave a live DDS participant behind
        try:
            self._init(connect_kwargs, tracker_kwargs, camera_kwargs, calibration_kwargs,
                       manager_kwargs)
        except Exception:
            self.destroy_node()
            raise

    def _init(self, connect_kwargs: dict | None, tracker_kwargs: dict | None,
              camera_kwargs: dict | None = None, calibration_kwargs: dict | None = None,
              manager_kwargs: dict | None = None) -> None:
        hand_param = self.declare_parameter("hand", "").value
        self.hand = C.check_choice("hand", str(hand_param), ("left", "right"))
        self.rate_hz = float(self.declare_parameter("rate_hz", 200.0).value)
        self.publish_tactile = C.check_choice(
            "publish_tactile", self.declare_parameter("publish_tactile", "auto").value, C.PUBLISH_TACTILE)
        self.require_tactile = bool(self.declare_parameter("require_tactile", False).value)
        # split tactile (default false): when enabled and supported by the firmware, ~/tactile is published per
        # sample (module rate, about 215Hz) and frame_stamp no longer matches a joint frame (see
        # TactileArray.msg); joints are unchanged
        self.split_tactile = bool(self.declare_parameter("split_tactile", False).value)
        # calibration services that write to the device: with the default false, calibrate / clear_calibration
        # are not created at all
        self.allow_calibration = bool(self.declare_parameter("allow_calibration", False).value)
        port = self.declare_parameter("port", "").value or None
        sysfs_root = self.declare_parameter("sysfs_root", "").value or None
        self.frame_id = self.declare_parameter("frame_id", f"revohuman_{self.hand}").value
        # manager_kwargs is only for injection by unit tests
        mgr = GloveManager(sysfs_root=sysfs_root, **(manager_kwargs or {}))
        query = T.query_devices_enabled(connect_kwargs, manager_kwargs)
        self.product_params = P.declare_product_params(self)
        self.tracker_params = T.declare_tracker_params(self)
        self.camera_params = K.declare_camera_params(self)
        self.camera_hand = K.declare_camera_hand_params(self, "camera_")
        # tracker serial number: parameter > the tracker physically attached to the HUB of this hand's glove
        # unit (the MCU firmware decides handedness). Before resolving, query handedness from any present glove
        # not queried yet: on recent firmware the serial number carries no handedness, so with a cold cache
        # sysfs alone cannot tell
        self.tracker_serial = T.require_serial(
            self, "tracker_serial", self.tracker_params["mode"],
            resolver=lambda: mgr.resolve_tracker(self.hand, query_devices=query))

        # —— base station geometry calibration generation: this process does not connect the glove and exits
        #    with exit code 5 when calibration ends ——
        if self.tracker_params["mode"] != "off" and T.calibration_pending(self.tracker_params):
            self.tracker_calibration = T.CalibrationGeneration(
                self, {self.tracker_serial: Handedness(self.hand)}, self.tracker_params,
                calibration_kwargs)
            return

        # —— connect the glove ——
        kwargs = dict(rate_hz=self.rate_hz, require_tactile=self.require_tactile,
                      split_tactile=self.split_tactile)
        kwargs.update(connect_kwargs or {})
        if port:
            self.glove = mgr.connect(port=port, **kwargs)
        else:
            self.glove = mgr.connect(Handedness(self.hand), **kwargs)
        try:
            T.confirm_serial(self, "tracker_serial", self.tracker_serial,
                             self.glove.unit_id, sysfs_root=sysfs_root)
        except Exception:
            self.glove.close()
            raise

        # —— product model: decides which sensors are enabled by default ——
        self.product_identity = P.identify_connected(
            self, self.product_params, {Handedness(self.hand): self.glove}, sysfs_root=sysfs_root)
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
            self.hand, sysfs_root=sysfs_root, warn=self.get_logger().warning)

        # —— glove topics: tactile is created only if the product model has tactile ——
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
        # split tactile: tactile is published per sample (the tactile on a frame is only the latest state,
        # _on_frame does not publish it)
        self._tactile_sample_sub = None
        if self.split_tactile and self.pub_tactile is not None:
            self._tactile_sample_sub = self.glove.subscribe_tactile(callback=self._on_tactile_sample)
        self._tare_group = ReentrantCallbackGroup()
        self._services = C.register_glove_services(
            self, self.glove, allow_calibration=self.allow_calibration,
            tare_callback_group=self._tare_group)
        # tactile tare state (latched, published on change plus once at startup): the callback only queues,
        # publishing happens in the executor
        self._tare_state = C.TareStatePublisher(self, {self.hand: self.glove})
        self.glove.on_tactile_tare = self._tare_state.changed
        self._tare_state.changed()
        self._sub = self.glove.subscribe(callback=self._on_frame)

        # —— tracker: same process, same time axis ——
        self.tracker_src = None
        self._tracker_sub = None
        self.pub_tracker = self.pub_tracker_delta = self.pub_tracker_status = None
        self._tracker_timer = None
        self.tracker_published = 0
        self.tracker_delta_published = 0
        self.tracker_no_stamp = 0
        if self.tracker_params["mode"] != "off":
            self._tracker_bringup(tracker_kwargs)

        # —— wrist camera ——
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
            # subscribe before capture starts: subscribing after it has started only receives the tail
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

    # runs in the callback thread of the tracker Subscription
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

    # ------------------------------------------------------------ wrist camera

    def _camera_bringup(self, camera_kwargs) -> None:
        cam = K.create_camera(self, self.camera_params, self.camera_hand, self.hand,
                              hub_resolver=self._camera_hub_resolver, extra_kwargs=camera_kwargs)
        if cam is None:
            return
        self.camera = cam
        self.pub_camera = self.create_publisher(CompressedImage, "wrist_camera/image/compressed", C.DATA_QOS)
        self.pub_camera_status = self.create_publisher(String, "wrist_camera/status", T.STATUS_QOS)
        # subscribe before start(); loaned=True gets a view on the driver buffer, returned right after
        # publishing in the callback
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

    # runs in the dispatch thread of the camera subscription. A zero-copy frame must be returned whether or not
    # publishing succeeded, otherwise the driver runs short of buffers
    def _on_camera_frame(self, frame) -> None:
        try:
            self.pub_camera.publish(K.make_compressed_image(frame, self.camera_frame_id))
            self.camera_published += 1
        except Exception as exc:  # noqa: BLE001 —— publisher already destroyed during shutdown, etc.
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

    # ------------------------------------------------------------ glove

    # runs in the Glove dispatch thread; an rclpy publisher may be called across threads
    def _on_frame(self, frame) -> None:
        stamp = C.frame_stamp_ns(frame)
        if stamp is None:
            self.frames_no_stamp += 1
            return
        self.pub_joints.publish(C.make_joint_state(stamp, self.frame_id, self.names,
                                                   C.joint_positions_rad(frame)))
        self.frames_published += 1
        if self.pub_tactile is not None and self.glove.tactile_transport != "split":
            t = frame.tactile
            if C.tactile_usable(t):
                self.pub_tactile.publish(C.make_tactile(frame, t, stamp, self.frame_id))
                self.tactile_published += 1
            else:
                self.tactile_skipped += 1

    # runs in the Glove dispatch thread (split tactile)
    def _on_tactile_sample(self, sample) -> None:
        if C.sample_publishable(sample):
            self.pub_tactile.publish(C.make_tactile_sample(sample, self.frame_id))
            self.tactile_published += 1
        elif sample.tactile.valid:
            self.tactile_skipped += 1

    def _publish_status(self) -> None:
        msg = String()
        msg.data = C.glove_status(self.glove, counters={
            "frames_published": self.frames_published,
            "frames_no_stamp": self.frames_no_stamp,
            "tactile_published": self.tactile_published,
            "tactile_skipped": self.tactile_skipped,
            "tactile_transport": self.glove.tactile_transport,
        }, extra=P.status_extra(self.product_identity))
        self.pub_status.publish(msg)

    def destroy_node(self):
        self._camera_teardown()
        self._tracker_teardown()
        for closer in (lambda: self.destroy_timer(self._status_timer),
                       lambda: self.destroy_publisher(self.pub_status),
                       lambda: self._sub.close(),
                       lambda: self._tactile_sample_sub and self._tactile_sample_sub.close(),
                       lambda: self.glove.close()):
            try:
                closer()
            except Exception:  # noqa: BLE001
                pass
        return super().destroy_node()


def spin_until_exit(node) -> int:
    """Runs until Ctrl-C or until ``node.exit_code`` is set (tracker mode switch), and returns the process exit
    code.

    Multi-threaded executor: the tare service is in its own Reentrant group; one hardware zeroing takes about
    3.5s and must neither block the 1Hz status timer nor serialize the tare of the two hands. All other
    callbacks stay in the default mutually exclusive group and remain serialized with each other."""
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        while rclpy.ok() and node.exit_code is None:
            executor.spin_once(timeout_sec=0.2)
    finally:
        executor.remove_node(node)
        executor.shutdown()
    return int(node.exit_code or 0)


def shutdown_node(node) -> None:
    """Process shutdown: ignore SIGINT → destroy_node → rclpy.shutdown.

    A Ctrl+C in the terminal goes to the whole process group, and launch forwards it once more. If the second
    one lands while glove.close() is joining the reader thread, KeyboardInterrupt interrupts the shutdown: the
    stream does not stop, the remaining gloves are not closed, the glove keeps streaming, and the next process
    that opens it cannot read device_info (in testing, the single-hand node refused to start because of this).
    If shutdown hangs, launch still escalates SIGTERM → SIGKILL, so it does not hang forever."""
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
        pass        # Ctrl-C / SIGINT goes through rclpy's shutdown; not a startup failure
    except Exception as exc:  # noqa: BLE001 —— a startup failure must show its cause
        # (product model mismatch / not registered / serial port busy)
        if rclpy.ok():
            print(f"glove_node 启动失败：{type(exc).__name__}: {exc}", file=sys.stderr)
            code = 1
        # otherwise it is an exception raised by publish in a callback after SIGINT invalidated the context:
        # shutdown, not a startup failure
    finally:
        shutdown_node(node)
    return code


if __name__ == "__main__":
    sys.exit(main())
