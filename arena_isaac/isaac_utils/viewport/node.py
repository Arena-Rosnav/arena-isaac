"""ROS surface for the Isaac viewport camera.

Services and topics sit under the global /arena/viewport namespace, the names every
backend of the contract advertises. Isaac drains them by spin_once from the loop that
also renders, so no state crosses a thread boundary and nothing needs a mutex. Two
things live on a second node with its own spinner thread instead: capture, whose
response is a frame the loop still has to draw (handed over via CaptureQueue), and the
tf listener a tracked TF frame is resolved through.
"""

from __future__ import annotations

import threading

import geometry_msgs.msg
import rclpy.executors
import rclpy.node
import rclpy.time
import tf2_ros
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from viewport_control_msgs.msg import ViewportView
from viewport_control_msgs.srv import ViewportCapture, ViewportSetProjection, ViewportSetReferenceFrame, ViewportSetView

from .backend import CameraBackend, entity_pose
from .capture import TIMEOUT_S, CaptureQueue, CaptureRequest
from .controller import Keyframe, Pose, ViewportController

NAMESPACE = "/arena/viewport"
# arena pins the world origin to map, so the camera world pose is map-frame
POSE_FRAME = "map"
# ~10 Hz at 60 fps
PUBLISH_EVERY_N_FRAMES = 6

# deep enough that every keyframe reaches the buffer, not just the latest
STREAM_QOS = QoSProfile(depth=64, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)


def _pose(msg: geometry_msgs.msg.Pose) -> Pose:
    return Pose(
        (msg.position.x, msg.position.y, msg.position.z),
        (msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z),
    )


class ViewportNode:
    """Owns the viewport ROS endpoints and pumps the controller once per frame."""

    def __init__(self, node: rclpy.node.Node) -> None:
        self._node = node
        self._controller = ViewportController()
        self._backend = CameraBackend()
        self._frames = 0
        self._warned_entity = ""
        self._resolved_entity = ""

        node.create_service(ViewportSetView, f"{NAMESPACE}/set_view", self._cb_set_view)
        node.create_service(ViewportSetReferenceFrame, f"{NAMESPACE}/set_reference_frame", self._cb_set_reference_frame)
        node.create_service(ViewportSetProjection, f"{NAMESPACE}/set_projection", self._cb_set_projection)
        node.create_subscription(ViewportView, f"{NAMESPACE}/cmd_view", self._cb_cmd_view, STREAM_QOS)
        self._pose_pub = node.create_publisher(geometry_msgs.msg.PoseStamped, f"{NAMESPACE}/camera_pose", 10)

        self._captures = CaptureQueue()
        self._side_node = rclpy.node.Node("arena_viewport_side")
        self._side_node.create_service(ViewportCapture, f"{NAMESPACE}/capture", self._cb_capture)
        self._tf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf, self._side_node, spin_thread=False)
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(self._side_node)
        threading.Thread(target=executor.spin, name="viewport_side", daemon=True).start()

    def _now(self) -> float:
        return self._node.get_clock().now().nanoseconds * 1e-9

    def _cb_set_view(self, request: ViewportSetView.Request, response: ViewportSetView.Response) -> ViewportSetView.Response:
        eye = (request.eye.x, request.eye.y, request.eye.z)
        target = (request.target.x, request.target.y, request.target.z)
        self._controller.set_view(eye, target, request.fov)
        response.success = True
        response.message = "ok"
        return response

    def _cb_set_reference_frame(self, request: ViewportSetReferenceFrame.Request, response: ViewportSetReferenceFrame.Response) -> ViewportSetReferenceFrame.Response:
        response.message = self._controller.set_reference_frame(request.entity, _pose(request.pose), request.has_pose, request.mode)
        self._warned_entity = ""
        response.success = True
        return response

    def _cb_set_projection(self, request: ViewportSetProjection.Request, response: ViewportSetProjection.Response) -> ViewportSetProjection.Response:
        response.success = self._controller.set_projection(request.projection)
        response.message = "ok" if response.success else "projection must be 'perspective' or 'orthographic'"
        return response

    def _cb_capture(self, request: ViewportCapture.Request, response: ViewportCapture.Response) -> ViewportCapture.Response:
        """Capture thread: park the request, block until the render loop has drawn it."""
        pending = CaptureRequest(
            local=_pose(request.pose),
            world_orientation=request.world_orientation,
            fov=request.fov,
            min_sim_time=rclpy.time.Time.from_msg(request.min_sim_time).nanoseconds * 1e-9,
        )
        self._captures.submit(pending)
        if not pending.done.wait(TIMEOUT_S):
            self._captures.drop(pending)
            pending.finish(None, "capture timed out, min_sim_time not reached")
        response.success = pending.image is not None
        response.message = pending.message
        if pending.image is not None:
            response.image = pending.image
        return response

    def _cb_cmd_view(self, msg: ViewportView) -> None:
        keyframe = Keyframe(
            time=rclpy.time.Time.from_msg(msg.target_time).nanoseconds * 1e-9,
            local=_pose(msg.pose),
            world_orientation=msg.world_orientation,
            fov=msg.fov,
        )
        self._controller.push_keyframe(keyframe, self._now())

    def _tf_pose(self, frame: str) -> Pose | None:
        """World pose of a TF frame, None while the transform is unknown."""
        try:
            tf = self._tf.lookup_transform(POSE_FRAME, frame, rclpy.time.Time()).transform
        except tf2_ros.TransformException:
            return None
        return Pose(
            (tf.translation.x, tf.translation.y, tf.translation.z),
            (tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z),
        )

    def _sample_reference(self) -> None:
        """Resolve the tracked entity's world pose for this frame: a prim path as is, else a TF frame."""
        entity = self._controller.tracked_entity
        if not entity:
            return
        pose = entity_pose(entity)
        source = "prim"
        if pose is None:
            pose = self._tf_pose(entity)
            source = "tf"
        if pose is None:
            if self._warned_entity != entity:
                self._warned_entity = entity
                self._node.get_logger().warning(f"tracked entity '{entity}' is neither a prim nor a TF frame, camera holds the world frame")
            return
        self._warned_entity = ""
        if self._resolved_entity != entity:
            self._resolved_entity = entity
            x, y, z = pose.position
            self._node.get_logger().info(f"tracking '{entity}' via {source}, now at ({x:.2f}, {y:.2f}, {z:.2f})")
        self._controller.set_reference_target(pose)

    def apply(self) -> None:
        """Drive the camera for one frame."""
        camera_pose = self._backend.world_pose()
        if camera_pose is None:  # no viewport yet
            return

        self._sample_reference()
        pending = self._captures.peek()
        if pending is not None:
            self._captures.serve(pending, self._node.sim_time, POSE_FRAME, self._controller.set_local)
        frame = self._controller.apply(self._now(), camera_pose)
        if self._controller.released is not None:
            offset, angle = self._controller.released
            self._controller.released = None
            self._node.get_logger().warning(f"camera moved off its tracked pose by {offset:.3f} m / {angle:.4f} rad, releasing the tracked entity")
        if frame.projection is not None:
            self._backend.set_projection(frame.projection)
        if frame.fov is not None and frame.fov > 0.0:
            self._backend.set_hfov(frame.fov)
        if frame.pose is not None:
            self._backend.set_world_pose(frame.pose)

        self._frames += 1
        if self._frames % PUBLISH_EVERY_N_FRAMES:
            return
        pose = frame.pose or camera_pose
        msg = geometry_msgs.msg.PoseStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = POSE_FRAME
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pose.position
        msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z = pose.orientation
        self._pose_pub.publish(msg)
