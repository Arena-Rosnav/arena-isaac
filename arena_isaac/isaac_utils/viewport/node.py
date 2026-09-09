"""ROS surface for the Isaac viewport camera.

Services and topics sit under the global /arena/viewport namespace, the names every
backend of the contract advertises. Isaac drains them by spin_once from the loop that
also renders, so no state crosses a thread boundary and nothing needs a mutex.
"""

from __future__ import annotations

import geometry_msgs.msg
import rclpy.node
import rclpy.time
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from viewport_control_msgs.msg import ViewportView
from viewport_control_msgs.srv import ViewportCapture, ViewportSetProjection, ViewportSetReferenceFrame, ViewportSetView

from .backend import CameraBackend, entity_pose
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

        node.create_service(ViewportSetView, f"{NAMESPACE}/set_view", self._cb_set_view)
        node.create_service(ViewportSetReferenceFrame, f"{NAMESPACE}/set_reference_frame", self._cb_set_reference_frame)
        node.create_service(ViewportSetProjection, f"{NAMESPACE}/set_projection", self._cb_set_projection)
        node.create_service(ViewportCapture, f"{NAMESPACE}/capture", self._cb_capture)
        node.create_subscription(ViewportView, f"{NAMESPACE}/cmd_view", self._cb_cmd_view, STREAM_QOS)
        self._pose_pub = node.create_publisher(geometry_msgs.msg.PoseStamped, f"{NAMESPACE}/camera_pose", 10)

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
        response.success = False
        response.message = "capture not yet supported"
        return response

    def _cb_cmd_view(self, msg: ViewportView) -> None:
        keyframe = Keyframe(
            time=rclpy.time.Time.from_msg(msg.target_time).nanoseconds * 1e-9,
            local=_pose(msg.pose),
            world_orientation=msg.world_orientation,
            fov=msg.fov,
        )
        self._controller.push_keyframe(keyframe, self._now())

    def _sample_reference(self) -> None:
        """Resolve the tracked entity's world pose for this frame."""
        entity = self._controller.tracked_entity
        if not entity:
            return
        pose = entity_pose(entity)
        if pose is None:
            if self._warned_entity != entity:
                self._warned_entity = entity
                self._node.get_logger().warning(f"tracked entity '{entity}' not found, camera holds the world frame")
            return
        self._warned_entity = ""
        self._controller.set_reference_target(pose)

    def apply(self) -> None:
        """Drive the camera for one frame."""
        camera_pose = self._backend.world_pose()
        if camera_pose is None:  # no viewport yet
            return

        self._sample_reference()
        frame = self._controller.apply(self._now(), camera_pose)
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
