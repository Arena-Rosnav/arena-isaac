import os
import xml.etree.ElementTree as ET
from collections.abc import Mapping, Sequence

import attrs
import omni
import omni.graph.core as og
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd
from isaacsim.ros2.bridge import read_camera_info
from isaacsim.sensors.camera import Camera

from isaac_utils.utils.geom import Rotation, Translation

from . import SensorBase


class SensorCamera(SensorBase):
    @attrs.define
    class Config:
        @attrs.define
        class Image:
            width: int = attrs.field(converter=attrs.converters.optional(int), default=640)
            height: int = attrs.field(converter=attrs.converters.optional(int), default=480)

        @attrs.define
        class Clip:
            near: float = attrs.field(converter=attrs.converters.optional(float), default=0.1)
            far: float = attrs.field(converter=attrs.converters.optional(float), default=100.0)

        image: Image
        clip: Clip
        update_rate: float = attrs.field(converter=attrs.converters.optional(float), default=1.0)

        @classmethod
        def parse(cls, config: ET.Element) -> "SensorCamera.Config":
            return cls(
                update_rate=config.findtext(".//update_rate"),
                image=cls.Image(
                    width=config.findtext(".//image/width"),
                    height=config.findtext(".//image/height"),
                ),
                clip=cls.Clip(
                    near=config.findtext(".//clip/near"),
                    far=config.findtext(".//clip/far"),
                ),
            )

    def __init__(
        self,
        robot_base_frame: str,
        parent_frame: str,
        name: str,
        config: "SensorCamera.Config",
        translation: Translation,
        rotation: Rotation,
        is_rgbd: bool = False,
    ):
        """
        Initializes the camera sensor.
        Args:
            robot_base_frame(str): The base frame of the robot.
            parent_frame(str): The frame ID of the parent frame.
            name(str): The name of the lidar sensor.
            config(SensorLidar.Config): The configuration of the lidar sensor.
            translation(Translation): Translation relative to parent prim.
            rotation(Rotation): Rotation relative to parent prim.
        """
        self.robot_base_frame: str = robot_base_frame
        self.parent_frame: str = parent_frame
        self.name: str = name
        self.config: "SensorCamera.Config" = config
        self.translation: Translation = translation
        self.rotation: Rotation = rotation
        self.is_rgbd: bool = is_rgbd

        self.prim_path: str | None = None
        self.camera: Camera | None = None

    def simulate(self, base_prim: str):
        camera = Camera(
            prim_path=os.path.join(base_prim, self.parent_frame, self.name),
            name=self.name,
            translation=self.translation.tuple(),
            orientation=self.rotation.quat(),
            frequency=self.config.update_rate,
            resolution=(self.config.image.width, self.config.image.height),
        )
        if camera:
            camera.initialize()
            self.prim_path = camera.prim_path
            self.camera = camera

    def paths(self) -> Sequence[str]:
        if self.prim_path is None:
            return ()
        return (self.prim_path,)

    def publish(self, base_topic: str):
        if self.prim_path is None or self.camera is None:
            raise RuntimeError('Camera not simulated. Call simulate() first.')

        camera_topic = os.path.join(base_topic, self.name)
        frame = f'{self.robot_base_frame}{self.parent_frame}'
        node_namespace = ''
        queue_size = 1

        render_product = self.camera._render_product_path
        step_size = int(60 / self.config.update_rate)

        self._publish_camera_info(render_product, frame, node_namespace, queue_size, camera_topic, step_size)
        self._publish_rgb(render_product, frame, node_namespace, queue_size, camera_topic, step_size)

        return render_product, frame, node_namespace, queue_size, camera_topic, step_size

    @staticmethod
    def _reshape_param(value, columns: int):
        if hasattr(value, "reshape"):
            return value.reshape([1, columns])
        if isinstance(value, (list, tuple)):
            return [list(value)]
        return value

    @staticmethod
    def _first_available(source, *keys):
        for key in keys:
            if isinstance(source, Mapping) and key in source:
                return source[key]
            if hasattr(source, key):
                return getattr(source, key)
        return None

    @classmethod
    def _normalize_camera_info(cls, camera_info):
        if isinstance(camera_info, Mapping):
            normalized = {
                "width": cls._first_available(camera_info, "width"),
                "height": cls._first_available(camera_info, "height"),
                "projectionType": cls._first_available(camera_info, "projectionType", "projection_type"),
                "k": cls._reshape_param(cls._first_available(camera_info, "k"), 9),
                "r": cls._reshape_param(cls._first_available(camera_info, "r"), 9),
                "p": cls._reshape_param(cls._first_available(camera_info, "p"), 12),
                "physicalDistortionModel": cls._first_available(camera_info, "physicalDistortionModel", "physical_distortion_model", "distortionModel", "distortion_model"),
                "physicalDistortionCoefficients": cls._first_available(camera_info, "physicalDistortionCoefficients", "physical_distortion_coefficients", "distortionCoefficients", "distortion_coefficients", "d"),
            }
            if normalized["projectionType"] is None:
                normalized["projectionType"] = "pinhole"
            return normalized

        if isinstance(camera_info, (tuple, list)) and len(camera_info) >= 8:
            width, height, projection_type, k, r, p, distortion_model, distortion_coefficients = camera_info[:8]
            return {
                "width": width,
                "height": height,
                "projectionType": projection_type,
                "k": cls._reshape_param(k, 9),
                "r": cls._reshape_param(r, 9),
                "p": cls._reshape_param(p, 12),
                "physicalDistortionModel": distortion_model,
                "physicalDistortionCoefficients": distortion_coefficients,
            }

        if isinstance(camera_info, (tuple, list)):
            if len(camera_info) == 1:
                return cls._normalize_camera_info(camera_info[0])

            if len(camera_info) == 2 and isinstance(camera_info[1], (Mapping, tuple, list)):
                try:
                    return cls._normalize_camera_info(camera_info[1])
                except TypeError:
                    pass

            for item in camera_info:
                try:
                    return cls._normalize_camera_info(item)
                except TypeError:
                    continue

        required_attrs = (
            "width",
            "height",
            "projectionType",
            "k",
            "r",
            "p",
            "physicalDistortionModel",
            "physicalDistortionCoefficients",
        )
        if all(hasattr(camera_info, attr) for attr in required_attrs):
            return {
                "width": getattr(camera_info, "width"),
                "height": getattr(camera_info, "height"),
                "projectionType": getattr(camera_info, "projectionType"),
                "k": cls._reshape_param(getattr(camera_info, "k"), 9),
                "r": cls._reshape_param(getattr(camera_info, "r"), 9),
                "p": cls._reshape_param(getattr(camera_info, "p"), 12),
                "physicalDistortionModel": getattr(camera_info, "physicalDistortionModel"),
                "physicalDistortionCoefficients": getattr(camera_info, "physicalDistortionCoefficients"),
            }

        alt_required_attrs = (
            "width",
            "height",
            "projection_type",
            "k",
            "r",
            "p",
            "distortion_model",
            "distortion_coefficients",
        )
        if all(hasattr(camera_info, attr) for attr in alt_required_attrs):
            return {
                "width": getattr(camera_info, "width"),
                "height": getattr(camera_info, "height"),
                "projectionType": getattr(camera_info, "projection_type"),
                "k": cls._reshape_param(getattr(camera_info, "k"), 9),
                "r": cls._reshape_param(getattr(camera_info, "r"), 9),
                "p": cls._reshape_param(getattr(camera_info, "p"), 12),
                "physicalDistortionModel": getattr(camera_info, "distortion_model"),
                "physicalDistortionCoefficients": getattr(camera_info, "distortion_coefficients"),
            }

        ros_camera_info_attrs = (
            "width",
            "height",
            "k",
            "r",
            "p",
            "distortion_model",
            "d",
        )
        if all(hasattr(camera_info, attr) for attr in ros_camera_info_attrs):
            return {
                "width": getattr(camera_info, "width"),
                "height": getattr(camera_info, "height"),
                "projectionType": "pinhole",
                "k": cls._reshape_param(getattr(camera_info, "k"), 9),
                "r": cls._reshape_param(getattr(camera_info, "r"), 9),
                "p": cls._reshape_param(getattr(camera_info, "p"), 12),
                "physicalDistortionModel": getattr(camera_info, "distortion_model"),
                "physicalDistortionCoefficients": getattr(camera_info, "d"),
            }

        tuple_len = len(camera_info) if isinstance(camera_info, (tuple, list)) else None
        raise TypeError(f"Unsupported camera info format: {type(camera_info)!r}, len={tuple_len}, value={camera_info!r}")

    @classmethod
    def _publish_camera_info(cls, render_product: str, frame: str, node_namespace: str, queue_size: int, camera_topic: str, step_size: int):
        camera_info = cls._normalize_camera_info(read_camera_info(render_product_path=render_product))

        writer_camera_info = rep.writers.get("ROS2PublishCameraInfo")
        writer_camera_info.initialize(
            frameId=frame,
            nodeNamespace=node_namespace,
            queueSize=queue_size,
            topicName=os.path.join(camera_topic, "camera_info"),
            width=camera_info["width"],
            height=camera_info["height"],
            projectionType=camera_info["projectionType"],
            k=camera_info["k"],
            r=camera_info["r"],
            p=camera_info["p"],
            physicalDistortionModel=camera_info["physicalDistortionModel"],
            physicalDistortionCoefficients=camera_info["physicalDistortionCoefficients"],
        )
        writer_camera_info.attach([render_product])
        gate_path_camera_info = omni.syntheticdata.SyntheticData._get_node_path(
            "PostProcessDispatch" + "IsaacSimulationGate", render_product
        )
        og.Controller.attribute(gate_path_camera_info + ".inputs:step").set(step_size)

    @classmethod
    def _publish_rgb(cls, render_product: str, frame: str, node_namespace: str, queue_size: int, camera_topic: str, step_size: int):
        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
        writer = rep.writers.get(rv + "ROS2PublishImage")
        writer.initialize(
            frameId=frame,
            nodeNamespace=node_namespace,
            queueSize=queue_size,
            topicName=os.path.join(camera_topic, 'image')
        )
        writer.attach([render_product])

        gate_path = omni.syntheticdata.SyntheticData._get_node_path(
            rv + "IsaacSimulationGate", render_product
        )
        og.Controller.attribute(gate_path + ".inputs:step").set(step_size)


class SensorCameraRGBD(SensorCamera):

    def publish(self, base_topic: str):
        args = super().publish(base_topic)
        self._publish_pointcloud(*args)
        self._publish_depth(*args)
        return args

    @classmethod
    def _publish_pointcloud(cls, render_product: str, frame: str, node_namespace: str, queue_size: int, camera_topic: str, step_size: int):
        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
            sd.SensorType.DistanceToImagePlane.name
        )

        writer = rep.writers.get(rv + "ROS2PublishPointCloud")
        writer.initialize(
            frameId=frame,
            nodeNamespace=node_namespace,
            queueSize=queue_size,
            topicName=os.path.join(camera_topic, 'points')
        )
        writer.attach([render_product])

        gate_path = omni.syntheticdata.SyntheticData._get_node_path(
            rv + "IsaacSimulationGate", render_product
        )
        og.Controller.attribute(gate_path + ".inputs:step").set(step_size)

    @classmethod
    def _publish_depth(cls, render_product: str, frame: str, node_namespace: str, queue_size: int, camera_topic: str, step_size: int):
        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
            sd.SensorType.DistanceToImagePlane.name
        )
        writer = rep.writers.get(rv + "ROS2PublishImage")
        writer.initialize(
            frameId=frame,
            nodeNamespace=node_namespace,
            queueSize=queue_size,
            topicName=os.path.join(camera_topic, 'depth')
        )
        writer.attach([render_product])

        gate_path = omni.syntheticdata.SyntheticData._get_node_path(
            rv + "IsaacSimulationGate", render_product
        )
        og.Controller.attribute(gate_path + ".inputs:step").set(step_size)
