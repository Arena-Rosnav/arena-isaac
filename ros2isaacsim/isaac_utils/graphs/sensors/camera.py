import os
import xml.etree.ElementTree as ET

import attrs
import omni
import omni.graph.core as og
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd
from isaac_utils.utils.geom import Rotation, Translation
from isaacsim.core.utils import extensions
from isaacsim.ros2.bridge import read_camera_info
from omni.isaac.sensor import Camera

from . import SensorBase

extensions.enable_extension("isaacsim.ros2.bridge")


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

    def publish(self, base_topic: str):
        if self.prim_path is None or self.camera is None:
            raise RuntimeError('Camera not simulated. Call simulate() first.')

        camera_topic = os.path.join(base_topic, self.name)
        frame = os.path.join(self.robot_base_frame, self.parent_frame)
        node_namespace = ''
        queue_size = 1

        render_product = self.camera._render_product_path
        step_size = int(60 / self.config.update_rate)

        self._publish_camera_info(render_product, frame, node_namespace, queue_size, camera_topic, step_size)
        self._publish_rgb(render_product, frame, node_namespace, queue_size, camera_topic, step_size)

        return render_product, frame, node_namespace, queue_size, camera_topic, step_size

    @classmethod
    def _publish_camera_info(cls, render_product: str, frame: str, node_namespace: str, queue_size: int, camera_topic: str, step_size: int):
        camera_info = read_camera_info(render_product_path=render_product)

        writer_camera_info = rep.writers.get("ROS2PublishCameraInfo")
        writer_camera_info.initialize(
            frameId=frame,
            nodeNamespace=node_namespace,
            queueSize=queue_size,
            topicName=os.path.join(camera_topic, "camera_info"),
            width=camera_info["width"],
            height=camera_info["height"],
            projectionType=camera_info["projectionType"],
            k=camera_info["k"].reshape([1, 9]),
            r=camera_info["r"].reshape([1, 9]),
            p=camera_info["p"].reshape([1, 12]),
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
