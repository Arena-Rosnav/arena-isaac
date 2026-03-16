import math
import os
import xml.etree.ElementTree as ET

import attrs
import carb
import numpy as np
import omni
import omni.graph.core as og
import omni.kit.commands
from isaacsim.core.utils.stage import get_current_stage

from isaac_utils.graphs import Graph
from isaac_utils.utils.geom import Rotation, Translation
from isaac_utils.utils.prim import ensure_path

from . import SensorBase


class SensorLidar(SensorBase):
    """
    Helper for creating and publishing a lidar sensor in Isaac Sim.

    Args:
        name (str): The name of the lidar sensor.
        frame (str): The frame ID of the lidar sensor.
        config (Config): The configuration of the lidar sensor.
    """

    @attrs.define
    class Config:
        @attrs.define
        class Dimension:
            samples: int = attrs.field(converter=attrs.converters.optional(int), default=1)
            resolution: float = attrs.field(converter=attrs.converters.optional(float), default=1.0)
            min_angle: float = attrs.field(converter=attrs.converters.optional(float), default=-math.pi)
            max_angle: float = attrs.field(converter=attrs.converters.optional(float), default=+math.pi)

        @attrs.define
        class Range:
            min: float = attrs.field(converter=attrs.converters.optional(float), default=0.0)
            max: float = attrs.field(converter=attrs.converters.optional(float), default=100.0)
            resolution: float = attrs.field(converter=attrs.converters.optional(float), default=1.0)

        horizontal: Dimension
        vertical: Dimension
        range: Range
        topic: str = attrs.field(validator=attrs.validators.instance_of(str))
        update_rate: float = attrs.field(converter=attrs.converters.optional(float), default=1.0)

        @classmethod
        def parse(cls, config: ET.Element) -> "SensorLidar.Config":
            return cls(
                topic=config.findtext(".//topic") or config.findtext(".//topicName") or 'lidar',
                update_rate=config.findtext(".//update_rate"),
                horizontal=SensorLidar.Config.Dimension(
                    samples=config.findtext(".//scan/horizontal/samples"),
                    resolution=config.findtext(".//scan/horizontal/resolution"),
                    min_angle=config.findtext(".//scan/horizontal/min_angle"),
                    max_angle=config.findtext(".//scan/horizontal/max_angle"),
                ),
                vertical=SensorLidar.Config.Dimension(
                    samples=config.findtext(".//scan/vertical/samples"),
                    resolution=config.findtext(".//scan/vertical/resolution"),
                    min_angle=config.findtext(".//scan/vertical/min_angle"),
                    max_angle=config.findtext(".//scan/vertical/max_angle"),
                ),
                range=SensorLidar.Config.Range(
                    min=config.findtext(".//range/min"),
                    max=config.findtext(".//range/max"),
                    resolution=config.findtext(".//range/resolution"),
                ),
            )

        def as_omnilidar_attributes(self) -> dict[str, object]:
            azimuth_deg = np.tile(
                np.linspace(
                    np.degrees(self.horizontal.min_angle),
                    np.degrees(self.horizontal.max_angle),
                    self.horizontal.samples,
                    endpoint=True,
                ),
                self.vertical.samples,
            ).tolist()

            elevation_deg = np.repeat(
                np.linspace(
                    np.degrees(self.vertical.min_angle),
                    np.degrees(self.vertical.max_angle),
                    self.vertical.samples,
                    endpoint=True,
                ),
                self.horizontal.samples,
            ).tolist()

            emitters_count = self.vertical.samples * self.horizontal.samples
            fire_time_ns = np.linspace(
                0,
                1e9 / max(self.update_rate, 1e-6),
                emitters_count,
            ).astype(int).tolist()

            return {
                "omni:sensor:Core:scanType": "SOLID_STATE",
                "omni:sensor:Core:intensityProcessing": "NORMALIZATION",
                "omni:sensor:Core:rayType": "IDEALIZED",
                "omni:sensor:Core:rotationDirection": "CW",
                "omni:sensor:Core:nearRangeM": float(self.range.min),
                "omni:sensor:Core:farRangeM": float(self.range.max),
                "omni:sensor:Core:rangeResolutionM": float(self.range.resolution),
                "omni:sensor:Core:rangeAccuracyM": 0.025,
                "omni:sensor:Core:waveLengthNm": 1550.0,
                "omni:sensor:Core:maxReturns": 2,
                "omni:sensor:Core:reportRateBaseHz": int(max(1, round(self.update_rate))),
                "omni:sensor:Core:scanRateBaseHz": int(max(1, round(self.update_rate))),
                "omni:sensor:Core:numberOfEmitters": int(emitters_count),
                "omni:sensor:Core:numberOfChannels": int(emitters_count),
                "omni:sensor:Core:numLines": int(self.vertical.samples),
                "omni:sensor:Core:numRaysPerLine": [int(self.horizontal.samples)] * int(self.vertical.samples),
                "omni:sensor:Core:rangeCount": 1,
                "omni:sensor:Core:rangesMinM": [float(self.range.min)],
                "omni:sensor:Core:rangesMaxM": [float(self.range.max)],
                "omni:sensor:Core:validStartAzimuthDeg": float(np.degrees(self.horizontal.min_angle)),
                "omni:sensor:Core:validEndAzimuthDeg": float(np.degrees(self.horizontal.max_angle)),
                "omni:sensor:Core:intensityMappingType": "LINEAR",
                "OmniSensorGenericLidarCoreEmitterStateAPI:s001:azimuthDeg": azimuth_deg,
                "OmniSensorGenericLidarCoreEmitterStateAPI:s001:elevationDeg": elevation_deg,
                "OmniSensorGenericLidarCoreEmitterStateAPI:s001:fireTimeNs": fire_time_ns,
            }

    _ARRAY_ATTRIBUTE_KEYS = {
        "omni:sensor:Core:numRaysPerLine",
        "omni:sensor:Core:rangesMinM",
        "omni:sensor:Core:rangesMaxM",
        "OmniSensorGenericLidarCoreEmitterStateAPI:s001:azimuthDeg",
        "OmniSensorGenericLidarCoreEmitterStateAPI:s001:elevationDeg",
        "OmniSensorGenericLidarCoreEmitterStateAPI:s001:fireTimeNs",
    }

    def __init__(
        self,
        robot_base_frame: str,
        parent_frame: str,
        name: str,
        config: "SensorLidar.Config",
        translation: Translation,
        rotation: Rotation,
    ):
        """
        Initializes the lidar sensor.
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
        self.config: SensorLidar.Config = config
        self.translation: Translation = translation
        self.rotation: Rotation = rotation

        self.prim_path: str | None = None

    def simulate(self, base_prim: str):
        """
        Simulates the lidar sensor in Isaac Sim.
        Args:
            base_prim(str): The base prim path for the robot.
            translation(Translation): The translation of the lidar sensor relative to the base prim.
            rotation(Rotation): The rotation of the lidar sensor relative to the base prim.
        """
        base_prim_path = f"/{str(base_prim).strip('/')}"
        parent_frame_path = str(self.parent_frame).strip('/')
        sensor_name = str(self.name).strip('/').split('/')[-1]

        parent_prim_path = f"{base_prim_path}/{parent_frame_path}" if parent_frame_path else base_prim_path
        prim_path = f"{parent_prim_path}/{sensor_name}"
        sensor_attributes = self.config.as_omnilidar_attributes()
        scalar_attributes = {
            key: value
            for key, value in sensor_attributes.items()
            if key not in self._ARRAY_ATTRIBUTE_KEYS
        }
        array_attributes = {
            key: value
            for key, value in sensor_attributes.items()
            if key in self._ARRAY_ATTRIBUTE_KEYS
        }

        try:
            ensure_path(parent_prim_path)

            _, lidar = omni.kit.commands.execute(
                "IsaacSensorCreateRtxLidar",
                path=prim_path,
                parent=None,
                config=None,
                translation=self.translation.tuple(),
                orientation=self.rotation.Quatd(),
                force_camera_prim=False,
                **scalar_attributes,
            )
        except Exception as error:
            self.prim_path = None
            carb.log_warn(
                f"Lidar simulate failed for '{self.name}' at '{prim_path}': {error}"
            )
            return

        if not lidar:
            self.prim_path = None
            carb.log_warn(
                f"Lidar simulate returned no prim for '{self.name}' at '{prim_path}'."
            )
            return

        created_prim_path = None
        try:
            created_prim_path = str(lidar.GetPath())
        except Exception:
            created_prim_path = None

        if created_prim_path:
            if created_prim_path != prim_path:
                carb.log_warn(
                    f"Lidar prim path remapped from '{prim_path}' to '{created_prim_path}' for '{self.name}'."
                )
            prim_path = created_prim_path

        stage = get_current_stage()
        prim = stage.GetPrimAtPath(prim_path) if stage is not None else None

        if prim is None or not prim.IsValid():
            self.prim_path = None
            carb.log_warn(
                f"Lidar prim '{prim_path}' is invalid after creation for '{self.name}'."
            )
            return

        for key, value in array_attributes.items():
            attr = prim.GetAttribute(key)
            if not attr.IsValid():
                carb.log_warn(
                    f"Lidar attribute '{key}' not found on '{prim_path}', skipping."
                )
                continue
            try:
                attr.Set(value)
            except Exception as error:
                carb.log_warn(
                    f"Failed to set lidar attribute '{key}' on '{prim_path}': {error}"
                )

        self.prim_path = prim_path

    def publish(self, base_topic: str):
        """
        Publishes the lidar sensor to ros2.
        Args:
            base_topic(str): The base topic path for the robot.
        """

        if self.prim_path is None:
            carb.log_warn(
                f"Lidar publish skipped for '{self.name}': sensor not simulated (prim_path is None)."
            )
            return False

        graph = Graph(os.path.join(self.prim_path, "LidarPublisher"))

        on_playback_tick = graph.node("on_playback_tick", "omni.graph.action.OnPlaybackTick")
        render_product = graph.node("render_product", "isaacsim.core.nodes.IsaacCreateRenderProduct")
        lidar_publisher = graph.node("lidar_publisher", "isaacsim.ros2.bridge.ROS2RtxLidarHelper")
        lidar_publisher_points = graph.node("lidar_publisher_points", "isaacsim.ros2.bridge.ROS2RtxLidarHelper")

        # ReadSimTime = graph.node("readSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime")
        # publishTF = graph.node("publishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree")

        render_product.attribute("cameraPrim", self.prim_path)
        render_product.attribute("width", 1)
        render_product.attribute("height", 1)

        lidar_publisher.attribute("topicName", os.path.join(base_topic, self.config.topic))
        lidar_publisher.attribute("frameId", os.path.join(self.robot_base_frame, self.parent_frame))
        lidar_publisher.attribute("type", 'laser_scan')

        lidar_publisher_points.attribute("topicName", os.path.join(base_topic, self.config.topic, 'points'))
        lidar_publisher_points.attribute("frameId", os.path.join(self.robot_base_frame, self.parent_frame))
        lidar_publisher_points.attribute("type", 'point_cloud')

        # publishTF.attribute("targetPrims", [self.prim_path, os.path.join(self.prim_path, self.frame)])

        on_playback_tick.connect("tick", render_product, "execIn")
        # OnPlaybackTick.connect("tick", publishTF, "execIn")
        # ReadSimTime.connect("simulationTime", publishTF, "timeStamp")
        render_product.connect("execOut", lidar_publisher, "execIn")
        render_product.connect("renderProductPath", lidar_publisher, "renderProductPath")
        render_product.connect("execOut", lidar_publisher_points, "execIn")
        render_product.connect("renderProductPath", lidar_publisher_points, "renderProductPath")

        graph.load_extensions()
        return graph.execute(og.Controller())
