import json
import math
import os
import tempfile
import xml.etree.ElementTree as ET

import attrs
import numpy as np
import omni
import omni.graph.core as og
import omni.kit.commands
from isaac_utils.graphs import Graph
from isaac_utils.utils.geom import Rotation, Translation
from isaacsim.core.utils import extensions
from isaacsim.core.utils.extensions import get_extension_path_from_name

from . import SensorBase

extensions.enable_extension("isaacsim.ros2.bridge")
extensions.enable_extension("isaacsim.core.nodes")


class SensorLidar(SensorBase):
    """
    Helper for creating and publishing a lidar sensor in Isaac Sim.

    Args:
        name (str): The name of the lidar sensor.
        frame (str): The frame ID of the lidar sensor.
        config (Config): The configuration of the lidar sensor.
    """

    config_base_dir = os.path.join(get_extension_path_from_name("omni.isaac.sensor"), "data/lidar_configs/CustomLidar_tmp_")
    config_base_prefix = os.path.basename(config_base_dir)
    os.makedirs(config_base_dir, exist_ok=True)

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

        def as_profile(self) -> dict:
            """
            Converts the configuration to a profile for the lidar sensor.
            Returns:
                dict: The profile for the lidar sensor.
            """
            return {
                "scanType": "solidState",
                "intensityProcessing": "normalization",
                "rayType": "IDEALIZED",
                "nearRangeM": self.range.min,
                "farRangeM": self.range.max,
                "rangeResolutionM": self.range.resolution,
                "rangeAccuracyM": 0.025,
                "rotationDirection": "CW",
                "wavelengthNm": 1550.0,
                "maxReturns": 2,
                "reportRateBaseHz": self.update_rate,

                "numberOfEmitters": self.vertical.samples * self.horizontal.samples,
                "numberOfChannels": self.vertical.samples * self.horizontal.samples,
                "scanRateBaseHz": self.update_rate,

                "numLines": self.vertical.samples,
                "numRaysPerLine": [self.horizontal.samples] * self.vertical.samples,

                "rangeCount": 1,
                "ranges": [{"min": self.range.min, "max": self.range.max}],

                "emitterStateCount": 1,
                "emitterStates": [
                    {
                        "azimuthDeg": np.tile(
                            np.linspace(
                                np.degrees(self.horizontal.min_angle),
                                np.degrees(self.horizontal.max_angle),
                                self.horizontal.samples,
                                endpoint=True
                            ),
                            self.vertical.samples
                        ).tolist(),
                        "elevationDeg": np.repeat(
                            np.linspace(
                                np.degrees(self.vertical.min_angle),
                                np.degrees(self.vertical.max_angle),
                                self.vertical.samples,
                                endpoint=True
                            ),
                            self.horizontal.samples
                        ).tolist(),
                        "fireTimeNs": np.linspace(0, 1e9 / self.update_rate, self.vertical.samples * self.horizontal.samples).astype(int).tolist(),
                    }
                ],

                "intensityMappingType": "LINEAR"
            }

    @classmethod
    def create_temp_config(cls, config: "SensorLidar.Config") -> str:
        """
        Creates a temporary config file for the lidar sensor.
        Args:
            config(SensorLidar.Config): The config to use for the lidar sensor.
        Returns:
            str: The path to the temporary config file.
        """
        with tempfile.NamedTemporaryFile(
            'w',
            dir=cls.config_base_dir,
            suffix='.json',
            delete=False,
        ) as f:
            f.write(
                json.dumps(
                    {
                        "class": "sensor",
                        "type": "lidar",
                        "name": "CustomLidar",
                        "driveWorksId": "GENERIC",
                        "profile": config.as_profile()
                    }
                )
            )
            import sys
            return os.path.join(cls.config_base_prefix, os.path.splitext(os.path.basename(f.name))[0])

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
        prim_path = os.path.join(base_prim, self.parent_frame, self.name)
        _, lidar = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path=prim_path,
            config=self.create_temp_config(self.config),
            translation=self.translation.tuple(),
            orientation=self.rotation.Quatd(),
        )
        if lidar:
            self.prim_path = prim_path

    def publish(self, base_topic: str):
        """
        Publishes the lidar sensor to ros2.
        Args:
            base_topic(str): The base topic path for the robot.
        """

        if self.prim_path is None:
            raise RuntimeError('Lidar not simulated. Call simulate() first.')

        graph = Graph(os.path.join(self.prim_path, "LidarPublisher"))

        get_camera_prim = graph.node('get_camera_prim', 'omni.replicator.core.OgnGetPrimAtPath')
        on_playback_tick = graph.node("on_playback_tick", "omni.graph.action.OnPlaybackTick")
        render_product = graph.node("render_product", "isaacsim.core.nodes.IsaacCreateRenderProduct")
        lidar_publisher = graph.node("lidar_publisher", "isaacsim.ros2.bridge.ROS2RtxLidarHelper")
        lidar_publisher_points = graph.node("lidar_publisher_points", "isaacsim.ros2.bridge.ROS2RtxLidarHelper")

        # ReadSimTime = graph.node("readSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime")
        # publishTF = graph.node("publishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree")

        get_camera_prim.attribute('paths', [self.prim_path])
        get_camera_prim.connect('prims', render_product, 'cameraPrim')

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

        graph.execute(og.Controller())
