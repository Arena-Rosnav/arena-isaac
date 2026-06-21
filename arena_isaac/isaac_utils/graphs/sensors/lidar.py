import math
import os
import xml.etree.ElementTree as ET
from collections.abc import Sequence

import attrs
import carb
import isaacsim.core.utils.prims as prim_utils
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor

from isaac_utils.utils.geom import Rotation, Translation

from . import SensorBase, resolve_link_prim

# Built-in RTX rotary configs carry the firing pattern (azimuth/elevation
# sampling) the sensor needs to cast rays. A lidar authored from bare attributes
# scans nothing. The 3D config feeds the point cloud, the 2D the LaserScan.
_LIDAR_CONFIG_POINTS = "Example_Rotary"
_LIDAR_CONFIG_SCAN = "Example_Rotary_2D"


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

        @attrs.define
        class Noise:
            type: str = attrs.field(converter=attrs.converters.optional(str), default='none')
            mean: float = attrs.field(converter=attrs.converters.optional(float), default=0.0)
            stddev: float = attrs.field(converter=attrs.converters.optional(float), default=0.0)

        horizontal: Dimension
        vertical: Dimension
        range: Range
        noise: Noise
        topic: str = attrs.field(validator=attrs.validators.instance_of(str))
        update_rate: float = attrs.field(converter=attrs.converters.optional(float), default=1.0)

        @classmethod
        def parse(cls, config: ET.Element) -> "SensorLidar.Config":
            return cls(
                topic=config.findtext(".//topic") or config.findtext(".//topicName") or 'lidar',
                update_rate=float(config.findtext(".//update_rate") or 1.0),
                horizontal=SensorLidar.Config.Dimension(
                    samples=int(config.findtext(".//scan/horizontal/samples") or 1),
                    resolution=float(config.findtext(".//scan/horizontal/resolution") or 1.0),
                    min_angle=float(config.findtext(".//scan/horizontal/min_angle") or -math.pi),
                    max_angle=float(config.findtext(".//scan/horizontal/max_angle") or +math.pi),
                ),
                vertical=SensorLidar.Config.Dimension(
                    samples=int(config.findtext(".//scan/vertical/samples") or 1),
                    resolution=float(config.findtext(".//scan/vertical/resolution") or 1.0),
                    min_angle=float(config.findtext(".//scan/vertical/min_angle") or -math.pi),
                    max_angle=float(config.findtext(".//scan/vertical/max_angle") or +math.pi),
                ),
                range=SensorLidar.Config.Range(
                    min=float(config.findtext(".//range/min") or 0.0),
                    max=float(config.findtext(".//range/max") or 100.0),
                    resolution=float(config.findtext(".//range/resolution") or 1.0),
                ),
                noise=SensorLidar.Config.Noise(
                    type=config.findtext(".//noise/type") or 'none',
                    mean=float(config.findtext(".//noise/mean") or 0.0),
                    stddev=float(config.findtext(".//noise/stddev") or 0.0),
                ),
            )

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

        self.prim_path_points: str | None = None
        self.prim_path_scan: str | None = None

        self._points_lidar: Lidar | None = None
        self._scan_lidar: Lidar | None = None

        # LidarSensor handles we own, for symmetric teardown.
        self._sensors: list = []

    def _create_lidar(self, prim_path: str, config_name: str) -> Lidar | None:
        try:
            lidar = Lidar.create(
                path=prim_path,
                config=config_name,
                translations=[list(self.translation.tuple())],
                orientations=[list(self.rotation.quat())],
                tick_rate=float(max(1e-3, self.config.update_rate)),
            )
        except Exception as error:
            carb.log_warn(
                f"Lidar create failed for '{self.name}' at '{prim_path}': {error}"
            )
            return None

        prim = prim_utils.get_prim_at_path(lidar.paths[0])
        prim.GetAttribute("omni:sensor:Core:nearRangeM").Set(float(self.config.range.min))
        prim.GetAttribute("omni:sensor:Core:farRangeM").Set(float(self.config.range.max))
        return lidar

    def simulate(self, base_prim: str):
        """
        Simulates the lidar sensor in Isaac Sim.
        Args:
            base_prim(str): The base prim path for the robot.
            translation(Translation): The translation of the lidar sensor relative to the base prim.
            rotation(Rotation): The rotation of the lidar sensor relative to the base prim.
        """

        parent_frame_path = str(self.parent_frame).strip('/')
        link_prim = resolve_link_prim(base_prim, parent_frame_path) if parent_frame_path else base_prim

        points_prim_path = os.path.join(link_prim, f"{self.name}_points")
        scan_prim_path = os.path.join(link_prim, f"{self.name}_scan")

        points_lidar = self._create_lidar(points_prim_path, _LIDAR_CONFIG_POINTS)
        scan_lidar = self._create_lidar(scan_prim_path, _LIDAR_CONFIG_SCAN)

        if points_lidar is None or scan_lidar is None:
            self.prim_path_points = None
            self.prim_path_scan = None
            return

        self._points_lidar = points_lidar
        self._scan_lidar = scan_lidar
        self.prim_path_points = points_prim_path
        self.prim_path_scan = scan_prim_path

    def paths(self) -> Sequence[str]:
        if self.prim_path_points is None or self.prim_path_scan is None:
            return ()
        return (self.prim_path_points, self.prim_path_scan)

    def destroy(self) -> None:
        for sensor in self._sensors:
            try:
                sensor.detach_writers()
            except Exception as error:
                carb.log_warn(f"SensorLidar sensor.detach_writers raised: {error}")
        self._sensors.clear()

    def publish(self, base_topic: str) -> bool:
        """
        Publishes the lidar sensor to ros2.
        Args:
            base_topic(str): The base topic path for the robot.
        """

        if self.prim_path_points is None or self.prim_path_scan is None:
            carb.log_warn(
                f"Lidar publish skipped for '{self.name}': sensor not simulated (points/scan prim paths missing)."
            )
            return False

        scan_topic = os.path.join(base_topic, self.config.topic)
        points_topic = os.path.join(base_topic, self.config.topic, 'points')
        frame_id = f'{self.robot_base_frame}{self.parent_frame}'

        points_ok = True
        try:
            points_sensor = LidarSensor(self._points_lidar, annotators=[])
            points_sensor.attach_writer(
                "RtxLidarROS2PublishPointCloud",
                topicName=points_topic,
                frameId=frame_id,
            )
            self._sensors.append(points_sensor)
        except Exception as error:
            carb.log_warn(
                f"Lidar PointCloud publish failed for '{self.name}': {error}"
            )
            points_ok = False

        scan_ok = True
        try:
            prim = prim_utils.get_prim_at_path(self._scan_lidar.paths[0])
            rotation_rate = float(prim.GetAttribute("omni:sensor:Core:scanRateBaseHz").Get() or 0)
            firing_rate = int(prim.GetAttribute("omni:sensor:Core:patternFiringRateHz").Get() or 0)
            near_range = float(prim.GetAttribute("omni:sensor:Core:nearRangeM").Get() or 0)
            far_range = float(prim.GetAttribute("omni:sensor:Core:farRangeM").Get() or 0)

            if rotation_rate <= 0 or firing_rate <= 0:
                carb.log_warn(
                    f"Lidar LaserScan writer skipped for '{self.name}': "
                    f"invalid rotation_rate={rotation_rate} firing_rate={firing_rate}."
                )
                scan_ok = False
            else:
                scan_sensor = LidarSensor(self._scan_lidar, annotators=[])
                scan_sensor.attach_writer(
                    "RtxLidarROS2PublishLaserScan",
                    topicName=scan_topic,
                    frameId=frame_id,
                    horizontalFov=360.0,
                    horizontalResolution=360.0 * rotation_rate / firing_rate,
                    depthRange=[near_range, far_range],
                    rotationRate=rotation_rate,
                    azimuthRange=[-180.0, 180.0],
                )
                self._sensors.append(scan_sensor)
        except Exception as error:
            carb.log_warn(
                f"Lidar LaserScan publish failed for '{self.name}': {error}"
            )
            scan_ok = False

        return points_ok and scan_ok
