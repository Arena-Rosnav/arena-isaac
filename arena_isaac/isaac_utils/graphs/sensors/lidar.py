from __future__ import annotations

import math
import os
import typing
import xml.etree.ElementTree as ET
from collections.abc import Sequence

import attrs
import carb
import isaacsim.core.utils.prims as prim_utils
from arena_robots.sensors import output_topics
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor

from isaac_utils.utils.geom import Rotation, Translation

from . import SensorBase, join_topic, monotonic_sensor_time, resolve_link_prim

if typing.TYPE_CHECKING:
    from pxr import Usd

# Built-in RTX rotary configs carry a complete firing pattern (azimuth/elevation
# sampling), a lidar authored from bare attributes scans nothing. The presets only
# seed the prim, the URDF scan block then overrides the pattern attributes on it.
# Lidars without a vertical fan use the planar config for both prims so the point
# cloud stays planar.
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
        def parse(cls, config: ET.Element) -> SensorLidar.Config:
            return cls(
                topic=config.findtext("./topic") or config.findtext(".//topic") or config.findtext(".//topicName") or 'lidar',
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
        config: SensorLidar.Config,
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
            carb.log_warn(f"Lidar create failed for '{self.name}' at '{prim_path}': {error}")
            return None

        prim = prim_utils.get_prim_at_path(lidar.paths[0])
        prim.GetAttribute("omni:sensor:Core:nearRangeM").Set(float(self.config.range.min))
        prim.GetAttribute("omni:sensor:Core:farRangeM").Set(float(self.config.range.max))
        self._apply_range_anchor(prim)
        self._apply_scan_pattern(prim)
        return lidar

    def _apply_range_anchor(self, prim: Usd.Prim) -> None:
        """Anchor intensity normalization (1.0 = min-reflectance target at anchor range) to this sensor's own max range."""
        value = float(self.config.range.max)
        for name in ('omni:sensor:Core:minReflectanceRange', 'omni:sensor:Core:minReflectanceRangeM'):
            attr = prim.GetAttribute(name)
            if attr.IsValid():
                current = attr.Get()
                attr.Set(type(current)(value) if current is not None else value)
                return
        available = sorted(a.GetName() for a in prim.GetAttributes() if a.GetName().startswith('omni:sensor:'))
        carb.log_warn(f"lidar '{self.name}': minReflectanceRange attribute not found, available: {available}")

    def _apply_scan_pattern(self, prim: Usd.Prim) -> None:
        """Override the preset firing pattern with the URDF scan block.

        The OmniLidar pattern attribute names are undocumented, so each value is
        resolved through candidate names, a miss logs the prim's sensor attributes
        so the table can be corrected from one live run.
        """
        horizontal = self.config.horizontal
        span = float(horizontal.max_angle) - float(horizontal.min_angle)
        if span <= 0.0:
            carb.log_warn(f"lidar '{self.name}': non-positive azimuth span, keeping preset pattern")
            return
        # The emitter fires uniformly over the full revolution, the sector only
        # gates output, so the firing rate must upscale samples to 360 degrees.
        per_revolution = float(horizontal.samples) * max(2.0 * math.pi / span, 1.0)

        # The azimuth domain is [0, 360] with the seam at 0 (negative values
        # clamp), shipped sector profiles center their FOV on 180, so a signed
        # URDF sector shifts by 180 and a full revolution stays 0..360.
        if span >= 2.0 * math.pi - 1e-6:
            start_deg, end_deg = 0.0, 360.0
        else:
            start_deg = 180.0 + math.degrees(float(horizontal.min_angle))
            end_deg = 180.0 + math.degrees(float(horizontal.max_angle))

        overrides: list[tuple[tuple[str, ...], float]] = [
            (('omni:sensor:Core:startAzimuthDeg', 'omni:sensor:Core:validStartAzimuthDeg'), start_deg),
            (('omni:sensor:Core:endAzimuthDeg', 'omni:sensor:Core:validEndAzimuthDeg'), end_deg),
            (('omni:sensor:Core:rotationRateHz', 'omni:sensor:Core:scanRateBaseHz'), float(self.config.update_rate)),
            (('omni:sensor:Core:reportRateBaseHz',), per_revolution * float(self.config.update_rate)),
        ]
        missing: list[str] = []
        for candidates, value in overrides:
            for name in candidates:
                attr = prim.GetAttribute(name)
                if attr.IsValid():
                    current = attr.Get()
                    attr.Set(type(current)(value) if current is not None else value)
                    break
            else:
                missing.append(candidates[0])
        if missing:
            available = sorted(a.GetName() for a in prim.GetAttributes() if a.GetName().startswith('omni:sensor:'))
            carb.log_warn(f"lidar '{self.name}': pattern attribute(s) {missing} not found, available: {available}")

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

        is_planar = self.config.vertical.samples <= 1
        points_lidar = self._create_lidar(points_prim_path, _LIDAR_CONFIG_SCAN if is_planar else _LIDAR_CONFIG_POINTS)
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
        self._sensors.clear()

    def publish(self, base_topic: str) -> bool:
        """
        Publishes the lidar sensor to ros2.
        Args:
            base_topic(str): The base topic path for the robot.
        """

        if self.prim_path_points is None or self.prim_path_scan is None:
            carb.log_warn(f"Lidar publish skipped for '{self.name}': sensor not simulated (points/scan prim paths missing).")
            return False

        outputs = output_topics('gpu_lidar', self.config.topic)
        scan_topic = join_topic(base_topic, outputs['laserscan'])
        points_topic = join_topic(base_topic, outputs['pointcloud'])
        frame_id = f'{self.robot_base_frame}{self.parent_frame}'

        points_ok = True
        try:
            points_sensor = LidarSensor(self._points_lidar, annotators=[])
            points_sensor.attach_writer(
                "RtxLidarROS2PublishPointCloud",
                topicName=points_topic,
                frameId=frame_id,
                outputIntensity=True,
            )
            self._sensors.append(points_sensor)
            monotonic_sensor_time(str(points_sensor.render_product.GetPath()))
        except Exception as error:
            carb.log_warn(f"Lidar PointCloud publish failed for '{self.name}': {error}")
            points_ok = False

        scan_ok = True
        try:
            prim = prim_utils.get_prim_at_path(self._scan_lidar.paths[0])
            rotation_rate = float(prim.GetAttribute("omni:sensor:Core:scanRateBaseHz").Get() or 0)
            firing_rate = int(prim.GetAttribute("omni:sensor:Core:patternFiringRateHz").Get() or 0)
            near_range = float(prim.GetAttribute("omni:sensor:Core:nearRangeM").Get() or 0)
            far_range = float(prim.GetAttribute("omni:sensor:Core:farRangeM").Get() or 0)

            if rotation_rate <= 0 or firing_rate <= 0:
                carb.log_warn(f"Lidar LaserScan writer skipped for '{self.name}': invalid rotation_rate={rotation_rate} firing_rate={firing_rate}.")
                scan_ok = False
            else:
                scan_sensor = LidarSensor(self._scan_lidar, annotators=[])
                scan_sensor.attach_writer(
                    "RtxLidarROS2PublishLaserScan",
                    topicName=scan_topic,
                    frameId=frame_id,
                    horizontalFov=360.0,
                    horizontalResolution=360.0 / float(self.config.horizontal.samples),
                    depthRange=[near_range, far_range],
                    rotationRate=rotation_rate,
                    azimuthRange=[-180.0, 180.0],
                )
                self._sensors.append(scan_sensor)
                monotonic_sensor_time(str(scan_sensor.render_product.GetPath()))
        except Exception as error:
            carb.log_warn(f"Lidar LaserScan publish failed for '{self.name}': {error}")
            scan_ok = False

        return points_ok and scan_ok
