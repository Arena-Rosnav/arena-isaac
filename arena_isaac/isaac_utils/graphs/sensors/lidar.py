import math
import os
import typing
import xml.etree.ElementTree as ET

import attrs
import carb
import omni
import omni.graph.core as og
import omni.kit.commands
from isaacsim.core.experimental.prims import Prim

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

            def sampled_angles_rad(self) -> list[float]:
                sample_count = int(max(1, self.samples))
                if sample_count == 1:
                    return [float(self.min_angle)]

                min_angle = float(self.min_angle)
                max_angle = float(self.max_angle)
                step = (max_angle - min_angle) / float(sample_count - 1)
                return [min_angle + step * index for index in range(sample_count)]

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

        def vertical_elevation_angles_rad(self) -> list[float]:
            return self.vertical.sampled_angles_rad()

        def as_omnilidar_attributes(self) -> dict[str, object]:
            horizontal_samples = int(max(1, self.horizontal.samples))
            vertical_samples = int(max(1, self.vertical.samples))
            scan_rate_hz = float(max(1e-3, self.update_rate))
            report_rate_hz = int(max(1, round(scan_rate_hz * horizontal_samples)))
            noise_type = str(self.noise.type or "none").strip().lower()
            noise_mean = float(self.noise.mean)
            noise_stddev = float(max(0.0, self.noise.stddev))

            elevation_deg = [
                float(math.degrees(angle_rad)) for angle_rad in self.vertical_elevation_angles_rad()
            ]

            azimuth_deg = [0.0] * vertical_samples
            fire_time_ns = [0] * vertical_samples
            channel_id = list(range(1, vertical_samples + 1))

            start_azimuth_deg_raw = float(math.degrees(self.horizontal.min_angle))
            end_azimuth_deg_raw = float(math.degrees(self.horizontal.max_angle))

            span_deg_raw = end_azimuth_deg_raw - start_azimuth_deg_raw
            span_deg = span_deg_raw
            while span_deg <= 0.0:
                span_deg += 360.0

            if span_deg >= 359.999:
                valid_start_azimuth_deg = 0.0
                valid_end_azimuth_deg = 360.0
            else:
                valid_start_azimuth_deg = start_azimuth_deg_raw
                valid_end_azimuth_deg = end_azimuth_deg_raw

            attributes: dict[str, object] = {
                "omni:sensor:Core:nearRangeM": float(self.range.min),
                "omni:sensor:Core:farRangeM": float(self.range.max),
                "omni:sensor:Core:rangeResolutionM": float(self.range.resolution),
                "omni:sensor:Core:maxReturns": 1,
                "omni:sensor:Core:reportRateBaseHz": report_rate_hz,
                "omni:sensor:Core:scanRateBaseHz": int(max(1, round(scan_rate_hz))),
                "omni:sensor:Core:numberOfChannels": vertical_samples,
                "omni:sensor:Core:numberOfEmitters": vertical_samples,
                "omni:sensor:Core:numLines": vertical_samples,
                "omni:sensor:Core:numRaysPerLine": [horizontal_samples] * vertical_samples,
                "omni:sensor:Core:validStartAzimuthDeg": valid_start_azimuth_deg,
                "omni:sensor:Core:validEndAzimuthDeg": valid_end_azimuth_deg,
                "OmniSensorGenericLidarCoreEmitterStateAPI:s000:azimuthDeg": azimuth_deg,
                "OmniSensorGenericLidarCoreEmitterStateAPI:s000:elevationDeg": elevation_deg,
                "OmniSensorGenericLidarCoreEmitterStateAPI:s000:fireTimeNs": fire_time_ns,
                "OmniSensorGenericLidarCoreEmitterStateAPI:s000:channelId": channel_id,
            }

            if noise_type == "gaussian":
                attributes["omni:sensor:Core:rangeAccuracyM"] = noise_stddev

            return attributes

    _ARRAY_ATTRIBUTE_KEYS = {
        "omni:sensor:Core:numRaysPerLine",
        "OmniSensorGenericLidarCoreEmitterStateAPI:s000:azimuthDeg",
        "OmniSensorGenericLidarCoreEmitterStateAPI:s000:elevationDeg",
        "OmniSensorGenericLidarCoreEmitterStateAPI:s000:fireTimeNs",
        "OmniSensorGenericLidarCoreEmitterStateAPI:s000:channelId",
    }
    _EMITTER_STATE_ARRAY_KEYS: set[str] = {
        "OmniSensorGenericLidarCoreEmitterStateAPI:s000:azimuthDeg",
        "OmniSensorGenericLidarCoreEmitterStateAPI:s000:elevationDeg",
        "OmniSensorGenericLidarCoreEmitterStateAPI:s000:fireTimeNs",
        "OmniSensorGenericLidarCoreEmitterStateAPI:s000:channelId",
    }
    _EMITTER_STATE_REQUIRED_KEYS: set[str] = {
        "OmniSensorGenericLidarCoreEmitterStateAPI:s000:azimuthDeg",
        "OmniSensorGenericLidarCoreEmitterStateAPI:s000:elevationDeg",
        "OmniSensorGenericLidarCoreEmitterStateAPI:s000:fireTimeNs",
        "OmniSensorGenericLidarCoreEmitterStateAPI:s000:channelId",
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

        self.prim_path_points: str | None = None
        self.prim_path_scan: str | None = None

    def _set_prim_attribute(self, prim, prim_path: str, key: str, value: object) -> bool:
        candidate_keys = [key]
        if key.startswith("OmniSensorGenericLidarCoreEmitterStateAPI:s000:"):
            field = key.rsplit(":", 1)[-1]
            candidate_keys.extend(
                [
                    f"omni:sensor:Core:emitterState:s001:{field}",
                    f"omni:sensor:Core:emitterState:s000:{field}",
                ]
            )

        for candidate_key in candidate_keys:
            attr = prim.GetAttribute(candidate_key)
            if not attr.IsValid():
                continue
            try:
                attr.Set(value)
                return True
            except Exception as error:
                carb.log_warn(
                    f"Failed to set lidar attribute '{candidate_key}' on '{prim_path}': {error}"
                )
                return False

        carb.log_warn(
            f"Lidar attribute '{key}' not found on '{prim_path}' (checked {candidate_keys}), skipping."
        )
        return False

    def _apply_prim_attributes(
        self,
        prim,
        prim_path: str,
        attributes: dict[str, object],
        required_keys: set[str] | None = None,
    ) -> bool:
        required_keys = required_keys or set()
        required_ok = True
        for key, value in attributes.items():
            if self._set_prim_attribute(prim, prim_path, key, value):
                continue
            if key in required_keys:
                required_ok = False
        return required_ok

    def _flat_lidar_attributes(self, elevation_rad: float) -> dict[str, object]:
        horizontal_samples = int(max(1, self.config.horizontal.samples))
        scan_rate_hz = float(max(1e-3, self.config.update_rate))
        report_rate_hz = int(max(1, round(scan_rate_hz * horizontal_samples)))
        start_azimuth_deg_raw = float(math.degrees(self.config.horizontal.min_angle))
        end_azimuth_deg_raw = float(math.degrees(self.config.horizontal.max_angle))

        span_deg_raw = end_azimuth_deg_raw - start_azimuth_deg_raw
        span_deg = span_deg_raw
        while span_deg <= 0.0:
            span_deg += 360.0

        if span_deg >= 359.999:
            valid_start_azimuth_deg = 0.0
            valid_end_azimuth_deg = 360.0
        else:
            valid_start_azimuth_deg = start_azimuth_deg_raw
            valid_end_azimuth_deg = end_azimuth_deg_raw

        attributes: dict[str, object] = {
            "omni:sensor:Core:nearRangeM": float(self.config.range.min),
            "omni:sensor:Core:farRangeM": float(self.config.range.max),
            "omni:sensor:Core:rangeResolutionM": float(self.config.range.resolution),
            "omni:sensor:Core:maxReturns": 1,
            "omni:sensor:Core:reportRateBaseHz": report_rate_hz,
            "omni:sensor:Core:scanRateBaseHz": int(max(1, round(scan_rate_hz))),
            "omni:sensor:Core:numberOfChannels": 1,
            "omni:sensor:Core:numberOfEmitters": 1,
            "omni:sensor:Core:numLines": 1,
            "omni:sensor:Core:numRaysPerLine": [horizontal_samples],
            "omni:sensor:Core:validStartAzimuthDeg": valid_start_azimuth_deg,
            "omni:sensor:Core:validEndAzimuthDeg": valid_end_azimuth_deg,
            "OmniSensorGenericLidarCoreEmitterStateAPI:s000:azimuthDeg": [0.0],
            "OmniSensorGenericLidarCoreEmitterStateAPI:s000:elevationDeg": [float(math.degrees(elevation_rad))],
            "OmniSensorGenericLidarCoreEmitterStateAPI:s000:fireTimeNs": [0],
            "OmniSensorGenericLidarCoreEmitterStateAPI:s000:channelId": [1],
        }

        noise_type = str(self.config.noise.type or "none").strip().lower()
        noise_stddev = float(max(0.0, self.config.noise.stddev))
        if noise_type == "gaussian":
            attributes["omni:sensor:Core:rangeAccuracyM"] = noise_stddev

        return attributes

    def _create_lidar_prim(self, prim_path: str, sensor_attributes: dict[str, object]) -> str | None:
        requested_prim_path = prim_path
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

        deferred_scalar_attributes: dict[str, object] = {}
        for deferred_key in (
            "omni:sensor:Core:numberOfEmitters",
            "omni:sensor:Core:numberOfChannels",
        ):
            if deferred_key in scalar_attributes:
                deferred_scalar_attributes[deferred_key] = scalar_attributes.pop(deferred_key)

        try:
            _, lidar = omni.kit.commands.execute(
                "IsaacSensorCreateRtxLidar",
                path=os.path.basename(requested_prim_path),
                parent=os.path.dirname(requested_prim_path),
                config=None,
                translation=self.translation.tuple(),
                orientation=self.rotation.Quatd(),
                force_camera_prim=False,
                **scalar_attributes,
            )
        except Exception as error:
            carb.log_warn(
                f"Lidar simulate failed for '{self.name}' at '{prim_path}': {error}"
            )
            return None

        if not lidar:
            carb.log_warn(
                f"Lidar simulate returned no prim for '{self.name}' at '{prim_path}'."
            )
            return None

        created_prim_path = None
        try:
            created_prim_path = str(lidar.GetPath())
        except Exception:
            created_prim_path = None

        if created_prim_path:
            prim_path = created_prim_path

        if prim_path != requested_prim_path:
            existing_paths, _ = Prim.resolve_paths([requested_prim_path])
            for existing_target in existing_paths:
                omni.kit.commands.execute(
                    "IsaacSimDestroyPrim",
                    prim_path=existing_target,
                )
            try:
                omni.kit.commands.execute(
                    "MovePrim",
                    path_from=prim_path,
                    path_to=requested_prim_path,
                    keep_world_transform=True,
                )
                prim_path = requested_prim_path
            except Exception as error:
                carb.log_warn(
                    f"Failed to move lidar prim from '{prim_path}' to '{requested_prim_path}': {error}"
                )

        prim_wrapper = Prim([prim_path])
        prim = prim_wrapper.prims[0] if prim_wrapper.valid and prim_wrapper.prims else None

        if prim is None:
            carb.log_warn(
                f"Lidar prim '{prim_path}' is invalid after creation for '{self.name}'."
            )
            return None

        emitter_arrays_set_ok = self._apply_prim_attributes(
            prim,
            prim_path,
            array_attributes,
            required_keys=self._EMITTER_STATE_REQUIRED_KEYS,
        )

        if deferred_scalar_attributes:
            if emitter_arrays_set_ok:
                self._apply_prim_attributes(
                    prim,
                    prim_path,
                    deferred_scalar_attributes,
                )
            else:
                carb.log_warn(
                    f"Skipped emitter count overrides on '{prim_path}' because emitter-state arrays were not fully set."
                )

        return prim_path

    def simulate(self, base_prim: str):
        """
        Simulates the lidar sensor in Isaac Sim.
        Args:
            base_prim(str): The base prim path for the robot.
            translation(Translation): The translation of the lidar sensor relative to the base prim.
            rotation(Rotation): The rotation of the lidar sensor relative to the base prim.
        """

        parent_frame_path = str(self.parent_frame).strip('/')

        parent_prim_path = os.path.join(base_prim, parent_frame_path) if parent_frame_path else base_prim
        points_prim_path = os.path.join(parent_prim_path, f"{self.name}_points")
        scan_prim_path = os.path.join(parent_prim_path, f"{self.name}_scan")

        try:
            ensure_path(parent_prim_path)
        except Exception as error:
            self.prim_path_points = None
            self.prim_path_scan = None
            carb.log_warn(
                f"Lidar simulate failed to ensure path '{parent_prim_path}' for '{self.name}': {error}"
            )
            return

        points_attributes = self.config.as_omnilidar_attributes()
        scan_attributes = self._flat_lidar_attributes(0.0)

        created_points = self._create_lidar_prim(points_prim_path, points_attributes)
        created_scan = self._create_lidar_prim(scan_prim_path, scan_attributes)

        if created_points is None or created_scan is None:
            self.prim_path_points = None
            self.prim_path_scan = None
            return

        self.prim_path_points = created_points
        self.prim_path_scan = created_scan

    def _create_lidar_publish_graph(
        self,
        *,
        prim_path: str,
        topic_name: str,
        frame_id: str,
        publish_type: typing.Literal['point_cloud', 'laser_scan'],
        graph_name: str,
    ) -> bool:
        graph = Graph(os.path.join(prim_path, graph_name))

        on_playback_tick = graph.node("on_playback_tick", "omni.graph.action.OnPlaybackTick")
        run_one_frame = graph.node("run_one_frame", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame")
        get_lidar_prim = graph.node("get_lidar_prim", "omni.replicator.core.OgnGetPrimAtPath")
        render_product = graph.node("render_product", "isaacsim.core.nodes.IsaacCreateRenderProduct")
        lidar_publisher = graph.node("lidar_publisher", "isaacsim.ros2.bridge.ROS2RtxLidarHelper")

        get_lidar_prim.attribute("paths", [prim_path])

        render_product.attribute("width", 1)
        render_product.attribute("height", 1)

        lidar_publisher.attribute("topicName", topic_name)
        lidar_publisher.attribute("frameId", frame_id)
        lidar_publisher.attribute("type", publish_type)
        lidar_publisher.attribute("fullScan", publish_type == 'point_cloud')
        lidar_publisher.attribute("frameSkipCount", 0)
        lidar_publisher.attribute("queueSize", 1)

        on_playback_tick.connect("tick", run_one_frame, "execIn")
        run_one_frame.connect("step", get_lidar_prim, "execIn")

        get_lidar_prim.connect("execOut", render_product, "execIn")
        get_lidar_prim.connect("prims", render_product, "cameraPrim")

        render_product.connect("execOut", lidar_publisher, "execIn")
        render_product.connect("renderProductPath", lidar_publisher, "renderProductPath")

        graph.load_extensions()
        return bool(graph.execute(og.Controller()))

    def publish(self, base_topic: str):
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

        points_prim_path = self.prim_path_points
        scan_prim_path = self.prim_path_scan

        scan_topic = os.path.join(base_topic, self.config.topic)
        points_topic = os.path.join(base_topic, self.config.topic, 'points')
        frame_id = f'{self.robot_base_frame}{self.parent_frame}'

        points_ok = self._create_lidar_publish_graph(
            prim_path=points_prim_path,
            topic_name=points_topic,
            frame_id=frame_id,
            publish_type='point_cloud',
            graph_name='LidarPointsPublisher',
        )
        scan_ok = self._create_lidar_publish_graph(
            prim_path=scan_prim_path,
            topic_name=scan_topic,
            frame_id=frame_id,
            publish_type='laser_scan',
            graph_name='LidarScanPublisher',
        )

        return points_ok and scan_ok
