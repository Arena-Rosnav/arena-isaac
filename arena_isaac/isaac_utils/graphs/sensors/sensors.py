
import xml.etree.ElementTree as ET

import carb

from isaac_utils.utils.geom import Rotation, Translation

from . import SensorBase
from .camera import SensorCamera, SensorCameraRGBD
from .contact import SensorContact
from .imu import SensorIMU
from .lidar import SensorLidar


class Sensors:
    def __init__(
        self,
        prim_path: str,
        base_frame: str,
        base_topic: str,
    ):
        self.prim_path: str = prim_path
        self.robot_base_frame: str = base_frame
        self.robot_base_topic: str = base_topic

    def parse_gazebo(self, urdf: str) -> list[SensorBase]:
        """Parse URDF <sensor> tags, returning sensor objects in creation order.

        Each sensor is created independently, a failure in one (unsupported prim,
        bad config) is logged and skipped so the rest of the robot's sensors still
        come up, the URDF order must not let one bad sensor mask the others.
        """

        root = ET.fromstring(urdf)
        links = {name for link in root.findall('.//link') if (name := link.get('name')) is not None}
        sensors_created: list[SensorBase] = []

        for gazebo in root.findall('.//gazebo'):
            reference = gazebo.get('reference')
            if reference is None:
                continue

            for sensor in gazebo.findall('.//sensor'):
                sensor_type = sensor.get('type')
                sensor_name = sensor.get('name')
                if sensor_type is None or sensor_name is None:
                    continue

                try:
                    created = self._spawn_sensor(sensor, sensor_type, sensor_name, reference, links)
                except Exception as error:
                    carb.log_error(f"sensor {sensor_name!r} ({sensor_type}) failed to initialize, skipping: {error}")
                    continue

                if created is not None:
                    sensors_created.append(created)

        return sensors_created

    @staticmethod
    def _optical_frame(sensor: ET.Element, reference: str, links: set[str]) -> str:
        """REP-103 optical frame for camera messages. Isaac's camera writers emit
        optical-axes data (z forward), so stamping the body-convention link frame
        tilts every consumer's view by 90 degrees."""
        declared = sensor.findtext('./optical_frame_id')
        if declared:
            return declared.strip()
        candidate = f"{reference.removesuffix('_frame')}_optical_frame"
        if candidate in links:
            return candidate
        carb.log_warn(f"camera under {reference!r} has no optical frame in the URDF, stamping the body frame")
        return reference

    def _spawn_sensor(self, sensor: ET.Element, sensor_type: str, sensor_name: str, reference: str, links: set[str]) -> SensorBase | None:
        """Build, simulate and publish one URDF sensor, None when the type is unsupported."""

        pose = list(map(float, sensor.findtext('./pose', '0 0 0 0 0 0').split(' ')))
        translation = Translation.parse(pose[:3])
        rotation = Rotation.parse(pose[3:])

        if sensor_type == 'gpu_lidar':
            created: SensorBase = SensorLidar(
                robot_base_frame=self.robot_base_frame,
                parent_frame=reference,
                name=sensor_name,
                config=SensorLidar.Config.parse(sensor),
                translation=translation,
                rotation=rotation,
            )
        elif sensor_type == 'imu':
            created = SensorIMU(
                robot_base_frame=self.robot_base_frame,
                config=SensorIMU.Config.parse(sensor),
                name=sensor_name,
                parent_frame=reference,
            )
        elif sensor_type == 'contact':
            created = SensorContact(
                robot_base_frame=self.robot_base_frame,
                config=SensorContact.Config.parse(sensor),
                name=sensor_name,
                parent_frame=reference,
            )
        elif sensor_type == 'camera':
            created = SensorCamera(
                robot_base_frame=self.robot_base_frame,
                parent_frame=reference,
                config=SensorCamera.Config.parse(sensor),
                name=sensor_name,
                translation=translation,
                rotation=rotation,
                optical_frame=self._optical_frame(sensor, reference, links),
            )
        elif sensor_type == 'rgbd_camera':
            created = SensorCameraRGBD(
                robot_base_frame=self.robot_base_frame,
                parent_frame=reference,
                config=SensorCameraRGBD.Config.parse(sensor),
                name=sensor_name,
                translation=translation,
                rotation=rotation,
                optical_frame=self._optical_frame(sensor, reference, links),
            )
        else:
            return None

        created.simulate(self.prim_path)
        created.publish(self.robot_base_topic)
        return created
