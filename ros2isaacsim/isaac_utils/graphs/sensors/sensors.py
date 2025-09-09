
import os
import xml.etree.ElementTree as ET

from isaac_utils.utils.geom import Rotation, Translation
from isaac_utils.utils.path import world_path

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

    def parse_gazebo(self, urdf: str):
        """
        Parses the URDF string as an XML file and finds all <sensor> tags.
        Processes each sensor based on its type attribute.
        Args:
            urdf(str): The URDF string.
        """

        root = ET.fromstring(urdf)

        for gazebo in root.findall('.//gazebo'):
            reference = gazebo.get('reference')
            if reference is None:
                continue

            for sensor in gazebo.findall('.//sensor'):
                try:
                    sensor_type = sensor.get('type')
                    sensor_name = sensor.get('name')

                    if sensor_type is None:
                        continue
                    if sensor_name is None:
                        continue

                    pose = list(map(float, sensor.findtext('./pose', '0 0 0 0 0 0').split(' ')))
                    translation = Translation.parse(pose[:3])
                    rotation = Rotation.parse(pose[3:])

                    if sensor_type == 'gpu_lidar':
                        lidar = SensorLidar(
                            robot_base_frame=self.robot_base_frame,
                            parent_frame=reference,
                            name=sensor_name,
                            config=SensorLidar.Config.parse(sensor),
                            translation=translation,
                            rotation=rotation,
                        )
                        lidar.simulate(self.prim_path)
                        lidar.publish(self.robot_base_topic)

                    elif sensor_type == "imu":
                        imu = SensorIMU(
                            robot_base_frame=self.robot_base_frame,
                            config=SensorIMU.Config.parse(sensor),
                            name=sensor_name,
                            parent_frame=reference,
                        )
                        imu.simulate(self.prim_path)
                        imu.publish(self.robot_base_topic)

                    elif sensor_type == 'contact':
                        contact = SensorContact(
                            robot_base_frame=self.robot_base_frame,
                            config=SensorContact.Config.parse(sensor),
                            name=sensor_name,
                        )
                        contact.simulate(self.prim_path)
                        contact.publish(self.robot_base_topic)

                    elif sensor_type == 'camera':
                        camera = SensorCamera(
                            robot_base_frame=self.robot_base_frame,
                            parent_frame=reference,
                            config=SensorCamera.Config.parse(sensor),
                            name=sensor_name,
                            translation=translation,
                            rotation=rotation,
                        )
                        camera.simulate(self.prim_path)
                        camera.publish(self.robot_base_topic)

                    elif sensor_type == 'rgbd_camera':
                        camera = SensorCameraRGBD(
                            robot_base_frame=self.robot_base_frame,
                            parent_frame=reference,
                            config=SensorCamera.Config.parse(sensor),
                            name=sensor_name,
                            translation=translation,
                            rotation=rotation,
                        )
                        camera.simulate(self.prim_path)
                        camera.publish(self.robot_base_topic)

                except Exception as e:
                    raise
                    pass
