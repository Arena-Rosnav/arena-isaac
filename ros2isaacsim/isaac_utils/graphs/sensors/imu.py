import os
import xml.etree.ElementTree as ET

import attrs
import omni.graph.core as og
from isaac_utils.graphs import Graph
from isaacsim.core.utils import extensions
from omni.isaac.sensor import IMUSensor

from . import SensorBase

extensions.enable_extension("isaacsim.ros2.bridge")


class SensorIMU(SensorBase):

    @attrs.define
    class Config:
        topic: str = attrs.field(validator=attrs.validators.instance_of(str))
        update_rate: float = attrs.field(converter=attrs.converters.optional(float), default=1.0)

        @classmethod
        def parse(cls, config: ET.Element) -> "SensorIMU.Config":
            return cls(
                topic=config.findtext('.//topic') or (config.findtext('.//remapping', '').split(':=')[-1] or None),
                update_rate=config.findtext('.//update_rate'),
            )

    def __init__(self, robot_base_frame: str, config: Config, name: str, parent_frame: str):
        """
        Initializes the IMU sensor utility.
        Args:
            robot_base_frame (str): The name of the base frame of the robot.
            config (Config): Configuration object for the IMU sensor.
            name (str): The name of the IMU sensor.
            parent_frame (str): The name of the parent frame to which the IMU sensor is attached.
        """
        self.robot_base_frame: str = robot_base_frame
        self.config: SensorIMU.Config = config
        self.name: str = name
        self.parent_frame: str = parent_frame

        self.prim_path: str | None = None

    def simulate(self, base_prim: str):
        imu_sensor = IMUSensor(
            prim_path=os.path.join(base_prim, self.parent_frame),
            name=self.name,
            # frequency=self.config.update_rate, # api is broken
            linear_acceleration_filter_size=10,
            angular_velocity_filter_size=10,
            orientation_filter_size=10,
        )
        if imu_sensor:
            imu_sensor.initialize()
            self.prim_path = imu_sensor.prim_path

    def publish(self, base_topic: str):
        if not self.prim_path:
            raise RuntimeError("Contact sensor not initialized. Call simulate() first.")

        graph = Graph(os.path.join(self.prim_path, 'IMUPublisher'))

        on_playback_tick = graph.node("on_playback_tick", "omni.graph.action.OnPlaybackTick")
        read_imu = graph.node("read_imu", "omni.isaac.sensor.IsaacReadIMU")
        ros2_publish_imu = graph.node("ros2_publish_imu", "isaacsim.ros2.bridge.ROS2PublishImu")
        get_imu_prim = graph.node('get_imu_prim', 'omni.replicator.core.OgnGetPrimAtPath')

        get_imu_prim.attribute('paths', [self.prim_path])
        get_imu_prim.connect('prims', read_imu, 'imuPrim')

        # Connect the nodes
        on_playback_tick.connect("tick", read_imu, "execIn")  # Trigger IMU data reading
        read_imu.connect("execOut", ros2_publish_imu, "execIn")  # Trigger IMU data publishing
        read_imu.connect("angVel", ros2_publish_imu, "angularVelocity")  # Pass angular velocity
        read_imu.connect("linAcc", ros2_publish_imu, "linearAcceleration")  # Pass linear acceleration
        read_imu.connect("orientation", ros2_publish_imu, "orientation")  # Pass orientation

        # Set the node parameters
        ros2_publish_imu.attribute("topicName", os.path.join(base_topic, self.config.topic))  # ROS2 topic name
        ros2_publish_imu.attribute("frameId", os.path.join(self.robot_base_frame, self.parent_frame))
        ros2_publish_imu.attribute("publishAngularVelocity", True)
        ros2_publish_imu.attribute("publishLinearAcceleration", True)
        ros2_publish_imu.attribute("publishOrientation", True)
        ros2_publish_imu.attribute("queueSize", 10)

        graph.execute(og.Controller())
