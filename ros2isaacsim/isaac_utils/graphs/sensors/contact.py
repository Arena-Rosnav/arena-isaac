import os
import xml.etree.ElementTree as ET

import attrs
import omni.graph.core as og
from isaacsim.core.utils import extensions
from omni.isaac.sensor import ContactSensor

from isaac_utils.graphs import Graph

from . import SensorBase

extensions.enable_extension("isaacsim.ros2.bridge")
extensions.enable_extension("omni.isaac.sensor")

# Contact Sensor


class SensorContact(SensorBase):

    @attrs.define
    class Config:
        collision: str = attrs.field(validator=attrs.validators.instance_of(str))
        update_rate: float = attrs.field(converter=attrs.converters.optional(float), default=1.0)

        @classmethod
        def parse(cls, config: ET.Element) -> "SensorContact.Config":
            return cls(
                collision=config.findtext('.//collision'),
                update_rate=config.findtext('.//update_rate'),
            )

    def __init__(self, robot_base_frame: str, config: "SensorContact.Config", name: str):
        """
        Initializes the contact sensor utility.
        Args:
            robot_base_frame (str): Base frame of the robot.
            config (SensorContact.Config): Configuration for the contact sensor.
            name (str): The name of the contact sensor.
        """
        self.robot_base_frame: str = robot_base_frame
        self.config: SensorContact.Config = config
        self.name: str = name

        self.prim_path: str | None = None

    def simulate(self, base_prim: str):
        contact_sensor = ContactSensor(
            prim_path=os.path.join(base_prim, self.name),
            name="Contact_Sensor",
            frequency=self.config.update_rate,
            min_threshold=0,
            max_threshold=10000000,
            radius=-1,
        )
        if contact_sensor:
            contact_sensor.initialize()
            self.prim_path = contact_sensor.prim_path

    def publish(self, base_topic: str):
        if self.prim_path is None:
            raise RuntimeError("Contact sensor not spawned. Call simulate() first.")

        graph = Graph(os.path.join(self.prim_path, 'ContactPublisher'))

        on_playback_tick = graph.node("on_playback_tick", "omni.graph.action.OnPlaybackTick")
        read_contact_sensor = graph.node("read_contact_sensor", "omni.isaac.sensor.IsaacReadContactSensor")
        ros2_publisher = graph.node("ros2_publisher", "isaacsim.ros2.bridge.ROS2Publisher")
        get_contact_prim = graph.node('get_contact_prim', 'omni.replicator.core.OgnGetPrimAtPath')

        get_contact_prim.attribute('paths', [self.prim_path])
        get_contact_prim.connect('prims', read_contact_sensor, 'csPrim')

        on_playback_tick.connect("tick", read_contact_sensor, "execIn")

        ros2_publisher.create_attribute("in_contact", "bool")
        ros2_publisher.create_attribute("force_value", "float")
        read_contact_sensor.connect("inContact", ros2_publisher, "in_contact")
        read_contact_sensor.connect("value", ros2_publisher, "force_value")
        read_contact_sensor.connect("execOut", ros2_publisher, "execIn")

        ros2_publisher.attribute("topicName", os.path.join(base_topic, self.config.collision))
        ros2_publisher.attribute("messagePackage", "isaacsim_msgs")
        ros2_publisher.attribute("messageName", "ContactSensor")

        graph.execute(og.Controller())
