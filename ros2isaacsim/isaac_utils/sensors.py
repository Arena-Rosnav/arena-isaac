from isaacsim.ros2.bridge import read_camera_info
import omni
import omni.graph.core as og
from isaacsim.core.utils import extensions
from isaacsim.core.nodes.scripts.utils import set_target_prims
from isaacsim.core.prims import SingleArticulation
from omni.isaac.sensor import Camera, ContactSensor, IMUSensor, LidarRtx
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd
import omni.isaac.core.utils.numpy.rotations as rot_utils
import omni.kit.commands as commands
from pxr import Gf
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformBroadcaster
from rclpy.node import Node
import subprocess
import time
from isaacsim.core.utils.prims import is_prim_path_valid, get_prim_at_path
extensions.enable_extension("isaacsim.ros2.bridge")


class LidarDataPublisher(Node):
    def __init__(self, env, num_envs=1, lidar_config="Hesai_XT32_SD10"):
        """
        Initializes the LidarDataPublisher node.

        :param env: The simulation environment.
        :param num_envs: Number of robotic environments.
        :param lidar_config: Configuration name for the LiDAR sensor.
        """
        super().__init__("lidar_data_publisher")
        self.create_ros_time_graph()
        self.env = env
        self.num_envs = num_envs
        self.lidar_config = lidar_config
        self.lidar_annotators = []
        self.lidar_publishers = []

        # Initialize Transform Broadcaster
        self.broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = TransformBroadcaster(self)  # For static transforms

        # Initialize ROS2 Publishers and Annotators
        self.initialize_publishers_and_lidars()

        # Initialize Timers
        self.lidar_freq = 15.0  # Hz
        self.lidar_timer = self.create_timer(1.0 / self.lidar_freq, self.publish_lidar_data)

        # Set ROS2 node to use simulation time
        self.set_use_sim_time()

        # Create Static Transforms for LiDAR Frames
        self.create_static_transforms()

    def create_ros_time_graph(self):
        og.Controller.edit(
            {"graph_path": "/ClockGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                    ("OnPlayBack", "omni.graph.action.OnPlaybackTick"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlayBack.outputs:tick", "PublishClock.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("PublishClock.inputs:topicName", "/clock"),
                ],
            },
        )

    def set_use_sim_time(self):
        """
        Configures the ROS2 node to use simulation time.
        """
        # Define the command as a list
        command = ["ros2", "param", "set", "/lidar_data_publisher", "use_sim_time", "true"]

        # Run the command in a non-blocking way
        subprocess.Popen(command)
        self.get_logger().info("Configured ROS2 node to use simulation time.")

    def initialize_publishers_and_lidars(self):
        """
        Initializes ROS2 publishers and LiDAR sensors with their annotators.
        Ensures synchronization between publishers and annotators.
        """
        for env_idx in range(self.num_envs):
            # Define the LiDAR sensor path
            if self.num_envs == 1:
                lidar_path = f"/World/envs/env_{env_idx}/Robot/left_shoulder_yaw_link/lidar"
                topic_name = "Robot/lidar/point_cloud"
                frame_id = "Robot/lidar_frame"
            else:
                lidar_path = f"/World/envs/env_{env_idx}/Robot/left_shoulder_yaw_link/lidar_{env_idx}"
                topic_name = f"Robot_{env_idx}/lidar/point_cloud"
                frame_id = f"Robot_{env_idx}/lidar_frame"

            # Create LiDAR sensor
            result, sensor = omni.kit.commands.execute(
                "IsaacSensorCreateRtxLidar",
                path="Lidar",
                parent=f"/World/envs/env_{env_idx}/Robot/left_shoulder_yaw_link",
                config=self.lidar_config,
                translation=(0.2, 0, 0.2),
                orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),  # w, x, y, z
            )

            if not result:
                self.get_logger().error(f"Failed to create LiDAR sensor at {lidar_path}")
                continue  # Skip to the next environment

            # Create a ROS2 publisher for PointCloud2 messages
            publisher = self.create_publisher(PointCloud2, topic_name, 10)
            self.lidar_publishers.append((publisher, frame_id))
            self.get_logger().info(f"Initialized LiDAR publisher on topic: {topic_name}")

            # Create Render Product
            render_product = rep.create.render_product(sensor.GetPath(), [1, 1], name="Isaac")

            # Attach Annotator
            annotator = rep.AnnotatorRegistry.get_annotator("RtxSensorCpuIsaacCreateRTXLidarScanBuffer")
            if annotator is None:
                self.get_logger().error(f"Failed to retrieve LiDAR annotator for {lidar_path}")
                # Remove the publisher since annotator is unavailable
                self.lidar_publishers.pop()
                continue  # Skip to the next environment

            annotator.attach(render_product.path)
            self.lidar_annotators.append(annotator)
            self.get_logger().info(f"Attached annotator to LiDAR at {lidar_path}")

    def create_static_transforms(self):
        """
        Publishes static transforms for each LiDAR frame.
        """
        for i in range(len(self.lidar_publishers)):
            publisher, frame_id = self.lidar_publishers[i]

            # Determine parent frame based on environment
            if self.num_envs == 1:
                parent_frame = "Robot/base_link"
                child_frame = "Robot/lidar_frame"
            else:
                parent_frame = f"Robot_{i}/base_link"
                child_frame = f"Robot_{i}/lidar_frame"

            # Define the static transform
            static_transform = TransformStamped()
            static_transform.header.stamp = self.get_clock().now().to_msg()
            static_transform.header.frame_id = parent_frame
            static_transform.child_frame_id = child_frame

            # Set translation relative to the base_link
            static_transform.transform.translation.x = 0.2
            static_transform.transform.translation.y = 0.0
            static_transform.transform.translation.z = 0.2

            # Set rotation (identity quaternion)
            static_transform.transform.rotation.x = 0.0
            static_transform.transform.rotation.y = 0.0
            static_transform.transform.rotation.z = 0.0
            static_transform.transform.rotation.w = 1.0

            # Broadcast the static transform
            self.static_broadcaster.sendTransform(static_transform)
            self.get_logger().info(f"Published static transform from {parent_frame} to {child_frame}")

    def publish_lidar_data(self):
        """
        Retrieves LiDAR data from annotators and publishes as PointCloud2 messages.
        """
        for i in range(len(self.lidar_publishers)):
            annotator = self.lidar_annotators[i]
            publisher, frame_id = self.lidar_publishers[i]

            # Retrieve LiDAR data
            data = annotator.get_data()
            if data is None or "data" not in data:
                self.get_logger().warn(f"No data available from LiDAR annotator {i}")
                continue

            points = data["data"].reshape(-1, 3)  # Ensure shape is (N, 3)

            if points.size == 0:
                self.get_logger().warn(f"No LiDAR points to publish for environment {i}")
                continue

            # Create PointCloud2 message
            point_cloud_msg = PointCloud2()
            point_cloud_msg.header.stamp = self.get_clock().now().to_msg()
            point_cloud_msg.header.frame_id = frame_id
            point_cloud_msg.height = 1
            point_cloud_msg.width = points.shape[0]
            point_cloud_msg.is_dense = False
            point_cloud_msg.is_bigendian = False
            point_cloud_msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            point_cloud_msg.point_step = 12  # 3 * 4 bytes
            point_cloud_msg.row_step = point_cloud_msg.point_step * points.shape[0]
            point_cloud_msg.data = points.astype(float).tobytes()

            # Publish the PointCloud2 message
            publisher.publish(point_cloud_msg)
            self.get_logger().debug(f"Published LiDAR PointCloud2 message for environment {i} with {points.shape[0]} points")


# Lidar
def lidar_setup(prim_path, name):
    _, lidar = commands.execute(
        "IsaacSensorCreateRtxLidar",
        path=prim_path + "/" + name,
        parent=None,
        config="Example_Rotary",
    )
    # _, lidar = omni.kit.commands.execute(
    #     "IsaacSensorCreateRtxLidar",
    #     path="/lidar",
    #     parent=prim_path,
    #     config="Hesai_XT32_SD10",
    #     # config="Velodyne_VLS128",
    #     translation=(0.2, 0, 0.2),
    #     orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),  # Gf.Quatd is w,i,j,k
    # )
    # lidar = LidarRtx(
    #     prim_path = prim_path + "/Lidar",
    #     name = name,
    #     config_file_name = 'Temp_Config_0',
    # )
    return lidar


def publish_lidar(name, prim_path, lidar):
    lidar_prim_path = lidar.prim_path
    keys = og.Controller.Keys
    (graph_handle, nodes, _, _) = og.Controller.edit(
        {"graph_path": f"{prim_path}/Lidar_Graph"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("RunOnce", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("LidarPublisher", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                ("readSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("LidarPointCloudPublisher", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                ("publishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),


            ],
            keys.SET_VALUES: [
                ("RenderProduct.inputs:cameraPrim", lidar_prim_path),
                ("LidarPublisher.inputs:topicName", f"{name}/lidar_scan"),
                ("LidarPublisher.inputs:frameId", f"base_scan"),
                ("LidarPublisher.inputs:type", f"laser_scan"),
                ("LidarPointCloudPublisher.inputs:frameId", f"base_scan"),
                ("LidarPointCloudPublisher.inputs:topicName", f"{name}/lidar_point_cloud"),
                ("LidarPointCloudPublisher.inputs:type", f"point_cloud"),
                ("publishTF.inputs:targetPrims", [prim_path, prim_path + '/base_scan']),
                # ("publishTF.inputs:topicName", f"{name}/tf"),


            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "RunOnce.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "publishTF.inputs:execIn"),
                ("readSimTime.outputs:simulationTime", "publishTF.inputs:timeStamp"),
                ("RenderProduct.outputs:execOut", "LidarPublisher.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "LidarPublisher.inputs:renderProductPath"),
                ("Context.outputs:context", "LidarPublisher.inputs:context"),
                ("Context.outputs:context", "publishTF.inputs:context"),
                ("RunOnce.outputs:step", "RenderProduct.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "LidarPointCloudPublisher.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "LidarPointCloudPublisher.inputs:renderProductPath"),
                ("Context.outputs:context", "LidarPointCloudPublisher.inputs:context"),
            ],
        },
    )

    # hydra_texture = rep.create.render_product(lidar.GetPath(), [1, 1], name="Isaac")

    # annotator = rep.AnnotatorRegistry.get_annotator("RtxSensorCpuIsaacCreateRTXLidarScanBuffer")
    # annotator.attach(hydra_texture)
    # writer = rep.writers.get("RtxLidarDebugDrawPointCloudBuffer")
    # writer.initialize(topicName=f"{name}/lidar_scan", frameId="sim_lidar")
    # writer.attach(hydra_texture)

    return

# Contact Sensor


def contact_sensor_setup(prim_path):
    contact_sensor = ContactSensor(

        prim_path=prim_path,
        name="Contact_Sensor",
        frequency=60,
        min_threshold=0,
        max_threshold=10000000,
        radius=-1,
    )
    contact_sensor.initialize()
    return contact_sensor


def publish_contact_sensor_info(name, prim_path, link, contact_sensor: ContactSensor):
    contact_sensor_prim = contact_sensor.prim_path
    (graph_handle, nodes, _, _) = og.Controller.edit(
        {"graph_path": f"{prim_path}/{link}_Contact_Sensor"},
        {
            # 2) Create the nodes needed
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadContactSensor", "omni.isaac.sensor.IsaacReadContactSensor"),
                ("ROS2Publisher", "isaacsim.ros2.bridge.ROS2Publisher"),  # ROS2Publisher setup
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ROS2Context.inputs:domain_id", 1),
                ("ReadContactSensor.inputs:csPrim", contact_sensor_prim),
                ("ROS2Publisher.inputs:topicName", f"{name}/{link}/contact_sensor_data"),        # Set topic name
                ("ROS2Publisher.inputs:messagePackage", "isaacsim_msgs"),              # Message package
                ("ROS2Publisher.inputs:messageName", "ContactSensor"),                   # ROS2 message type
            ],
            # 3) Connect each node's pins
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ReadContactSensor.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ROS2Publisher.inputs:execIn"),
                ("ROS2Context.outputs:context", "ROS2Publisher.inputs:context"),
                # ("ReadContactSensor.outputs:inContact","ROS2Publisher.inputs:in_contact"),
                # ("ReadContactSensor.outputs:value", "ROS2Publisher.inputs:force_value"),
            ],

        }
    )

    og_path = graph_handle.get_path_to_graph()
    og.Controller.connect(
        og.Controller.attribute(og_path + "/ReadContactSensor.outputs:inContact"),
        og.Controller.attribute(og_path + "/ROS2Publisher.inputs:in_contact"),
    )
    og.Controller.connect(
        og.Controller.attribute(og_path + "/ReadContactSensor.outputs:value"),
        og.Controller.attribute(og_path + "/ROS2Publisher.inputs:force_value"),
    )

    return

# IMU


def imu_setup(prim_path):
    imu = IMUSensor(
        prim_path=prim_path,
        name="imu",
        frequency=60,
        linear_acceleration_filter_size=10,
        angular_velocity_filter_size=10,
        orientation_filter_size=10,
    )
    # imu.initialize()
    return imu


def publish_imu(name, prim_path, link, imu):
    imu_sensor_prim_path = imu.prim_path
    og.Controller.edit(
        {"graph_path": f"{prim_path}/{link}_IMU"},  # Define the graph path
        {
            # Create the required nodes
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("IsaacReadIMU", "omni.isaac.sensor.IsaacReadIMU"),
                ("ToString", "omni.graph.nodes.ToString"),
                ("PrintText", "omni.graph.ui_nodes.PrintText"),
                ("ROS2PublishImu", "isaacsim.ros2.bridge.ROS2PublishImu")
            ],
            # Connect the nodes
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "IsaacReadIMU.inputs:execIn"),  # Trigger IMU data reading
                ("IsaacReadIMU.outputs:execOut", "ROS2PublishImu.inputs:execIn"),  # Trigger IMU data publishing
                ("ROS2Context.outputs:context", "ROS2PublishImu.inputs:context"),  # Connect ROS2 context
                ("IsaacReadIMU.outputs:angVel", "ROS2PublishImu.inputs:angularVelocity"),  # Pass angular velocity
                ("IsaacReadIMU.outputs:linAcc", "ROS2PublishImu.inputs:linearAcceleration"),  # Pass linear acceleration
                ("IsaacReadIMU.outputs:orientation", "ROS2PublishImu.inputs:orientation"),  # Pass orientation
                ("IsaacReadIMU.outputs:angVel", "ToString.inputs:value"),  # Convert angular velocity for debugging
                ("ToString.outputs:converted", "PrintText.inputs:text")  # Print IMU data to console
            ],
            # Set the node parameters
            og.Controller.Keys.SET_VALUES: [
                ("ROS2Context.inputs:domain_id", 1),  # Set the ROS2 domain ID
                ("IsaacReadIMU.inputs:imuPrim", imu_sensor_prim_path),  # Set the IMU sensor prim path
                ("ROS2PublishImu.inputs:topicName", f"{name}/{link}/imu_data"),  # ROS2 topic name
                ("ROS2PublishImu.inputs:frameId", "imu_link"),  # Frame ID for the ROS2 message
                ("ROS2PublishImu.inputs:publishAngularVelocity", True),  # Enable angular velocity publishing
                ("ROS2PublishImu.inputs:publishLinearAcceleration", True),  # Enable linear acceleration publishing
                ("ROS2PublishImu.inputs:publishOrientation", True),  # Enable orientation publishing
                ("ROS2PublishImu.inputs:queueSize", 10)  # Set the queue size
            ],
        }
    )
    return

# =================================================================================
# ===================================Sensors Camera================================


def camera_set_up(prim_path, name):
    prim = SingleArticulation(prim_path=prim_path)
    pos, ori = prim.get_world_pose()
    camera = Camera(
        prim_path=prim_path + "/" + name,
        name=name,
        translation=pos,
        orientation=ori,
        frequency=10,
        resolution=(640, 480),
    )
    camera.initialize()
    return camera


def publish_camera_info(name, camera: Camera, freq):
    # The following code will link the camera's render product and publish the data to the specified topic name.
    render_product = camera._render_product_path
    step_size = int(60 / freq)
    topic_name = name + "/" + camera.name + "_camera_info"
    queue_size = 1
    node_namespace = ""
    frame_id = camera.prim_path.split("/")[-1]  # This matches what the TF tree is publishing.

    writer = rep.writers.get("ROS2PublishCameraInfo")
    camera_info = read_camera_info(render_product_path=render_product)
    writer.initialize(
        frameId=frame_id,
        nodeNamespace=node_namespace,
        queueSize=queue_size,
        topicName=topic_name,
        width=camera_info["width"],
        height=camera_info["height"],
        projectionType=camera_info["projectionType"],
        k=camera_info["k"].reshape([1, 9]),
        r=camera_info["r"].reshape([1, 9]),
        p=camera_info["p"].reshape([1, 12]),
        physicalDistortionModel=camera_info["physicalDistortionModel"],
        physicalDistortionCoefficients=camera_info["physicalDistortionCoefficients"],
    )
    writer.attach([render_product])

    gate_path = omni.syntheticdata.SyntheticData._get_node_path(
        "PostProcessDispatch" + "IsaacSimulationGate", render_product
    )

    # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)
    return


def publish_pointcloud_from_depth(name, camera: Camera, freq):
    # The following code will link the camera's render product and publish the data to the specified topic name.
    render_product = camera._render_product_path
    step_size = int(60 / freq)
    topic_name = name + "/" + camera.name + "_pointcloud"  # Set topic name to the camera's name
    queue_size = 1
    node_namespace = ""
    frame_id = camera.prim_path.split("/")[-1]  # This matches what the TF tree is publishing.

    # Note, this pointcloud publisher will simply convert the Depth image to a pointcloud using the Camera intrinsics.
    # This pointcloud generation method does not support semantic labelled objects.
    rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
        sd.SensorType.DistanceToImagePlane.name
    )

    writer = rep.writers.get(rv + "ROS2PublishPointCloud")
    writer.initialize(
        frameId=frame_id,
        nodeNamespace=node_namespace,
        queueSize=queue_size,
        topicName=topic_name
    )
    writer.attach([render_product])

    # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
    gate_path = omni.syntheticdata.SyntheticData._get_node_path(
        rv + "IsaacSimulationGate", render_product
    )
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)

    return


def publish_rgb(name, camera: Camera, freq):
    # The following code will link the camera's render product and publish the data to the specified topic name.
    render_product = camera._render_product_path
    step_size = int(60 / freq)
    topic_name = name + "/" + camera.name + "_rgb"
    queue_size = 1
    node_namespace = ""
    frame_id = camera.prim_path.split("/")[-1]  # This matches what the TF tree is publishing.

    rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
    writer = rep.writers.get(rv + "ROS2PublishImage")
    writer.initialize(
        frameId=frame_id,
        nodeNamespace=node_namespace,
        queueSize=queue_size,
        topicName=topic_name
    )
    writer.attach([render_product])

    # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
    gate_path = omni.syntheticdata.SyntheticData._get_node_path(
        rv + "IsaacSimulationGate", render_product
    )
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)

    return


def publish_depth(name, camera: Camera, freq):
    # The following code will link the camera's render product and publish the data to the specified topic name.
    render_product = camera._render_product_path
    step_size = int(60 / freq)
    topic_name = name + "/" + camera.name + "_depth"
    queue_size = 1
    node_namespace = ""
    frame_id = camera.prim_path.split("/")[-1]  # This matches what the TF tree is publishing.

    rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
        sd.SensorType.DistanceToImagePlane.name
    )
    writer = rep.writers.get(rv + "ROS2PublishImage")
    writer.initialize(
        frameId=frame_id,
        nodeNamespace=node_namespace,
        queueSize=queue_size,
        topicName=topic_name
    )
    writer.attach([render_product])

    # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
    gate_path = omni.syntheticdata.SyntheticData._get_node_path(
        rv + "IsaacSimulationGate", render_product
    )
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)

    return


def publish_camera_tf(name, prim_path, camera: Camera):
    camera_prim = camera.prim_path

    if not is_prim_path_valid(camera_prim):
        raise ValueError(f"Camera path '{camera_prim}' is invalid.")

    try:
        # Generate the camera_frame_id. OmniActionGraph will use the last part of
        # the full camera prim path as the frame name, so we will extract it here
        # and use it for the pointcloud frame_id.
        camera_frame_id = camera_prim.split("/")[-1]

        # Generate an action graph associated with camera TF publishing.
        ros_camera_graph_path = f"{prim_path}/CameraTFActionGraph"

        # If a camera graph is not found, create a new one.
        if not is_prim_path_valid(ros_camera_graph_path):
            (ros_camera_graph, _, _, _) = og.Controller.edit(
                {
                    "graph_path": ros_camera_graph_path,
                    "evaluator_name": "execution",
                    "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
                },
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnTick", "omni.graph.action.OnTick"),
                        ("IsaacClock", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("RosPublisher", "isaacsim.ros2.bridge.ROS2PublishClock"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnTick.outputs:tick", "RosPublisher.inputs:execIn"),
                        ("IsaacClock.outputs:simulationTime", "RosPublisher.inputs:timeStamp"),
                    ]
                }
            )

        # Generate 2 nodes associated with each camera: TF from world to ROS camera convention, and world frame.
        og.Controller.edit(
            ros_camera_graph_path,
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("PublishTF_" + camera_frame_id, "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                    ("PublishRawTF_" + camera_frame_id + "_world", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("PublishTF_" + camera_frame_id + ".inputs:topicName", f"{name}/camera_tf"),
                    # Note if topic_name is changed to something else besides "/tf",
                    # it will not be captured by the ROS tf broadcaster.
                    ("PublishRawTF_" + camera_frame_id + "_world.inputs:topicName", f"{name}/camera_tf"),
                    ("PublishRawTF_" + camera_frame_id + "_world.inputs:parentFrameId", camera_frame_id),
                    ("PublishRawTF_" + camera_frame_id + "_world.inputs:childFrameId", camera_frame_id + "_world"),
                    # Static transform from ROS camera convention to world (+Z up, +X forward) convention:
                    ("PublishRawTF_" + camera_frame_id + "_world.inputs:rotation", [0.5, -0.5, 0.5, 0.5]),
                ],
                og.Controller.Keys.CONNECT: [
                    (ros_camera_graph_path + "/OnTick.outputs:tick",
                        "PublishTF_" + camera_frame_id + ".inputs:execIn"),
                    (ros_camera_graph_path + "/OnTick.outputs:tick",
                        "PublishRawTF_" + camera_frame_id + "_world.inputs:execIn"),
                    (ros_camera_graph_path + "/IsaacClock.outputs:simulationTime",
                        "PublishTF_" + camera_frame_id + ".inputs:timeStamp"),
                    (ros_camera_graph_path + "/IsaacClock.outputs:simulationTime",
                        "PublishRawTF_" + camera_frame_id + "_world.inputs:timeStamp"),
                ],
            },
        )
    except Exception as e:
        print(e)

    # Add target prims for the USD pose. All other frames are static.
    set_target_prims(
        primPath=ros_camera_graph_path + "/PublishTF_" + camera_frame_id,
        inputName="inputs:targetPrims",
        targetPrimPaths=[camera_prim],
    )
    return
