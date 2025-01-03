# Use the isaacsim to import SimulationApp
from isaacsim import SimulationApp

# Setting the config for simulation and make an simulation.
CONFIG = {"renderer": "RayTracedLighting", "headless": False}
simulation_app = SimulationApp(CONFIG)

# Import dependencies.
import carb
import omni
import omni.graph.core as og
import usdrt.Sdf
import numpy as np
import matplotlib.pyplot as plt
import yaml
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import extensions, stage, nucleus
from omni.isaac.nucleus import get_assets_root_path
from omni.kit.viewport.utility import get_active_viewport
from omni.isaac.core.utils.extensions import get_extension_path_from_name
from omni.isaac.core.utils.prims import delete_prim,get_prim_at_path,set_prim_attribute_value,get_prim_attribute_value,get_prim_attribute_names, is_prim_path_valid
from omni.isaac.core_nodes.scripts.utils import set_target_prims
from omni.isaac.core.world import World
from omni.importer.urdf import _urdf
from omni.isaac.sensor import Camera, ContactSensor, IMUSensor
from omni.isaac.range_sensor import _range_sensor
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd
import omni.isaac.core.utils.numpy.rotations as rot_utils
from pxr import Gf, Usd, UsdGeom
import rclpy
from rclpy.node import Node
from isaacsim_msgs.msg import Euler, Quat, Env
from isaacsim_msgs.srv import ImportUsd, ImportUrdf, UrdfToUsd, ImportYaml
from sensor_msgs.msg import JointState

#======================================Base======================================
# Setting up world and enable ros2_bridge extentions.
BACKGROUND_STAGE_PATH = "/background"

BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"

world = World()
extensions.enable_extension("omni.isaac.ros2_bridge")
simulation_app.update() #update the simulation once for update ros2_bridge.
simulation_context = SimulationContext(stage_units_in_meters=1.0) #currently we use 1m for simulation.

assets_root_path = nucleus.get_assets_root_path()
stage.add_reference_to_stage(assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH)
# Setting up URDF importer.
status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
import_config.merge_fixed_joints = True
import_config.convex_decomp = False
import_config.import_inertia_tensor = False
import_config.self_collision = False
import_config.fix_base = False
import_config.distance_scale = 1
import_config.make_default_prim = True
import_config.default_drive_type = (_urdf.UrdfJointTargetType.JOINT_DRIVE_VELOCITY)
extension_path = _urdf.ImportConfig()

# list devices.
robots = []
environments = []
robot_positions = []
robot_orientation = []
environment_positions = []
environment_orientation = []

#================================================================================
#============================read yaml file===============================
def read_yaml_config(yaml_path):
    with open(yaml_path, 'r') as file:
        config = yaml.safe_load(file)
    return config

def yaml_importer(request, response):
    # Read configuration from YAML file
    yaml_path = request.yaml_path
    config = read_yaml_config(yaml_path)
    
    # Extract parameters
    name = config['robot']['name']
    usd_path = config['robot']['usd_path']
    prim_path = config['robot']['prim_path']
    control = config['robot']['control']
    position = config['robot']['position']
    orientation = config['robot']['orientation']

    # Prepare the request for ImportUsd service
    yaml_request = ImportUsd.Request()
    yaml_request.name = name
    yaml_request.usd_path = usd_path
    yaml_request.prim_path = prim_path
    yaml_request.control = control
    yaml_request.position = np.array(position,dtype=np.float32)
    yaml_request.orientation = np.array(orientation,dtype=np.float32)

    usd_response = usd_importer(yaml_request, response)
    
    # Pass the response back (optional, depending on how you want to structure your service)
    response.ret = usd_response.ret
    return response

#================================================================================
#============================urdf converter service===============================
# URDF convert to usd (service) -> usd_path.
def urdf_to_usd(request, response):
    name = request.name
    urdf_path = request.urdf_path
    usd_path = f"robot_models/Arena_rosnav/User/{request.name}.usd"
    
    status, stage_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        dest_path=usd_path,
        import_config=import_config,
        get_articulation_root=False,
    )
    
    response.usd_path = usd_path
    return response
    
# Urdf importer service callback.
def convert_urdf_to_usd(controller):
    service = controller.create_service(srv_type=UrdfToUsd, 
                        srv_name='urdf_to_usd', 
                        callback=urdf_to_usd)
    return service
#================================================================================
#========================publish environment information=========================
def publish_environemnt_information(node):
    msg = Env()
    msg.robots = robots
    msg.environments = environments
    msg.robot_positions = robot_positions
    msg.robot_orientation = robot_orientation
    msg.environment_positions = environment_positions
    msg.environment_orientaion = environment_orientation
    node.publish(msg)

def create_publish_environment_information(controller):
    controller.create_publisher()
#================================================================================
#=========================multiple usd importer service==========================
## pass
#================================================================================
#============================usd importer service================================
# Usd importer (service) -> bool.
def usd_importer(request, response):
    name = request.name
    usd_path = request.usd_path
    prim_path = request.prim_path + "/" + name
    position = request.position
    orientation = request.orientation
    stage.add_reference_to_stage(usd_path, prim_path)
    set_prim_attribute_value(prim_path, attribute_name="xformOp:translate", value=np.array(position))
    set_prim_attribute_value(prim_path, attribute_name="xformOp:orient", value=np.array(orientation))
    response.ret = True
    if not request.control:
        environments.append(prim_path)
        return response 
    camera_prim_path = prim_path + "/" + "Camera"

    camera = camera_set_up(camera_prim_path)
    camera.initialize()
    publish_camera_info(camera, 20)
    publish_depth(camera, 20)
    publish_rgb(camera, 20)
    publish_pointcloud_from_depth(camera, 20)
    publish_camera_tf(camera)

    lidar_prim_path = prim_path + "/" + "Lidar"
    lidar = lidar_setup(lidar_prim_path)
    publish_lidar(lidar)


    contact_prim_path = prim_path + "/" + "ContactSensor"
    contact_sensor = contact_sensor_setup(contact_prim_path)
    publish_contact_sensor_info(contact_sensor)

    imu_prim_path = prim_path + "/" + "IMU"
    imu = imu_setup(imu_prim_path)
    publish_imu(imu)

    robots.append(prim_path)
    # create default graph.
    og.Controller.edit(
        # default graph name for robots.
        {"graph_path": f"/{name}"},
        {
            # create default nodes.
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ROS2Context", "omni.isaac.ros2_bridge.ROS2Context"),
                ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
                ("SubcribeJoinState", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
            ],
            # connect node inputs and outputs.
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "SubcribeJoinState.inputs:execIn"),
                ("SubcribeJoinState.outputs:execOut", "ArticulationController.inputs:execIn"),
                ("ROS2Context.outputs:context", "PublishJointState.inputs:context"),
                ("ROS2Context.outputs:context", "SubcribeJoinState.inputs:context"),
                ("SubcribeJoinState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"), #config publisher and subcriber.
                ("SubcribeJoinState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                ("SubcribeJoinState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                ("SubcribeJoinState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
            ],
            # set default values for nodes.
            og.Controller.Keys.SET_VALUES: [
                ("ROS2Context.inputs:domain_id", 1),
                ("PublishJointState.inputs:targetPrim", [prim_path + "/base_footprint"]),
                ("ArticulationController.inputs:targetPrim", [prim_path]),
                ("ArticulationController.inputs:robotPath", prim_path),
                ("SubcribeJoinState.inputs:topicName", f"{name}_command"),
                
                ("PublishJointState.inputs:topicName", f"{name}_states"),
            ]
        }
    )
    return response

#=================================================================================
#===================================Sensors Lidar=================================

def lidar_setup(prim_path):
    _, lidar = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path= prim_path,
    parent=None,
    config="Example_Rotary",
    orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
    )
    return lidar

def publish_lidar(lidar):
    hydra_texture = rep.create.render_product(lidar.GetPath(), [1, 1], name="Isaac")

    writer = rep.writers.get("RtxLidar" + "ROS2PublishPointCloud")
    writer.initialize(topicName="lidar_point_cloud", frameId="sim_lidar")
    writer.attach([hydra_texture])

    # Create the debug draw pipeline in the post process graph
    writer = rep.writers.get("RtxLidar" + "DebugDrawPointCloud")
    writer.attach([hydra_texture])

    # Create LaserScan publisher pipeline in the post process graph
    writer = rep.writers.get("RtxLidar" + "ROS2PublishLaserScan")
    writer.initialize(topicName="lidar_scan", frameId="sim_lidar")
    writer.attach([hydra_texture])

    return
#=================================================================================
#===================================Contact Sensor================================
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

def publish_contact_sensor_info(contact_sensor: ContactSensor):
    contact_sensor_prim = contact_sensor.prim_path
    og.Controller.edit(
        {"graph_path": f"/Contact_Sensor"},
        {
            # 2) Create the nodes needed
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ROS2Context", "omni.isaac.ros2_bridge.ROS2Context"),
                ("ReadContactSensor", "omni.isaac.sensor.IsaacReadContactSensor"),
                ("ToString", "omni.graph.nodes.ToString"),
                ("PrintText", "omni.graph.ui_nodes.PrintText"),
                ("ROS2Publisher", "omni.isaac.ros2_bridge.ROS2Publisher")  # ROS2Publisher setup
            ],
            # 3) Connect each node's pins
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ReadContactSensor.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PrintText.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ROS2Publisher.inputs:execIn"),  # Connect exec to ROS2Publisher
                ("ReadContactSensor.outputs:value", "ToString.inputs:value"),  # Convert to string
                ("ToString.outputs:converted", "PrintText.inputs:text"),       # Connect to PrintText
                ("ToString.outputs:converted", "ROS2Publisher.inputs:messageName"),  # Message name is the converted string
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ROS2Context.inputs:domain_id", 1),
                ("ReadContactSensor.inputs:csPrim", contact_sensor_prim),
                ("ROS2Publisher.inputs:topicName", "/contact_sensor_data"),        # Set topic name
                ("ROS2Publisher.inputs:messagePackage", "std_msgs"),              # Message package
                ("ROS2Publisher.inputs:messageSubfolder", "msg"),                 # Message subfolder
                ("ROS2Publisher.inputs:messageName", "String"),                   # ROS2 message type                       # Queue size
                ("ROS2Publisher.inputs:qosProfile", "default"),                   # QoS profile
            ],
        }
    )
    return

#===================================Sensors IMU=================================

def imu_setup(prim_path):
    imu = IMUSensor(
    prim_path=prim_path,
    name="imu",
    frequency=60,
    linear_acceleration_filter_size = 10,
    angular_velocity_filter_size = 10,
    orientation_filter_size = 10,
)
    imu.initialize()
    return imu

def publish_imu(imu):
    imu_sensor_prim_path = imu.prim_path
    og.Controller.edit(
        {"graph_path": f"/IMU_Publisher"},
        {
            # Create necessary nodes
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("IsaacReadIMU", "omni.isaac.sensor.IsaacReadIMU"),
                ("ToString", "omni.graph.nodes.ToString"),
                ("PrintText", "omni.graph.ui_nodes.PrintText"),
                ("ROS2PublishImu", "omni.isaac.ros2_bridge.ROS2PublishImu")
            ],
            # Connect nodes
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "IsaacReadIMU.inputs:execIn"),  # Trigger IMU read on playback tick
                ("IsaacReadIMU.outputs:execOut", "ROS2PublishImu.inputs:execIn"),  # Trigger publishing after IMU read
                ("IsaacReadIMU.outputs:angVel", "ROS2PublishImu.inputs:angularVelocity"),  # Corrected angular velocity
                ("IsaacReadIMU.outputs:linAcc", "ROS2PublishImu.inputs:linearAcceleration"),  # Corrected linear acceleration
                ("IsaacReadIMU.outputs:orientation", "ROS2PublishImu.inputs:orientation"),  # Pass orientation
                ("IsaacReadIMU.outputs:angVel", "ToString.inputs:value"),  # Convert angular velocity for display
                ("ToString.outputs:converted", "PrintText.inputs:text")  # Print IMU data
            ],
            # Set node parameters
            og.Controller.Keys.SET_VALUES: [
                ("IsaacReadIMU.inputs:imuPrim", imu_sensor_prim_path),  # Set IMU sensor prim path
                ("ROS2PublishImu.inputs:topicName", "/imu_data"),  # ROS 2 topic name
                ("ROS2PublishImu.inputs:frameId", "imu_link"),  # Frame ID for the IMU message
                ("ROS2PublishImu.inputs:publishAngularVelocity", True),  # Enable angular velocity publishing
                ("ROS2PublishImu.inputs:publishLinearAcceleration", True),  # Enable linear acceleration publishing
                ("ROS2PublishImu.inputs:publishOrientation", True),  # Enable orientation publishing
                ("ROS2PublishImu.inputs:qosProfile", "default"),  # QoS profile
                ("ROS2PublishImu.inputs:queueSize", 10)  # Queue size
            ],
        }
    )
    return

#=================================================================================

#=================================================================================
#===================================Sensors Camera================================

def camera_set_up(prim_path):
    camera = Camera(
        prim_path = prim_path,
        name = 'camera',
        frequency=20,
        resolution=(640, 480),
    )
    camera.initialize()
    return camera

def publish_camera_info(camera: Camera, freq):
    from omni.isaac.ros2_bridge import read_camera_info
    # The following code will link the camera's render product and publish the data to the specified topic name.
    render_product = camera._render_product_path
    step_size = int(60/freq)
    topic_name = camera.name+"_camera_info"
    queue_size = 1
    node_namespace = ""
    frame_id = camera.prim_path.split("/")[-1] # This matches what the TF tree is publishing.

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

def publish_pointcloud_from_depth(camera: Camera, freq):
    # The following code will link the camera's render product and publish the data to the specified topic name.
    render_product = camera._render_product_path
    step_size = int(60/freq)
    topic_name = camera.name+"_pointcloud" # Set topic name to the camera's name
    queue_size = 1
    node_namespace = ""
    frame_id = camera.prim_path.split("/")[-1] # This matches what the TF tree is publishing.

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

def publish_rgb(camera: Camera, freq):
    # The following code will link the camera's render product and publish the data to the specified topic name.
    render_product = camera._render_product_path
    step_size = int(60/freq)
    topic_name = camera.name+"_rgb"
    queue_size = 1
    node_namespace = ""
    frame_id = camera.prim_path.split("/")[-1] # This matches what the TF tree is publishing.

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

def publish_depth(camera: Camera, freq):
    # The following code will link the camera's render product and publish the data to the specified topic name.
    render_product = camera._render_product_path
    step_size = int(60/freq)
    topic_name = camera.name+"_depth"
    queue_size = 1
    node_namespace = ""
    frame_id = camera.prim_path.split("/")[-1] # This matches what the TF tree is publishing.

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

def publish_camera_tf(camera: Camera):
    camera_prim = camera.prim_path

    if not is_prim_path_valid(camera_prim):
        raise ValueError(f"Camera path '{camera_prim}' is invalid.")

    try:
        # Generate the camera_frame_id. OmniActionGraph will use the last part of
        # the full camera prim path as the frame name, so we will extract it here
        # and use it for the pointcloud frame_id.
        camera_frame_id=camera_prim.split("/")[-1]

        # Generate an action graph associated with camera TF publishing.
        ros_camera_graph_path = "/CameraTFActionGraph"

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
                        ("IsaacClock", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                        ("RosPublisher", "omni.isaac.ros2_bridge.ROS2PublishClock"),
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
                    ("PublishTF_"+camera_frame_id, "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                    ("PublishRawTF_"+camera_frame_id+"_world", "omni.isaac.ros2_bridge.ROS2PublishRawTransformTree"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("PublishTF_"+camera_frame_id+".inputs:topicName", "/tf"),
                    # Note if topic_name is changed to something else besides "/tf",
                    # it will not be captured by the ROS tf broadcaster.
                    ("PublishRawTF_"+camera_frame_id+"_world.inputs:topicName", "/tf"),
                    ("PublishRawTF_"+camera_frame_id+"_world.inputs:parentFrameId", camera_frame_id),
                    ("PublishRawTF_"+camera_frame_id+"_world.inputs:childFrameId", camera_frame_id+"_world"),
                    # Static transform from ROS camera convention to world (+Z up, +X forward) convention:
                    ("PublishRawTF_"+camera_frame_id+"_world.inputs:rotation", [0.5, -0.5, 0.5, 0.5]),
                ],
                og.Controller.Keys.CONNECT: [
                    (ros_camera_graph_path+"/OnTick.outputs:tick",
                        "PublishTF_"+camera_frame_id+".inputs:execIn"),
                    (ros_camera_graph_path+"/OnTick.outputs:tick",
                        "PublishRawTF_"+camera_frame_id+"_world.inputs:execIn"),
                    (ros_camera_graph_path+"/IsaacClock.outputs:simulationTime",
                        "PublishTF_"+camera_frame_id+".inputs:timeStamp"),
                    (ros_camera_graph_path+"/IsaacClock.outputs:simulationTime",
                        "PublishRawTF_"+camera_frame_id+"_world.inputs:timeStamp"),
                ],
            },
        )
    except Exception as e:
        print(e)

    # Add target prims for the USD pose. All other frames are static.
    set_target_prims(
        primPath=ros_camera_graph_path+"/PublishTF_"+camera_frame_id,
        inputName="inputs:targetPrims",
        targetPrimPaths=[camera_prim],
    )
    return

# Usd importer service callback.
def import_yaml(controller):
    service = controller.create_service(srv_type=ImportYaml, 
                        srv_name='import_yaml', 
                        callback=yaml_importer)
    
def import_usd(controller):
    service = controller.create_service(srv_type=ImportUsd, 
                        srv_name='import_usd', 
                        callback=usd_importer)
    return service
#=================================================================================
#===================================controller====================================
# create controller node for isaacsim.
def create_controller(time=120):
    # init controller.
    controller = rclpy.create_node('controller')
    controller.create_publisher
    # init services.
    import_usd_service = import_usd(controller)
    urdf_to_usd_service = convert_urdf_to_usd(controller)
    importYaml_service = import_yaml(controller)
    ##
    return controller

# update the simulation.
def run():
    simulation_app.update()
    simulation_context.play()
#=================================================================================
#======================================main=======================================
def main(arg=None):
    rclpy.init()
    world.scene.add_default_ground_plane()
    controller = create_controller()
    while True:
        run()
        rclpy.spin_once(controller, timeout_sec=0.0)
    controller.destroy_node()
    rclpy.shutdown()
    return
#=================================================================================
if __name__ == "__main__":
    main()
