#autopep8: off
#import order untouched

# Use the isaacsim to import SimulationApp
from isaacsim import SimulationApp

# Setting the config for simulation and make an simulation.
CONFIG = {
    "renderer": "RayTracedLighting",
    "headless": False
}
simulation_app = SimulationApp(CONFIG)

# Import dependencies.
import carb
import math
import omni
import omni.graph.core as og
import usdrt.Sdf
import numpy as np
import matplotlib.pyplot as plt
import yaml
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils.rotations import quat_to_euler_angles
from omni.isaac.core.utils import extensions, stage, nucleus
from omni.isaac.nucleus import get_assets_root_path
from omni.kit.viewport.utility import get_active_viewport
from omni.isaac.core.utils.extensions import get_extension_path_from_name
from omni.isaac.core.utils.prims import delete_prim,get_prim_at_path,set_prim_attribute_value,get_prim_attribute_value,get_prim_attribute_names
from omni.isaac.core.world import World
from omni.importer.urdf import _urdf
from omni.isaac.sensor import Camera, LidarRtx
from omni.isaac.range_sensor import _range_sensor
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd
import omni.isaac.core.utils.numpy.rotations as rot_utils
from pxr import Gf, Usd, UsdGeom
import omni.kit.commands as commands
import numpy as np
import rclpy
from rclpy.node import Node
from isaacsim_msgs.msg import Euler, Quat, Env, Values
from isaacsim_msgs.srv import ImportUsd, ImportUrdf, UrdfToUsd, DeletePrim, GetPrimAttributes, MovePrim, ImportYaml, ScalePrim, SpawnWall, SdfToUsd
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
    usd_path = f"/home/ubuntu/arena4_ws/src/arena/isaac/robot_models/{request.name}.usd"

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
                        srv_name='isaac/urdf_to_usd', 
                        callback=urdf_to_usd)
    return service
#================================================================================
#============================sdf converter service===============================
# URDF convert to usd (service) -> usd_path.
def sdf_to_usd(request, response):
    name = request.name
    sdf_path = request.sdf_path
    usd_path = f"/home/ubuntu/arena4_ws/src/arena/isaac/robot_models/{request.name}.usd"

    status, stage_path = omni.kit.commands.execute(
        "SDFParseAndImportFile",
        sdf_path=sdf_path,
        dest_path=usd_path,
        import_config=import_config,
        get_articulation_root=False,
    )

    response.usd_path = usd_path
    return response

# Urdf importer service callback.
def convert_sdf_to_usd(controller):
    service = controller.create_service(srv_type=SdfToUsd, 
                        srv_name='isaac/sdf_to_usd', 
                        callback=sdf_to_usd)
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

#=========================Get Prims attribute service============================
def get_prim_attributes(request,response):
    name = request.name
    prim = get_prim_at_path(request.prim_path)
    response.translate = np.array(prim.GetAttribute("xformOp:translate").Get(),dtype=np.float32)
    quat = prim.GetAttribute("xformOp:orient").Get()
    response.orient =  np.array([quat.real, quat.imaginary[0], quat.imaginary[1], quat.imaginary[2]], dtype=np.float32)
    response.scale = np.array(prim.GetAttribute("xformOp:scale").Get(),dtype=np.float32)

    return response

def get_prim_attr(controller):
    service = controller.create_service(srv_type=GetPrimAttributes, 
                        srv_name='isaac/get_prim_attributes', 
                        callback=get_prim_attributes)
    return service
#================================================================================

#============================Move Prims service================================
def prim_mover(request,response):
    name = request.name
    prim_path = request.prim_path
    position, orientation = request.values
    position = tuple(position.values)
    orientation = tuple(orientation.values)
    commands.execute(
        "IsaacSimTeleportPrim",
        prim_path = prim_path,
        translation = position,
        rotation = orientation,
    )

    response.ret = True
    return response

def move_prim(controller):
    service = controller.create_service(srv_type=MovePrim, 
                        srv_name='isaac/move_prim', 
                        callback=prim_mover)
    return service
#================================================================================

#============================Scale Prims service=================================
def prim_scaler(request,response):
    name = request.name
    prim_path = request.prim_path
    scale = tuple(request.values)
    commands.execute(
        "IsaacSimScalePrim",
        prim_path = prim_path,
        scale = scale,
    )

    response.ret = True
    return response

def scale_prim(controller):
    service = controller.create_service(srv_type=ScalePrim, 
                        srv_name='isaac/scale_prim', 
                        callback=prim_scaler)
    return service
#================================================================================

#============================Delete Prims service================================
def prim_deleter(request,response):
    name = request.name
    prim_path = request.prim_path
    commands.execute(
        "IsaacSimDestroyPrim",
        prim_path = prim_path,
    )
    response.ret = True
    return response

def _delete_prim(controller):
    service = controller.create_service(srv_type=DeletePrim, 
                        srv_name='isaac/delete_prim', 
                        callback=prim_deleter)
    return service
#================================================================================

#============================Delete Prims service================================
def wall_spawner(request,response):
    #Get service attributes
    name = request.name
    world_path = request.world_path
    height = request.height
    start = np.append(np.array(request.start),height/2)
    end =  np.append(np.array(request.end),height/2)
    start_vec = Gf.Vec3d(*start)
    end_vec = Gf.Vec3d(*end)
    #fixed attributes
    scale=Gf.Vec3f(*[0.05, 1, height/2])
    color=np.array([.2,.2,0.])
    vector_ab = end - start 

    center = (start_vec + end_vec)/2
    length = np.linalg.norm(start[:2] - end[:2])
    angle = math.atan2(vector_ab[1], vector_ab[0])

    #create wall
    stage = omni.usd.get_context().get_stage()
    wall = UsdGeom.Cube.Define(stage, f"{world_path}/{name}")
    wall_transform = UsdGeom.XformCommonAPI(wall)
    wall.AddTranslateOp().Set(center)
    wall_transform.SetScale(scale)
    wall.AddRotateZOp().Set(math.degrees(angle))



    response.ret = True
    return response

def spawn_wall(controller):
    service = controller.create_service(srv_type=SpawnWall, 
                        srv_name='isaac/spawn_wall', 
                        callback=wall_spawner)
    return service
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
#===================================Sensors=======================================

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

# Usd importer service callback.
def import_yaml(controller):
    service = controller.create_service(srv_type=ImportYaml, 
                        srv_name='isaac/import_yaml', 
                        callback=yaml_importer)

def import_usd(controller):
    service = controller.create_service(srv_type=ImportUsd, 
                        srv_name='isaac/import_usd', 
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
    sdf_to_usd_service = convert_sdf_to_usd(controller)
    get_prim_attribute_service = get_prim_attr(controller)
    move_prim_service = move_prim(controller)
    delete_prim_service = _delete_prim(controller)
    prim_scale_service = scale_prim(controller)
    wall_spawn_service = spawn_wall(controller)
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
