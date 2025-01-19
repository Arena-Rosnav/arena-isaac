# Use the isaacsim to import SimulationApp
from isaacsim import SimulationApp

# Setting the config for simulation and make an simulation.
CONFIG = {"renderer": "RayTracedLighting", "headless": False}
simulation_app = SimulationApp(CONFIG)

#import parent directory
from pathlib import Path
import sys
parent_dir = Path(__file__).resolve().parent.parent
sys.path.append(str(parent_dir))

# Import dependencies.
import omni
import omni.graph.core as og
import numpy as np
import yaml
import os
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import extensions, stage
import omni.isaac.nucleus as nucleus
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import set_prim_attribute_value
from omni.isaac.core.world import World
from omni.importer.urdf import _urdf
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd
import omni.kit.commands as commands
import numpy as np
import rclpy
from isaacsim_msgs.srv import ImportUsd, ImportYaml

import isaac_utils
from isaac_utils.services.SpawnWall import spawn_wall
from isaac_utils.services.MovePrim import move_prim
from isaac_utils.services.GetPrimAttributes import get_prim_attr
from isaac_utils.services.DeletePrim import _delete_prim
from isaac_utils.services.UrdfToUsd import convert_urdf_to_usd

from isaac_utils.sensors import imu_setup,publish_imu, contact_sensor_setup, publish_contact_sensor_info, camera_set_up,publish_camera_tf,publish_depth,publish_camera_info,publish_pointcloud_from_depth,publish_rgb, lidar_setup,publish_lidar 

#======================================Base======================================
# Setting up world and enable ros2_bridge extentions.
# BACKGROUND_STAGE_PATH = "/background"
# BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"

world = World()
extensions.enable_extension("omni.isaac.ros2_bridge")
simulation_app.update() #update the simulation once for update ros2_bridge.
simulation_context = SimulationContext(stage_units_in_meters=1.0) #currently we use 1m for simulation.

assets_root_path = get_assets_root_path()
print(assets_root_path)
# stage.add_reference_to_stage(assets_root_path, BACKGROUND_STAGE_PATH + BACKGROUND_USD_PATH)

# Setting up URDF importer.
status, import_config = commands.execute("URDFCreateImportConfig")
import_config.merge_fixed_joints = False
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
    publish_camera_info(name, camera, 20)
    publish_depth(name, camera, 20)
    publish_rgb(name, camera, 20)
    publish_pointcloud_from_depth(name, camera, 20)
    publish_camera_tf(name,camera)

    lidar_prim_path = prim_path + "/" + "Lidar"
    lidar = lidar_setup(lidar_prim_path)
    publish_lidar(name, lidar)

    links = ["wheel_left_link","wheel_right_link"]
    for link in links:
        imu_prim_path = prim_path + "/" + link + "/" + "IMU"
        contact_prim_path = prim_path + "/" + link + "/" + "ContactSensor"
        imu = imu_setup(imu_prim_path)
        contact_sensor = contact_sensor_setup(contact_prim_path)
        publish_contact_sensor_info(name,prim_path,link,contact_sensor)
        publish_imu(name,prim_path,link,imu)

    robots.append(prim_path)
    # create default graph.
    og.Controller.edit(
        # default graph name for robots.
        {"graph_path": f"{prim_path}/controller"},
        {
            #Create nodes for the OmniGraph
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick",        "omni.graph.action.OnPlaybackTick"),
                ("ROS2Context",           "omni.isaac.ros2_bridge.ROS2Context"),
                ("ROS2SubscribeTwist",    "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
                ("ScaleStageUnits",       "omni.isaac.core_nodes.OgnIsaacScaleToFromStageUnit"),       
                ("Break3Vector_Linear",   "omni.graph.nodes.BreakVector3"),
                ("Break3Vector_Angular",  "omni.graph.nodes.BreakVector3"),
                ("DifferentialController","omni.isaac.wheeled_robots.DifferentialController"),
                ("ConstantToken0",        "omni.graph.nodes.ConstantToken"),
                ("ConstantToken1",        "omni.graph.nodes.ConstantToken"),
                ("MakeArray",             "omni.graph.nodes.ConstructArray"),
                ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
                ("SubscribeJointState", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
                ("ArticulationController","omni.isaac.core_nodes.IsaacArticulationController"),
                # ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),

            ],
            
            og.Controller.Keys.SET_VALUES: [
                #ROS2 domain ID
                ("ROS2Context.inputs:domain_id", 30),

                #MakeArray size
                ("MakeArray.inputs:arraySize",2),
                
                #ROS2 Subscriber for controlling
                ("ROS2SubscribeTwist.inputs:topicName", "/cmd_vel"),

                #DifferentialController
                ("DifferentialController.inputs:wheelDistance",   0.16),
                ("DifferentialController.inputs:wheelRadius",     0.033),
                ("DifferentialController.inputs:maxWheelSpeed",   10.0),
                ("DifferentialController.inputs:maxLinearSpeed",  2.0),
                ("DifferentialController.inputs:maxAngularSpeed", 2.0),
                ("DifferentialController.inputs:maxAcceleration", 0.0),     
                ("DifferentialController.inputs:maxDeceleration", 0.0),
                ("DifferentialController.inputs:maxAngularAcceleration", 0.0),
                
                
                #SubscribeJointState
                ("PublishJointState.inputs:targetPrim",f"{prim_path}/base_footprint"),
                
                # ArticulationController
                ("ArticulationController.inputs:targetPrim", prim_path),
                ("ConstantToken0.inputs:value",'wheel_left_joint'),
                ("ConstantToken1.inputs:value",'wheel_right_joint'),

            ],
            
            og.Controller.Keys.CREATE_ATTRIBUTES: [
                ("MakeArray.inputs:input1", "token"),
            ],
            # 3) Connect each node's pins
            og.Controller.Keys.CONNECT: [
                # -- Execution flow
                ("OnPlaybackTick.outputs:tick",            "ROS2SubscribeTwist.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick",            "ArticulationController.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                ("ROS2SubscribeTwist.outputs:execOut",      "DifferentialController.inputs:execIn"),

                # -- ROS context to the subscriber
                ("ROS2Context.outputs:context", "ROS2SubscribeTwist.inputs:context"),
                ("ROS2Context.outputs:context", "PublishJointState.inputs:context"),
                
                # -- Scale the linear velocity before splitting it
                ("ROS2SubscribeTwist.outputs:linearVelocity", "ScaleStageUnits.inputs:value"),
                ("ScaleStageUnits.outputs:result",           "Break3Vector_Linear.inputs:tuple"),

                # -- Break the scaled linear velocity into x,y,z
                ("Break3Vector_Linear.outputs:x", "DifferentialController.inputs:linearVelocity"),

                # -- Break the angular velocity into x,y,z (only z used typically)
                ("ROS2SubscribeTwist.outputs:angularVelocity", "Break3Vector_Angular.inputs:tuple"),
                ("Break3Vector_Angular.outputs:z",             "DifferentialController.inputs:angularVelocity"),

                # -- Constant tokens to MakeArray for joint indices
                ("ConstantToken0.inputs:value", "MakeArray.inputs:input0"),
                ("ConstantToken1.inputs:value", "MakeArray.inputs:input1"),
                ("MakeArray.outputs:array",      "ArticulationController.inputs:jointNames"),

                # -- DifferentialController outputs to ArticulationController
                ("DifferentialController.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
            ],
        }
    )
    og.Controller.edit(
        {"graph_path": f"{prim_path}/Odom_Publisher", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("onPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("context", "omni.isaac.ros2_bridge.ROS2Context"),
                ("readSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("computeOdom", "omni.isaac.core_nodes.IsaacComputeOdometry"),
                ("publishOdom", "omni.isaac.ros2_bridge.ROS2PublishOdometry"),
                ("publishRawTF", "omni.isaac.ros2_bridge.ROS2PublishRawTransformTree"),
                ("publishRawTF2", "omni.isaac.ros2_bridge.ROS2PublishRawTransformTree"),
                ("publishTF", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                
            ],
            og.Controller.Keys.SET_VALUES: [
                ("context.inputs:domain_id", 30),
                
                ("computeOdom.inputs:chassisPrim", prim_path + '/base_link'),
                
                ("publishRawTF.inputs:childFrameId",'base_link'),
                # ("publishRawTF.inputs:topicName","/tf"),
                ("publishRawTF.inputs:parentFrameId", 'odom'),
            
                ("publishOdom.inputs:odomFrameId", 'odom'),
                ("publishOdom.inputs:chassisFrameId","base_link"),
                
                ("publishTF.inputs:targetPrims", prim_path + '/base_link'),
                ("publishTF.inputs:parentPrim", prim_path + '/base_link'),
                # ("publishTF.inputs:topicName","/tf"),
                
                ("publishRawTF2.inputs:childFrameId",'odom'),
                # ("publishRawTF2.inputs:topicName","/tf"),
                ("publishRawTF2.inputs:parentFrameId", 'world'),
            ],
            og.Controller.Keys.CONNECT: [
                ("onPlaybackTick.outputs:tick", "computeOdom.inputs:execIn"),
                ("onPlaybackTick.outputs:tick", "publishOdom.inputs:execIn"),
                ("onPlaybackTick.outputs:tick", "publishRawTF.inputs:execIn"),
                ("onPlaybackTick.outputs:tick", "publishRawTF2.inputs:execIn"),
                ("onPlaybackTick.outputs:tick", "publishTF.inputs:execIn"),
                
                ("readSimTime.outputs:simulationTime", "publishOdom.inputs:timeStamp"),
                ("readSimTime.outputs:simulationTime", "publishRawTF.inputs:timeStamp"),
                ("readSimTime.outputs:simulationTime", "publishRawTF2.inputs:timeStamp"),
                ("readSimTime.outputs:simulationTime", "publishTF.inputs:timeStamp"),
                
                ("context.outputs:context", "publishOdom.inputs:context"),
                ("context.outputs:context", "publishRawTF.inputs:context"),
                ("context.outputs:context", "publishRawTF2.inputs:context"),
                ("context.outputs:context", "publishTF.inputs:context"),
                
                ("computeOdom.outputs:angularVelocity", "publishOdom.inputs:angularVelocity"),
                ("computeOdom.outputs:linearVelocity", "publishOdom.inputs:linearVelocity"),
                ("computeOdom.outputs:orientation", "publishOdom.inputs:orientation"),
                ("computeOdom.outputs:position", "publishOdom.inputs:position"),
                ("computeOdom.outputs:orientation", "publishRawTF.inputs:rotation"),
                ("computeOdom.outputs:position", "publishRawTF.inputs:translation"),
                
            ],
        }
    )
    world.stop()
    return response

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
    get_prim_attribute_service = get_prim_attr(controller)
    move_prim_service = move_prim(controller)
    delete_prim_service = _delete_prim(controller)
    wall_spawn_service = spawn_wall(controller)
    import_yaml_service = import_yaml(controller)
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