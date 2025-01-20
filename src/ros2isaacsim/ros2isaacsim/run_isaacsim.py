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

# Import Isaac Sim dependencies
import numpy as np
import yaml
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import extensions, stage
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import set_prim_attribute_value
from omni.isaac.core.world import World
from omni.importer.urdf import _urdf
import omni.kit.commands as commands
import omni.usd 
from omni.isaac.core.utils import prims
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.utils.stage import get_current_stage, open_stage
from pxr import Gf
import random

#Import world generation dependencies
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd

import rclpy

from isaacsim_msgs.srv import ImportUsd, ImportYaml

#Import robot models
from isaac_utils.robot_graphs import assign_robot_model

#Import services
from isaac_utils.services import spawn_wall
from isaac_utils.services import move_prim
from isaac_utils.services import get_prim_attr
from isaac_utils.services import delete_prim
from isaac_utils.services import convert_urdf_to_usd
from isaac_utils.services import import_obstacle

#Import sensors
from isaac_utils.sensors import imu_setup,publish_imu, contact_sensor_setup, publish_contact_sensor_info, camera_set_up,publish_camera_tf,publish_depth,publish_camera_info,publish_pointcloud_from_depth,publish_rgb, lidar_setup,publish_lidar 

#======================================Base======================================
# Setting up world and enable ros2_bridge extentions.

BACKGROUND_STAGE_PATH = "/background"
BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"

world = World()
extensions.enable_extension("omni.isaac.ros2_bridge")
simulation_app.update() #update the simulation once for update ros2_bridge.
simulation_context = SimulationContext(stage_units_in_meters=1.0) #currently we use 1m for simulation.

assets_root_path = get_assets_root_path()

stage.add_reference_to_stage(assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH)

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
    usd_response = ImportUsd.Response()
    # Extract parameters
    name = config['robot']['name']
    model = config['robot']['model']
    usd_path = config['robot']['usd_path']
    prim_path = config['robot']['prim_path']
    control = config['robot']['control']
    position = config['robot']['position']
    orientation = config['robot']['orientation']
    robot_numbers = config['robot']['robot_numbers']
    response.ret = True
    # Prepare the request for ImportUsd service
    for i in range(robot_numbers):
        yaml_request = ImportUsd.Request()
        yaml_request.name = name + f"_{i+1}"
        yaml_request.model = model
        yaml_request.usd_path = usd_path
        yaml_request.prim_path = prim_path
        yaml_request.control = control
        yaml_request.position = np.array(position[i] ,dtype=np.float32)
        yaml_request.orientation = np.array(orientation[i] ,dtype=np.float32)
        print(yaml_request)
        usd_response = usd_importer(yaml_request, usd_response)
        if usd_response.ret is False:
            response.ret = False

    # Pass the response back (optional, depending on how you want to structure your service)
    return response

#============================usd importer service================================
# Usd importer (service) -> bool.
def usd_importer(request, response):
    name = request.name
    model = request.model
    usd_path = request.usd_path
    prim_path = request.prim_path + "/" + name
    position = request.position
    orientation = request.orientation
    
    model_prim = prims.create_prim(
    prim_path=f"/World/{name}",
    position=np.array(position),
    orientation=np.array(orientation),
    usd_path=usd_path,
    semantic_label=model,
    )
    
    # print(model_prim)

    response.ret = True
    if not request.control:
        environments.append(prim_path)
        return response 
    camera_prim_path = prim_path + "/camera_link/" + "Camera"

    camera = camera_set_up(camera_prim_path)
    camera.initialize()
    publish_camera_info(name, camera, 20)
    publish_depth(name, camera, 20)
    publish_rgb(name, camera, 20)
    publish_pointcloud_from_depth(name, camera, 20)
    publish_camera_tf(name,prim_path,camera)

    lidar_prim_path = prim_path + "/base_link/" + "Lidar"
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
    
    model = assign_robot_model(name,prim_path,model)
    
    #publish joint_states and control
    model.control_and_publish_joint_states()
    model.publish_odom_and_tf()
    world.reset()
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
    delete_prim_service = delete_prim(controller)
    wall_spawn_service = spawn_wall(controller)
    import_yaml_service = import_yaml(controller)
    import_obstacle_service = import_obstacle(controller)
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