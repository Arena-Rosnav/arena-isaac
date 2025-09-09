# fmt: off

# preload attrs
import arena_simulation_setup
import arena_simulation_setup.utils.cattrs

# Use the isaacsim to import SimulationApp
from isaacsim import SimulationApp

# Setting the config for simulation and make an simulation.
CONFIG = {"renderer": "Wireframe", "headless": False}
#import parent directory
import sys
from pathlib import Path

simulation_app = SimulationApp(CONFIG)
parent_dir = Path(__file__).resolve().parent.parent
sys.path.insert(0,str(parent_dir))


# Import Isaac Sim dependencies
import random

import carb
import omni.kit.commands as commands
import omni.timeline
import omni.usd
import yaml
from isaac_utils.utils.assets import get_assets_root_path_safe
from omni.importer.urdf import _urdf
from omni.isaac.core import SimulationContext, World
from omni.isaac.core.utils import extensions, prims, stage
from pxr import Sdf

EXTENSIONS_PEOPLE = [
    'omni.anim.people', 
    'omni.anim.navigation.bundle', 
    'omni.anim.timeline',
    'omni.anim.graph.bundle', 
    'omni.anim.graph.core', 
    'omni.anim.graph.ui',
    'omni.anim.retarget.bundle', 
    'omni.anim.retarget.core',
    'omni.anim.retarget.ui', 
    'omni.kit.scripting',
    'omni.graph.nodes',
    'omni.anim.curve.core',
    'omni.anim.navigation.core'
]

EXTENSIONS_MATERIAL = [
    'omni.kit.material.library',
    'omni.kit.browser.material',
    'omni.kit.browser.asset',
    'omni.kit.window.material'
]
for ext_people in EXTENSIONS_PEOPLE:
    extensions.enable_extension(ext_people)

for ext_material in EXTENSIONS_MATERIAL:
    extensions.enable_extension(ext_material)
# Update the simulation app with the new extensions
simulation_app.update()

# -------------------------------------------------------------------------------------------------
# These lines are needed to restart the USD stage and make sure that the people extension is loaded
# -------------------------------------------------------------------------------------------------
omni.usd.get_context().new_stage()

extensions.disable_extension("omni.isaac.ros_bridge")
extensions.enable_extension("omni.isaac.ros2_bridge")

import random

import numpy as np

#Import world generation dependencies
import omni.anim.graph.core as ag

#imprt navmesh gen
import omni.anim.navigation.core as nav
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd
import rclpy

# graphs
from isaac_utils.graphs.time import PublishTime
from isaac_utils.managers.door_manager import door_manager

#Import services
from isaac_utils.services import services
from pedestrian.simulator.logic.people_manager import PeopleManager
from rclpy.qos import QoSProfile

# fmt: on
# ======================================Base======================================
# Setting up world and enable ros2_bridge extentions.
# BACKGROUND_STAGE_PATH = "/background"
# BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"
plane_material_paths = [
    'https://omniverse-content-production.s3.us-west-2.amazonaws.com/Materials/2023_1/Base/Wood/Walnut_Planks.mdl',
    # 'https://omniverse-content-production.s3.us-west-2.amazonaws.com/Materials/2023_1/vMaterials_2/Ceramic/Ceramic_Tiles_Glazed_Diamond.mdl',
    # 'https://omniverse-content-production.s3.us-west-2.amazonaws.com/Materials/2023_1/vMaterials_2/Ceramic/Ceramic_Tiles_Glazed_Diamond.mdl'
]
world = World()
world.scene.add_ground_plane(size=100, z_position=0.0)
_stage = omni.usd.get_context().get_stage()
plane_mdl_path = random.choice(plane_material_paths)
plane_mtl_name = plane_mdl_path.split('/')[-1][:-4]
plane_mtl_path = "/World/Looks/PlaneMaterial"
plane_mtl = _stage.GetPrimAtPath(plane_mtl_path)
# if not (plane_mtl and plane_mtl.IsValid()):
#     create_res = omni.kit.commands.execute('CreateMdlMaterialPrimCommand',
#                                                 mtl_url=plane_mdl_path,
#                                                 mtl_name=plane_mtl_name,
#                                                 mtl_path=plane_mtl_path)

#     bind_res = omni.kit.commands.execute('BindMaterialCommand',
#                                             prim_path="/World/groundPlane",
#                                             material_path=plane_mtl_path)
simulation_app.update()  # update the simulation once for update ros2_bridge.
simulation_context = SimulationContext(stage_units_in_meters=1.0)  # currently we use 1m for simulation.
light_1 = prims.create_prim(
    "/World/Light_1",
    "DomeLight",
    position=np.array([1.0, 1.0, 1.0]),
    attributes={
        "inputs:texture:format": "latlong",
        "inputs:intensity": 1000.0,
        "inputs:color": (1.0, 1.0, 1.0)
    }
)
assets_root_path = get_assets_root_path_safe()

# Navmesh config and baking
simulation_app.update()
stage = omni.usd.get_context().get_stage()

omni.kit.commands.execute("CreateNavMeshVolumeCommand",
                          parent_prim_path=Sdf.Path("/World"),
                          layer=stage.GetRootLayer()
                          )
simulation_app.update()

omni.kit.commands.execute(
    'ChangeSetting',
    path='/exts/omni.anim.navigation.core/navMesh/config/agentRadius',
    value=35.0)

omni.kit.commands.execute(
    'ChangeSetting',
    path='/exts/omni.anim.people/navigation_settings/dynamic_avoidance_enabled',
    value=True)
omni.kit.commands.execute(
    'ChangeSetting',
    path='/exts/omni.anim.people/navigation_settings/navmesh_enabled',
    value=True)

inav = nav.acquire_interface()
x = inav.start_navmesh_baking()
simulation_app.update()


# =================================================================================

# ===================================controller====================================
# create controller node for isaacsim.


def create_controller(time=120):
    rclpy.init()
    controller = rclpy.create_node("isaac_controller")

    PublishTime('/World/publish_time')
    for service in services:
        service.create(controller, qos_profile=QoSProfile(depth=2000))
    # Let the DoorManager subscribe to ROS topics on this controller node
    try:
        door_manager.register_node(controller)
    except Exception as e:
        controller.get_logger().warning(f'Failed to register DoorManager with controller: {e}')
    # Enable per-entity logging and filter to show only jackal-related outputs
    try:
        door_manager._log_every_tick = False
        door_manager._log_entity_filter = ['jackal']
        controller.get_logger().info('DoorManager per-tick logging enabled (filter=jackal)')
    except Exception as e:
        controller.get_logger().warning(f'Failed to set DoorManager logging flags: {e}')
    return controller

# =================================================================================

# ======================================main=======================================


def main(args=None):
    """
    Main function to initialize the simulation, create the ROS 2 node,
    and run the simulation loop.
    """
    # Create the ROS 2 controller node. This also calls rclpy.init().
    controller = create_controller()

    world.reset()
    SimulationContext().play()

    try:
        # Main simulation loop
        while simulation_app.is_running():
            # Step the simulation
            simulation_app.update()

            # Update door logic
            door_manager.update()

            # Tick the ROS 2 node
            rclpy.spin_once(controller, timeout_sec=0.0)

    except KeyboardInterrupt:
        controller.get_logger().info('Received KeyboardInterrupt, shutting down.')
    except Exception as e:
        controller.get_logger().error(f'Exception in main loop: {e}')
        import traceback
        import sys
        controller.get_logger().error(traceback.format_exc())
        traceback.print_exc(file=sys.stdout)
    finally:
        # Cleanly shut down the simulation and ROS 2
        controller.get_logger().info('Shutting down ROS 2 node and simulation.')
        controller.destroy_node()
        rclpy.shutdown()
        simulation_app.close()


# =================================================================================
if __name__ == "__main__":
    main()
