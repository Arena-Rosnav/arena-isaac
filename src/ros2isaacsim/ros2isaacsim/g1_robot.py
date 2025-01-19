from isaacsim import SimulationApp

# Setting the config for simulation and make an simulation.
CONFIG = {"renderer": "RayTracedLighting", "headless": False}
simulation_app = SimulationApp(CONFIG)

import omni
from omni.isaac.core import SimulationContext
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.world import World
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot
import time
import carb
import numpy as np

world = World()
world.scene.add_default_ground_plane()

# Add robot
path = get_assets_root_path()
asset_path = path + '/Isaac/Robots/Unitree/G1/g1.usd'
add_reference_to_stage(usd_path=asset_path, prim_path="/World/G1_Robot")

g1_robot = world.scene.add(Robot(prim_path="/World/G1_Robot", name="g1_robot", position=(0, 0 , 0.7)))

# Teleport parameters
teleport_distance_per_tick = 0.033  # Approx. 1 meter over 30 ticks (assuming 30Hz)

# Main simulation loop
while simulation_app.is_running():
    world.step()
    
    # Get the current position of the pedestrian
    current_position, current_orientation = g1_robot.get_world_pose() 
    
    
    # Teleport the pedestrian forward by a small distance
    new_position = current_position + np.array([teleport_distance_per_tick, 0, 0])  # Moving along X-axis

    omni.kit.commands.execute(
        "IsaacSimTeleportPrim",
        prim_path = "/World/G1_Robot",
        translation = new_position,
    )
    
    # Update previous position
    previous_position = new_position

# Close the simulation app
simulation_app.close()