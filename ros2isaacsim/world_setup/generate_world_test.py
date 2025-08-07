from pxr import Gf
from isaacsim.core.api.world import World
from isaac_utils.utils.assets import get_assets_root_path_safe
from isaacsim_msgs.srv import SpawnWall
from isaacsim.core.utils.stage import get_current_stage, open_stage
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils import prims
from isaac_utils.services.SpawnWall import wall_spawner
from isaac_utils import scene_based_sdg_utils
from isaac_utils import world_generation_utils
import numpy as np
import sys
from pathlib import Path
import omni.usd
import omni.replicator.core as rep
import carb
from isaacsim import SimulationApp
import yaml
import random
import os
import math
import json
import argparse
config = {}


# Check if there are any config files (yaml or json) are passed as arguments
parser = argparse.ArgumentParser()
parser.add_argument("--config", required=False, help="Include specific config parameters (json or yaml))")
args, unknown = parser.parse_known_args()
args_config = {}
if args.config and os.path.isfile(args.config):
    print("File exist")
    with open(args.config, "r") as f:
        if args.config.endswith(".json"):
            args_config = json.load(f)
        elif args.config.endswith(".yaml"):
            args_config = yaml.safe_load(f)
        else:
            carb.log_warn(f"File {args.config} is not json or yaml, will use default config")
else:
    carb.log_warn(f"File {args.config} does not exist, will use default config")

# If there are specific writer parameters in the input config file make sure they are not mixed with the default ones
if "writer_config" in args_config:
    config["writer_config"].clear()

# Update the default config dictionay with any new parameters or values from the config file
config.update(args_config)

# Create the simulation app with the given launch_config
simulation_app = SimulationApp(launch_config=config["launch_config"])

# Late import of runtime modules (the SimulationApp needs to be created before loading the modules)

parent_dir = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(parent_dir))

# Custom util functions for the example


NUM_FRAMES = 1000

world = World()
world.scene.add_default_ground_plane()

# spawn bounding walls
bounding_walls = config.get("bounding_walls", [])
objects = config.get("objects", [])
for wall in bounding_walls.items():
    response = SpawnWall.Response()
    wall_name = wall[0]
    params = wall[1]
    request = SpawnWall.Request()
    request.name = wall_name
    request.world_path = "/World/BoundingWalls"
    request.start = params["start"]
    request.end = params["end"]
    request.height = params["height"]
    wall_response = wall_spawner(request, response)

# reset world
world.reset()

# Get server path
assets_root_path = get_assets_root_path_safe()
if assets_root_path is None:
    carb.log_error("Could not get nucleus server path, closing application..")
    simulation_app.close()


# Disable capture on play (data generation will be triggered manually)
rep.orchestrator.set_capture_on_play(False)

# Clear any previous semantic data in the loaded stage
if config["clear_previous_semantics"]:
    stage = get_current_stage()
    scene_based_sdg_utils.remove_previous_semantics(stage)

# Spawn a new robot at a random pose
robot_prim = prims.create_prim(
    prim_path="/World/waffle",
    position=(random.uniform(-20, -2), random.uniform(-1, 3), 0),
    orientation=euler_angles_to_quat([0, 0, random.uniform(0, math.pi)]),
    usd_path=config["robots"]['waffle']["usd_path"],
    semantic_label=config["robots"]['waffle']["model"],
)

# Spawn the pallet in front of the robot with a random offset on the Y (pallet's forward) axis
robot_tf = omni.usd.get_world_transform_matrix(robot_prim)
robot_quat_gf = robot_tf.ExtractRotationQuat()
robot_quat_xyzw = (robot_quat_gf.GetReal(), *robot_quat_gf.GetImaginary())

# Register randomization graphs
world_generation_utils.register_objects_spawner(objects, assets_root_path, NUM_FRAMES)
scene_based_sdg_utils.register_cone_placement(robot_prim, assets_root_path, config)
scene_based_sdg_utils.register_wall_spawner("/home/ubuntu/arena4_ws/src/arena/isaac/robot_models/world_1.yaml")

# Spawn a camera in the driver's location looking at the pallet
robot_pos_gf = robot_tf.ExtractTranslation()
driver_cam_pos_gf = robot_pos_gf + Gf.Vec3d(0.0, 0.0, 1.9)

driver_cam = rep.create.camera(
    focus_distance=400.0, focal_length=24.0, clipping_range=(0.1, 10000000.0), name="DriverCam"
)


# Camera looking at the robot from a top view with large min clipping to see the scene through the ceiling
top_view_cam = rep.create.camera(clipping_range=(6.0, 1000000.0), name="TopCam")

# Create render products for the custom cameras and attach them to the writer
resolution = config.get("resolution", (512, 512))
robot_rp = rep.create.render_product(top_view_cam, resolution, name="TopView")
driver_rp = rep.create.render_product(driver_cam, resolution, name="DriverView")
# Disable the render products until SDG to improve perf by avoiding unnecessary rendering
rps = [robot_rp, driver_rp]
for rp in rps:
    rp.hydra_texture.set_updates_enabled(False)


# Setup the randomizations to be triggered every frame
with rep.trigger.on_frame():
    rep.randomizer.spawn_objects()
    # rep.randomizer.spawn_walls()

# Setup the randomizations to be triggered at every nth frame (interval)
with rep.trigger.on_frame(interval=4):
    top_view_cam_min = (robot_pos_gf[0], robot_pos_gf[1], 9)
    top_view_cam_max = (robot_pos_gf[0], robot_pos_gf[1], 11)
    with top_view_cam:
        rep.modify.pose(
            position=rep.distribution.uniform(top_view_cam_min, top_view_cam_max),
            rotation=rep.distribution.uniform((0, -90, -30), (0, -90, 30)),
        )

# Start the SDG
for i in range(NUM_FRAMES):
    print(f"[scene_based_sdg] \t Capturing frame {i}")
    # Trigger the custom event to randomize the cones at specific frames
    if i % 2 == 0:
        rep.utils.send_og_event(event_name="randomize_cones")
    # Trigger any on_frame registered randomizers and the writers (delta_time=0.0 to avoid advancing the timeline)
    rep.orchestrator.step(delta_time=0.0)
    i += 1

# Wait for the data to be written to disk
rep.orchestrator.wait_until_complete()

# Check if the application should keep running after the data generation (debug purposes)
close_app_after_run = config.get("close_app_after_run", False)
if config["launch_config"]["headless"]:
    if not close_app_after_run:
        print(
            "[scene_based_sdg] 'close_app_after_run' is ignored when running headless. The application will be closed."
        )
elif not close_app_after_run:
    print("[scene_based_sdg] The application will not be closed after the run. Make sure to close it manually.")
    while simulation_app.is_running():
        simulation_app.update()
simulation_app.close()
