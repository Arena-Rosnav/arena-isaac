from isaac_utils.sensors import imu_setup, publish_imu, contact_sensor_setup, publish_contact_sensor_info, camera_set_up, publish_camera_tf, publish_depth, publish_camera_info, publish_pointcloud_from_depth, publish_rgb, lidar_setup, publish_lidar
from isaac_utils.sensors import imu_setup, publish_imu, contact_sensor_setup, publish_contact_sensor_info, camera_set_up, publish_camera_tf, publish_depth, publish_camera_info, publish_pointcloud_from_depth, publish_rgb, lidar_setup, publish_lidar, LidarDataPublisher
from omni.isaac.lab.assets import AssetBaseCfg  # , ArticulationCfg
import omni.replicator.core as rep
from isaac_utils.utils.assets import get_assets_root_path_safe
from isaacsim.core.utils.prims import define_prim, get_prim_at_path
from omni.isaac.lab.sensors import CameraCfg, ContactSensorCfg, RayCasterCfg, patterns
from omni.isaac.lab.utils.assets import ISAACLAB_NUCLEUS_DIR
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.managers import TerminationTermCfg as DoneTerm
from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.managers import RewardTermCfg as RewTerm
from omni.isaac.lab.managers import ObservationTermCfg as ObsTerm
from omni.isaac.lab.managers import ObservationGroupCfg as ObsGroup
from omni.isaac.lab.managers import EventTermCfg as EventTerm
from omni.isaac.lab.scene import InteractiveScene, InteractiveSceneCfg
from omni.isaac.lab.envs import mdp
from omni.isaac.lab.envs import ManagerBasedRLEnvCfg
from omni.isaac.lab.assets import ArticulationCfg
from omni.isaac.lab.actuators import ImplicitActuatorCfg
import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.envs import ManagerBasedRLEnv
import omni.isaac.core.utils.numpy.rotations as rot_utils
from omni.isaac.sensor import Camera
import rclpy
import numpy as np
import omni
from pxr import Gf
import os
import math
import torch
import sys
from pathlib import Path
import argparse

from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Training for wheeled quadruped robot.")
parser.add_argument("--num_envs", type=int, default=128, help="Number of environments to spawn.")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()
parent_dir = Path(__file__).resolve().parent.parent
sys.path.append(str(parent_dir))
# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# import math
#


parent_dir = Path(__file__).resolve().parent.parent
sys.path.append(str(parent_dir))


def setup_sensors(prim_path, name):
    camera_prim_path = prim_path
    camera = camera_set_up(camera_prim_path, "Camera")
    camera.initialize()
    publish_camera_info(name, camera, 20)
    publish_depth(name, camera, 20)
    publish_rgb(name, camera, 20)
    publish_pointcloud_from_depth(name, camera, 20)
    publish_camera_tf(name, prim_path, camera)
    # lidar_prim_path = prim_path
    # lidar = lidar_setup(lidar_prim_path, "Lidar")
    # publish_lidar(name, prim_path, lidar)


H1_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/H1/h1.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=4, solver_velocity_iteration_count=4
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 1.05),
        joint_pos={
            ".*_hip_yaw": 0.0,
            ".*_hip_roll": 0.0,
            ".*_hip_pitch": -0.28,  # -16 degrees
            ".*_knee": 0.79,  # 45 degrees
            ".*_ankle": -0.52,  # -30 degrees
            "torso": 0.0,
            ".*_shoulder_pitch": 0.28,
            ".*_shoulder_roll": 0.0,
            ".*_shoulder_yaw": 0.0,
            ".*_elbow": 0.52,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_yaw", ".*_hip_roll", ".*_hip_pitch", ".*_knee", "torso"],
            effort_limit=300,
            velocity_limit=100.0,
            stiffness={
                ".*_hip_yaw": 150.0,
                ".*_hip_roll": 150.0,
                ".*_hip_pitch": 200.0,
                ".*_knee": 200.0,
                "torso": 200.0,
            },
            damping={
                ".*_hip_yaw": 5.0,
                ".*_hip_roll": 5.0,
                ".*_hip_pitch": 5.0,
                ".*_knee": 5.0,
                "torso": 5.0,
            },
        ),
        "feet": ImplicitActuatorCfg(
            joint_names_expr=[".*_ankle"],
            effort_limit=100,
            velocity_limit=100.0,
            stiffness={".*_ankle": 20.0},
            damping={".*_ankle": 4.0},
        ),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[".*_shoulder_pitch", ".*_shoulder_roll", ".*_shoulder_yaw", ".*_elbow"],
            effort_limit=300,
            velocity_limit=100.0,
            stiffness={
                ".*_shoulder_pitch": 40.0,
                ".*_shoulder_roll": 40.0,
                ".*_shoulder_yaw": 40.0,
                ".*_elbow": 40.0,
            },
            damping={
                ".*_shoulder_pitch": 10.0,
                ".*_shoulder_roll": 10.0,
                ".*_shoulder_yaw": 10.0,
                ".*_elbow": 10.0,
            },
        ),
    },
)


@configclass
class UnitreeSceneCfg(InteractiveSceneCfg):
    # ground
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(size=(100, 100)),
    )

    robot: ArticulationCfg = H1_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # light
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=500.0),
    )
    distant_light = AssetBaseCfg(
        prim_path="/World/DistantLight",
        spawn=sim_utils.DistantLightCfg(color=(0.9, 0.9, 0.9), intensity=2500.0),
        init_state=AssetBaseCfg.InitialStateCfg(rot=(0.738, 0.477, 0.477, 0.0)),
    )

    height_scanner = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/joints",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20)),
        attach_yaw_only=True,
        pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[1.6, 1.0]),
        debug_vis=False,
        mesh_prim_paths=["/World/ground"],
    )


@configclass
class ActionsCfg:
    joint_pos = mdp.JointPositionActionCfg(asset_name="robot", joint_names=[".*"], scale=0.5, use_default_offset=True)


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # observation terms (order preserved)
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel)
        actions = ObsTerm(func=mdp.last_action)

        height_scan = ObsTerm(func=mdp.height_scan,
                              params={"sensor_cfg": SceneEntityCfg("height_scanner")},
                              clip=(-1.0, 1.0))

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Configuration for events."""

    reset_scene = EventTerm(func=mdp.reset_scene_to_default, mode="reset")


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # (1) Constant running reward
    alive = RewTerm(func=mdp.is_alive, weight=1.0)
    # (2) Failure penalty
    terminating = RewTerm(func=mdp.is_terminated, weight=-2.0)
    # (3) Primary task: keep robot upright
    pole_pos = RewTerm(
        func=mdp.base_height_l2,
        weight=1.0,
        params={"asset_cfg": SceneEntityCfg("robot"), "target_height": 0.828},
    )


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    # (1) Time out
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    # (2) The robot fell on the ground
    '''
    robot_on_the_ground = DoneTerm(
        func=mdp.root_height_below_minimum,
        params={"asset_cfg": SceneEntityCfg("robot"), "minimum_height": 0.4},
    )
    '''
    robot_on_the_ground = DoneTerm(
        func=mdp.bad_orientation,
        params={"asset_cfg": SceneEntityCfg("robot"), "limit_angle": math.pi / 3},
    )


@configclass
class CommandsCfg:
    """Command terms for the MDP."""

    # no commands for this MDP
    null = mdp.NullCommandCfg()


@configclass
class CurriculumCfg:
    """Configuration for the curriculum."""

    pass


@configclass
class UnitreeEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the wheeled quadruped environment."""

    # Scene settings
    scene: UnitreeSceneCfg = UnitreeSceneCfg(num_envs=4096, env_spacing=4.0)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    # MDP settings
    curriculum: CurriculumCfg = CurriculumCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    # No command generator
    commands: CommandsCfg = CommandsCfg()

    # Post initialization
    def __post_init__(self) -> None:
        """Post initialization."""
        # general settings
        self.decimation = 2
        # simulation settings
        self.sim.dt = 0.005  # simulation timestep -> 200 Hz physics
        self.episode_length_s = 5
        # viewer settings
        self.viewer.eye = (8.0, 0.0, 5.0)
        # simulation settings
        self.sim.render_interval = self.decimation
        self.sim.disable_contact_processing = True
        self.sim.render.antialiasing_mode = None

        if self.scene.height_scanner is not None:
            self.scene.height_scanner.update_period = self.decimation * self.sim.dt


def create_office_env():
    assets_root_path = get_assets_root_path_safe()
    prim = get_prim_at_path("/World/Office")
    prim = define_prim("/World/Office", "Xform")
    asset_path = assets_root_path + "/Isaac/Environments/Office/office.usd"
    prim.GetReferences().AddReference(asset_path)


def create_warehouse_env():
    assets_root_path = get_assets_root_path_safe()
    prim = get_prim_at_path("/World/Warehouse")
    prim = define_prim("/World/Warehouse", "Xform")
    asset_path = assets_root_path + "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
    prim.GetReferences().AddReference(asset_path)


def main():
    """Main function."""
    # create environment configuration
    env_cfg = UnitreeEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    # setup RL environment
    env = ManagerBasedRLEnv(cfg=env_cfg)
    for i in range(8):
        prim_path = f"/World/envs/env_{i}/Robot/left_shoulder_yaw_link"
        name = f"Robot_{i}"
        setup_sensors(prim_path, name)
    # create_warehouse_env()
    # simulate physics
    count = 0

    rclpy.init()
    lidar_publisher = LidarDataPublisher(env=env, num_envs=8, lidar_config="Hesai_XT32_SD10")
    while simulation_app.is_running():
        with torch.inference_mode():
            # reset
            lidar_publisher.publish_lidar_data()
            rclpy.spin_once(lidar_publisher)
            if count % 300 == 0:
                count = 0
                env.reset()
                print("-" * 80)
                print("[INFO]: Resetting environment...")
            # sample random actions
            joint_vel = torch.randn_like(env.action_manager.action)
            # step the environment
            obs, rew, terminated, truncated, info = env.step(joint_vel)
            # print current orientation of pole
            # print("[Env 0]: Pole joint: ", obs["policy"][0][1].item())

            # update counter
            count += 1

    # close the environment
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
