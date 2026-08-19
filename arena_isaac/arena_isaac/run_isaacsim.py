# fmt: off


# preload attrs
import os
import sys
import arena_simulation_setup
import arena_simulation_setup.utils.cattrs

# Use the isaacsim to import SimulationApp
from isaacsim import SimulationApp

def _arg_bool(name: str, default: bool) -> bool:
    if name in sys.argv:
        i = sys.argv.index(name)
        if i + 1 < len(sys.argv):
            return sys.argv[i + 1].lower() in ("true", "1")
    prefix = f"{name.lstrip('-')}:="
    for arg in sys.argv:
        if arg.startswith(prefix):
            return arg[len(prefix):].lower() in ("true", "1")
    return default

def _arg_str(name: str, default: str) -> str:
    if name in sys.argv:
        i = sys.argv.index(name)
        if i + 1 < len(sys.argv):
            return sys.argv[i + 1]
    prefix = f"{name.lstrip('-')}:="
    for arg in sys.argv:
        if arg.startswith(prefix):
            return arg[len(prefix):]
    return default

CONFIG = {
    "renderer": "Wireframe",
    "headless": _arg_bool("--headless", False),
}
PHYSICS_ENGINE = _arg_str("--physics", "physx")
if PHYSICS_ENGINE not in ("physx", "newton"):
    raise ValueError(f"unknown physics engine: {PHYSICS_ENGINE}")
#import parent directory
from pathlib import Path

simulation_app = SimulationApp(CONFIG)
parent_dir = Path(__file__).resolve().parent.parent
sys.path.insert(0,str(parent_dir))

# stdlib
import queue
import random
import traceback

# Import Isaac Sim dependencies

import carb
import omni.kit.commands as commands
import omni.timeline
import omni.usd
import yaml
from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.asset.importer.urdf")
from isaacsim.core.api import SimulationContext, World
from isaacsim.core.utils import extensions, prims

EXTENSIONS_MATERIAL = [
    'omni.kit.material.library',
]

if not CONFIG["headless"]:
    EXTENSIONS_MATERIAL += [
        'omni.kit.browser.material',
        'omni.kit.browser.asset',
        'omni.kit.window.material',
    ]

for ext_material in EXTENSIONS_MATERIAL:
    extensions.enable_extension(ext_material)

import tomllib

def enable_extensions_from_kit(kit_path):
    with open(kit_path, "rb") as f:
        data = tomllib.load(f)
        dependencies = data.get("dependencies", {})

        for ext_name in dependencies.keys():
            print(f"Enabling: {ext_name}")
            enable_extension(ext_name)

KIT_FILE_PATH = (
    "/isaac-sim/apps/isaacsim.exp.base.kit"
    if CONFIG["headless"]
    else "/isaac-sim/apps/isaacsim.exp.full.kit"
)
enable_extensions_from_kit(KIT_FILE_PATH)

# Update the simulation app with the new extensions
for _ in range(100):
    simulation_app.update()

# -------------------------------------------------------------------------------------------------
# These lines are needed to restart the USD stage and make sure that the people extension is loaded
# -------------------------------------------------------------------------------------------------
omni.usd.get_context().new_stage()

for _ext in (
    "isaacsim.ros2.bridge",
    "isaacsim.sensors.physics",
    "isaacsim.sensors.physics.nodes",
    "isaacsim.sensors.camera",
    "isaacsim.sensors.experimental.rtx",
    "omni.graph.nodes",
):
    if not extensions.enable_extension(_ext):
        carb.log_error(f"failed to enable extension: {_ext}")

# Let the freshly-enabled extensions finish starting (native plugins and the
# OG nodes the sensor graphs use register over the next few app updates).
for _ in range(20):
    simulation_app.update()

# RTX lidars and cameras render through the experimental.rtx multi-tick pipeline:
# supportMultiTickRate must be on or their render products never tick and they
# publish nothing. perSensorTickTlas (the per-sensor motion BVH) must stay off,
# with it on Isaac segfaults a few minutes in and basic lidar/camera do not need it.
import carb.settings
_carb_settings = carb.settings.get_settings()
_carb_settings.set("/rtx/hydra/supportMultiTickRate", True)
_carb_settings.set("/rtx/rendering/perSensorTickTlas", False)

# ros2 publisher nodes skip publishing when getSubscriptionCount() sees no
# subscribers, and that count is unreliable (matched DDS subscribers read as 0,
# silently muting joint_states/odom/tf), publish unconditionally like NVIDIA's
# own test configs do.
_carb_settings.set("/exts/isaacsim.ros2.bridge/publish_without_verification", True)

import numpy as np

import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd
from isaacsim.core.simulation_manager import SimulationManager

# rclpy
import rclpy
import rclpy.node
import std_srvs.srv
from isaacsim_msgs.srv import StepSimulation

# graphs
from isaac_utils.graphs.time import PublishTime
from isaac_utils.utils.material import Material, PhysicsParams
from isaac_utils.utils.path import world_path

#Import services
# isaacsim.sensors.physics ships in extsDeprecated, but `isaacsim.sensors` was
# already imported from the sibling sensor extensions, so its cached __path__
# never picks up the deprecated portion and the physics import resolves to an
# empty namespace ("unknown location"). Drop the cached namespace so the import
# below rebuilds __path__ across every enabled portion.
import importlib
importlib.invalidate_caches()
for _mod in [_m for _m in sys.modules if _m == "isaacsim.sensors" or _m.startswith("isaacsim.sensors.physics")]:
    del sys.modules[_mod]
from arena_isaac.services import services, subscriptions
from peds import runtime as pedestrian_runtime
from rclpy.qos import QoSProfile
from arena_isaac import run_after_tick_queue

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
class _NewtonStaleGuard:
    """Tracks structural stage edits that newton cannot see.

    Newton registers no prim-change callbacks with the stage-update interface, so a
    robot spawned during a pause hold never enters the mjwarp model, its articulation
    view then matches nothing ("Physics backend not found"). The main loop bounces
    play-time edits through a pause and rebuilds on the stop->play resume.
    """

    # resyncs on these prim types (or their descendants) never change the physics
    # model, and graph/material/render churn arrives every frame, which would keep
    # the guard dirty forever and starve the sim in a pause/rebuild cycle
    _SKIP_TYPES = frozenset((
        'Shader', 'Material', 'OmniGraph', 'OmniGraphNode',
        'RenderProduct', 'RenderVar', 'RenderSettings',
    ))

    def __init__(self, stage) -> None:
        from pxr import Tf, Usd
        self._stage = stage
        self._dirty = False
        self._quiet = 0
        self._key = Tf.Notice.Register(Usd.Notice.ObjectsChanged, self._on_changed, stage)

    def _relevant(self, path) -> bool:
        prim = self._stage.GetPrimAtPath(path.GetPrimPath())
        while prim and prim.IsValid():
            if str(prim.GetTypeName()) in self._SKIP_TYPES:
                return False
            prim = prim.GetParent()
        return True

    def _on_changed(self, notice, _sender) -> None:
        if any(self._relevant(p) for p in notice.GetResyncedPaths()):
            self._dirty = True
            self._quiet = 0

    def tick(self) -> None:
        self._quiet += 1

    def pending(self, settle_frames: int = 0) -> bool:
        return self._dirty and self._quiet >= settle_frames

    def flush(self, settle_frames: int = 0) -> bool:
        if self.pending(settle_frames):
            self._dirty = False
            return True
        return False


def _newton_sim_time() -> float:
    import isaacsim.physics.newton as newton_ext
    ns = newton_ext.acquire_stage()
    return 0.0 if ns is None else float(ns.sim_time)


def _newton_restore_sim_time(t: float) -> None:
    # newton's init() zeroes physics time on every model rebuild, which publishes a
    # backward /clock jump and wedges every use_sim_time consumer (controller_manager
    # update loop, controller switches), keep it monotonic across rebuilds
    import isaacsim.physics.newton as newton_ext
    ns = newton_ext.acquire_stage()
    if ns is not None and ns.initialized and ns.sim_time < t:
        ns.sim_time = t


def _newton_apply_solver_cfg() -> None:
    import isaacsim.physics.newton as newton_ext
    ns = newton_ext.acquire_stage()
    if ns is None:
        carb.log_warn("arena: newton stage not attached yet, solver cfg not applied")
        return
    cfg = ns.cfg.solver_cfg
    if cfg.solver_type != "mujoco":
        return
    # elliptic cone at impratio 1 can neither turn a skid-steer in place nor
    # grip quadruped feet, 0.05 is bench-validated for both
    cfg.cone = "pyramidal"
    cfg.impratio = 0.05
    # mjwarp specializes its tile kernels on these sizes
    cfg.nconmax = 400
    cfg.njmax = 2400


# matches Isaac's implicit default, pinned so lockstep step projections are exact
PHYSICS_DT = 1.0 / 60.0


def _sim_clock() -> float:
    # what /clock publishes, world.current_time zeroes on every timeline stop
    return float(SimulationManager._simulation_manager_interface.get_simulation_time_monotonic())

newton_guard = None
if PHYSICS_ENGINE == "newton":
    # python.sh does not autoload the newton extensions (only isaac-sim.newton.sh does),
    # and the engine switch must happen before any physics scene exists.
    for _ext in ("isaacsim.physics.newton", "isaacsim.physics.newton.tensors"):
        if not extensions.enable_extension(_ext):
            carb.log_error(f"failed to enable extension: {_ext}")
    for _ in range(20):
        simulation_app.update()
    SimulationManager.switch_physics_engine("newton")
    newton_guard = _NewtonStaleGuard(omni.usd.get_context().get_stage())

world = World(physics_dt=PHYSICS_DT, rendering_dt=PHYSICS_DT)
if PHYSICS_ENGINE == "newton":
    _newton_apply_solver_cfg()
world.scene.add_ground_plane(size=100, z_position=-1.0)
Material.physics(
    parent_prim_path=world_path(),
    key='ground_default',
    params=PhysicsParams(
        static_friction=1.0,
        dynamic_friction=1.0,
        restitution=0.0,
        combine_mode='min',
    ),
).bind_to('/World/groundPlane/collisionPlane')
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
# =================================================================================

# ===================================controller====================================
# create controller node for isaacsim.


class IsaacController(rclpy.node.Node):
    def __init__(self, *args, **kwargs):
        super().__init__(node_name="isaac", *args, **kwargs)
        self._running = True
        self._pending_steps = 0

        self.__pause_srv = self.create_service(
            std_srvs.srv.Trigger,
            os.path.join('isaac/PauseSimulation'),
            self._cb_pause,
        )
        self.__unpause_srv = self.create_service(
            std_srvs.srv.Trigger,
            os.path.join('isaac/UnpauseSimulation'),
            self._cb_unpause,
        )
        self.__step_srv = self.create_service(
            std_srvs.srv.Trigger,
            os.path.join('isaac/StepSimulation'),
            self._cb_step,
        )
        self.__step_n_srv = self.create_service(
            StepSimulation,
            os.path.join('isaac/StepSimulationN'),
            self._cb_step_n,
        )

    def _cb_pause(self, request: std_srvs.srv.Trigger.Request, response: std_srvs.srv.Trigger.Response):
        self._running = False
        response.success = True
        return response

    def _cb_unpause(self, request: std_srvs.srv.Trigger.Request, response: std_srvs.srv.Trigger.Response):
        self._running = True
        response.success = True
        return response

    def _cb_step(self, request: std_srvs.srv.Trigger.Request, response: std_srvs.srv.Trigger.Response):
        self._pending_steps += 1
        response.success = True
        return response

    def _cb_step_n(self, request: StepSimulation.Request, response: StepSimulation.Response):
        if request.steps == 0:
            response.success = False
            response.target_sim_time = 0.0
            response.error_msg = "steps must be > 0"
            return response
        self._pending_steps += request.steps
        response.success = True
        response.target_sim_time = _sim_clock() + request.steps * PHYSICS_DT
        response.error_msg = ""
        return response

    def consume_steps(self, n: int) -> None:
        """Consume the steps a gated frame advanced /clock by. No-op while free-running."""
        if not self._running:
            self._pending_steps = max(0, self._pending_steps - n)

    @property
    def running(self):
        return self._pending_steps > 0 or self._running

    @classmethod
    def wait_for_bridge(cls):
        extensions.enable_extension("isaacsim.ros2.bridge")
        simulation_app.update()

        from isaacsim.ros2.core.bindings._ros2_core import acquire_ros2_core_interface
        ros2_bridge = acquire_ros2_core_interface()
        while not ros2_bridge.get_startup_status():
            simulation_app.update()

        carb.log_info("ROS 2 bridge started successfully!")


# ======================================main=======================================


def main(args=None):
    """
    Main function to initialize the simulation, create the ROS 2 node,
    and run the simulation loop.
    """

    sim = SimulationContext()

    IsaacController.wait_for_bridge()
    rclpy.init(args=[])
    controller = IsaacController()

    for service in services:
        service.create(controller, qos_profile=QoSProfile(depth=2000))

    # latest-wins state streams, shallow history discards stale samples
    for subscription in subscriptions:
        subscription.create(controller, qos_profile=QoSProfile(depth=10))

    PublishTime('/World/publish_time')
    world.reset()
    if PHYSICS_ENGINE == "newton":
        # the solver consumes the extension cfg at first play, before the resume path
        _newton_apply_solver_cfg()

    pedestrian_runtime.initialize(world)

    # set photoreal settings
    import isaac_utils.config.photoreal as photoreal
    if os.environ.get('RENDER_PRESET', 'photoreal') != 'boring':
        photoreal.PRESET_PHOTOREAL.apply()
    else:
        photoreal.PRESET_DEFAULT.apply()

    # hard reset once
    omni.timeline.get_timeline_interface().stop()

    # mainloop
    was_playing: bool = False
    newton_bounce_hold: int = 0
    try:
        while simulation_app.is_running():
            stepped_this_iteration: bool = False
            # bounded drain, a single callback per frame backs up under call bursts
            for _ in range(16):
                rclpy.spin_once(controller, timeout_sec=0)
            if controller.running:
                if newton_guard is not None:
                    newton_guard.tick()
                if newton_bounce_hold > 0:
                    newton_bounce_hold -= 1
                    simulation_app.update()
                elif not was_playing:
                    restore_time = 0.0
                    if newton_guard is not None:
                        # newton's per-frame pump does not survive a plain pause->play
                        # resume (only warmup's two direct steps run afterwards), the
                        # timeline stop->play cycle is the transition the simulation
                        # manager rebuilds around (stop invalidates, play triggers full
                        # warmup), and authored poses are the spawn/teleport poses so
                        # the stage reset is a no-op at arena's episode boundaries
                        newton_guard.flush()
                        restore_time = _newton_sim_time()
                        timeline = omni.timeline.get_timeline_interface()
                        timeline_time = timeline.get_current_time()
                        world.stop()
                        _newton_apply_solver_cfg()
                        if timeline_time > 0.0:
                            # stop rewinds the timeline to zero, but rtx sensor
                            # writers stamp from it, keep it continuous or their
                            # messages fall behind /clock and tf lookups fail
                            timeline.set_current_time(timeline_time)
                    else:
                        # prim deletions during the pause invalidate the tensor
                        # views cached in bridge graph nodes
                        from isaac_utils.graphs import rebuild_graphs
                        rebuild_graphs()
                    world.play()
                    was_playing = True
                    clock_before = _sim_clock()
                    world.step(render=True)
                    controller.consume_steps(round((_sim_clock() - clock_before) / PHYSICS_DT))
                    if restore_time > 0.0:
                        _newton_restore_sim_time(restore_time)
                    stepped_this_iteration = True
                elif newton_guard is not None and newton_guard.pending(settle_frames=30):
                    # spawns during an open clock window never cross a pause->play
                    # boundary, hold a real pause and rebuild on the resume path
                    carb.log_info("arena: newton rebuild pending, bouncing timeline")
                    world.pause()
                    was_playing = False
                    newton_bounce_hold = 30
                else:
                    clock_before = _sim_clock()
                    world.step(render=True)
                    controller.consume_steps(round((_sim_clock() - clock_before) / PHYSICS_DT))
                    stepped_this_iteration = True
            else:
                if was_playing:
                    world.pause()
                    was_playing = False
                simulation_app.update()

            if stepped_this_iteration:
                pending_actions = run_after_tick_queue.qsize()
                for _ in range(pending_actions):
                    try:
                        deferred_action = run_after_tick_queue.get_nowait()
                    except queue.Empty:
                        break

                    try:
                        deferred_action()
                    except Exception as e:
                        carb.log_error(f"Deferred action failed: {e}\n{traceback.format_exc()}")

    except KeyboardInterrupt:
        controller.get_logger().info('Received KeyboardInterrupt, shutting down.')
    except Exception as e:
        controller.get_logger().error(f'Exception in main loop: {e}')
        controller.get_logger().error(traceback.format_exc())
        traceback.print_exc(file=sys.stdout)
    finally:
        controller.get_logger().info('Shutting down ROS 2 node and simulation.')
        controller.destroy_node()
        rclpy.shutdown()
        simulation_app.close()


# =================================================================================
if __name__ == "__main__":
    main()
