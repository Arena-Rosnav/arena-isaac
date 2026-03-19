import os
import sys
from pathlib import Path

import carb
import isaac_utils.graphs.joint_states as joint_states
import isaac_utils.graphs.odom as odom
import isaac_utils.graphs.sensors.sensors as sensors
import isaac_utils.graphs.tf as tf
import omni.kit.commands as commands
from isaac_utils.graphs import control
from isaac_utils.managers.door_manager import DoorManager
from isaac_utils.managers.elevator_manager import ElevatorManager
from isaac_utils.utils import geom
from isaac_utils.utils.path import world_path
from isaac_utils.utils.prim import ensure_path

from isaacsim_msgs.srv import SpawnUrdf

from .utils import Service, on_exception

parent_dir = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(parent_dir))


@on_exception('')
def spawn_urdf(request: SpawnUrdf.Request) -> str:
    name = request.name
    urdf_path = request.urdf_path
    robot_model = request.robot_model

    prim_path = world_path(name)

    status, import_config = commands.execute("URDFCreateImportConfig")
    import_config.set_merge_fixed_joints(False)
    import_config.set_convex_decomp(False)
    import_config.set_import_inertia_tensor(False)
    import_config.set_make_default_prim(False)
    import_config.set_distance_scale(1.0)
    import_config.set_fix_base(False)
    import_config.set_default_drive_type(2)
    import_config.set_self_collision(False)

    ensure_path(os.path.dirname(prim_path))
    status, usd_path = commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=import_config,
        dest_path='',
    )

    if usd_path is None:
        raise ValueError(f"Failed to import URDF from '{urdf_path}'")

    commands.execute(
        "MovePrim",
        path_from=usd_path,
        path_to=prim_path,
        keep_world_transform=True
    )

    # print(usd_path)
    if request.localization:
        if not odom.odom(
            os.path.join(prim_path, 'odom_publisher'),
            prim_path=os.path.join(prim_path, request.base_frame),
            base_frame_id=os.path.join(request.tf_prefix, request.base_frame),
            odom_frame_id=os.path.join(request.tf_prefix, request.odom_frame),
            odom_topic=request.odom_topic,
        ):
            carb.log_error("Failed to create odom graph")

    if not tf.tf(
        os.path.join(prim_path, 'tf_publisher'),
        prim_path=os.path.join(prim_path, request.base_frame),
        tf_prefix=request.tf_prefix,
    ):
        carb.log_error("Failed to create tf graph")

    if request.joint_states_topic:
        if not joint_states.joint_states(
            os.path.join(prim_path, 'joint_states_publisher'),
            prim_path=os.path.join(prim_path, request.base_frame),
            joint_states_topic=request.joint_states_topic,
        ):
            carb.log_error("Failed to create joint_states graph")

    if request.cmd_vel_topic:
        if not control.Control(
            prim_path=prim_path,
            cmd_vel_topic=request.cmd_vel_topic,
        ).parse(
            robot_model=robot_model,
        ):
            carb.log_error("Failed to create control graph")

    with open(request.urdf_path, 'r') as f:
        sensors.Sensors(
            prim_path=prim_path,
            base_frame=request.tf_prefix,
            base_topic=os.path.dirname(request.cmd_vel_topic)
        ).parse_gazebo(f.read())

    # Spawn robot at (1, 1, 0)
    geom.move(
        prim_path=prim_path,
        translation=geom.Translation(2.0, 2.0, 0.0),
        rotation=geom.Rotation.parse(request.pose.orientation),
    )

    DoorManager.instance().add_robot(prim_path, request.odom_topic)
    carb.log_error(f"Prim path of robot: {str(prim_path)}")
    carb.log_info(f"Added robot: {prim_path}")
    ElevatorManager.instance().add_robot(prim_path)
    carb.log_error(f"Check robot in ElevatorManager: {str(ElevatorManager.instance().get_robots())}")
    return prim_path


def spawn_urdf_callback(request, response):
    response.path = spawn_urdf(request)
    return response

# Urdf importer service callback.


spawn_urdf_service = Service(
    srv_type=SpawnUrdf,
    srv_name='isaac/SpawnUrdf',
    callback=spawn_urdf_callback
)

__all__ = ['spawn_urdf_service']
