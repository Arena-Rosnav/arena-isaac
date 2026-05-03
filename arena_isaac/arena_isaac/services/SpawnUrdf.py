import xml.etree.ElementTree as ET
import tempfile
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
from typing import Dict

parent_dir = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(parent_dir))


def sanitize_urdf_for_isaac(urdf_path: str) -> str:
    # usd hates dashes in names, so i hate usd
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    link_name_map: Dict[str, str] = {}
    joint_name_map: Dict[str, str] = {}

    for tag in root.iter():
        if tag.tag == 'link':
            name = tag.attrib.get('name')
            if name and '-' in name:
                link_name_map[name] = name.replace('-', '_')
        elif tag.tag == 'joint':
            name = tag.attrib.get('name')
            if name and '-' in name:
                joint_name_map[name] = name.replace('-', '_')

    tmp_mesh_dir_path = tempfile.mkdtemp(prefix="isaac_urdf_")

    for tag in root.iter():
        if tag.tag in ['robot', 'link', 'joint']:
            name = tag.attrib.get('name')
            if name and '-' in name:
                tag.attrib['name'] = name.replace('-', '_')
        elif tag.tag in ['parent', 'child']:
            link = tag.attrib.get('link')
            if link in link_name_map:
                tag.attrib['link'] = link_name_map[link]
        elif tag.tag in ['mimic', 'actuator']:
            joint = tag.attrib.get('joint')
            if joint in joint_name_map:
                tag.attrib['joint'] = joint_name_map[joint]
        elif tag.tag == 'gazebo':
            reference = tag.attrib.get('reference')
            if reference in link_name_map:
                tag.attrib['reference'] = link_name_map[reference]

        elif tag.tag == 'mesh':
            original_abs_path = tag.attrib.get('filename')
            if not original_abs_path:
                continue

            filename = os.path.basename(original_abs_path)

            if '-' in filename:
                sanitized_filename = filename.replace('-', '_')
                symlink_path = os.path.join(tmp_mesh_dir_path, sanitized_filename)

                if not os.path.exists(symlink_path):
                    os.symlink(original_abs_path, symlink_path)

                tag.attrib['filename'] = symlink_path

    tmp_urdf = tempfile.NamedTemporaryFile(delete=False, suffix="_sanitized.urdf", mode='w')
    tree.write(tmp_urdf.name, encoding='unicode', xml_declaration=True)

    return tmp_urdf.name


@on_exception('')
def spawn_urdf(request: SpawnUrdf.Request) -> str:
    name = request.name
    urdf_path = request.urdf_path
    robot_model = request.robot_model

    prim_path = world_path(name)

    urdf_path = sanitize_urdf_for_isaac(urdf_path)

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
    )

    if usd_path is None:
        raise ValueError(f"Failed to import URDF from '{urdf_path}'. Status {status}")

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
            target_prim_path=os.path.join(prim_path, request.base_frame),
            cmd_vel_topic=request.cmd_vel_topic,
        ).parse(
            robot_model=robot_model,
        ):
            carb.log_error("Failed to create control graph")

    with open(request.urdf_path, 'r') as f:
        sensors.Sensors(
            prim_path=prim_path,
            base_frame=request.tf_prefix,
            base_topic=os.path.dirname(request.cmd_vel_topic),
        ).parse_gazebo(f.read())

    articulation_path = os.path.join(prim_path, request.base_frame)
    geom.register_robot(
        robot_prim_path=prim_path,
        articulation_prim_path=articulation_path,
    )

    geom.move(
        prim_path=prim_path,
        translation=geom.Translation.parse(request.pose.position),
        rotation=geom.Rotation.parse(request.pose.orientation),
    )

    DoorManager.instance().add_robot(prim_path, request.odom_topic)
    carb.log_info(f"Added robot: {prim_path}")
    ElevatorManager.instance().add_robot(prim_path)
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
