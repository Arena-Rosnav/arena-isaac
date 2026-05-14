import os
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict

import carb
import isaac_utils.graphs.joint_states as joint_states
import isaac_utils.graphs.odom as odom
import isaac_utils.graphs.sensors.sensors as sensors
import omni.kit.commands as commands
import omni.usd
from isaac_utils.graphs import control
from isaac_utils.managers import entity_lifecycle
from isaac_utils.managers.door_manager import DoorManager
from isaac_utils.managers.elevator_manager import ElevatorManager
from isaac_utils.utils import geom
from isaac_utils.utils.material import Material, PhysicsParams
from isaac_utils.utils.path import world_path
from isaac_utils.utils.prim import ensure_path
from pxr import UsdPhysics

from isaacsim_msgs.srv import SpawnUrdf

from .utils import Service, on_exception

parent_dir = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(parent_dir))


def _resolve_articulation_prim(prim_path: str, base_frame: str) -> str:
    """Return the prim that joint_states / IsaacArticulationController target.

    These need any prim inside the articulation (the controller walks up to
    the root). Prefers `<prim_path>/<base_frame>`; falls back to the first
    descendant with ArticulationRootAPI; finally to prim_path itself.
    """
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return os.path.join(prim_path, base_frame)

    explicit = os.path.join(prim_path, base_frame)
    if stage.GetPrimAtPath(explicit).IsValid():
        return explicit

    root = stage.GetPrimAtPath(prim_path)
    if root.IsValid():
        for prim in root.GetAllChildren():
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                return str(prim.GetPath())
            for grandchild in prim.GetAllChildren():
                if grandchild.HasAPI(UsdPhysics.ArticulationRootAPI):
                    return str(grandchild.GetPath())

    return prim_path


def _resolve_body_prim(prim_path: str, base_frame: str) -> str:
    """Return the prim whose world transform tracks the robot body.

    odom.odom reads this prim's world transform to publish odom -> base.
    Prefers `<prim_path>/<base_frame>` (e.g. base_link); falls back to the
    first descendant with RigidBodyAPI (the first physics body, which is
    the chassis for our robots); finally to prim_path itself.
    """
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return os.path.join(prim_path, base_frame)

    explicit = os.path.join(prim_path, base_frame)
    if stage.GetPrimAtPath(explicit).IsValid():
        return explicit

    root = stage.GetPrimAtPath(prim_path)
    if root.IsValid():
        for prim in root.GetAllChildren():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                return str(prim.GetPath())
            for grandchild in prim.GetAllChildren():
                if grandchild.HasAPI(UsdPhysics.RigidBodyAPI):
                    return str(grandchild.GetPath())

    return prim_path


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

            if original_abs_path.startswith('file://'):
                original_abs_path = original_abs_path[len('file://'):]

            filename = os.path.basename(original_abs_path)

            if '-' in filename:
                sanitized_filename = filename.replace('-', '_')
                symlink_path = os.path.join(tmp_mesh_dir_path, sanitized_filename)

                if not os.path.lexists(symlink_path):
                    os.symlink(original_abs_path, symlink_path)

                tag.attrib['filename'] = symlink_path

    for link in root.iter('link'):
        link_name = link.attrib.get('name', '')
        inertial = link.find('inertial')
        if inertial is None:
            continue

        mass_el = inertial.find('mass')
        inertia_el = inertial.find('inertia')
        if mass_el is None or inertia_el is None:
            continue

        try:
            mass = float(mass_el.attrib.get('value', '0'))
        except ValueError:
            continue

        try:
            ixx = float(inertia_el.attrib.get('ixx', '0'))
            ixy = float(inertia_el.attrib.get('ixy', '0'))
            ixz = float(inertia_el.attrib.get('ixz', '0'))
            iyy = float(inertia_el.attrib.get('iyy', '0'))
            iyz = float(inertia_el.attrib.get('iyz', '0'))
            izz = float(inertia_el.attrib.get('izz', '0'))
        except ValueError:
            continue

        det = (
            ixx * (iyy * izz - iyz * iyz)
            - ixy * (ixy * izz - iyz * ixz)
            + ixz * (ixy * iyz - iyy * ixz)
        )

        degenerate = (
            mass < 1e-6
            or ixx < 1e-6
            or iyy < 1e-6
            or izz < 1e-6
            or det < 1e-12
        )

        if degenerate:
            link.remove(inertial)

    tmp_urdf = tempfile.NamedTemporaryFile(delete=False, suffix="_sanitized.urdf", mode='w')
    tree.write(tmp_urdf.name, encoding='unicode', xml_declaration=True)

    return tmp_urdf.name


def _extract_gazebo_physics(urdf_path: str) -> dict[str, PhysicsParams]:
    """Parse <gazebo reference="X"> mu1/mu2 blocks into PhysicsParams per link."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    mu1_per_link: dict[str, float] = {}
    mu2_per_link: dict[str, float] = {}

    for gazebo in root.iter('gazebo'):
        ref = gazebo.attrib.get('reference')
        if not ref:
            continue

        for child in gazebo:
            if child.tag not in ('mu1', 'mu2'):
                continue
            raw = child.attrib.get('value')
            if raw is None:
                raw = child.text
            if raw is None:
                continue
            raw = raw.strip()
            try:
                val = float(raw)
            except ValueError:
                continue
            if child.tag == 'mu1':
                mu1_per_link[ref] = val
            else:
                mu2_per_link[ref] = val

    all_links = set(mu1_per_link) | set(mu2_per_link)
    warned_asymmetric = False
    result: dict[str, PhysicsParams] = {}

    for link_name in all_links:
        mu1 = mu1_per_link.get(link_name)
        mu2 = mu2_per_link.get(link_name)

        if mu1 is None and mu2 is None:
            continue

        mu = ((mu1 or 0.0) + (mu2 or 0.0)) / (
            (1 if mu1 is not None else 0) + (1 if mu2 is not None else 0)
        )

        if not warned_asymmetric and mu1 is not None and mu2 is not None and mu1 != mu2:
            carb.log_warn(
                f'{urdf_path}: anisotropic friction (mu1 != mu2) is not supported '
                'in USD-PhysX, collapsing to the mean'
            )
            warned_asymmetric = True

        result[link_name] = PhysicsParams(
            static_friction=mu,
            dynamic_friction=mu,
            restitution=0.0,
            combine_mode=None,
        )

    return result


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
    import_config.set_import_inertia_tensor(True)
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

    stage = omni.usd.get_context().get_stage()

    friction_params = _extract_gazebo_physics(urdf_path)
    for link_name, params in friction_params.items():
        collider_root = f'/colliders/{link_name}'
        if not stage.GetPrimAtPath(collider_root).IsValid():
            continue
        key = f'wheel_{round(params.static_friction * 1000):d}_{round(params.dynamic_friction * 1000):d}_{round(params.restitution * 1000):d}_{params.combine_mode or "def"}'
        material = Material.physics(parent_prim_path=world_path(), key=key, params=params)
        if not material.bind_to(collider_root):
            carb.log_error(f'failed to bind physx material at {collider_root}')

    articulation_path = _resolve_articulation_prim(prim_path, request.base_frame)
    body_path = _resolve_body_prim(prim_path, request.base_frame)

    manifest = entity_lifecycle.register_robot(prim_path, articulation_path)

    if request.localization:
        odom_graph_path = os.path.join(prim_path, 'odom_publisher')
        if not odom.odom(
            odom_graph_path,
            prim_path=body_path,
            base_frame_id=f'{request.tf_prefix}{request.base_frame}',
            odom_frame_id=f'{request.tf_prefix}{request.odom_frame}',
            odom_topic=request.odom_topic,
        ):
            carb.log_error("Failed to create odom graph")
        else:
            manifest.graph_paths.append(odom_graph_path)

    # Joint TF (base_link -> wheel/sensor links) comes from robot_state_publisher
    # launched on the arena_runtime side. Isaac only owns world-pose TF (odom.odom).

    if request.joint_states_topic:
        joint_states_graph_path = os.path.join(prim_path, 'joint_states_publisher')
        if not joint_states.joint_states(
            joint_states_graph_path,
            prim_path=articulation_path,
            joint_states_topic=request.joint_states_topic,
        ):
            carb.log_error("Failed to create joint_states graph")
        else:
            manifest.graph_paths.append(joint_states_graph_path)

    if request.cmd_vel_topic:
        if not control.Control(
            prim_path=prim_path,
            target_prim_path=articulation_path,
            cmd_vel_topic=request.cmd_vel_topic,
            urdf_path=request.urdf_path,
        ).parse(
            robot_model=robot_model,
        ):
            carb.log_error("Failed to create control graph")
        else:
            manifest.graph_paths.append(os.path.join(prim_path, 'topic_bridge'))

    with open(request.urdf_path, 'r') as f:
        manifest.sensors.extend(
            sensors.Sensors(
                prim_path=prim_path,
                base_frame=request.tf_prefix,
                base_topic=os.path.dirname(request.cmd_vel_topic),
            ).parse_gazebo(f.read())
        )

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
