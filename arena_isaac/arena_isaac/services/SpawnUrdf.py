import os
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import carb
import isaacsim.core.utils.prims as prim_utils
import omni.usd
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
from isaacsim_msgs.srv import SpawnUrdf
from pxr import Usd, UsdGeom, UsdPhysics

import isaac_utils.graphs.joint_states as joint_states
import isaac_utils.graphs.odom as odom
import isaac_utils.graphs.sensors.sensors as sensors
from isaac_utils.graphs import control
from isaac_utils.managers import entity_lifecycle
from isaac_utils.utils import geom
from isaac_utils.utils.material import Material, PhysicsParams
from isaac_utils.utils.path import world_path
from isaac_utils.utils.prim import ensure_path

from .utils import Service, on_exception

parent_dir = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(parent_dir))


def _resolve_articulation_prim(prim_path: str, base_frame: str) -> str:
    """Return the prim carrying ArticulationRootAPI.

    The URDF importer relocates ArticulationRootAPI onto the parent of the
    root rigid body, nested under `<prim_path>/Geometry/...`, so we walk the
    whole subtree rather than assuming a fixed depth. physx matches the
    articulation at this exact prim path, so the joint-state / controller
    graphs must target it directly. Falls back to a prim named `base_frame`,
    then prim_path itself.
    """
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return prim_path

    root = stage.GetPrimAtPath(prim_path)
    if root.IsValid():
        for prim in Usd.PrimRange(root):
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                return str(prim.GetPath())
        for prim in Usd.PrimRange(root):
            if prim.GetName() == base_frame:
                return str(prim.GetPath())

    return prim_path


def _resolve_body_prim(robot_prim: str, articulation_prim: str) -> str:
    """A physics-moved rigid body rigidly fixed to the base link, for odom to track.

    Drops bodies behind an articulated joint (wheels spin) and the static holder,
    then takes the one closest to the holder. odom composes its pose to the base.
    """
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return articulation_prim

    holder = stage.GetPrimAtPath(articulation_prim)
    robot = stage.GetPrimAtPath(robot_prim)
    if not holder.IsValid() or not robot.IsValid():
        return articulation_prim

    bodies = [prim for prim in Usd.PrimRange(holder) if prim.GetPath() != holder.GetPath() and prim.HasAPI(UsdPhysics.RigidBodyAPI)]
    if not bodies:
        return articulation_prim

    articulated: set[str] = set()
    for prim in Usd.PrimRange(robot):
        for joint_type in (UsdPhysics.RevoluteJoint, UsdPhysics.PrismaticJoint, UsdPhysics.SphericalJoint):
            if prim.IsA(joint_type):
                articulated.update(str(t) for t in joint_type(prim).GetBody1Rel().GetTargets())

    candidates = [body for body in bodies if str(body.GetPath()) not in articulated] or bodies

    cache = UsdGeom.XformCache()
    hx, hy, _ = cache.GetLocalToWorldTransform(holder).ExtractTranslation()

    def _offset(prim: Usd.Prim) -> float:
        tx, ty, _ = cache.GetLocalToWorldTransform(prim).ExtractTranslation()
        return (tx - hx) ** 2 + (ty - hy) ** 2

    return str(min(candidates, key=_offset).GetPath())


def sanitize_urdf_for_isaac(urdf_path: str) -> str:
    # usd hates dashes in names, so i hate usd
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    link_name_map: dict[str, str] = {}
    joint_name_map: dict[str, str] = {}

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
        # usd cannot name xml-namespaced attributes (gz:expressed_in), strip them
        for key in [k for k in tag.attrib if k.startswith('{')]:
            del tag.attrib[key]

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
                original_abs_path = original_abs_path[len('file://') :]

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

        det = ixx * (iyy * izz - iyz * iyz) - ixy * (ixy * izz - iyz * ixz) + ixz * (ixy * iyz - iyy * ixz)

        degenerate = mass < 1e-6 or ixx < 1e-6 or iyy < 1e-6 or izz < 1e-6 or det < 1e-12

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
    directional: set[str] = set()

    for gazebo in root.iter('gazebo'):
        ref = gazebo.attrib.get('reference')
        if not ref:
            continue

        for child in gazebo:
            if child.tag == 'fdir1':
                raw = child.attrib.get('value') or child.text or ''
                try:
                    if any(float(c) != 0.0 for c in raw.split()):
                        directional.add(ref)
                except ValueError:
                    pass
                continue
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
    for ref in sorted(all_links & directional):
        carb.log_warn(f'{urdf_path}: link {ref!r} declares directional friction (fdir1), a gz tuning an isotropic material cannot honor, keeping the solver default')
    all_links -= directional
    warned_asymmetric = False
    result: dict[str, PhysicsParams] = {}

    for link_name in all_links:
        mu1 = mu1_per_link.get(link_name)
        mu2 = mu2_per_link.get(link_name)

        if mu1 is None and mu2 is None:
            continue

        mu = ((mu1 or 0.0) + (mu2 or 0.0)) / ((1 if mu1 is not None else 0) + (1 if mu2 is not None else 0))

        if not warned_asymmetric and mu1 is not None and mu2 is not None and mu1 != mu2:
            carb.log_warn(f'{urdf_path}: anisotropic friction (mu1 != mu2) is not supported in USD-PhysX, collapsing to the mean')
            warned_asymmetric = True

        result[link_name] = PhysicsParams(
            static_friction=mu,
            dynamic_friction=mu,
            restitution=0.0,
            combine_mode=None,
        )

    return result


def _link_collider_prims(stage: Usd.Stage, robot_prim: str, link_name: str) -> list[str]:
    """Collider prims of one imported link. The importer applies CollisionAPI to
    collision children inside the nested Geometry link tree (there is no flat
    /colliders scope), and child links nest under their parent link, so the walk
    prunes at descendant rigid bodies to not capture their colliders."""
    root = stage.GetPrimAtPath(robot_prim)
    if not root.IsValid():
        return []
    link = next(
        (p for p in Usd.PrimRange(root) if p.GetName() == link_name and p.HasAPI(UsdPhysics.RigidBodyAPI)),
        None,
    )
    if link is None:
        return []
    out: list[str] = []
    it = iter(Usd.PrimRange(link))
    for prim in it:
        if prim.GetPath() != link.GetPath() and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            it.PruneChildren()
            continue
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            out.append(str(prim.GetPath()))
    return out


@on_exception('')
def spawn_urdf(request: SpawnUrdf.Request) -> str:
    name = request.name
    urdf_path = request.urdf_path
    robot_model = request.robot_model

    prim_path = world_path(name)

    urdf_path = sanitize_urdf_for_isaac(urdf_path)

    import_config = URDFImporterConfig(
        urdf_path=urdf_path,
        merge_fixed_joints=False,
        allow_self_collision=False,
        fix_base=False,
        joint_target_type="velocity",
    )
    usd_path = URDFImporter(import_config).import_urdf()

    if not usd_path:
        raise ValueError(f"Failed to import URDF from '{urdf_path}'.")

    ensure_path(os.path.dirname(prim_path))
    prim_utils.create_prim(prim_path, "Xform", usd_path=usd_path)

    stage = omni.usd.get_context().get_stage()

    friction_params = _extract_gazebo_physics(urdf_path)
    for link_name, params in friction_params.items():
        colliders = _link_collider_prims(stage, prim_path, link_name)
        if not colliders:
            carb.log_warn(f'{name}: no collider prims for link {link_name!r}, friction mu={params.static_friction} not applied')
            continue
        key = f'friction_{round(params.static_friction * 1000):d}_{round(params.dynamic_friction * 1000):d}_{round(params.restitution * 1000):d}_{params.combine_mode or "def"}'
        material = Material.physics(parent_prim_path=world_path(), key=key, params=params)
        for collider in colliders:
            if not material.bind_to(collider):
                carb.log_error(f'failed to bind physx material at {collider}')

    articulation_path = _resolve_articulation_prim(prim_path, request.base_frame)
    body_path = _resolve_body_prim(prim_path, articulation_path)

    manifest = entity_lifecycle.register_robot(prim_path, articulation_path)

    if request.localization:
        odom_graph_path = os.path.join(prim_path, 'odom_publisher')
        if not odom.odom(
            odom_graph_path,
            prim_path=body_path,
            base_prim=articulation_path,
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

    with open(request.urdf_path) as f:
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

    carb.log_info(f"Added robot: {prim_path}")
    return prim_path


def spawn_urdf_callback(request: SpawnUrdf.Request, response: SpawnUrdf.Response) -> SpawnUrdf.Response:
    response.path = spawn_urdf(request)
    return response


# Urdf importer service callback.


spawn_urdf_service = Service(srv_type=SpawnUrdf, srv_name='isaac/SpawnUrdf', callback=spawn_urdf_callback)

__all__ = ['spawn_urdf_service']
