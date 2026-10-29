"""SpawnArticulated and DriveArticulated: articulated objects as fixed-base articulations whose joints damp, rub, spring back and switch their position drives."""

from __future__ import annotations

import math
import traceback
from dataclasses import dataclass, field

import carb
import numpy as np
import omni.usd
from geometry_msgs.msg import Pose
from isaacsim.core.experimental.prims import Articulation
from isaacsim_msgs.msg import ArticulatedLink, JointCommand, JointReading
from isaacsim_msgs.srv import DriveArticulated, SpawnArticulated
from pxr import Gf, PhysxSchema, Tf, Usd, UsdGeom, UsdPhysics, UsdShade

from isaac_utils.utils import light, prim
from isaac_utils.utils.path import world_path

from .utils import Service, on_exception

JOINTS = 'joints'
WORLD_JOINT = 'world'
GEOMETRY = 'geometry'
MATERIAL = 'physics_material'
FLOORS = 'Floors'


@dataclass
class _Joint:
    """One joint of an object: its DOF name, passive damping, friction and spring, and its drive's gains and effort."""

    dof: str
    damping: float
    friction: float
    armature: float
    effort: float
    stiffness: float
    drive_damping: float
    wraps: bool
    spring_stiffness: float = 0.0
    spring_rest: float = 0.0
    on: bool = False

    def passive(self, reference: float) -> tuple[float, float, float]:
        """Drive stiffness, damping and target while the drive is off: the spring toward its rest, else no pull."""
        return self.spring_stiffness, self.damping, self.spring_rest if self.spring_stiffness > 0.0 else reference


@dataclass
class _Object:
    path: str
    initial: dict[str, float]
    joints: dict[str, _Joint] = field(default_factory=dict)
    view: Articulation | None = None
    dofs: dict[str, int] = field(default_factory=dict)


_objects: dict[str, _Object] = {}


def _row(values: list[float]) -> np.ndarray:
    return np.array([values], dtype=np.float32)


def _rotation(axis: Gf.Vec3d) -> Gf.Quatf:
    """Rotation taking +X onto the axis, the frame of a joint whose axis is X."""
    quat = Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), axis).GetQuat()
    return Gf.Quatf(quat.GetReal(), Gf.Vec3f(quat.GetImaginary()))


def _floor_colliders(stage: Usd.Stage, name: str) -> list[str]:
    root = stage.GetPrimAtPath(world_path(name.split('/')[0], FLOORS))
    if not root.IsValid():
        return []
    return [str(item.GetPath()) for item in Usd.PrimRange(root) if item.HasAPI(UsdPhysics.CollisionAPI)]


def _decompose(root: Usd.Prim) -> None:
    """Collide every mesh under root through its convex decomposition."""
    for item in Usd.PrimRange(root):
        if item.IsA(UsdGeom.Mesh) and item.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.MeshCollisionAPI.Apply(item).CreateApproximationAttr(UsdPhysics.Tokens.convexDecomposition)


def _pin(stage: Usd.Stage, path: str, pose: Pose) -> None:
    """Hold the root link at pose through the joint fixing it to the world."""
    joint = UsdPhysics.FixedJoint(stage.GetPrimAtPath(f'{path}/{JOINTS}/{WORLD_JOINT}'))
    orientation = pose.orientation
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(pose.position.x, pose.position.y, pose.position.z))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(orientation.w, orientation.x, orientation.y, orientation.z))


def _add_link(stage: Usd.Stage, path: str, link: ArticulatedLink, scale: Gf.Vec3f, floors: list[str]) -> str:
    body = UsdGeom.Xform.Define(stage, f'{path}/{Tf.MakeValidIdentifier(link.name)}').GetPrim()
    if link.usd_path and (geometry := prim.create_prim_safe(prim_path=str(body.GetPath().AppendChild(GEOMETRY)), usd_path=link.usd_path, scale=np.array(scale))) is not None:
        prim.sanitize_asset(geometry)
        _decompose(geometry)
        light.demote_emission(geometry)
    UsdPhysics.RigidBodyAPI.Apply(body)
    mass = UsdPhysics.MassAPI.Apply(body)
    mass.CreateMassAttr(link.mass)
    mass.CreateCenterOfMassAttr(Gf.Vec3f(link.center.x, link.center.y, link.center.z))
    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(link.moments.x, link.moments.y, link.moments.z))
    mass.CreatePrincipalAxesAttr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    if floors:
        UsdPhysics.FilteredPairsAPI.Apply(body).CreateFilteredPairsRel().SetTargets(floors)
    if link.has_material:
        material = UsdShade.Material.Define(stage, body.GetPath().AppendChild(MATERIAL))
        surface = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        surface.CreateStaticFrictionAttr(link.static_friction)
        surface.CreateDynamicFrictionAttr(link.dynamic_friction)
        surface.CreateRestitutionAttr(link.restitution)
        UsdShade.MaterialBindingAPI.Apply(body).Bind(material, UsdShade.Tokens.strongerThanDescendants, 'physics')
    return str(body.GetPath())


def _add_joint(stage: Usd.Stage, scope: str, link: ArticulatedLink, bodies: dict[str, str]) -> _Joint | None:
    name = Tf.MakeValidIdentifier(link.joint)
    joint_path = f'{scope}/{name}'
    angular = link.type in (ArticulatedLink.REVOLUTE, ArticulatedLink.CONTINUOUS)
    if angular:
        joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path)
        joint.CreateAxisAttr(UsdPhysics.Tokens.x)
        if link.type == ArticulatedLink.REVOLUTE:
            joint.CreateLowerLimitAttr(math.degrees(link.lower))
            joint.CreateUpperLimitAttr(math.degrees(link.upper))
    elif link.type == ArticulatedLink.PRISMATIC:
        joint = UsdPhysics.PrismaticJoint.Define(stage, joint_path)
        joint.CreateAxisAttr(UsdPhysics.Tokens.x)
        joint.CreateLowerLimitAttr(link.lower)
        joint.CreateUpperLimitAttr(link.upper)
    else:
        joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
    joint.CreateBody0Rel().SetTargets([bodies[link.parent]])
    joint.CreateBody1Rel().SetTargets([bodies[link.name]])
    origin = Gf.Vec3f(link.origin.x, link.origin.y, link.origin.z)
    rotation = _rotation(Gf.Vec3d(link.axis.x, link.axis.y, link.axis.z))
    joint.CreateLocalPos0Attr().Set(origin)
    joint.CreateLocalPos1Attr().Set(origin)
    joint.CreateLocalRot0Attr().Set(rotation)
    joint.CreateLocalRot1Attr().Set(rotation)
    if link.type == ArticulatedLink.FIXED:
        return None
    dynamics = link.dynamics
    unit = math.pi / 180.0 if angular else 1.0
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), 'angular' if angular else 'linear')
    drive.CreateTypeAttr('force')
    drive.CreateStiffnessAttr(dynamics.spring_stiffness * unit)
    drive.CreateDampingAttr(dynamics.damping * unit)
    drive.CreateMaxForceAttr(dynamics.effort)
    drive.CreateTargetPositionAttr(math.degrees(dynamics.spring_rest) if angular else dynamics.spring_rest)
    return _Joint(
        dof=name,
        damping=dynamics.damping,
        friction=dynamics.friction,
        armature=dynamics.armature,
        effort=dynamics.effort,
        stiffness=dynamics.stiffness,
        drive_damping=dynamics.drive_damping,
        wraps=link.type == ArticulatedLink.CONTINUOUS,
        spring_stiffness=dynamics.spring_stiffness,
        spring_rest=dynamics.spring_rest,
    )


def _author(stage: Usd.Stage, path: str, request: SpawnArticulated.Request) -> _Object:
    root = UsdGeom.Xform.Define(stage, path)
    position, orientation = request.pose.position, request.pose.orientation
    root.AddTranslateOp().Set(Gf.Vec3d(position.x, position.y, position.z))
    root.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(orientation.w, orientation.x, orientation.y, orientation.z))
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    PhysxSchema.PhysxArticulationAPI.Apply(root.GetPrim()).CreateEnabledSelfCollisionsAttr(False)
    scale = Gf.Vec3f(request.scale.x, request.scale.y, request.scale.z)
    floors = _floor_colliders(stage, request.name)
    bodies = {link.name: _add_link(stage, path, link, scale, floors) for link in request.links}
    scope = str(UsdGeom.Scope.Define(stage, f'{path}/{JOINTS}').GetPath())
    UsdPhysics.FixedJoint.Define(stage, f'{scope}/{WORLD_JOINT}').CreateBody1Rel().SetTargets([bodies[list(request.links)[0].name]])
    _pin(stage, path, request.pose)
    obj = _Object(path=path, initial={link.joint: link.value for link in request.links if link.parent and link.type != ArticulatedLink.FIXED})
    for link in list(request.links)[1:]:
        if (joint := _add_joint(stage, scope, link, bodies)) is not None:
            obj.joints[link.joint] = joint
    light.bind_glow()
    return obj


def spawn_articulated(request: SpawnArticulated.Request) -> bool:
    """Author the object under its world path, a failed one taken back out whole."""
    stage = omni.usd.get_context().get_stage()
    path = world_path(request.name)
    if stage.GetPrimAtPath(path).IsValid() or not request.links:
        return False
    try:
        _objects[request.name] = _author(stage, path, request)
    except Exception as error:
        stage.RemovePrim(path)
        carb.log_error(f"articulated object {request.name!r} not spawned: {error}\n{traceback.format_exc()}")
        return False
    return True


def follow_root(name: str, pose: Pose) -> None:
    """Move the world pin of a moved object with it."""
    obj = _objects.get(name)
    if obj is not None:
        _pin(omni.usd.get_context().get_stage(), obj.path, pose)


def _view(obj: _Object) -> Articulation | None:
    """The object's articulation view once physics has parsed it, its DOFs configured on first sight, None before."""
    if obj.view is not None and obj.view.is_physics_tensor_entity_valid():
        return obj.view
    view = Articulation(obj.path)
    if not view.is_physics_tensor_entity_valid():
        return None
    obj.dofs = {joint: view.dof_names.index(entry.dof) for joint, entry in obj.joints.items() if entry.dof in view.dof_names}
    joints = list(obj.dofs)
    indices = [obj.dofs[joint] for joint in joints]
    if indices:
        entries = [obj.joints[joint] for joint in joints]
        frictions = _row([entry.friction for entry in entries])
        view.set_dof_friction_properties(static_frictions=frictions, dynamic_frictions=frictions, dof_indices=indices)
        view.set_dof_armatures(_row([entry.armature for entry in entries]), dof_indices=indices)
        view.set_dof_max_efforts(_row([entry.effort for entry in entries]), dof_indices=indices)
        starts = [obj.initial.get(joint, 0.0) for joint in joints]
        passive = [entry.passive(start) for entry, start in zip(entries, starts, strict=True)]
        view.set_dof_gains(_row([gains[0] for gains in passive]), _row([gains[1] for gains in passive]), dof_indices=indices)
        view.set_dof_position_targets(_row([gains[2] for gains in passive]), dof_indices=indices)
        view.set_dof_positions(_row(starts), dof_indices=indices)
        view.set_dof_velocities(_row([0.0] * len(entries)), dof_indices=indices)
        for entry in entries:
            entry.on = False
    obj.view = view
    return view


def _drive(command: JointCommand) -> JointReading:
    reading = JointReading(name=command.name, found=False)
    obj = _objects.get(command.name)
    stage = omni.usd.get_context().get_stage()
    if obj is None:
        return reading
    if not stage.GetPrimAtPath(obj.path).IsValid():
        del _objects[command.name]
        return reading
    view = _view(obj)
    if view is None:
        return reading
    current = view.get_dof_positions().numpy()[0]
    placed: list[tuple[int, float]] = []
    targets: list[tuple[int, float]] = []
    for name, reference, drive, place in zip(command.joints, command.reference, command.drive, command.place, strict=True):
        joint, index = obj.joints.get(name), obj.dofs.get(name)
        if joint is None or index is None:
            continue
        if place:
            placed.append((index, reference))
        stiffness, damping, rest = (joint.stiffness, joint.drive_damping + joint.damping, reference) if drive else joint.passive(reference)
        if drive != joint.on:
            view.set_dof_gains(_row([stiffness]), _row([damping]), dof_indices=[index])
            joint.on = drive
        value = float(current[index])
        targets.append((index, value + math.remainder(rest - value, 2 * math.pi) if joint.wraps and not place else rest))
    if placed:
        indices = [index for index, _ in placed]
        view.set_dof_positions(_row([value for _, value in placed]), dof_indices=indices)
        view.set_dof_velocities(np.zeros((1, len(placed)), dtype=np.float32), dof_indices=indices)
    if targets:
        view.set_dof_position_targets(_row([value for _, value in targets]), dof_indices=[index for index, _ in targets])
    positions = view.get_dof_positions().numpy()[0] if placed else current
    reading.found = True
    reading.joints = list(obj.dofs)
    reading.position = [float(positions[obj.dofs[joint]]) for joint in reading.joints]
    return reading


def spawn_articulated_callback(request: SpawnArticulated.Request, response: SpawnArticulated.Response) -> SpawnArticulated.Response:
    response.ret = spawn_articulated(request)
    return response


def drive_articulated_callback(request: DriveArticulated.Request, response: DriveArticulated.Response) -> DriveArticulated.Response:
    response.readings = [on_exception(JointReading(name=command.name, found=False))(_drive)(command) for command in request.commands]
    return response


spawn_articulated_service = Service(srv_type=SpawnArticulated, srv_name='isaac/SpawnArticulated', callback=spawn_articulated_callback)
drive_articulated_service = Service(srv_type=DriveArticulated, srv_name='isaac/DriveArticulated', callback=drive_articulated_callback)

__all__ = ['drive_articulated_service', 'follow_root', 'spawn_articulated_service']
