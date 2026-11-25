# arena_isaac

Isaac Sim backend behind `sim:=isaac`.

## Robot import and joint drives

`SpawnUrdf` sanitizes the URDF, imports it with `URDFImporterConfig(joint_target_type="velocity")` and builds the joint-state and command bridge from the `<ros2_control>` block of the URDF it was given. The importer writes every collider as an instanceable reference, and instanced mesh colliders on the robot links make no contacts, so `isaac_utils/utils/colliders.py` turns every instanceable prim holding a collider into a plain prim right after the import. The importer creates every drive without gains (its log says "Stiffness and damping not available ... actuator will be created without gain parameters"), so `isaac_utils/graphs/control` writes the gains afterwards, per command interface:

| Command interface | Drive |
| --- | --- |
| `velocity` | stiffness 0, damping 1e4 (physx) or 0.35 with a 16 force clamp (newton) |
| `position` | gains and effort limit from the joint's `kp`, `kd` and `effort` params, see below |
| `effort` | the `position` drive holds the spawn pose until the first finite effort command (`graphs/control/release.py`), then stiffness 0, damping 0, max force from the `effort` param, the commanded effort is applied raw |

`isaac_utils/utils/drives.py` reads, without Isaac, the `kp`, `kd` and `effort` params of each `revolute`, `continuous` or `prismatic` joint that has a `position` or `effort` command interface in the served URDF's `<ros2_control>` block. arena_robots derives them (SI units: N m/rad and N m s/rad, N/m and N s/m for prismatic, effort in N m or N) and an overridable rule sizes the defaults. The drive force is clamped to `effort`.

Angular gains are converted to per degree before they reach USD, prismatic gains are written as is. Under newton, angular joints take 0.08 times kp and kd with an armature, prismatic joints take the params as is. A position joint without params gets stiffness 4e5 and damping 4e3.

Effort-commanded joints read `<ns>/isaac/joint_commands_effort` next to `joint_commands_position` and `joint_commands_velocity`.

`<dynamics damping>` of position-commanded joints is removed from the URDF before import, so the joint's passive damping cannot resist the drive. Friction and the damping of velocity-commanded joints stay.

The bridge logs one `topic_bridge: position drive <joint> kp=... kd=... max_force=...` line per joint with params.

## Removing robots

Deleting a robot's prims invalidates the physics tensor simulation view that every articulation shares, and the first step on that view crashes PhysX. When the sim resumes after a stage edit, the main loop drops the invalid views (`renew_physics_views` in `isaac_utils/graphs`), so the play that follows recreates them, and then rebuilds the bridge graphs that cached them.
