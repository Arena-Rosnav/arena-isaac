# arena_isaac

Isaac Sim backend behind `sim:=isaac`.

## Robot import and joint drives

`SpawnUrdf` sanitizes the URDF, imports it with `URDFImporterConfig(joint_target_type="velocity")` and builds the joint-state and command bridge from the `<ros2_control>` block of the URDF it was given. The importer writes every collider as an instanceable reference, and instanced mesh colliders on the robot links make no contacts, so `isaac_utils/utils/colliders.py` turns every instanceable prim holding a collider into a plain prim right after the import. The importer creates every drive without gains (its log says "Stiffness and damping not available ... actuator will be created without gain parameters"), so `isaac_utils/graphs/control` writes the gains afterwards, per command interface:

| Command interface | Drive |
| --- | --- |
| `velocity` | stiffness 0, damping 1e4 (physx) or 0.35 with a 16 force clamp (newton) |
| `position` | sized per joint by `isaac_utils/utils/drives.py` from the URDF, see below |

`position_drive` computes, in SI units and without Isaac, the drive of each joint that has a `position` command interface and is `revolute`, `continuous` or `prismatic`:

- `max_force` is the `<limit effort>` (25 when absent).
- Revolute and continuous joints: stiffness 100 N m/rad per N m of effort, damping stiffness/20.
- Prismatic joints: stiffness 8 N/m per N of effort, raised to carried mass * 9.81 / 2 mm so gravity sags the joint under 2 mm, damping `2 sqrt(stiffness * mass)` (critical) for the mass of the child link and everything below it.
- Damping never exceeds effort / `<limit velocity>`, so the damping force at the speed limit stays within the effort limit.

Revolute gains are converted to per degree before they reach USD, prismatic gains are written as is. Under newton, revolute joints keep stiffness 8 * effort with damping stiffness/20 and an armature, prismatic joints take the sized drive.

`<dynamics damping>` of position-commanded joints is removed from the URDF before import, so the joint's passive damping cannot resist the drive. Friction and the damping of velocity-commanded joints stay.

The bridge logs one `topic_bridge: position drive <joint> kp=... kd=... max_force=...` line per sized joint.
