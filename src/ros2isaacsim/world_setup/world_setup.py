import omni
import omni.replicator.isaac as dr
import omni.replicator.core as rep
import numpy as np

def generate_world(simulation_app,world,object_views,articulation_views):
    
    dr.physics_view.register_simulation_context(world)
    for object_view in object_views:
        dr.physics_view.register_rigid_prim_view(object_view)
    for articulation_view in articulation_views:
        dr.physics_view.register_articulation_view(articulation_view)
    
    with dr.gate.on_interval(interval=20):
        dr.physics_view.randomize_simulation_context(
            operation="scaling",
            gravity=rep.distribution.uniform((1, 1, 0.0), (1, 1, 2.0)),
        )
    with dr.gate.on_interval(interval=50):
        dr.physics_view.randomize_rigid_prim_view(
            view_name=object_view.name,
            operation="direct",
            force=rep.distribution.uniform((0, 0, 2.5), (0, 0, 5.0)),
        )
    with dr.gate.on_interval(interval=10):
        dr.physics_view.randomize_articulation_view(
            view_name=articulation_view.name,
            operation="direct",
            joint_velocities=rep.distribution.uniform(tuple([-2]*num_dof), tuple([2]*num_dof)),
        )
    with dr.gate.on_env_reset():
        dr.physics_view.randomize_rigid_prim_view(
            view_name=object_view.name,
            operation="additive",
            position=rep.distribution.normal((0.0, 0.0, 0.0), (0.2, 0.2, 0.0)),
            velocity=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        dr.physics_view.randomize_articulation_view(
            view_name=franka_view.name,
            operation="additive",
            joint_positions=rep.distribution.uniform(tuple([-0.5]*num_dof), tuple([0.5]*num_dof)),
            position=rep.distribution.normal((0.0, 0.0, 0.0), (0.2, 0.2, 0.0)),
        )
    rep.orchestrator.run()
    frame_idx = 0
    while simulation_app.is_running():
        if world.is_playing():
            reset_inds = list()
            if frame_idx % 200 == 0:
                # triggers reset every 200 steps
                reset_inds = np.arange(1)
            dr.physics_view.step_randomization(reset_inds)
            world.step(render=True)
            frame_idx += 1
