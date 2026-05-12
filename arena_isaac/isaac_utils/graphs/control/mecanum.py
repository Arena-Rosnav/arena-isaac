import omni.graph.core as og
from isaacsim.core.utils import extensions

from isaac_utils.graphs import Graph

extensions.enable_extension("isaacsim.core.nodes")
extensions.enable_extension("isaacsim.ros2.bridge")


def mecanum(
    graph_path: str,
    prim_path: str,
    cmd_vel_topic: str,
    joint_names: list[str],
    wheel_radius: float,
    wheel_base_sum: float,
    max_linear_speed: float,
    max_lateral_speed: float,
    max_angular_speed: float,
) -> bool:
    """Action graph driving a 4-wheel mecanum base from a ROS2 Twist topic.

    joint_names ordering must be [front_left, back_left, back_right, front_right]
    so the wheel-velocity output array lines up with the articulation controller.
    wheel_base_sum is ros2_control's
    ``kinematics.sum_of_robot_center_projection_on_X_Y_axis`` (half-wheelbase + half-track).
    """
    if len(joint_names) != 4:
        return False

    del max_linear_speed, max_lateral_speed, max_angular_speed  # parity with differential, unused for now

    graph = Graph(graph_path)

    on_playback_tick = graph.node('on_playback_tick', 'omni.graph.action.OnPlaybackTick')
    ros2_subscribe_twist = graph.node('ros2_subscribe_twist', 'isaacsim.ros2.bridge.ROS2SubscribeTwist')
    scale_stage_units = graph.node('scale_stage_units', 'isaacsim.core.nodes.OgnIsaacScaleToFromStageUnit')
    break3vector_linear = graph.node('break3vector_linear', 'omni.graph.nodes.BreakVector3')
    break3vector_angular = graph.node('break3vector_angular', 'omni.graph.nodes.BreakVector3')
    mecanum_ik = graph.node('mecanum_ik', 'omni.graph.scriptnode.ScriptNode')
    make_array = graph.node('make_array', 'omni.graph.nodes.ConstructArray')
    get_target_prim = graph.node('get_target_prim', 'omni.replicator.core.OgnGetPrimAtPath')
    articulation_controller = graph.node('articulation_controller', 'isaacsim.core.nodes.IsaacArticulationController')

    ros2_subscribe_twist.attribute('topicName', cmd_vel_topic)
    make_array.attribute('arraySize', len(joint_names))
    get_target_prim.attribute('paths', [prim_path])

    mecanum_ik.create_attribute('inputs:vx', 'double')
    mecanum_ik.create_attribute('inputs:vy', 'double')
    mecanum_ik.create_attribute('inputs:wz', 'double')
    mecanum_ik.create_attribute('inputs:wheelRadius', 'double')
    mecanum_ik.create_attribute('inputs:wheelBaseSum', 'double')
    mecanum_ik.create_attribute('outputs:wheelVelocities', 'double[]')
    mecanum_ik.attribute('wheelRadius', wheel_radius)
    mecanum_ik.attribute('wheelBaseSum', wheel_base_sum)
    mecanum_ik.attribute('script', MECANUM_SCRIPT)

    on_playback_tick.connect('tick', ros2_subscribe_twist, 'execIn')
    on_playback_tick.connect('tick', articulation_controller, 'execIn')
    on_playback_tick.connect('tick', get_target_prim, 'execIn')

    ros2_subscribe_twist.connect('execOut', mecanum_ik, 'execIn')
    ros2_subscribe_twist.connect('linearVelocity', scale_stage_units, 'value')
    scale_stage_units.connect('result', break3vector_linear, 'tuple')
    ros2_subscribe_twist.connect('angularVelocity', break3vector_angular, 'tuple')

    break3vector_linear.connect('x', mecanum_ik, 'vx')
    break3vector_linear.connect('y', mecanum_ik, 'vy')
    break3vector_angular.connect('z', mecanum_ik, 'wz')

    mecanum_ik.connect('wheelVelocities', articulation_controller, 'velocityCommand')
    get_target_prim.connect('prims', articulation_controller, 'targetPrim')

    for i, joint_name in enumerate(joint_names):
        if i > 0:
            make_array.create_attribute(f'inputs:input{i}', 'token')
        token_node = graph.node(f'make_array_const_{i}', 'omni.graph.nodes.ConstantToken')
        token_node.attribute('value', joint_name)
        token_node.connect('value', make_array, f'input{i}', outputs_prefix='inputs:')

    make_array.connect('array', articulation_controller, 'jointNames')

    graph.load_extensions()
    return graph.execute(og.Controller())


MECANUM_SCRIPT = """
def compute(db: og.Database):
    vx = db.inputs.vx
    vy = db.inputs.vy
    wz = db.inputs.wz
    r = db.inputs.wheelRadius
    lw = db.inputs.wheelBaseSum
    if r == 0.0:
        return False
    db.outputs.wheelVelocities = [
        (vx - vy - lw * wz) / r,
        (vx + vy - lw * wz) / r,
        (vx + vy + lw * wz) / r,
        (vx - vy + lw * wz) / r,
    ]
    return True
"""
