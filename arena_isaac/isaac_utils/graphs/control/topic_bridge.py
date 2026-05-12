import omni.graph.core as og
from isaacsim.core.utils import extensions

from isaac_utils.graphs import Graph

extensions.enable_extension("isaacsim.core.nodes")
extensions.enable_extension("isaacsim.ros2.bridge")


def topic_bridge(
    graph_path: str,
    prim_path: str,
    joints_velocity: list[str],
    joints_position: list[str],
    commands_topic: str,
    states_topic: str,
) -> bool:
    """Action graph bridging an external ros2_control controller_manager to Isaac.

    Subscribes to sensor_msgs/JointState on commands_topic, routes velocity
    commands to a velocity articulation controller and position commands to a
    position articulation controller. Publishes joint state feedback on
    states_topic for the union of all bridged joints.

    If either joints_velocity or joints_position is empty, that command path
    is skipped entirely; state publishing always runs.
    """
    all_joints = joints_velocity + joints_position
    if not all_joints:
        return False

    graph = Graph(graph_path)

    on_playback_tick = graph.node('on_playback_tick', 'omni.graph.action.OnPlaybackTick')
    get_target_prim = graph.node('get_target_prim', 'omni.replicator.core.OgnGetPrimAtPath')
    ros2_subscribe_joint_state = graph.node(
        'ros2_subscribe_joint_state', 'isaacsim.ros2.bridge.ROS2SubscribeJointState'
    )
    isaac_read_simulation_time = graph.node(
        'isaac_read_simulation_time', 'isaacsim.core.nodes.IsaacReadSimulationTime'
    )
    ros2_publish_joint_state = graph.node(
        'ros2_publish_joint_state', 'isaacsim.ros2.bridge.ROS2PublishJointState'
    )

    get_target_prim.attribute('paths', [prim_path])
    ros2_subscribe_joint_state.attribute('topicName', commands_topic)
    ros2_publish_joint_state.attribute('topicName', states_topic)

    on_playback_tick.connect('tick', get_target_prim, 'execIn')
    on_playback_tick.connect('tick', ros2_subscribe_joint_state, 'execIn')
    on_playback_tick.connect('tick', ros2_publish_joint_state, 'execIn')

    isaac_read_simulation_time.connect('simulationTime', ros2_publish_joint_state, 'timeStamp')
    get_target_prim.connect('prims', ros2_publish_joint_state, 'targetPrim')
    # ROS2PublishJointState auto-discovers the joint set from targetPrim;
    # there is no jointNames input filter in Isaac SDK 5.x.

    # ScriptNode: given inbound name+position+velocity arrays, compute per-kind
    # index/value arrays for the articulation controllers.
    if joints_velocity or joints_position:
        bridge_script_node = graph.node('bridge_script', 'omni.graph.scriptnode.ScriptNode')
        bridge_script_node.create_attribute('inputs:inNames', 'token[]')
        bridge_script_node.create_attribute('inputs:inPositions', 'double[]')
        bridge_script_node.create_attribute('inputs:inVelocities', 'double[]')
        bridge_script_node.create_attribute('inputs:velJoints', 'token[]')
        bridge_script_node.create_attribute('inputs:posJoints', 'token[]')
        bridge_script_node.create_attribute('outputs:velIndices', 'int[]')
        bridge_script_node.create_attribute('outputs:velValues', 'double[]')
        bridge_script_node.create_attribute('outputs:posIndices', 'int[]')
        bridge_script_node.create_attribute('outputs:posValues', 'double[]')

        bridge_script_node.attribute('velJoints', joints_velocity)
        bridge_script_node.attribute('posJoints', joints_position)
        bridge_script_node.attribute('script', _BRIDGE_SCRIPT)

        ros2_subscribe_joint_state.connect('execOut', bridge_script_node, 'execIn')
        ros2_subscribe_joint_state.connect('jointNames', bridge_script_node, 'inNames')
        ros2_subscribe_joint_state.connect('positionCommand', bridge_script_node, 'inPositions')
        ros2_subscribe_joint_state.connect('velocityCommand', bridge_script_node, 'inVelocities')

        on_playback_tick.connect('tick', bridge_script_node, 'execIn')

    if joints_velocity:
        vel_controller = graph.node(
            'vel_articulation_controller', 'isaacsim.core.nodes.IsaacArticulationController'
        )
        on_playback_tick.connect('tick', vel_controller, 'execIn')
        get_target_prim.connect('prims', vel_controller, 'targetPrim')

        # joint name array for velocity controller
        vel_names_array = graph.node('vel_names_array', 'omni.graph.nodes.ConstructArray')
        vel_names_array.attribute('arraySize', len(joints_velocity))
        for i, joint_name in enumerate(joints_velocity):
            if i > 0:
                vel_names_array.create_attribute(f'inputs:input{i}', 'token')
            token_node = graph.node(f'vel_name_const_{i}', 'omni.graph.nodes.ConstantToken')
            token_node.attribute('value', joint_name)
            token_node.connect('value', vel_names_array, f'input{i}', outputs_prefix='inputs:')
        vel_names_array.connect('array', vel_controller, 'jointNames')

        bridge_script_node.connect('velValues', vel_controller, 'velocityCommand')

    if joints_position:
        pos_controller = graph.node(
            'pos_articulation_controller', 'isaacsim.core.nodes.IsaacArticulationController'
        )
        on_playback_tick.connect('tick', pos_controller, 'execIn')
        get_target_prim.connect('prims', pos_controller, 'targetPrim')

        # joint name array for position controller
        pos_names_array = graph.node('pos_names_array', 'omni.graph.nodes.ConstructArray')
        pos_names_array.attribute('arraySize', len(joints_position))
        for i, joint_name in enumerate(joints_position):
            if i > 0:
                pos_names_array.create_attribute(f'inputs:input{i}', 'token')
            token_node = graph.node(f'pos_name_const_{i}', 'omni.graph.nodes.ConstantToken')
            token_node.attribute('value', joint_name)
            token_node.connect('value', pos_names_array, f'input{i}', outputs_prefix='inputs:')
        pos_names_array.connect('array', pos_controller, 'jointNames')

        bridge_script_node.connect('posValues', pos_controller, 'positionCommand')

    graph.load_extensions()
    return graph.execute(og.Controller())


_BRIDGE_SCRIPT = """
def compute(db: og.Database):
    name_to_idx = {name: i for i, name in enumerate(db.inputs.inNames)}
    positions = db.inputs.inPositions
    velocities = db.inputs.inVelocities

    vel_indices = []
    vel_values = []
    for joint in db.inputs.velJoints:
        idx = name_to_idx.get(joint)
        if idx is not None:
            vel_indices.append(idx)
            vel_values.append(float(velocities[idx]) if idx < len(velocities) else 0.0)

    pos_indices = []
    pos_values = []
    for joint in db.inputs.posJoints:
        idx = name_to_idx.get(joint)
        if idx is not None:
            pos_indices.append(idx)
            pos_values.append(float(positions[idx]) if idx < len(positions) else 0.0)

    db.outputs.velIndices = vel_indices
    db.outputs.velValues = vel_values
    db.outputs.posIndices = pos_indices
    db.outputs.posValues = pos_values
    return True
"""
