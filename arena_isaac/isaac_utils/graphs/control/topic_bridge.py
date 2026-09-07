import omni.graph.core as og
from isaacsim.core.utils import extensions

from isaac_utils.graphs import Graph

extensions.enable_extension("isaacsim.core.nodes")
extensions.enable_extension("isaacsim.ros2.bridge")
extensions.enable_extension("isaacsim.sensors.physics.nodes")


def topic_bridge(
    graph_path: str,
    prim_path: str,
    joints_velocity: list[str],
    joints_position: list[str],
    velocity_commands_topic: str,
    position_commands_topic: str,
    states_topic: str,
) -> bool:
    """Action graph bridging an external ros2_control controller_manager to Isaac.

    One JointState subscriber per command-interface kind: each upstream
    JointStateTopicSystem hardware component emits a homogeneous message (name[]
    co-indexed with the kind's value array), so the velocity and position arrays
    feed straight into their respective articulation controllers with no
    re-indexing required.
    """
    if not (joints_velocity or joints_position):
        return False

    graph = Graph(graph_path)

    on_playback_tick = graph.node('on_playback_tick', 'omni.graph.action.OnPlaybackTick')
    get_target_prim = graph.node('get_target_prim', 'omni.replicator.core.OgnGetPrimAtPath')
    isaac_read_simulation_time = graph.node('isaac_read_simulation_time', 'isaacsim.core.nodes.IsaacReadSimulationTime')
    isaac_read_simulation_time.attribute('resetOnStop', False)
    # publisher targetPrim is deprecated and its reader finds no joints under asset
    # structure 3.0 (joints live in the sibling Physics scope), feed it from the
    # tensor-backed read node instead
    read_joint_state = graph.node('read_joint_state', 'isaacsim.sensors.physics.IsaacReadJointState')
    ros2_publish_joint_state = graph.node('ros2_publish_joint_state', 'isaacsim.ros2.bridge.ROS2PublishJointState')

    get_target_prim.attribute('paths', [prim_path])
    ros2_publish_joint_state.attribute('topicName', states_topic)

    on_playback_tick.connect('tick', get_target_prim, 'execIn')
    on_playback_tick.connect('tick', read_joint_state, 'execIn')
    read_joint_state.connect('execOut', ros2_publish_joint_state, 'execIn')

    isaac_read_simulation_time.connect('simulationTime', ros2_publish_joint_state, 'timeStamp')
    get_target_prim.connect('prims', read_joint_state, 'prim')
    for attr in (
        'jointNames',
        'jointPositions',
        'jointVelocities',
        'jointEfforts',
        'jointDofTypes',
        'stageMetersPerUnit',
    ):
        read_joint_state.connect(attr, ros2_publish_joint_state, attr)

    def _wire_kind(kind: str, joints: list[str], topic: str, command_attr: str) -> None:
        subscriber = graph.node(f'ros2_subscribe_joint_state_{kind}', 'isaacsim.ros2.bridge.ROS2SubscribeJointState')
        subscriber.attribute('topicName', topic)
        on_playback_tick.connect('tick', subscriber, 'execIn')

        controller = graph.node(f'{kind}_articulation_controller', 'isaacsim.core.nodes.IsaacArticulationController')
        on_playback_tick.connect('tick', controller, 'execIn')
        get_target_prim.connect('prims', controller, 'targetPrim')

        names_array = graph.node(f'{kind}_names_array', 'omni.graph.nodes.ConstructArray')
        names_array.attribute('arraySize', len(joints))
        for i, joint_name in enumerate(joints):
            if i > 0:
                names_array.create_attribute(f'inputs:input{i}', 'token')
            token_node = graph.node(f'{kind}_name_const_{i}', 'omni.graph.nodes.ConstantToken')
            token_node.attribute('value', joint_name)
            token_node.connect('value', names_array, f'input{i}', outputs_prefix='inputs:')
        names_array.connect('array', controller, 'jointNames')

        subscriber.connect(command_attr, controller, f'{kind}Command')

    if joints_velocity:
        _wire_kind('velocity', joints_velocity, velocity_commands_topic, 'velocityCommand')
    if joints_position:
        _wire_kind('position', joints_position, position_commands_topic, 'positionCommand')

    graph.load_extensions()
    return graph.execute(og.Controller())
