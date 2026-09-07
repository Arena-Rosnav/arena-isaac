import functools

import omni.graph.core as og

from isaac_utils.graphs import Graph, register_rebuilder


def joint_states(graph_path: str, prim_path: str, joint_states_topic: str) -> bool:
    """
    Creates an OmniGraph Action Graph to publish nav2 - type odometry information for a given prim
    using ROS2.

    Args:
        graph_path(str): The USD path where the Action Graph will be created(e.g., '/ActionGraph').
        prim_path(str): The USD path to the prim for which to publish odometry(e.g., '/World/MyRobot/chassis').
        base_frame_id(str): The name of the base frame for the robot(e.g., 'base_link').
        odom_frame_id(str): The name of the odometry frame(e.g., 'odom').
        map_frame_id(str): The name of the map frame(included for completeness but not used in this graph).

    Returns:
        bool: True if the graph was created successfully, False otherwise.
    """

    graph = Graph(graph_path)

    # Create nodes
    # publisher targetPrim is deprecated and its reader finds no joints under asset
    # structure 3.0 (joints live in the sibling Physics scope), feed it from the
    # tensor-backed read node instead
    on_playback_tick = graph.node('on_playback_tick', 'omni.graph.action.OnPlaybackTick')
    read_joint_state = graph.node('read_joint_state', 'isaacsim.sensors.physics.IsaacReadJointState')
    publish_joint_state = graph.node('publish_joint_state', 'isaacsim.ros2.bridge.ROS2PublishJointState')
    read_sim_time = graph.node('read_sim_time', 'isaacsim.core.nodes.IsaacReadSimulationTime')

    # Set values
    read_sim_time.attribute('resetOnStop', False)
    read_joint_state.attribute('prim', prim_path)
    publish_joint_state.attribute('topicName', joint_states_topic)

    # Connect nodes
    on_playback_tick.connect('tick', read_joint_state, 'execIn')
    read_joint_state.connect('execOut', publish_joint_state, 'execIn')
    for attr in (
        'jointNames',
        'jointPositions',
        'jointVelocities',
        'jointEfforts',
        'jointDofTypes',
        'stageMetersPerUnit',
    ):
        read_joint_state.connect(attr, publish_joint_state, attr)
    read_sim_time.connect('simulationTime', publish_joint_state, 'timeStamp')

    graph.load_extensions()
    register_rebuilder(graph_path, functools.partial(joint_states, graph_path, prim_path, joint_states_topic))
    return graph.execute(og.Controller())
