import omni.graph.core as og
from isaac_utils.graphs import Graph


def joint_states(
    graph_path: str,
    prim_path: str,
    joint_states_topic: str
) -> bool:
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
    on_playback_tick = graph.node('on_playback_tick', 'omni.graph.action.OnPlaybackTick')
    publish_joint_state = graph.node('publish_joint_state', 'isaacsim.ros2.bridge.ROS2PublishJointState')
    read_sim_time = graph.node('read_sim_time', 'isaacsim.core.nodes.IsaacReadSimulationTime')

    # Set values
    publish_joint_state.attribute('targetPrim', prim_path)
    publish_joint_state.attribute('topicName', joint_states_topic)

    # Connect nodes
    on_playback_tick.connect('tick', publish_joint_state, 'execIn')
    read_sim_time.connect('simulationTime', publish_joint_state, 'timeStamp')

    graph.execute(og.Controller())

    return True
