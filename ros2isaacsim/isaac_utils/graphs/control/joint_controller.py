import omni.graph.core as og
from isaac_utils.graphs import Graph

from omni.isaac.core.utils import extensions

extensions.enable_extension("omni.isaac.core_nodes")
extensions.enable_extension("omni.isaac.ros2_bridge")


def joint_controller(
    graph_path: str,
    prim_path: str,
    joint_topic_name: str,
) -> bool:
    """
    Creates an OmniGraph Action Graph to control a joint using ROS2.
    Args:
        graph_path(str): The USD path where the Action Graph will be created(e.g., '/ActionGraph').
        prim_path(str): The USD path to the robot's base prim.
        joint_topic_name(str): A joint topic name for the joint controller in isaacsim.
    Returns:
        bool: True if the graph was created successfully, False otherwise.
    """

    graph = Graph(graph_path)

    # Create nodes
    on_playback_tick = graph.node('on_playback_tick', 'omni.graph.action.OnPlaybackTick')
    ros2_publish_joint_state = graph.node('ros2_publish_joint_state', 'omni.isaac.ros2_bridge.ROS2PublishJointState')
    isaac_read_simulation_time = graph.node('isaac_read_simulation_time', 'omni.isaac.core_nodes.IsaacReadSimulationTime')
    ros2_subscribe_joint_state = graph.node('ros2_subscribe_joint_state', 'omni.graph.nodes.ROS2SubscribeJointState')
    articulation_controller = graph.node('articulation_controller', 'omni.isaac.core_nodes.IsaacArticulationController')

    # Connect nodes
    on_playback_tick.connect('tick', ros2_publish_joint_state, 'execIn')
    on_playback_tick.connect('tick', ros2_subscribe_joint_state, 'execIn')
    on_playback_tick.connect('tick', articulation_controller, 'execIn')
    isaac_read_simulation_time.connect('simulationTime', ros2_publish_joint_state, 'timeStamp')
    ros2_subscribe_joint_state.connect('jointNames', articulation_controller, 'jointNames')
    ros2_subscribe_joint_state.connect('positionCommand', articulation_controller, 'positionCommand')
    ros2_subscribe_joint_state.connect('velocityCommand', articulation_controller, 'velocityCommand')
    
    # Set values
    ros2_publish_joint_state.attribute('targetPrim', prim_path)
    ros2_publish_joint_state.attribute('topicName', joint_topic_name)

    graph.execute(og.Controller())

    return True
