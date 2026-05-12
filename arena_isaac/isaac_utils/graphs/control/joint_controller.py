import omni.graph.core as og
from isaacsim.core.utils import extensions

from isaac_utils.graphs import Graph

extensions.enable_extension("isaacsim.core.nodes")
extensions.enable_extension("isaacsim.ros2.bridge")


def joint_controller(
    graph_path: str,
    prim_path: str,
    state_topic: str,
    command_topic: str,
) -> bool:
    """Action graph subscribing to JointState commands and publishing JointState
    feedback for an articulated robot.

    Whatever joints the inbound message names get driven on the articulation;
    callers should publish only the joints they intend to actuate.
    """

    graph = Graph(graph_path)

    on_playback_tick = graph.node('on_playback_tick', 'omni.graph.action.OnPlaybackTick')
    ros2_publish_joint_state = graph.node('ros2_publish_joint_state', 'isaacsim.ros2.bridge.ROS2PublishJointState')
    isaac_read_simulation_time = graph.node('isaac_read_simulation_time', 'isaacsim.core.nodes.IsaacReadSimulationTime')
    ros2_subscribe_joint_state = graph.node('ros2_subscribe_joint_state', 'isaacsim.ros2.bridge.ROS2SubscribeJointState')
    get_target_prim = graph.node('get_target_prim', 'omni.replicator.core.OgnGetPrimAtPath')
    articulation_controller = graph.node('articulation_controller', 'isaacsim.core.nodes.IsaacArticulationController')

    get_target_prim.attribute('paths', [prim_path])
    ros2_publish_joint_state.attribute('topicName', state_topic)
    ros2_subscribe_joint_state.attribute('topicName', command_topic)

    on_playback_tick.connect('tick', get_target_prim, 'execIn')
    on_playback_tick.connect('tick', ros2_publish_joint_state, 'execIn')
    on_playback_tick.connect('tick', ros2_subscribe_joint_state, 'execIn')
    on_playback_tick.connect('tick', articulation_controller, 'execIn')

    isaac_read_simulation_time.connect('simulationTime', ros2_publish_joint_state, 'timeStamp')
    get_target_prim.connect('prims', ros2_publish_joint_state, 'targetPrim')
    get_target_prim.connect('prims', articulation_controller, 'targetPrim')

    ros2_subscribe_joint_state.connect('jointNames', articulation_controller, 'jointNames')
    ros2_subscribe_joint_state.connect('positionCommand', articulation_controller, 'positionCommand')
    ros2_subscribe_joint_state.connect('velocityCommand', articulation_controller, 'velocityCommand')

    graph.load_extensions()
    return graph.execute(og.Controller())
