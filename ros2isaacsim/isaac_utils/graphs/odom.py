import os
import omni.graph.core as og
from isaac_utils.graphs import Graph
from isaacsim.core.utils import extensions

extensions.enable_extension("omni.graph.nodes")
extensions.enable_extension("isaacsim.core.nodes")
extensions.enable_extension("isaacsim.ros2.bridge")


def odom(
    graph_path: str,
    prim_path: str,
    base_frame_id: str = 'base_link',
    odom_frame_id: str = 'odom',
    map_frame_id: str = 'map',
) -> bool:
    """
    Creates an OmniGraph Action Graph to publish nav2 - type odometry information for a given prim
    using ROS2.

    Args:
        graph_path(str): The USD path where the Action Graph will be created(e.g., '/ActionGraph').
        prim_path(str): The USD path to the prim for which to publish odometry(e.g., '/World/MyRobot/chassis').
        tf_prefix(str): The prefix to apply to the TF frames published by this graph(e.g., 'jackal').
        base_frame_id(str): The name of the base frame for the robot(e.g., 'base_link').
        odom_frame_id(str): The name of the odometry frame(e.g., 'odom').
        map_frame_id(str): The name of the map frame(included for completeness but not used in this graph).

    Returns:
        bool: True if the graph was created successfully, False otherwise.
    """

    controller = og.Controller()

    graph = Graph(graph_path)

    on_playback_tick = graph.node('on_playback_tick', 'omni.graph.action.OnPlaybackTick')
    read_simulation_time = graph.node('read_simulation_time', 'isaacsim.core.nodes.IsaacReadSimulationTime')
    get_transform = graph.node('get_transform', 'omni.graph.nodes.GetPrimLocalToWorldTransform')
    extract_translation = graph.node('extract_translation', 'omni.graph.nodes.GetMatrix4Translation')
    extract_rotation = graph.node('extract_rotation', 'omni.graph.nodes.GetMatrix4Quaternion')
    publish_map = graph.node('publish_odom_static', 'isaacsim.ros2.bridge.ROS2PublishRawTransformTree')
    publish_odom = graph.node('publish_odom', 'isaacsim.ros2.bridge.ROS2PublishRawTransformTree')

    on_playback_tick.connect('tick', publish_odom, 'execIn')
    on_playback_tick.connect('tick', publish_map, 'execIn')
    read_simulation_time.connect('simulationTime', publish_odom, 'timeStamp')
    read_simulation_time.connect('simulationTime', publish_map, 'timeStamp')

    get_transform.attribute('primPath', prim_path)
    get_transform.connect('localToWorldTransform', extract_translation, 'matrix')
    get_transform.connect('localToWorldTransform', extract_rotation, 'matrix')

    publish_odom.attribute('parentFrameId', odom_frame_id)
    publish_odom.attribute('childFrameId', base_frame_id)
    extract_translation.connect('translation', publish_odom, 'translation')
    extract_rotation.connect('quaternion', publish_odom, 'rotation')

    publish_map.attribute('parentFrameId', map_frame_id)
    publish_map.attribute('childFrameId', odom_frame_id)
    publish_map.attribute('translation', [0., 0., 0.])
    publish_map.attribute('rotation', [0., 0., 0., 1.])

    return graph.execute(controller)
