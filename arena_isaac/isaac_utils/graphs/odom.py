import omni.graph.core as og

from isaac_utils.graphs import Graph, physics_engine
from isaac_utils.graphs.transform import base_pose


def odom(
    graph_path: str,
    prim_path: str,
    base_prim: str,
    base_frame_id: str = 'base_link',
    odom_frame_id: str = 'odom',
    map_frame_id: str = 'map',
    odom_topic: str = '',
) -> bool:
    """Action graph publishing nav2 odometry. `prim_path` is a tracked rigid body
    fixed to the base; `base_prim` is the base-link holder. Publishes the base pose
    (body composed with its constant body->base offset) on odom->base, identity
    map->odom (reset-safe). `odom_topic` empty skips the nav_msgs/Odometry topic."""

    controller = og.Controller()

    graph = Graph(graph_path)

    newton = physics_engine() == 'newton'

    on_playback_tick = graph.node('on_playback_tick', 'omni.graph.action.OnPlaybackTick')
    read_simulation_time = graph.node('read_simulation_time', 'isaacsim.core.nodes.IsaacReadSimulationTime')
    read_simulation_time.attribute('resetOnStop', False)
    get_base_transform = graph.node('get_base_transform', 'omni.graph.nodes.GetPrimLocalToWorldTransform')
    base = base_pose(graph, 'base_link_pose', body_is_matrix=not newton)
    publish_map = graph.node('publish_odom_static', 'isaacsim.ros2.bridge.ROS2PublishRawTransformTree')
    publish_odom = graph.node('publish_odom', 'isaacsim.ros2.bridge.ROS2PublishRawTransformTree')

    on_playback_tick.connect('tick', base, 'execIn')
    on_playback_tick.connect('tick', publish_map, 'execIn')
    base.connect('execOut', publish_odom, 'execIn')
    read_simulation_time.connect('simulationTime', publish_odom, 'timeStamp')
    read_simulation_time.connect('simulationTime', publish_map, 'timeStamp')

    # the usd body matrix serves the one-time offset capture under both engines,
    # and stays the live pose source under physx
    get_transform = graph.node('get_transform', 'omni.graph.nodes.GetPrimLocalToWorldTransform')
    get_transform.attribute('primPath', prim_path)
    get_transform.connect('localToWorldTransform', base, 'body_matrix')

    if newton:
        # newton syncs body poses to fabric only, usd xforms keep their spawn
        # values, so the live body pose must come from the fabric world matrix
        read_world_pose = graph.node('read_world_pose', 'isaacsim.core.nodes.IsaacReadWorldPose')
        read_world_pose.attribute('prim', prim_path)
        read_world_pose.connect('translation', base, 'body_translation')
        read_world_pose.connect('orientation', base, 'body_orientation')

    get_base_transform.attribute('primPath', base_prim)
    get_base_transform.connect('localToWorldTransform', base, 'base_matrix')

    publish_odom.attribute('parentFrameId', odom_frame_id)
    publish_odom.attribute('childFrameId', base_frame_id)
    base.connect('translation', publish_odom, 'translation')
    base.connect('quaternion', publish_odom, 'rotation')

    publish_map.attribute('parentFrameId', map_frame_id)
    publish_map.attribute('childFrameId', odom_frame_id)
    publish_map.attribute('translation', [0.0, 0.0, 0.0])
    publish_map.attribute('rotation', [0.0, 0.0, 0.0, 1.0])

    if odom_topic:
        publish_odom_topic = graph.node('publish_odom_topic', 'isaacsim.ros2.bridge.ROS2PublishOdometry')
        base.connect('execOut', publish_odom_topic, 'execIn')
        read_simulation_time.connect('simulationTime', publish_odom_topic, 'timeStamp')
        publish_odom_topic.attribute('topicName', odom_topic)
        publish_odom_topic.attribute('odomFrameId', odom_frame_id)
        publish_odom_topic.attribute('chassisFrameId', base_frame_id)
        base.connect('translation', publish_odom_topic, 'position')
        base.connect('quaternion', publish_odom_topic, 'orientation')

    graph.load_extensions()
    return graph.execute(controller)
