import os

import omni.graph.core as og

from isaac_utils.graphs import Graph


def tf(
    graph_path: str,
    prim_path: str,
    tf_prefix: str,
    throttle: int = 1,
) -> bool:
    """
    Creates an OmniGraph Action Graph to publish tf subtree of a given prim.

    Args:
        graph_path(str): The USD path where the Action Graph will be created(e.g., '/ActionGraph').
        prim_path(str): The USD path to the prim for which to publish odometry(e.g., '/World/MyRobot/chassis').
        tf_prefix(str): The prefix to apply to the TF frames published by this graph(e.g., 'jackal').
        throttle(int): The throttle ratio (every n ticks) to run the graph at (e.g., 300).

    Returns:
        bool: True if the graph was created successfully, False otherwise.
    """

    controller = og.Controller()

    graph = Graph(graph_path)

    prefixer_node_name = f"tf_prefixer_node_{graph_path.replace('/', '_')}"

    internal_tf_topic = os.path.join(prefixer_node_name, 'tf_internal')

    on_tick = graph.node('on_tick', 'omni.graph.action.OnTick')
    on_tick.attribute('framePeriod', throttle)

    read_simulation_time = graph.node('read_simulation_time', 'isaacsim.core.nodes.IsaacReadSimulationTime')
    read_simulation_time.attribute('resetOnStop', False)
    get_base_prim = graph.node('get_base_prim', 'omni.replicator.core.OgnGetPrimAtPath')
    publish_tf = graph.node('publish_tf', 'isaacsim.ros2.bridge.ROS2PublishTransformTree')
    prefix_transform = graph.node('prefix_transform', 'omni.graph.scriptnode.ScriptNode')

    on_tick.connect('tick', get_base_prim, 'execIn')
    get_base_prim.connect('execOut', publish_tf, 'execIn')
    get_base_prim.connect('execOut', prefix_transform, 'execIn')
    read_simulation_time.connect('simulationTime', publish_tf, 'timeStamp')

    get_base_prim.attribute('paths', [prim_path])
    get_base_prim.connect('prims', publish_tf, 'parentPrim')
    get_base_prim.connect('prims', publish_tf, 'targetPrims')

    publish_tf.attribute('topicName', internal_tf_topic)

    prefix_transform.create_attribute('inputs:prefix', 'string')
    prefix_transform.attribute('prefix', tf_prefix)
    prefix_transform.create_attribute('inputs:topic', 'string')
    prefix_transform.attribute('topic', internal_tf_topic)
    prefix_transform.create_attribute('inputs:name', 'string')
    prefix_transform.attribute('name', prefixer_node_name)
    prefix_transform.create_attribute('inputs:timestamp', 'double')
    read_simulation_time.connect('simulationTime', prefix_transform, 'timestamp')

    prefix_transform.attribute('script', PREFIX_SCRIPT)

    return graph.execute(controller)


PREFIX_SCRIPT = """
import rclpy
from tf2_msgs.msg import TFMessage
import os

def setup(db: og.Database):
    node_name = db.inputs.name
    if not node_name:
        return False

    try:
        rclpy.init()
        db.per_instance_state.rclpy_was_shutdown = True
    except:
        db.per_instance_state.rclpy_was_shutdown = False

    db.per_instance_state.node = rclpy.create_node(node_name)
    db.per_instance_state.last_message = None

    def tf_callback(msg):
        db.per_instance_state.last_message = msg

    input_topic = db.inputs.topic
    if not input_topic:
        return False

    db.per_instance_state.subscriber = db.per_instance_state.node.create_subscription(
        TFMessage,
        input_topic,
        tf_callback,
        10)

    db.per_instance_state.publisher = db.per_instance_state.node.create_publisher(
        TFMessage,
        "/tf",
        10)

    db.per_instance_state.node.get_logger().info(
        f"'{node_name}' active. Subscribed to '{input_topic}', publishing to '/tf'."
    )

def cleanup(db: og.Database):
    if db.per_instance_state.node:
        db.per_instance_state.node.get_logger().info("Shutting down tf_prefixer node.")
        db.per_instance_state.node.destroy_node()
        db.per_instance_state.node = None
    if db.per_instance_state.rclpy_was_shutdown:
        rclpy.shutdown()

def compute(db: og.Database):
    node = db.per_instance_state.node
    if not node or not rclpy.ok():
        return True

    rclpy.spin_once(node, timeout_sec=0)

    if db.per_instance_state.last_message is not None:
        incoming_message = db.per_instance_state.last_message
        db.per_instance_state.last_message = None

        prefix = db.inputs.prefix
        timestamp = db.inputs.timestamp

        modified_message = TFMessage()
        for transform in incoming_message.transforms:
            new_transform = transform

            # Only apply prefix if the input string is not empty
            if prefix:
                new_transform.header.frame_id = os.path.join(prefix, transform.header.frame_id)
                new_transform.child_frame_id = os.path.join(prefix, transform.child_frame_id)
                if timestamp:
                    new_transform.header.stamp.sec = int(timestamp)
                    new_transform.header.stamp.nanosec = int((timestamp - int(timestamp)) * 1e9)

            modified_message.transforms.append(new_transform)

        if modified_message.transforms:
            db.per_instance_state.publisher.publish(modified_message)

    return True
"""
