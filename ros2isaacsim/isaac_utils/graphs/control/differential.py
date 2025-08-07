import omni.graph.core as og
from isaac_utils.graphs import Graph

from isaacsim.core.utils import extensions

extensions.enable_extension("isaacsim.core.nodes")
extensions.enable_extension("isaacsim.ros2.bridge")
extensions.enable_extension("omni.isaac.wheeled_robots")


def differential(
    graph_path: str,
    prim_path: str,
    cmd_vel_topic: str,
    joint_names: list[str],
    wheel_distance: float,
    wheel_radius: float,
    max_linear_speed: float,
    min_linear_speed: float,
    max_angular_speed: float,
    min_angular_speed: float,
) -> bool:
    """
    Creates an OmniGraph Action Graph to control a robot using ROS2.
    Args:
        graph_path(str): The USD path where the Action Graph will be created(e.g., '/ActionGraph').
        prim_path(str): The USD path to the robot's base prim.
        cmd_vel_topic(str): The ROS2 topic for controlling the robot.
        joint_names(list[str]): A list of joint names for the robot.
    Returns:
        bool: True if the graph was created successfully, False otherwise.
    """

    graph = Graph(graph_path)

    # Create nodes
    on_playback_tick = graph.node('on_playback_tick', 'omni.graph.action.OnPlaybackTick')
    ros2_subscribe_twist = graph.node('ros2_subscribe_twist', 'isaacsim.ros2.bridge.ROS2SubscribeTwist')
    scale_stage_units = graph.node('scale_stage_units', 'isaacsim.core.nodes.OgnIsaacScaleToFromStageUnit')
    break3vector_linear = graph.node('break3vector_linear', 'omni.graph.nodes.BreakVector3')
    break3vector_angular = graph.node('break3vector_angular', 'omni.graph.nodes.BreakVector3')
    differential_controller = graph.node('differential_controller', 'omni.isaac.wheeled_robots.DifferentialController')
    make_array = graph.node('make_array', 'omni.graph.nodes.ConstructArray')
    articulation_controller = graph.node('articulation_controller', 'isaacsim.core.nodes.IsaacArticulationController')

    # Set values
    make_array.attribute('arraySize', len(joint_names))
    ros2_subscribe_twist.attribute('topicName', cmd_vel_topic)
    differential_controller.attribute('wheelDistance', wheel_distance)
    differential_controller.attribute('wheelRadius', wheel_radius)
    differential_controller.attribute('maxWheelSpeed', 10.0)
    differential_controller.attribute('maxLinearSpeed', max_linear_speed)
    differential_controller.attribute('maxAngularSpeed', max_angular_speed)
    articulation_controller.attribute('targetPrim', prim_path)

    # Connect nodes
    on_playback_tick.connect('tick', ros2_subscribe_twist, 'execIn')
    on_playback_tick.connect('tick', articulation_controller, 'execIn')
    ros2_subscribe_twist.connect('execOut', differential_controller, 'execIn')
    ros2_subscribe_twist.connect('linearVelocity', scale_stage_units, 'value')
    scale_stage_units.connect('result', break3vector_linear, 'tuple')
    break3vector_linear.connect('x', differential_controller, 'linearVelocity')
    ros2_subscribe_twist.connect('angularVelocity', break3vector_angular, 'tuple')
    break3vector_angular.connect('z', differential_controller, 'angularVelocity')
    differential_controller.connect('velocityCommand', articulation_controller, 'velocityCommand')

    for i, joint_name in enumerate(joint_names):
        if i > 0:
            make_array.create_attribute(f'inputs:input{i}', 'token')
        token_node = graph.node(f'make_array_const_{i}', 'omni.graph.nodes.ConstantToken')
        token_node.attribute('value', joint_name)
        token_node.connect('value', make_array, f'input{i}', outputs_prefix='inputs:')

    make_array.connect('array', articulation_controller, 'jointNames')

    graph.execute(og.Controller())

    return True
