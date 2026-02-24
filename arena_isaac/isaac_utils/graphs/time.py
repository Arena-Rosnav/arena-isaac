import omni.graph.core as og
from isaac_utils.graphs import Graph
from isaacsim.core.utils import extensions

extensions.enable_extension("isaacsim.core.nodes")
extensions.enable_extension("isaacsim.ros2.bridge")


def PublishTime(graph_path: str):
    graph = Graph(graph_path)

    on_playback_tick = graph.node('on_playback_tick', 'omni.graph.action.OnPlaybackTick')
    read_simulation_time = graph.node('read_simulation_time', 'isaacsim.core.nodes.IsaacReadSimulationTime')
    publish_clock = graph.node('publish_clock', 'isaacsim.ros2.bridge.ROS2PublishClock')

    on_playback_tick.connect('tick', publish_clock, 'execIn')
    read_simulation_time.connect('simulationTime', publish_clock, 'timeStamp')

    graph.execute(og.Controller())
