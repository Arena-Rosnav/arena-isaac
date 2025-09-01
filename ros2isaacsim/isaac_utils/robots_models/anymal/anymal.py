from isaac_utils.yaml_utils import read_yaml_config
import omni.graph.core as og
from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy, AnymalFlatTerrainPolicy


class Anymal(Node):
    def __init__(self):
        super().__init__('spot_teleop')
        self.subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )
        self.latest_cmd = np.array([0.0, 0.0, 0.0])

    def listener_callback(self, msg: Twist):
        forward = msg.linear.x
        lateral = msg.linear.y
        yaw_rate = msg.angular.z
        self.latest_cmd = np.array([forward, lateral, yaw_rate])

rclpy.init()
ros2_node = Anymal()
ros2_thread = threading.Thread(target=lambda: rclpy.spin(ros2_node), daemon=True)
ros2_thread.start()

world = World(stage_units_in_meters=1.0, physics_dt=1.0/500.0, rendering_dt=10.0/500.0)
world.scene.add_default_ground_plane(z_position=0)

robot_name = "Anymal"

spot = SpotFlatTerrainPolicy(
    prim_path="/World/Spot",
    name="Spot",
    position=np.array([0, 0, 0.8]),
)

def on_physics_step(step_size) -> None:
    # anymal.forward(step_size, ros2_node.latest_cmd)
    spot.forward(step_size, ros2_node.latest_cmd)

# anymal.initialize()
spot.initialize()
world.add_physics_callback("physics_step", callback_fn=on_physics_step)
