#!/usr/bin/env python3
from isaac_utils.sensors import imu_setup, publish_imu, contact_sensor_setup, publish_contact_sensor_info, camera_set_up, publish_camera_tf, publish_depth, publish_camera_info, publish_pointcloud_from_depth, publish_rgb, lidar_setup, publish_lidar
from sensor_msgs.msg import LaserScan, Image
import omni.replicator.core as rep
from pxr import UsdGeom
from isaacsim.core.prims import XFormPrim
from isaac_utils.robot_graphs import assign_robot_model
from isaacsim.core.api.world import World
from isaacsim.core.utils import extensions, stage
from omni.isaac.core import SimulationContext
from isaacsim.core.utils import prims
from isaac_utils import world_generation_utils
import omni
from isaacsim import SimulationApp
from cv_bridge import CvBridge, CvBridgeError
import cv2
import threading
import copy
from geometry_msgs.msg import Twist
import numpy as np
from tf2_msgs.msg import TFMessage
import rclpy
import rclpy.context
import rclpy.executors
from rclpy.node import Node
import torch

# import parent directory
from pathlib import Path
import sys
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from tf2_ros import TransformStamped
# from sac import SAC
# from torch.utils.tensorboard import SummaryWriter
# from replay_buffer import ReplayMemory
parent_dir = Path(__file__).resolve().parent.parent
sys.path.append(str(parent_dir))


# Setting the config for simulation and make an simulation.
CONFIG = {"renderer": "RayTracedLighting", "headless": False}
simulation_app = SimulationApp(CONFIG)


extensions.enable_extension("isaacsim.ros2.bridge")


class NavigationController(Node):
    def __init__(self, name, context):
        super().__init__('navigation_controller', context=context)
        """
        Initialize the NavigationController with the robot's name.

        :param name: The name of the robot (string)
        """
        self.name = name
        self.cmd_vel_topic = f"/{self.name}/cmd_vel"
        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.get_logger().info(f"Controller initialized for robot '{self.name}' on topic '{self.cmd_vel_topic}'.")

        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0

    def _publish_velocity(self, linear_x=1.0, angular_z=0.0):
        """
        Publish a Twist message with specified linear and angular velocities for a certain duration.

        :param linear_x: Linear velocity in the x-direction (float)
        :param angular_z: Angular velocity around the z-axis (float)
        :param duration: Time in seconds to publish the velocity (float)
        """

        self.twist.linear.x = linear_x
        self.twist.angular.z = angular_z

        self.publisher.publish(self.twist)
        # rospy.loginfo("Movement stopped.")

    def turn_right(self, angular_z=0.5):
        """
        Turn the robot to the right.

        :param angular_z: Angular velocity around the z-axis (default: 0.5)
        :param duration: Time in seconds to perform the turn (default: 1.0)
        """

        self.get_logger().info("turning right")
        self._publish_velocity(linear_x=self.twist.linear.x, angular_z=angular_z)

    def turn_left(self, angular_z=-0.5):
        """
        Turn the robot to the left.

        :param angular_z: Angular velocity around the z-axis (default: -0.5)
        :param duration: Time in seconds to perform the turn (default: 1.0)
        """
        self.get_logger().info("turning left")
        self._publish_velocity(linear_x=self.twist.linear.x, angular_z=angular_z)

    def go_straight(self, linear_x=0.5, angular_z=0.0):
        self.get_logger().info("going straight")

        self._publish_velocity(linear_x=linear_x, angular_z=angular_z)

    def reverse(self, linear_x=-0.5, angular_z=0.0):
        """
        Move the robot in reverse.

        :param linear_x: Linear velocity in the x-direction (negative for reverse, default: -0.5)
        :param duration: Time in seconds to move in reverse (default: 2.0)
        """
        self.get_logger().info("reversing")
        self._publish_velocity(linear_x=linear_x, angular_z=angular_z)

    def stop(self):
        self.get_logger().info("stopping")
        self._publish_velocity(linear_x=0.0, angular_z=0.0)


# spawn world
world = World()
omni.usd.get_context().open_stage("/home/ubuntu/arena4_ws/src/arena/isaac/robot_models/map_add_colli_scaled.usd")


# wait for things to load
simulation_app.update()
while isaacsim.core.utils.stage.is_stage_loading():
    simulation_app.update()

simulation_context = omni.isaac.core.SimulationContext(stage_units_in_meters=1.0)
# simulation_context.initialize_physics()
# simulation_context.play()


name = "waffle_1"
prim_path = f"/World/{name}"

model_prim = prims.create_prim(
    prim_path=f"/World/waffle_1",
    position=np.array([-4.8, 4.8, 0.5]),
    orientation=np.array([1.0, 0.0, 0.0, 0.0]),
    usd_path="/home/ubuntu/arena4_ws/src/arena/isaac/robot_models/waffle.usd",
    semantic_label="waffle",
)
camera_prim_path = prim_path + "/camera_link"
camera = camera_set_up(camera_prim_path, "Camera")
camera.initialize()
publish_camera_info(name, camera, 20)
publish_depth(name, camera, 20)
publish_rgb(name, camera, 20)
publish_pointcloud_from_depth(name, camera, 20)
publish_camera_tf(name, prim_path, camera)

lidar_prim_path = prim_path + "/base_scan"
lidar = lidar_setup(lidar_prim_path, "Lidar")
publish_lidar(name, prim_path, lidar)

links = ["wheel_left_link", "wheel_right_link"]
for link in links:
    imu_prim_path = prim_path + "/" + link + "/" + "IMU"
    contact_prim_path = prim_path + "/" + link + "/" + "ContactSensor"
    imu = imu_setup(imu_prim_path)
    contact_sensor = contact_sensor_setup(contact_prim_path)
    publish_contact_sensor_info(name, prim_path, link, contact_sensor)
    publish_imu(name, prim_path, link, imu)

model = assign_robot_model(name, prim_path, "waffle")
model.control_and_publish_joint_states()


body_pose = np.array([0.0, 0.0, 0.0], float)  # x, y, theta
lidar_data = np.zeros(20)
image_L = 720
H = image_L
W = image_L
pix2m = 0.1
L = 8  # length of a box
stage_W = image_L * pix2m
stage_H = image_L * pix2m
clash_sum = 0

rgb_image = np.zeros((H, W, 3), np.uint8)
image_for_clash_calc = np.zeros((H, W), np.uint8)
# start region
pt27 = (int((stage_W / 2 - L / 2 + 24) / pix2m), int((stage_H / 2 + L / 2 + 24) / pix2m))
pt28 = (int((stage_W / 2 + L / 2 + 24) / pix2m), int((stage_H / 2 - L / 2 + 24) / pix2m))
cv2.rectangle(rgb_image, pt27, pt28, (0, 0, 255), cv2.FILLED, cv2.LINE_8)
# goal region
pt27 = (int((stage_W / 2 - L / 2 - 24) / pix2m), int((stage_H / 2 + L / 2 - 24) / pix2m))
pt28 = (int((stage_W / 2 + L / 2 - 24) / pix2m), int((stage_H / 2 - L / 2 - 24) / pix2m))
cv2.rectangle(rgb_image, pt27, pt28, (255, 0, 0), cv2.FILLED, cv2.LINE_8)


class Get_Model_State(Node):
    def __init__(self, context):
        super().__init__('get_modelstate', context=context)
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, data):
        global body_pose

        pose = data.transforms[1].transform.translation
        orientation = data.transforms[1].transform.rotation

        body_pose[0] = pose.x
        body_pose[1] = pose.y
        q0 = orientation.x
        q1 = orientation.y
        q2 = orientation.z
        q3 = orientation.w
        body_pose[2] = -np.atan2(2 * (q0 * q1 + q2 * q3), (q0**2 - q1**2 - q2**2 + q3**2))


class Lidar_Subscriber(Node):

    def __init__(self, name, context):
        super().__init__('lidar_subscriber', context=context)
        self.subscription = self.create_subscription(
            LaserScan,
            f'{name}/lidar_scan',
            self.listener_callback,
            10)
        self.subscription

        self.lidar_data_prev_step = np.zeros(20)

    def listener_callback(self, data):
        global lidar_data

        for i in range(20):
            value = data.ranges[180 * i:180 * i + 8]
            lidar_data[i] = np.max(value)
            if (lidar_data[i] <= 0):
                lidar_data[i] = self.lidar_data_prev_step[i]
            self.lidar_data_prev_step = copy.copy(lidar_data)


class RGB_Subscriber(Node):
    def __init__(self, name, context):
        super().__init__('rgb_subscriber', context=context)
        # Initialize the subscription to the RGB camera topic
        self.subscription = self.create_subscription(
            Image,
            f'{name}/Camera_rgb',
            self.listener_callback,
            10  # QoS history depth
        )
        self.subscription  # Prevent unused variable warning

        # Initialize previous step's image as a NumPy array of zeros
        self.rgb_image_prev_step = np.zeros((480, 640, 3), dtype=np.uint8)  # Adjust dimensions as needed

        # Initialize CvBridge for converting ROS Image messages to OpenCV images
        self.bridge = CvBridge()

    def listener_callback(self, data):
        global rgb_image

        try:
            # Convert ROS Image message to OpenCV image (BGR format)
            current_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

            if current_image is not None:
                rgb_image = current_image
            else:
                # If current image is invalid, retain the previous image
                rgb_image = self.rgb_image_prev_step

        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            # In case of conversion failure, retain the previous image
            rgb_image = self.rgb_image_prev_step

        # Store a copy of the current image for the next step
        self.rgb_image_prev_step = copy.deepcopy(rgb_image)

# class CollisionDetector(Node):
#     def __init__(self, env):
#         super().__init__('collision_detector')
#         self.env = env
#         self.collision = False

#         # Acquire PhysX interface
#         self.physx_interface = physx.acquire_physx_interface()

#         # Subscribe to collision events involving the robot
#         # Replace "/World/Robot" with your robot's actual prim path
#         robot_prim_path = "/World/waffle_1"  # Example path
#         self.physx_interface.collision_event_subscribe(robot_prim_path, self.collision_callback)

#     def collision_callback(self, collision_event: CollisionEvent):
#         # Implement collision detection logic
#         # Example condition based on impulse
#         if collision_event.impulse > 0.1:  # Threshold value
#             self.collision = True
        # self.env.done = True
        # self.get_logger().info(f"Collision detected with {collision_event.other_prim_path}")


class Env(Node):
    def __init__(self,
                 agent_name,
                 world,
                 context):
        super().__init__('isaac_env', context=context)

        self.world = world

        self.model = assign_robot_model(name=agent_name, prim_path="/World/waffle_1", model="waffle")
        self.model.controller = NavigationController(name=agent_name, context=context)

        self.get_model_state = Get_Model_State(context)
        self.lidar_subscriber = Lidar_Subscriber(agent_name, context)
        self.rgb_subscriber = RGB_Subscriber(agent_name, context)

        # self.collision_detector = CollisionDetector(self)

        self.state = {
            'position': np.zeros(2),
            'orientation': 0.0,
            'lidar': np.zeros(20),
            'image': np.zeros((H, W, 3), dtype=np.uint8)
        }

        self.goal = np.array([-4.8, 4.8])

        self.max_episode_steps = 120
        self.current_step = 0
        self.done = False

        self.cube_size = 1.6
        self.obstacles = self.initialize_obstacles()

    def initialize_obstacles(self):
        # # Randomize and place obstacles in the simulation
        # rng = np.random.default_rng()
        # self.obstacles = []
        # for i in range(7):  # Assuming 7 obstacle pairs
        #     pos1 = rng.uniform(-32, 32, size=2)  # Example range
        #     pos2 = rng.uniform(-32, 32, size=2)
        #     self.obstacles.append({'pos1': pos1, 'pos2': pos2})
        #     self.place_obstacle(pos1, f"Obstacle_{i}_1")
        #     self.place_obstacle(pos2, f"Obstacle_{i}_2")
        world_generation_utils.register_cube_spawner(count=21)
        with rep.trigger.on_custom_event('randomize_cubes'):
            rep.randomizer.spawn_cubes()

    # def place_obstacle(self, count):
    #     x_coords = np.random.uniform(-4.5, 4.5, 500).astype(float)
    #     y_coords = np.random.uniform(-4.5, 4.5, 500).astype(float)
    #     z_coords = np.zeros(500).astype(float)
    #     pos = [
    #     (float(x), float(y), float(z))
    #     for x, y, z in zip(x_coords, y_coords, z_coords)
    #     ]

    #     cube = UsdGeom.Cube.Define(self.world.stage, prim_path + "/Geom")
    #     cube.GetSizeAttr().Set(self.cube_size)
    #     xform = XFormPrim(prim_path)
    #     xform.set_world_pose([position[0], position[1], 0.0], [0.0, 0.0, 0.0, 1.0])
    #     objs = rep.create.cube(
    #         count=count
    #     )
    #     with objs:
    #         rep.modify.pose(
    #             position = rep.distribution.choice(pos)
    #         )

    def update_state(self):
        global body_pose, lidar_data, rgb_image

        self.state["position"] = body_pose[:2].copy()
        self.state["orientation"] = body_pose[2].copy()
        self.state["lidar"] = lidar_data.copy()
        self.state["image"] = rgb_image.copy()

    def step(self, action):
        if self.done:
            self.get_logger().warn("Episode has terminated. Please reset the environment.")
            return self.state, 0.0, self.done

        for _ in range(20):  # Adjust the number of steps as needed
            self.apply_action(action)
            self.world.step(render=True)
            # rep.orchestrator.step(delta_time=0.0)

        self.update_state()

        reward = self.calculate_reward()

        self.check_terminated()

        self.current_step += 1

        if self.current_step % 20 == 0:
            self.reset()

        return self.state, reward, self.done

    def apply_action(self, action):
        if action == 0:
            self.model.controller.go_straight()
        elif action == 1:
            self.model.controller.turn_right()
        elif action == 2:
            self.model.controller.turn_left()
        elif action == 3:
            self.model.controller.reverse()
        elif action == 4:
            self.model.controller.stop()

    def reset(self):
        self.update_state()
        simulation_context.reset()
        print('world resetting')
        # context = rclpy.utilities.get_default_context(shutting_down=False)
        # print(context)
        rep.utils.send_og_event(event_name="randomize_cubes")
        return

    def calculate_reward(self):
        distance_to_goal = np.linalg.norm(self.goal - self.state['position'])
        reward = -distance_to_goal

        # Penalty for collision
        if self.check_terminated:
            reward -= 10

        # Reward for reaching the goal
        if distance_to_goal < 1.0:
            reward += 100

        return reward

    def check_terminated(self):
        self.done = False
        return False


class RLAgent(Node):
    def __init__(self, name, num_steps):
        super().__init__('rl_agent')
        self.name = name

        # Initialize the environment
        self.env = Env(name)

        # Initialize RL components (SAC agent, replay buffer, etc.)
        state_dim = 22  # 2 for position, 1 for orientation, 20 for LIDAR
        action_dim = 2  # [linear_velocity, angular_velocity]
        # self.agent = SACAgent(state_dim=state_dim, action_dim=action_dim, args=args)
        # self.memory = ReplayMemory(args.replay_size, args.seed)

        # # Initialize TensorBoard
        # self.writer = SummaryWriter('runs/{}_SAC_{}'.format(
        #     datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S"), name))

        # Initialize training parameters
        self.total_steps = 0
        self.updates = 0
        self.max_steps = num_steps

        # Start training loop in a separate thread
        self.training_thread = threading.Thread(target=self.train, daemon=True)
        self.training_thread.start()

    def train(self):
        while rclpy.ok() and self.total_steps < self.max_steps:
            state = self.env.reset()
            episode_reward = 0
            done = False

            while not done:
                # Select action
                if self.total_steps < self.agent.start_steps:
                    # Explore: random actions
                    action = self.agent.select_random_action()
                else:
                    # Exploit: select action based on policy
                    action = self.agent.select_action(state)

                # Take action in the environment
                next_state, reward, done = self.env.step(action)

                # Store transition in replay buffer
                self.memory.push(state, action, reward, next_state, done)

                state = next_state
                episode_reward += reward
                self.total_steps += 1

                # Update RL agent
                if len(self.memory) > self.agent.batch_size:
                    for _ in range(self.agent.updates_per_step):
                        critic_1_loss, critic_2_loss, policy_loss, ent_loss, alpha = self.agent.update_parameters(self.memory)
                        self.writer.add_scalar('loss/critic_1', critic_1_loss, self.updates)
                        self.writer.add_scalar('loss/critic_2', critic_2_loss, self.updates)
                        self.writer.add_scalar('loss/policy', policy_loss, self.updates)
                        self.writer.add_scalar('loss/entropy_loss', ent_loss, self.updates)
                        self.writer.add_scalar('entropy_alpha', alpha, self.updates)
                        self.updates += 1

            # Log episode reward
            self.writer.add_scalar('reward/episode', episode_reward, self.total_steps)
            self.get_logger().info(f"Episode Reward: {episode_reward}")

            # Optionally, save checkpoints
            if self.total_steps % 1000 == 0:
                self.agent.save_checkpoint(f"{self.name}_checkpoint_{self.total_steps}.pth")


def main():
    context = rclpy.context.Context()
    rclpy.init(context=context)
    env = Env(agent_name="waffle_1", world=World.instance(), context=context)
    executor = rclpy.executors.MultiThreadedExecutor(context=context)
    executor.add_node(env)
    executor.add_node(env.get_model_state)
    executor.add_node(env.lidar_subscriber)
    executor.add_node(env.rgb_subscriber)
    # executor.add_node(env.model.controller)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # # agent = RLAgent("waffle", 500)

    env.reset()
    while True:
        action = np.random.choice(5)
        env.step(action)


if __name__ == '__main__':
    main()
