import rclpy
from rclpy.node import Node
from isaacsim_msgs.srv import ImportUsd
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import cv2
import numpy

class controller(Node):
    def __init__(self):
        super().__init__("controller")
        self.client = self.create_client(
            srv_type=ImportUsd,
            srv_name='/import_usd',
        )
        self.request = ImportUsd.Request()
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.send(name='waffle', 
                           usd_path='robot_models/Arena_rosnav/User/waffle.usd', 
                           prim_path='/World', 
                           control=True)
        rclpy.spin_until_future_complete(self, future)
        self.response = future.result()
        self.get_logger().info("Spawn complete!")
        self.info = JointState()
        self.subscriber_ = self.create_subscription(
            msg_type=JointState,
            topic='/waffle_states',
            callback=self.get_data,
            qos_profile=1
        )
        self.left_wheel_vel = .0
        self.right_wheel_vel = .0
        self.publisher_ = self.create_publisher(
            msg_type=JointState,
            topic='/waffle_command',
            qos_profile=1
        )
        self.timer_ = self.create_timer(1/60, self.control)
        self.canvas = numpy.zeros((300, 300))
        
    def control(self):
        cv2.imshow('control', self.canvas)
        key = cv2.waitKey(1)
        if key == ord('s'):
            self.left_wheel_vel += 1
        if key == ord('a'):
            self.left_wheel_vel -= 1
        if key == ord('k'):
            self.right_wheel_vel += 1
        if key == ord('l'):
            self.right_wheel_vel -= 1
        control_signal = JointState()
        control_signal.header = self.info.header
        control_signal.name = self.info.name
        control_signal.position = self.info.position
        control_signal.velocity = [float(self.left_wheel_vel), float(self.right_wheel_vel)]
        control_signal.effort = self.info.effort
        self.publisher_.publish(control_signal)

    def get_data(self, msg):
        self.info.header = msg.header
        self.info.name = msg.name
        self.info.position = msg.position
        self.info.velocity = msg.velocity
        self.info.effort = msg.effort
        self.get_logger().info(f"""
        header: {self.info.header}
        name: {self.info.name},
        position: {self.info.position},
        velocity: {self.info.velocity},
        effort: {self.info.effort}
        """)

    def send(self, name, usd_path, prim_path, control):
        self.request.name = name
        self.request.usd_path = usd_path
        self.request.prim_path = prim_path
        self.request.control = control
        return self.client.call_async(self.request)

def main(args=None):
    rclpy.init()
    node = controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()