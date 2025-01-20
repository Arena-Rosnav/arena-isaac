#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')

        # Declare parameters with default values
        self.declare_parameter('linear_x', -0.2)
        self.declare_parameter('linear_y', 0.0)
        self.declare_parameter('linear_z', 0.0)
        self.declare_parameter('angular_x', 0.0)
        self.declare_parameter('angular_y', 0.0)
        self.declare_parameter('angular_z', 0.0)
        self.declare_parameter('publish_frequency', 10.0)

        # Retrieve parameter values
        linear_x = self.get_parameter('linear_x').get_parameter_value().double_value
        linear_y = self.get_parameter('linear_y').get_parameter_value().double_value
        linear_z = self.get_parameter('linear_z').get_parameter_value().double_value
        angular_x = self.get_parameter('angular_x').get_parameter_value().double_value
        angular_y = self.get_parameter('angular_y').get_parameter_value().double_value
        angular_z = self.get_parameter('angular_z').get_parameter_value().double_value
        publish_frequency = self.get_parameter('publish_frequency').get_parameter_value().double_value

        # Create a publisher to /cmd_vel topic with a queue size of 10
        self.publisher_ = self.create_publisher(Twist, 'waffle/cmd_vel', 10)

        # Timer to control the publishing rate
        timer_period = 1.0 / publish_frequency  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize Twist message
        self.twist = Twist()
        self.twist.linear.x = linear_x
        self.twist.linear.y = linear_y
        self.twist.linear.z = linear_z
        self.twist.angular.x = angular_x
        self.twist.angular.y = angular_y
        self.twist.angular.z = angular_z

        self.get_logger().info('CmdVelPublisher node has been started with parameters.')

    def timer_callback(self):
        # Publish the Twist message
        self.publisher_.publish(self.twist)
        self.get_logger().debug(f'Published Twist: {self.twist}')

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_publisher = CmdVelPublisher()
    try:
        rclpy.spin(cmd_vel_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        cmd_vel_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
