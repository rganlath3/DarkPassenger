#!/usr/bin/env python3

#Dexter-MCU-Bridge base_differential_drive_controller.py by Ranil Ganlath. 
#This node is for controlling the robot without ROS2_Control. (If you are using ros2_control, see base_ros2_control_hardware_interface.py)
#This ROS2 Python Node creates a publisher and subscriber setup.
# The subscriber reads /cmd_vel Twist topic published by a ROS controller such as teleop_twist_keyboard 
# It then converts the Twist message to left and right wheel motor commands and publishes them on the /MCU/Base/serial_tx topic with message format <M,LEFT_WHEEL_VELOCITY,RIGHT_WHEEL_VELOCITY>
# Copyright (C) 2025 Ranil Ganlath

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math


class DifferentialDriveController(Node):

    def __init__(self):
        super().__init__('base_differential_drive_controller')

        # Declare parameters
        self.declare_parameter('wheel_radius', 0.035)  # in meters
        self.declare_parameter('wheel_separation', 0.218)  # in meters
        self.declare_parameter('max_linear_velocity', 1.0)  # in m/s
        self.declare_parameter('max_angular_velocity', 2.0)  # in rad/s
        self.declare_parameter('velocity_scale', 5.0)  # scale factor for motor velocity

        # Get parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        self.velocity_scale = self.get_parameter('velocity_scale').value


        # Create subscriber
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)


        # Initialize publishers

        self.publisher = self.create_publisher(String, 'MCU/Base/serial_tx', 10)

        self.get_logger().info('Differential Drive Controller Node started')



    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Apply velocity limits
        linear_x = max(min(linear_x, self.max_linear_velocity), -self.max_linear_velocity)
        angular_z = max(min(angular_z, self.max_angular_velocity), -self.max_angular_velocity)
        
        # Apply angular scaling
        angular_z = 3 * angular_z

        # Differential drive kinematics
        # v_l = (linear_x - angular_z * wheel_separation / 2) / wheel_radius
        # v_r = (linear_x + angular_z * wheel_separation / 2) / wheel_radius
        left_wheel_velocity = (linear_x - angular_z * self.wheel_separation / 2) / self.wheel_radius
        right_wheel_velocity = (linear_x + angular_z * self.wheel_separation / 2) / self.wheel_radius
        
        # Scale to motor control units
        left_wheel_cmd = int(left_wheel_velocity * self.velocity_scale)
        right_wheel_cmd = int(right_wheel_velocity * self.velocity_scale)
        
        # Apply velocity limits (Max is 255)
        left_wheel_cmd = min(255, max(-255, left_wheel_cmd))
        right_wheel_cmd = min(255, max(-255, right_wheel_cmd))
        

        # Format the command string
        cmd_str = f"<M,{left_wheel_cmd},{right_wheel_cmd}>"
        
        # Create and publish the message
        msg = String()
        msg.data = cmd_str
        self.publisher.publish(msg)
        
        self.get_logger().debug(f'Published: {cmd_str}')


def main(args=None):
    rclpy.init(args=args)


    differential_drive_controller = DifferentialDriveController()
    
    try:
        rclpy.spin(differential_drive_controller)
    except KeyboardInterrupt:
        pass
    finally:
        differential_drive_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()