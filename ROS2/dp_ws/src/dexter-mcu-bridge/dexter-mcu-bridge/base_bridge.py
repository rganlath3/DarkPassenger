#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import threading
import time
from math import pi

# ROS2 Messages
from std_msgs.msg import Float32, Int32, String, Bool
from sensor_msgs.msg import Imu, NavSatFix, BatteryState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, Point, Pose, Vector3, TransformStamped
import tf2_ros
from tf2_ros import TransformBroadcaster

# Constants for serial communication
MSG_START_MARKER = '<'
MSG_END_MARKER = '>'
MSG_SEPARATOR = ','

# Message types from ESP32
MSG_ODOM = 'O'
MSG_BATTERY = 'B'
MSG_IMU = 'I'
MSG_GPS = 'G'
MSG_STATUS = 'S'

class ESP32RobotSerialBridge(Node):
    def __init__(self):
        super().__init__('esp32_robot_bridge')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('publish_tf', True)
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value
        self.odom_frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        
        # Physical properties
        self.wheel_diameter = 0.070  # 70mm in meters
        self.wheel_separation = 0.235  # distance between wheels in meters
        
        # Initialize TF broadcaster if needed
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # Initialize publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.battery_pub = self.create_publisher(BatteryState, 'battery', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.gps_pub = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.status_pub = self.create_publisher(String, 'robot_status', 10)
        
        # Initialize subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
            
        self.headlight_sub = self.create_subscription(
            Bool,
            'headlight',
            self.headlight_callback,
            10)
            
        self.brakelight_sub = self.create_subscription(
            Bool,
            'brakelight',
            self.brakelight_callback,
            10)
            
        self.reset_odom_sub = self.create_subscription(
            Bool,
            'reset_odometry',
            self.reset_odometry_callback,
            10)
        
        # Initialize serial port
        try:
            self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to {self.serial_port} at {self.baud_rate} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            rclpy.shutdown()
            return
            
        # Initialize serial reception variables
        self.received_data = ''
        self.receiving_message = False
        
        # Initialize odometry data
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left_encoder = 0
        self.last_right_encoder = 0
        
        # Start the serial reading thread
        self.serial_thread = threading.Thread(target=self.serial_read_loop)
        self.serial_thread.daemon = True
        self.serial_thread.start()
        
        self.get_logger().info('ESP32 Robot bridge node initialized')
    
    def serial_read_loop(self):
        """Thread to continuously read from serial port"""
        while rclpy.ok():
            if self.serial_conn.in_waiting > 0:
                try:
                    data = self.serial_conn.read().decode()
                    
                    if data == MSG_START_MARKER:
                        self.received_data = ''
                        self.receiving_message = True
                    elif data == MSG_END_MARKER:
                        self.receiving_message = False
                        self.process_message(self.received_data)
                    elif self.receiving_message:
                        self.received_data += data
                except Exception as e:
                    self.get_logger().error(f"Error reading from serial: {e}")
            
            # Small delay to prevent CPU hogging
            time.sleep(0.001)
    
    def process_message(self, message):
        """Process received message from ESP32"""
        if not message:
            return
            
        # Split message by separator
        parts = message.split(MSG_SEPARATOR)
        
        if not parts:
            return
            
        # Get message type (first character)
        msg_type = parts[0]
        
        try:
            if msg_type == MSG_ODOM:
                # Format: O,left_encoder,right_encoder,left_dist_mm,right_dist_mm
                left_encoder = int(parts[1])
                right_encoder = int(parts[2])
                left_dist_mm = float(parts[3])
                right_dist_mm = float(parts[4])
                
                # Process odometry
                self.process_odometry(left_encoder, right_encoder, left_dist_mm, right_dist_mm)
                
            elif msg_type == MSG_BATTERY:
                # Format: B,voltage_mv,percentage
                voltage_mv = int(parts[1])
                percentage = int(parts[2])
                
                # Publish battery state
                battery_msg = BatteryState()
                battery_msg.header.stamp = self.get_clock().now().to_msg()
                battery_msg.voltage = voltage_mv / 1000.0  # Convert mV to V
                battery_msg.percentage = float(percentage)
                battery_msg.present = True
                
                self.battery_pub.publish(battery_msg)
                
            elif msg_type == MSG_IMU:
                # Format: I,ax,ay,az,gx,gy,gz,mx,my,mz,direction
                ax = float(parts[1])
                ay = float(parts[2])
                az = float(parts[3])
                gx = float(parts[4])
                gy = float(parts[5])
                gz = float(parts[6])
                mx = float(parts[7])
                my = float(parts[8])
                mz = float(parts[9])
                direction = float(parts[10])  # heading in degrees
                
                # Publish IMU data
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = self.base_frame_id
                
                # Set linear acceleration (convert to m/s^2 if needed)
                imu_msg.linear_acceleration.x = ax
                imu_msg.linear_acceleration.y = ay
                imu_msg.linear_acceleration.z = az
                
                # Set angular velocity (convert to rad/s if needed)
                imu_msg.angular_velocity.x = gx
                imu_msg.angular_velocity.y = gy
                imu_msg.angular_velocity.z = gz
                
                # Note: We're not setting orientation as it requires quaternion conversion
                
                self.imu_pub.publish(imu_msg)
                
            elif msg_type == MSG_GPS:
                # Format: G,lat,lng,satellites,valid
                lat = float(parts[1])
                lng = float(parts[2])
                satellites = int(parts[3])
                valid = int(parts[4])
                
                # Publish GPS fix
                gps_msg = NavSatFix()
                gps_msg.header.stamp = self.get_clock().now().to_msg()
                gps_msg.header.frame_id = "gps_link"
                
                gps_msg.latitude = lat
                gps_msg.longitude = lng
                
                # Set status based on validity
                gps_msg.status.status = 0 if valid else -1  # 0 = FIX, -1 = NO_FIX
                gps_msg.status.service = 1  # 1 = GPS
                
                # Set covariance based on number of satellites
                # This is a rough approximation
                covariance = 100.0 if satellites < 4 else (10.0 if satellites < 7 else 1.0)
                gps_msg.position_covariance[0] = covariance
                gps_msg.position_covariance[4] = covariance
                gps_msg.position_covariance[8] = covariance * 2  # Altitude uncertainty higher
                gps_msg.position_covariance_type = 1  # APPROXIMATED
                
                self.gps_pub.publish(gps_msg)
                
            elif msg_type == MSG_STATUS:
                # Format: S,sw1,sw2,wheel1_sw,wheel2_sw,headlight,brakelight
                sw1 = int(parts[1])
                sw2 = int(parts[2])
                wheel1_sw = int(parts[3])
                wheel2_sw = int(parts[4])
                headlight = int(parts[5])
                brakelight = int(parts[6])
                
                # Create status message as JSON-like string
                status_str = f'{{"switches":{{"sw1":{sw1},"sw2":{sw2},"wheel1":{wheel1_sw},"wheel2":{wheel2_sw}}}, "lights":{{"headlight":{headlight},"brakelight":{brakelight}}}}}'
                status_msg = String()
                status_msg.data = status_str
                
                self.status_pub.publish(status_msg)
                
        except Exception as e:
            self.get_logger().error(f"Error processing message '{message}': {e}")
    
    def process_odometry(self, left_encoder, right_encoder, left_dist_mm, right_dist_mm):
        """Process encoder data and publish odometry"""
        # Convert distances to meters
        left_dist_m = left_dist_mm / 1000.0
        right_dist_m = right_dist_mm / 1000.0
        
        # Calculate encoder changes since last reading
        delta_left = left_encoder - self.last_left_encoder
        delta_right = right_encoder - self.last_right_encoder
        
        # Save current encoder values for next calculation
        self.last_left_encoder = left_encoder
        self.last_right_encoder = right_encoder
        
        # Skip first reading or if no encoder changes
        if delta_left == 0 and delta_right == 0:
            return
            
        # Calculate robot motion
        # This is a simple differential drive model
        # More accurate models would account for wheel slippage and other factors
        
        # Distance traveled by each wheel in this interval
        wheel_left_dist = (delta_left / 508.8) * (pi * self.wheel_diameter)
        wheel_right_dist = (delta_right / 508.8) * (pi * self.wheel_diameter)
        
        # Total distance traveled by robot center
        dist = (wheel_left_dist + wheel_right_dist) / 2.0
        
        # Change in orientation
        d_theta = (wheel_right_dist - wheel_left_dist) / self.wheel_separation
        
        # Update robot pose
        self.theta += d_theta
        
        # Normalize theta to [-pi, pi]
        self.theta = ((self.theta + pi) % (2*pi)) - pi
        
        # Update position
        self.x += dist * cos(self.theta)
        self.y += dist * sin(self.theta)
        
        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id
        
        # Set position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Set orientation (as quaternion)
        quat = self.euler_to_quaternion(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        
        # Set covariance
        # This is a simplified diagonal covariance matrix
        # In a real implementation, you would estimate the uncertainty based on the robot's motion
        odom_msg.pose.covariance = [0.01, 0, 0, 0, 0, 0,
                                    0, 0.01, 0, 0, 0, 0,
                                    0, 0, 0.01, 0, 0, 0,
                                    0, 0, 0, 0.01, 0, 0,
                                    0, 0, 0, 0, 0.01, 0,
                                    0, 0, 0, 0, 0, 0.01]
        
        # Set twist (linear and angular velocities)
        # This is an instantaneous velocity calculated from the wheel movements
        dt = 0.1  # Assume fixed time interval for simplicity
        odom_msg.twist.twist.linear.x = dist / dt
        odom_msg.twist.twist.angular.z = d_theta / dt
        
        # Set twist covariance (similar to pose covariance)
        odom_msg.twist.covariance = [0.01, 0, 0, 0, 0, 0,
                                    0, 0.01, 0, 0, 0, 0,
                                    0, 0, 0.01, 0, 0, 0,
                                    0, 0, 0, 0.01, 0, 0,
                                    0, 0, 0, 0, 0.01, 0,
                                    0, 0, 0, 0, 0, 0.01]
        
        # Publish odometry
        self.odom_pub.publish(odom_msg)
        
        # Publish transform if enabled
        if self.publish_tf:
            self.broadcast_transform(self.x, self.y, quat)
    
    def broadcast_transform(self, x, y, quat):
        """Broadcast transform from odom to base_link"""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.odom_frame_id
        transform.child_frame_id = self.base_frame_id
        
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]
        
        self.tf_broadcaster.sendTransform(transform)
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return [qx, qy, qz, qw]
    
    def cmd_vel_callback(self, msg):
        """Handle velocity commands from ROS2"""
        # Extract linear and angular velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Simple differential drive kinematics
        # v_left = linear_x - (angular_z * wheel_separation / 2)
        # v_right = linear_x + (angular_z * wheel_separation / 2)
        
        # Convert to wheel velocities
        v_left = linear_x - (angular_z * self.wheel_separation / 2.0)
        v_right = linear_x + (angular_z * self.wheel_separation / 2.0)
        
        # Convert to motor speeds (-255 to 255)
        # This mapping depends on your motor characteristics
        # You might need to adjust the scaling factor
        speed_left = int(v_left * 255.0 / 0.5)  # Assuming 0.5 m/s max speed
        speed_right = int(v_right * 255.0 / 0.5)  # Assuming 0.5 m/s max speed
        
        # Constrain values
        speed_left = max(-255, min(255, speed_left))
        speed_right = max(-255, min(255, speed_right))
        
        # Send command to ESP32
        command = f"<M,{speed_left},{speed_right}>"
        self.send_command(command)
    
    def headlight_callback(self, msg):
        """Handle headlight command"""
        headlight = 1 if msg.data else 0
        # Keep brakelight state unchanged (use -1 as "don't change" value)
        command = f"<L,{headlight},-1>"
        self.send_command(command)
    
    def brakelight_callback(self, msg):
        """Handle brakelight command"""
        brakelight = 1 if msg.data else 0
        # Keep headlight state unchanged (use -1 as "don't change" value)
        command = f"<L,-1,{brakelight}>"
        self.send_command(command)
    
    def reset_odometry_callback(self, msg):
        """Reset odometry when requested"""
        if msg.data:
            # Reset internal odometry
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            self.last_left_encoder = 0
            self.last_right_encoder = 0
            
            # Send reset command to ESP32
            command = "<R>"
            self.send_command(command)
            
            self.get_logger().info("Odometry reset")
    
    def send_command(self, command):
        """Send command to ESP32 over serial"""
        try:
            self.serial_conn.write(command.encode())
            self.get_logger().debug(f"Sent command: {command}")
        except Exception as e:
            self.get_logger().error(f"Failed to send command: {e}")
    
    def shutdown(self):
        """Cleanup when node is shutting down"""
        # Stop the robot
        self.send_command("<M,0,0>")
        
        # Close serial connection
        if hasattr(self, 'serial_conn') and self.serial_conn.is_open:
            self.serial_conn.close()
            self.get_logger().info("Serial connection closed")


def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create node
    node = ESP32RobotSerialBridge()
    
    try:
        # Spin the node to process callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()