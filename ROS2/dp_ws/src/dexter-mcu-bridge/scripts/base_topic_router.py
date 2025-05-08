#!/usr/bin/env python3

#Dexter-MCU-Bridge base_rx_topic_router.py by Ranil Ganlath. 
#This ROS2 Python Node creates a publisher and subscriber setup. The subscriber reads /MCU/Base/serial_rx topic published by the Base MCU (ESP32). 
# It then unpacks the message and routes it to a topic depending on the type of response. 
#Ex. If the MCU sends out encoder info, this node will forward the message to the /encoder topic.
# Copyright (C) 2025 Ranil Ganlath

import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String, Bool
from sensor_msgs.msg import Imu, MagneticField, NavSatFix, BatteryState


class SerialSubscriber(Node):

    def __init__(self):
        super().__init__('base_serial_topic_router')

        # Declare parameters
        self.declare_parameter('base_frame_id', 'base_link')

        # Get parameters
        self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value

        # Initialize subscribers
        self.subscription = self.create_subscription(String, 'MCU/Base/serial_rx', self.callback, 10)

        # Initialize publishers
        self.odometry_publisher = self.create_publisher(String, 'MCU/Base/odometry', 10)
        self.battery_publisher = self.create_publisher(BatteryState, 'MCU/Base/battery', 10)
        self.IMU_publisher = self.create_publisher(Imu, 'MCU/Base/imu/data_raw', 10)
        self.MagneticField_publisher = self.create_publisher(MagneticField, 'MCU/Base/imu/mag', 10)
        self.GPS_publisher = self.create_publisher(NavSatFix, 'MCU/Base/gps/fix', 10)
        self.status_publisher = self.create_publisher(String, 'MCU/Base/status', 10)


        self.encoder_publisher = self.create_publisher(String,'MCU/Base/encoder_info', 10)
        self.motor_target_publisher = self.create_publisher(String, 'MCU/Base/motor_target_info', 10)
        self.motor_velocity_publisher = self.create_publisher(String, 'MCU/Base/motor_velocity_info', 10)
        # Add more publishers for other commands as needed

    def callback(self, msg):
        for indcmd in msg.data.split('<'): 
            if indcmd != '':
                # Remove the trailing ">" character from the command
                indcmd = indcmd[:-1]
                #At this point the message went from "<s,0,1>" to s,0,1 so we need to split it up.
                message_parts = indcmd.split(',')
                cmd = message_parts[0] 

                switcher = {
                    'K': self.handle_connected,
                    'B': self.handle_battery,
                    'I': self.handle_IMU,
                    'G': self.handle_GPS,
                    'S': self.handle_status,       
                    'O': self.handle_odometry            
                    # Add more cases for other commands as needed
                }


                # Get the function to handle the command from the switcher dictionary
                handler = switcher.get(cmd, self.handle_unknown_command)

                # Call the handler function with the message and message_parts arguments
                handler(msg, message_parts)

    def handle_connected(self, msg, message_parts):
        #This function is called the first time the robot makes a serial connection (runs in setup function of MCU code)
        new_msg = String()
        new_msg.data = "ESP32 Base MCU Connected to Topic Router"
        self.status_publisher.publish(new_msg)

    def handle_battery(self, msg, message_parts):
        #This repackages raw battery info into a proper BatteryState format topic.
        voltage_mv = float(message_parts[1])
        percentage = float(message_parts[2])

        # Publish battery state
        battery_msg = BatteryState()
        battery_msg.header.stamp = self.get_clock().now().to_msg()
        battery_msg.voltage = voltage_mv / 1000.0  # Convert mV to V
        battery_msg.percentage = percentage
        battery_msg.present = True
        battery_msg.power_supply_status = 2 #Discharging
        battery_msg.power_supply_technology = 2 #DeWalt Battery with 18650 Lithium Ion Cells
        
        self.battery_publisher.publish(battery_msg)

    def handle_IMU(self, msg, message_parts):
        #This repackages raw IMU and magnetic field info from the MPU9250 into a proper Imu and Magnetic Field format topics
        aX = float(message_parts[1])
        aY = float(message_parts[2])
        aZ = float(message_parts[3])
        gX = float(message_parts[4])
        gY = float(message_parts[5])
        gZ = float(message_parts[6])
        mX = float(message_parts[7])
        mY = float(message_parts[8])
        mZ = float(message_parts[9])
        mDirection = float(message_parts[10])  # heading in degrees

        # Publish IMU data
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.base_frame_id
        
        # Set linear acceleration (convert to m/s^2 if needed)
        imu_msg.linear_acceleration.x = aX
        imu_msg.linear_acceleration.y = aY
        imu_msg.linear_acceleration.z = aZ
        
        # Set angular velocity (convert to rad/s if needed)
        imu_msg.angular_velocity.x = gX
        imu_msg.angular_velocity.y = gY
        imu_msg.angular_velocity.z = gZ
        
        # Note: For now we're not setting orientation as it requires quaternion conversion
        
        self.IMU_publisher.publish(imu_msg)

        # Publish mag data
        mag_msg = MagneticField()
        mag_msg.header.stamp = self.get_clock().now().to_msg()
        mag_msg.header.frame_id = self.base_frame_id
        
        #Set magnetic field (convert from MicroTeslas to Tesla if needed)
        mag_msg.magnetic_field.x = mX
        mag_msg.magnetic_field.y = mY
        mag_msg.magnetic_field.z = mZ
        
        self.MagneticField_publisher.publish(mag_msg)


    def handle_GPS(self, msg, message_parts):
        #This repackages raw GPS info into a proper GPS NavSatFix format topic.
        lat = float(message_parts[1])
        lng = float(message_parts[2])
        satellites = float(message_parts[3])
        valid = float(message_parts[4])

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
        self.get_logger().info("Number of Satellites: "+str(satellites)) #FOR DEBUGGING
        self.GPS_publisher.publish(gps_msg)

    def handle_status(self, msg, message_parts):
        #This repackages raw status info into a proper JSON format status format topic.
        toggle_sw1_up = int(message_parts[1])
        toggle_sw2_up = int(message_parts[2])
        left_wheel_grounded = int(message_parts[3])
        right_wheel_grounded = int(message_parts[4])
        headlight_on = int(message_parts[5])
        brakelight_on = int(message_parts[6])

        status_str = f'{{"switches":{{"toggle_sw1_up":{toggle_sw1_up},"toggle_sw2_up":{toggle_sw2_up},"left_wheel_grounded":{left_wheel_grounded},"right_wheel_grounded":{right_wheel_grounded}}}, "lights":{{"headlight_on":{headlight_on},"brakelight_on":{brakelight_on}}}}}'
        status_msg = String()
        status_msg.data = status_str
        
        self.status_publisher.publish(status_msg)


    def handle_odometry(self, msg, message_parts):
        #TO DO
        #For now this function will just echo odometry data as a string. Later on, I need to repackage info to interface with the correct ROS2 topic message formats for other plugins.
        variableA = float(message_parts[1])
        variableB = float(message_parts[2])
        variableC = float(message_parts[3])
        variableD = float(message_parts[4])
        new_msg = String()
        new_msg.data = "Left Encoder Count: " + str(variableA) + " Right Encoder Count: " + str(variableB) + " Left Distance MM: " + str(variableC) + " Right Distance MM: " + str(variableD)
        self.odometry_publisher.publish(new_msg)

    def handle_unknown_command(self, msg, message_parts):
        self.get_logger().warn(f"Unknown command: {message_parts[0]}")
        new_msg = String()
        new_msg.data = "WARN: Received unknown command type: " + str(message_parts[0])
        self.status_publisher.publish(new_msg)



def main(args=None):
    rclpy.init(args=args)

    serial_subscriber = SerialSubscriber()

    rclpy.spin(serial_subscriber)

    serial_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()