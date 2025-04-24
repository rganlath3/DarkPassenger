#!/usr/bin/env python3

#Dexter-MCU-Bridge base_rx_topic_router.py by Ranil Ganlath. 
#This ROS2 Python Node creates a publisher and subscriber setup. The subscriber reads /MCU/Base/serial_rx topic published by the Base MCU (ESP32). 
# It then unpacks the message and routes it to a topic depending on the type of response. 
#Ex. If the MCU sends out encoder info, this node will forward the message to the /encoder topic.
# Copyright (C) 2025 Ranil Ganlath


import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32


class SerialSubscriber(Node):

    def __init__(self):
        super().__init__('serial_subscriber')
        self.subscription = self.create_subscription(String, 'MCU/Base/serial_rx', self.callback, 10)


        self.odometry_publisher = self.create_publisher(String, 'MCU/Base/odometry', 10)
        self.battery_publisher = self.create_publisher(String, 'MCU/Base/battery', 10)
        self.IMU_publisher = self.create_publisher(String, 'MCU/Base/IMU', 10)
        self.GPS_publisher = self.create_publisher(String, 'MCU/Base/GPS', 10)
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


    def handle_battery(self, msg, message_parts):
        #For now this function will just echo battery data as a string. Later on, I need to repackage info to interface with the correct ROS2 topic message formats for other plugins.
        variableA = float(message_parts[1])
        variableB = float(message_parts[2])
        new_msg = String()
        new_msg.data = "Battery Voltage: " + str(variableA) + " Battery Percentage: " + str(variableB)
        self.battery_publisher.publish(new_msg)

    def handle_IMU(self, msg, message_parts):
        #For now this function will just echo odometry data as a string. Later on, I need to repackage info to interface with the correct ROS2 topic message formats for other plugins.
        aX = float(message_parts[1])
        aY = float(message_parts[2])
        aZ = float(message_parts[3])
        gX = float(message_parts[4])
        gY = float(message_parts[5])
        gZ = float(message_parts[6])
        mX = float(message_parts[7])
        mY = float(message_parts[8])
        mZ = float(message_parts[9])
        mDirection = float(message_parts[10])
        new_msg = String()
        new_msg.data = "aX: " + str(aX) + " aY " + str(aY) + " aZ " + str(aZ) + " gX " + str(gX) + " gY " + str(gY) + " gZ " + str(gZ) + " mX " + str(mX) + " mY " + str(mY) + " mZ " + str(mZ)+ " mDirection " + str(mDirection)
        self.IMU_publisher.publish(new_msg)

    def handle_GPS(self, msg, message_parts):
        #For now this function will just echo odometry data as a string. Later on, I need to repackage info to interface with the correct ROS2 topic message formats for other plugins.
        variableA = float(message_parts[1])
        variableB = float(message_parts[2])
        variableC = float(message_parts[3])
        variableD = float(message_parts[4])
        new_msg = String()
        new_msg.data = "Lat: " + str(variableA) + " Long: " + str(variableB) + " Satellites Connected: " + str(variableC) + " Valid Data: " + str(variableD)
        self.GPS_publisher.publish(new_msg)

    def handle_status(self, msg, message_parts):
        #For now this function will just echo odometry data as a string. Later on, I need to repackage info to interface with the correct ROS2 topic message formats for other plugins.
        variableA = float(message_parts[1])
        variableB = float(message_parts[2])
        variableC = float(message_parts[3])
        variableD = float(message_parts[4])
        variableE = float(message_parts[5])
        variableF = float(message_parts[6])
        new_msg = String()
        new_msg.data = "Toggle Switch 1: " + str(variableA) + "Toggle Switch 2: " + str(variableB) + " Left Wheel Grounded: " + str(variableC) + " Right Wheel Grounded: " + str(variableD) + " Head Light On: " + str(variableE) + " Brake Light On: " + str(variableF)
        self.status_publisher.publish(new_msg)

    def handle_odometry(self, msg, message_parts):
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

    ##################OLD STUFF BELOW###################

def main(args=None):
    rclpy.init(args=args)

    serial_subscriber = SerialSubscriber()

    rclpy.spin(serial_subscriber)

    serial_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()