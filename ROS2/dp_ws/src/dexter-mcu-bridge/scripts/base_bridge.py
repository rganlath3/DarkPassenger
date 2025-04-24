#!/usr/bin/env python3

#Dexter-MCU-Bridge base_bridge.py by Ranil Ganlath. 
#This ROS2 Python Node creates a publisher and subscriber setup. The subscriber reads /MCU/Base/serial_tx topic and transmits it via serial to the Base MCU (ESP32).
#This node also reads serial data in from the Base MCU (ESP32) and publishes it to the /MCU/Base/serial_rx topic.
# Copyright (C) 2025 Ranil Ganlath

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.port = '/dev/ttyACM0'  # change this to the correct port for your Arduino, for example ACM0 or USB0
        self.baudrate = 115200  # change this to the correct baudrate for your Arduino
        self.serial = None
        self.subscription = self.create_subscription(
            String,
            'MCU/Base/serial_tx',
            self.serial_write_callback,
            10)
        self.publisher = self.create_publisher(String, 'MCU/Base/serial_rx', 10)
        self.timer = self.create_timer(0.1, self.serial_read_callback)
        self.connect_to_serial()

    def connect_to_serial(self):
        while not self.serial:
            try:
                self.serial = serial.Serial(self.port, self.baudrate, timeout=0.1)
                self.serial.flushInput()
                self.get_logger().info(f"Connected to Base ESP32 on {self.port} at {self.baudrate} baud")
            except serial.SerialException:
                self.get_logger().warning(f"Failed to connect to Base ESP32 on {self.port} at {self.baudrate} baud. Retrying in 5 seconds...")
                time.sleep(5)

    def serial_write_callback(self, msg):
        #OLD WAY, it would crash if multiple messages were sent at once, ex. "<s,0,1><o,5,5,5,5>"
        #data = msg.data + '\n'  # add newline character to end of message
        #self.serial.write(data.encode('utf-8'))
        for cmd in msg.data.split('<'):
            if cmd != '':
                # Add the "<" character back to the command
                cmd = "<" + cmd + '\n' # add newline character to end of message
                # Encode the command as bytes and send it out via serial write
                self.serial.write(cmd.encode('utf-8'))



    def serial_read_callback(self):
        if self.serial:
            if self.serial.inWaiting() > 0:
                data = self.serial.readline().decode('utf-8').rstrip()
                msg = String()
                msg.data = data
                self.publisher.publish(msg)
        else:
            self.connect_to_serial()

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    rclpy.spin(node)
    if node.serial:
        node.serial.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#To send a single message try: ros2 topic pub -1 MCU/Base/serial_tx std_msgs/msg/String "{data: '<L,1,1>'}"