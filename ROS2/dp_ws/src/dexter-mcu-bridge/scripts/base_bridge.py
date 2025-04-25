#!/usr/bin/env python3

# Dexter-MCU-Bridge base_bridge.py by Ranil Ganlath.
# This ROS2 Python Node creates a publisher and subscriber setup. The subscriber reads /MCU/Base/serial_tx topic and transmits it via serial to the Base MCU (ESP32).
# This node also reads serial data in from the Base MCU (ESP32) and publishes it to the /MCU/Base/serial_rx topic.
# Copyright (C) 2025 Ranil Ganlath

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
import serial
import time
import threading
from queue import Queue
import re

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        
        # Declare parameters with default values
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('reconnect_delay', 5.0)
        self.declare_parameter('read_interval', 0.01)  # 10ms for more responsive reads
        
        # Get parameters
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.timeout = self.get_parameter('timeout').value
        self.reconnect_delay = self.get_parameter('reconnect_delay').value
        self.read_interval = self.get_parameter('read_interval').value
        
        # Serial connection
        self.serial = None
        self.is_connected = False
        self.connection_lock = threading.Lock()
        
        # Message buffers
        self.incoming_buffer = ""
        self.outgoing_queue = Queue()
        
        # Statistics
        self.rx_count = 0
        self.tx_count = 0
        self.error_count = 0
        
        # Create publishers and subscribers
        self.subscription = self.create_subscription(
            String,
            'MCU/Base/serial_tx',
            self.serial_write_callback,
            10)
        self.publisher = self.create_publisher(String, 'MCU/Base/serial_rx', 10)
        self.diag_publisher = self.create_publisher(DiagnosticStatus, 'diagnostics/serial_node', 10)
        
        # Create timers
        self.read_timer = self.create_timer(self.read_interval, self.serial_read_callback)
        self.diag_timer = self.create_timer(1.0, self.publish_diagnostics)
        
        # Start connection
        self.connect_to_serial()

    def connect_to_serial(self):
        """Attempt to connect to the serial port"""
        with self.connection_lock:
            if self.serial:
                try:
                    self.serial.close()
                except Exception as e:
                    self.get_logger().error(f"Error closing existing serial connection: {str(e)}")
                self.serial = None
                
            try:
                self.serial = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=self.timeout)
                self.serial.reset_input_buffer()
                self.serial.reset_output_buffer()
                self.is_connected = True
                self.get_logger().info(f"Connected to Base ESP32 on {self.port} at {self.baudrate} baud")
                return True
            except serial.SerialException as e:
                self.is_connected = False
                self.error_count += 1
                self.get_logger().warning(f"Failed to connect to Base ESP32 on {self.port}: {str(e)}")
                return False

    def serial_write_callback(self, msg):
        """Handle incoming messages to send to Arduino"""
        try:
            # Parse and queue commands
            for cmd in re.findall(r'<[^>]+>', msg.data):
                self.outgoing_queue.put(cmd)
                
            # Process the queue
            self._process_outgoing_queue()
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f"Error in serial write callback: {str(e)}")

    def _process_outgoing_queue(self):
        """Process the outgoing message queue"""
        if not self.is_connected:
            if not self.connect_to_serial():
                return
                
        while not self.outgoing_queue.empty():
            try:
                cmd = self.outgoing_queue.get()
                
                # Validate command format before sending
                if not (cmd.startswith('<') and cmd.endswith('>')):
                    self.get_logger().warning(f"Invalid command format: {cmd}")
                    continue
                    
                # Add newline and send
                with self.connection_lock:
                    if self.serial and self.is_connected:
                        self.serial.write((cmd + '\n').encode('utf-8'))
                        self.tx_count += 1
                        self.get_logger().debug(f"Sent: {cmd}")
                    else:
                        # Re-queue the command if connection was lost
                        self.outgoing_queue.put(cmd)
                        break
            except serial.SerialException as e:
                self.get_logger().error(f"Serial write error: {str(e)}")
                self.is_connected = False
                self.error_count += 1
                # Re-queue the command
                self.outgoing_queue.put(cmd)
                break
            except Exception as e:
                self.get_logger().error(f"Error sending command: {str(e)}")
                self.error_count += 1

    def serial_read_callback(self):
        """Read data from serial port and publish complete messages"""
        if not self.is_connected:
            if not self.connect_to_serial():
                return
                
        try:
            with self.connection_lock:
                if self.serial and self.is_connected and self.serial.in_waiting > 0:
                    # Read available data
                    data = self.serial.readline()
                    
                    try:
                        # Try to decode the data
                        decoded_data = data.decode('utf-8').rstrip()
                        
                        # Add to buffer and check for complete messages
                        self.incoming_buffer += decoded_data
                        
                        # Extract complete messages
                        messages = self._extract_complete_messages()
                        
                        # Publish complete messages
                        for message in messages:
                            msg = String()
                            msg.data = message
                            self.publisher.publish(msg)
                            self.rx_count += 1
                            self.get_logger().debug(f"Published: {message}")
                    
                    except UnicodeDecodeError:
                        self.get_logger().warning("Failed to decode serial data")
                        self.error_count += 1
        
        except serial.SerialException as e:
            self.get_logger().error(f"Serial read error: {str(e)}")
            self.is_connected = False
            self.error_count += 1
            # Schedule reconnection
            self.create_timer(self.reconnect_delay, self.connect_to_serial, oneshot=True)
        
        except Exception as e:
            self.get_logger().error(f"Error reading from serial: {str(e)}")
            self.error_count += 1

    def _extract_complete_messages(self):
        """Extract complete messages from the buffer"""
        messages = []
        
        # Find complete commands in the format <cmd>
        pattern = r'<[^>]*>'
        matches = re.finditer(pattern, self.incoming_buffer)
        
        last_end = 0
        for match in matches:
            messages.append(match.group(0))
            last_end = match.end()
        
        # Keep any partial message for the next iteration
        if last_end > 0:
            self.incoming_buffer = self.incoming_buffer[last_end:]
        
        return messages

    def publish_diagnostics(self):
        """Publish diagnostic information"""
        status = DiagnosticStatus()
        status.name = "Serial Communication Bridge"
        
        if self.is_connected:
            status.level = DiagnosticStatus.OK
            status.message = "Connected"
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = "Disconnected"
        
        # Add values
        status.values = [
            KeyValue(key="port", value=self.port),
            KeyValue(key="baudrate", value=str(self.baudrate)),
            KeyValue(key="connected", value=str(self.is_connected)),
            KeyValue(key="rx_count", value=str(self.rx_count)),
            KeyValue(key="tx_count", value=str(self.tx_count)),
            KeyValue(key="error_count", value=str(self.error_count)),
            KeyValue(key="queue_size", value=str(self.outgoing_queue.qsize())),
        ]
        
        self.diag_publisher.publish(status)

    def on_shutdown(self):
        """Clean up resources on shutdown"""
        self.get_logger().info("Shutting down serial node...")
        with self.connection_lock:
            if self.serial:
                try:
                    self.serial.close()
                    self.get_logger().info("Serial port closed")
                except Exception as e:
                    self.get_logger().error(f"Error closing serial port: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {str(e)}")
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# Example usage:
# ros2 topic pub -1 MCU/Base/serial_tx std_msgs/msg/String "{data: '<L,1,1>'}"
# ros2 topic echo MCU/Base/serial_rx
# ros2 service call /serial_node/set_parameters rcl_interfaces/srv/SetParameters "{parameters: [{name: 'port', value: {type: 4, string_value: '/dev/ttyUSB0'}}]}"