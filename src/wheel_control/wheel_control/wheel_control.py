import rclpy
import rclpy.logging
import serial
import serial.tools.list_ports
from rclpy.node import Node
from interfaces.msg import WheelCommands
import time

# Node Class:
class WheelControlNode(Node):
    # Init
    def __init__(self):
        super().__init__('wheel_control_node')
        self.get_logger().info('Wheel Control Node has been started.')

        # Initialize serial communication with Teensy:
        port = self.find_teensy_port(vid=0x16C0, pid=0x0483)  # VID and PID
        self.ser = serial.Serial(port, baudrate=115200, timeout=1)
        time.sleep(2)

        # Subscriber for wheel commands
        self.wheel_command_sub = self.create_subscription(
            WheelCommands,
            'wheel_commands',
            self.wheel_commands_callback,
            10
        )
        self.wheel_command_sub  # Prevent unused variable warning

    def wheel_commands_callback(self, msg):
        self.send_serial_command(0, msg.m0_command, msg.m0_value)
        self.send_serial_command(1, msg.m1_command, msg.m1_value)
        self.send_serial_command(2, msg.m2_command, msg.m2_value)
        self.send_serial_command(3, msg.m3_command, msg.m3_value)
        self.send_serial_command(4, msg.m4_command, msg.m4_value)
        self.send_serial_command(5, msg.m5_command, msg.m5_value)
        self.send_serial_command(6, msg.m6_command, msg.m6_value)
        self.send_serial_command(7, msg.m7_command, msg.m7_value)

    def send_serial_command(self, motorId, command, value):
        """Send a command to the Teensy via serial."""
        ser_msg = f"{motorId},{command},{value}\n"
        self.ser.write(ser_msg.encode('ascii'))

    # Helper function to find Teensy serial port:
    def find_teensy_port(self, vid, pid):
        """Find the serial port for the Teensy device."""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.vid == vid and port.pid == pid:
                self.get_logger().info(f'Found Teensy on port: {port.device}')
                return port.device
            else:
                self.get_logger().warn(f'Could not find Teensy.')
        self.get_logger().info('WheelComs node has been started.')



def main():
    rclpy.init()    # Initialize the ROS 2 Python client library
    wheel_control_node = WheelControlNode() # Create an instance of the WheelControlNode
    rclpy.spin(wheel_control_node)  # Spin the node to keep it active and processing callbacks
    wheel_control_node.destroy_node()   # Clean up the node when done
    rclpy.shutdown()    # Shutdown the ROS 2 Python client library

if __name__ == '__main__':
    main()