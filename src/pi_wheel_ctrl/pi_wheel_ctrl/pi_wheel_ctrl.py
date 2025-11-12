import rclpy
import rclpy.logging

from . import Motor
from rclpy.node import Node
from std_msgs.msg import Float32
from interfaces.msg import BoomWheelCmds

# Define GPIO pins for motor control
M1_P_PIN = 27  # PWM pin for Motor 1
M1_A_PIN = 26  # Direction pin A for Motor 1
M1_B_PIN = 17  # Direction pin B for Motor 1

M2_P_PIN = 23  # PWM pin for Motor 2
M2_A_PIN = 24  # Direction pin A for Motor 2
M2_B_PIN = 22  # Direction pin B for Motor 2

class PIWheelCtrl(Node):
    def __init__(self):
        super().__init__('pi_wheel_node')
        qos_profile = rclpy.qos.QoSProfile(depth=10)
        
        self.wheel_command_sub = self.create_subscription(
            msg_type = BoomWheelCmds,
            topic = 'wheel_commands',
            callback = self.wheel_commands_callback,
            qos_profile = qos_profile
        )
        # Initialize brushed wheel motor objects
        self.wheel1 = Motor.BrushedMotor(M1_P_PIN, M1_A_PIN, M1_B_PIN)
        self.wheel2 = Motor.BrushedMotor(M2_P_PIN, M2_A_PIN, M2_B_PIN)

    def wheel_commands_callback(self, msg):
        # self.get_logger().info(f"Wheel duty cycle command: {msg.duty1}")
        self.wheel1.drive(msg.duty1)
        self.wheel2.drive(msg.duty2)

def main():
    rclpy.init()
    wheel_ctrl_node = PIWheelCtrl()
    rclpy.spin(wheel_ctrl_node)
    wheel_ctrl_node.destroy_node()
    rclpy.shutdown()
