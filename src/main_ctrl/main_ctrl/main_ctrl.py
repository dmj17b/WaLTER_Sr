import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from interfaces.msg import WheelCommands
from sensor_msgs.msg import Joy

'''This is the main control loop for the robot. Here is where we will subscribe to joystick commands, process outputs, and then publish wheel/leg commands.'''

# Create a node for the main control loop:
class MainControlLoop(Node):
    def __init__(self):
        super().__init__('main_ctrl')
        qos_profile=rclpy.qos.QoSProfile(depth=10)

        # Wheel control variables:
        self.max_wheel_vel = 8.0
        self.wheel_commands = WheelCommands()

        # Create joystick subscriber:
        self.joystick_subscriber = self.create_subscription(msg_type = Joy, topic = 'joy', callback=self.joy_callback, qos_profile=qos_profile)
        self.publisher_ = self.create_publisher(WheelCommands, 'wheel_commands', qos_profile)
        self.timer = self.create_timer(0.01, self.publish_wheel_commands)
    
    def joy_callback(self, msg):
        # Here we would process the joystick command and set wheel speeds accordingly.
        left_stick_ud = msg.axes[0]  # Left stick left/right
        left_stick_lr = msg.axes[1]  # Left stick up/down

        # Map joystick inputs to differential wheel speeds:
        left_wheel_speed = self.max_wheel_vel * left_stick_ud + self.max_wheel_vel * left_stick_lr
        right_wheel_speed = self.max_wheel_vel * left_stick_ud - self.max_wheel_vel * left_stick_lr

        self.wheel_commands.m0_command = "setVel"
        self.wheel_commands.m1_command = "setVel"
        self.wheel_commands.m2_command = "setVel"
        self.wheel_commands.m3_command = "setVel"
        self.wheel_commands.m4_command = "setVel"
        self.wheel_commands.m5_command = "setVel"
        self.wheel_commands.m6_command = "setVel"
        self.wheel_commands.m7_command = "setVel"

        self.wheel_commands.m0_value = left_wheel_speed
        self.wheel_commands.m1_value = left_wheel_speed
        self.wheel_commands.m2_value = right_wheel_speed
        self.wheel_commands.m3_value = right_wheel_speed
        self.wheel_commands.m4_value = left_wheel_speed
        self.wheel_commands.m5_value = right_wheel_speed
        self.wheel_commands.m6_value = left_wheel_speed
        self.wheel_commands.m7_value = right_wheel_speed


    def publish_wheel_commands(self):
        self.publisher_.publish(self.wheel_commands)
        # self.get_logger().info('Publishing: "%s"' % msg)
def main():
    rclpy.init()
    main_ctrl_loop = MainControlLoop()
    rclpy.spin(main_ctrl_loop)
    main_ctrl_loop.destroy_node()
    rclpy.shutdown()


def main_ctrl():
    pass