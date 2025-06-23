import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from interfaces.msg import WheelCommands
from sensor_msgs.msg import Joy
from odrive_can.msg import ODriveStatus, ControlMessage, ControllerStatus
from odrive_can.srv import AxisState

'''This is the main control loop for the robot. Here is where we will subscribe to joystick commands, process outputs, and then publish wheel/leg commands.'''

# Create a node for the main control loop:
class MainControlLoop(Node):
    def __init__(self):
        # Initialize the node with a name:
        super().__init__('main_ctrl_node')
        qos_profile=rclpy.qos.QoSProfile(depth=10)

        # General control variables:
        self.safety_on = True
        self.prev_start_button = 0  # Track previous button state for debouncing

        # Wheel control variables:
        self.max_wheel_vel = 16.0
        self.wheel_commands = WheelCommands()

        # ODrive control/status variables:
        self.fr_hip_pos = 0.0
        self.fr_hip_vel = 0.0
        self.fr_knee_pos = 0.0
        self.fr_knee_vel = 0.0
        self.max_knee_vel = 5.0

        # Create axis state clients for each hip/knee motor:
        self.axis_state_clients = {}
        odrive_nodes = ['fr_hip', 'fr_knee']#, 'fl_hip', 'fl_knee', 'rr_hip', 'rr_knee', 'rl_hip', 'rl_knee']
        for node_name in odrive_nodes:
            self.axis_state_clients[node_name] = self.create_client(AxisState, f'/{node_name}/request_axis_state')

        # ODrive control messages:
        self.fr_hip_msg = ControlMessage()
        self.fr_knee_msg = ControlMessage()

        # # ODrive control message publishers:
        # self.fr_hip_pub = self.create_publisher(ControlMessage, '/fr_hip/control', qos_profile)
        self.fr_knee_pub = self.create_publisher(ControlMessage, '/fr_knee/control_message', qos_profile)

        # Create subscriber for controller status messages:
        self.fr_hip_sub = self.create_subscription(
            msg_type=ControllerStatus,
            topic='/fr_hip/status',
            callback=self.fr_hip_callback,
            qos_profile=qos_profile
        )
        self.fr_knee_sub = self.create_subscription(
            msg_type=ControllerStatus,
            topic='/fr_knee/status',
            callback=self.fr_knee_callback,
            qos_profile=qos_profile
        )
        # Create joystick subscriber:
        self.joystick_subscriber = self.create_subscription(msg_type = Joy, topic = 'joy', callback=self.joy_callback, qos_profile=qos_profile)
        self.wheel_publisher_ = self.create_publisher(WheelCommands, 'wheel_commands', qos_profile)

        # Timer to publish wheel commands at a regular interval:
        self.wheel_timer = self.create_timer(0.01, self.publish_wheel_commands)
    

    # Callbacks for ODrive status messages:
    # These will update the ACTUAL position and velocity of the hip/knee motors:
    def fr_hip_callback(self, msg):
        self.fr_hip_pos = msg.position
        self.fr_hip_vel = msg.velocity
    
    def fr_knee_callback(self, msg):
        self.fr_knee_pos = msg.position
        self.fr_knee_vel = msg.velocity


    # Function to set ODrive axis state:
    def set_odrive_axis_state(self, node_name, state):
        if node_name not in self.axis_state_clients:
            self.get_logger().error(f"No client for node: {node_name}")
            return
            
        client = self.axis_state_clients[node_name]
        if not client.service_is_ready():
            self.get_logger().warn(f"Service not ready for {node_name}")
            return
            
        request = AxisState.Request()
        request.axis_requested_state = state
        
        future = client.call_async(request)
        future.add_done_callback(lambda f: self.service_response_callback(f, node_name))

    # Callback function for service responses:
    def service_response_callback(self, future, node_name):
        try:
            response = future.result()
            self.get_logger().info(f"Service call to {node_name} completed")
        except Exception as e:
            self.get_logger().error(f"Service call to {node_name} failed: {e}")
    
    # Callback function for joystick messages:
    def joy_callback(self, msg):
        # If start button is pressed, toggle safety mode (with debouncing):
        if msg.buttons[9] == 1 and self.prev_start_button == 0:  # Rising edge detection
            self.safety_on = not self.safety_on
            self.get_logger().info(f"Safety mode {'enabled' if self.safety_on else 'disabled'}")
            if(self.safety_on):
                # Set all ODrive axes to idle mode:
                for node_name in self.axis_state_clients.keys():
                    self.set_odrive_axis_state(node_name, 1)
            else:
                # Set all ODrive axes to closed loop control mode:
                for node_name in self.axis_state_clients.keys():
                    self.set_odrive_axis_state(node_name, 8)
        
        # Update previous button state
        self.prev_start_button = msg.buttons[9]

        right_stick_ud = msg.axes[2]
        right_stick_lr = msg.axes[3]

        # Map joystick inputs to knee velocities:
        knee_vel = right_stick_ud * self.max_knee_vel
        self.fr_knee_pub.publish(self.fr_knee_msg)

        fr_knee_des_vel = knee_vel
        self.fr_knee_msg.control_mode = 2
        self.fr_knee_msg.input_mode = 1
        self.fr_knee_msg.input_pos = 0.0  # Not used in velocity
        self.fr_knee_msg.input_vel = fr_knee_des_vel
        self.fr_knee_msg.input_torque = 0.0  # Not used in velocity control


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
        self.wheel_commands.m5_value = left_wheel_speed
        self.wheel_commands.m6_value = right_wheel_speed
        self.wheel_commands.m7_value = right_wheel_speed



    def publish_wheel_commands(self):
        self.wheel_publisher_.publish(self.wheel_commands)
        # self.get_logger().info('Publishing: "%s"' % msg)

def main():
    rclpy.init()
    main_ctrl_loop = MainControlLoop()
    rclpy.spin(main_ctrl_loop)
    main_ctrl_loop.destroy_node()
    rclpy.shutdown()


def main_ctrl():
    pass