import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from interfaces.msg import BoomWheelCmds
from sensor_msgs.msg import Joy
from odrive_can.msg import ODriveStatus, ControlMessage, ControllerStatus
from odrive_can.srv import AxisState
import time
import numpy as np

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
        self.dt = 0.1  # Control loop period in seconds (10ms)

        # Wheel control variables:
        self.max_wheel_vel = 20
        self.hip_input_mode = 5
        self.knee_input_mode = 3

        # Mechanism:
        self.knee_gear_ratio = 6.0*30.0/15.0  # Gear ratio for knee motors (6:1 planetary * 30:19 pulley)
        self.hip_gear_ratio = 6.0 # Gear ratio for hip motors (6:1 planetary)

        # ODrive control/status variables:
        self.des_hip_splay = 0.0
        self.max_knee_vel = 0.50
        self.max_hip_angle = 1.0  # radians
        self.min_hip_angle = -1.0  # radians
        self.max_hip_vel = 0.1  # radians per second

        self.hip_pos = 0.0
        self.hip_vel = 0.0
        self.hip_torque = 0.0
        self.knee_pos = 0.0
        self.knee_vel = 0.0
        self.knee_torque = 0.0

        self.knee_des_pos = 0.0

        # Joystick control variables:
        self.right_stick_ud = 0
        self.right_stick_lr = 0
        self.left_stick_ud = 0
        self.left_stick_lr = 0
        self.dpad_ud = 0

        # Initialize ODrive-related objects as None - create them later
        self.axis_state_clients = {}
        self.odrive_publishers = {}
        self.odrive_subscribers = {}
        self.odrive_messages = {}
        self.odrive_timer = None
        self.odrive_initialized = False
        

        # Create joystick subscriber:
        self.joystick_subscriber = self.create_subscription(msg_type = Joy, topic = 'joy', callback=self.joy_callback, qos_profile=qos_profile)
        self.wheel_publisher_ = self.create_publisher(BoomWheelCmds, 'wheel_commands', qos_profile)

        # Timer to publish wheel commands at a regular interval:
        self.wheel_timer = self.create_timer(0.05, self.publish_wheel_commands)
        self.odrive_init_timer = self.create_timer(0.5, callback = self.initialize_odrives)  # Initialize ODrives after 3 seconds
    


    def initialize_odrives(self):
        qos_profile=rclpy.qos.QoSProfile(depth=10)
        if self.odrive_initialized:
            return  # Avoid re-initialization

        
        self.get_logger().info("Initializing ODrive objects...")

        # Create axis state clients for each hip/knee motor:
        self.axis_state_clients = {}
        odrive_nodes = ['hip', 'knee']
        for node_name in odrive_nodes:
            # Add error handling for service creation
            try:
                self.axis_state_clients[node_name] = self.create_client(AxisState, f'/{node_name}/request_axis_state')
                self.get_logger().info(f"Created service client for {node_name}")
            except Exception as e:
                self.get_logger().error(f"Failed to create service client for {node_name}: {e}")


        # ODrive control messages:
        self.hip_msg = ControlMessage(control_mode = 3, input_mode = 5)
        self.knee_msg = ControlMessage(control_mode = 3, input_mode = 5)
        self.wheel_msg = BoomWheelCmds()

        self.get_logger().info("ODrive control messages initialized")



        # Create subscriber for controller status messages:
        subscriber_qos = rclpy.qos.QoSProfile(
            depth=1,  # Smaller queue
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,  # Less strict
            durability=rclpy.qos.DurabilityPolicy.VOLATILE  # Don't store messages
        )
        self.hip_sub = self.create_subscription(
            msg_type=ControllerStatus,
            topic='/hip/controller_status',
            callback=self.fr_hip_callback,
            qos_profile=subscriber_qos
        )
        time.sleep(0.1)  # Small delay to ensure subscribers are ready
        self.knee_sub = self.create_subscription(
            msg_type=ControllerStatus,
            topic='/knee/controller_status',
            callback=self.fr_knee_callback,
            qos_profile=subscriber_qos
        )
        self.get_logger().info("ODrive status subscribers initialized")

        # ODrive control message publishers:
        self.hip_pub = self.create_publisher(ControlMessage, '/hip/control_message', qos_profile)
        self.knee_pub = self.create_publisher(ControlMessage, '/knee/control_message', qos_profile)

        self.get_logger().info("ODrive control message publishers initialized")

        # Create a timer to publish ODrive commands at a regular interval:
        self.odrive_timer = self.create_timer(0.005, self.publish_odrive_commands)
        self.get_logger().info("ODrive command publisher initialized")
        self.odrive_initialized = True




    # Callback function for joystick messages:
    '''-------------- Essentially our main control loop for now--------------'''

    def joy_callback(self, msg):
        # Check if ODrive is initialized before processing joystick commands. 
        # Need to make sure ODrives are actually running before we try to control them.
        if not self.odrive_initialized:
            self.get_logger().warn("ODrive not initialized yet, skipping joystick processing")
            return
        

        # --------------- Safety Mode Toggle ---------------
        # If start button is pressed, toggle safety mode (with debouncing):
        if msg.buttons[7] == 1 and self.prev_start_button == 0:  # Rising edge detection
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
        self.prev_start_button = msg.buttons[7]

        # --------------- Joystick Control (ODRIVE) ---------------
        # IF SAFETY IS OFF, CONTROL:
        if not self.safety_on:

            # Read right joystick for knee control:
            self.right_stick_ud = msg.axes[4]
            self.right_stick_lr = msg.axes[3]

            # Read D-Pad for hip angle:
            self.dpad_ud = msg.axes[7]

            # ---------------- Joystick Control (Wheels) ---------------
            # Here we would process the joystick command and set wheel speeds accordingly.
            self.left_stick_ud = msg.axes[1]  # Left stick left/right
            self.left_stick_lr = msg.axes[0]  # Left stick up/down

            # Map joystick inputs to differential wheel speeds:
            wheel_speed = self.max_wheel_vel * self.left_stick_ud + self.max_wheel_vel * self.left_stick_lr
            wheel_duty = -self.intmap(wheel_speed, -self.max_wheel_vel, self.max_wheel_vel, -100, 100)
            
            self.wheel_msg.duty1 = wheel_duty
            self.wheel_msg.duty2 = wheel_duty
        # IF SAFETY IS ON: 
        else:
            wheel_duty = 0
            self.wheel_msg.duty1 = wheel_duty
            self.wheel_msg.duty2 = wheel_duty

    # Map float to int
    def intmap(self, val, val_min, val_max, int_min, int_max):
        # Clamp val to be within val_min and val_max
        val = max(min(val, val_max), val_min)
        # Map the value
        mapped_val = int((val - val_min) / (val_max - val_min) * (int_max - int_min) + int_min)
        return mapped_val
    # Nearest Pi function to calculate closest multiple of pi:
    def nearest_pi_knee(self, angle):
        value = 0.5*6*30/15
        near_pi = np.round(angle/value) * value
        return near_pi

    # Publish the wheel commands to the wheel command topic:
    def publish_wheel_commands(self):
        if not self.odrive_initialized:
            return
        self.wheel_publisher_.publish(self.wheel_msg)


    # Callbacks for ODrive status messages:
    # These will update the ACTUAL position and velocity of the hip/knee motors:
    def fr_hip_callback(self, msg):
        self.hip_pos = msg.pos_estimate
        self.hip_vel = msg.vel_estimate
        self.hip_torque = msg.torque_estimate

    def fr_knee_callback(self, msg):
        self.knee_pos = msg.pos_estimate
        self.knee_vel = msg.vel_estimate
        self.knee_torque = msg.torque_estimate

    def publish_odrive_commands(self):
        # Map joystick inputs to knee velocities:
        right_knee_vel = self.max_knee_vel * self.right_stick_ud + self.max_knee_vel * self.right_stick_lr
        left_knee_vel = self.max_knee_vel * self.right_stick_ud - self.max_knee_vel * self.right_stick_lr
        
        # Ensure knee velocities are within limits:
        left_knee_vel = max(min(left_knee_vel, self.max_knee_vel), -self.max_knee_vel)
        right_knee_vel = max(min(right_knee_vel, self.max_knee_vel), -self.max_knee_vel)

        # If desired knee position is zero, set desired knee position to current knee position to avoid looping back to zero on startup
        if self.knee_des_pos == 0.0:
            self.knee_des_pos = self.knee_pos

        # Increment desired knee position by joystick knee velocity
        self.knee_des_pos = self.knee_des_pos + right_knee_vel * self.dt  # Assuming 10ms control loop


        # Map dpad inputs to hip velocities:
        self.des_hip_splay = self.des_hip_splay + self.dpad_ud * self.max_hip_vel * self.dt  # Adjust splay angle based on dpad input
        self.des_hip_splay = max(min(self.des_hip_splay, self.max_hip_angle), self.min_hip_angle)  # Clamp splay angle

        # ---------------- ODrive Control Messages ----------------

        # Construct ODrive control messages:
        # Front Right Knee:
        self.knee_msg.control_mode = 3
        self.knee_msg.input_mode = self.knee_input_mode
        self.knee_msg.input_pos = self.knee_des_pos  # Not used in velocity
        self.knee_msg.input_vel = right_knee_vel
        self.knee_msg.input_torque = 0.0  # Not used in velocity control
    
        # Front Right Hip:
        self.hip_msg.control_mode = 3
        self.hip_msg.input_mode = self.hip_input_mode
        self.hip_msg.input_pos = -self.des_hip_splay  # Desired hip splay position
        self.hip_msg.input_vel = 0.0  # Not used
        self.hip_msg.input_torque = 0.0  # Not used in velocity control
        self.knee_pub.publish(self.knee_msg)
        self.hip_pub.publish(self.hip_msg)



    # Function to set ODrive axis state:
    def set_odrive_axis_state(self, node_name, state):
        if node_name not in self.axis_state_clients:
            self.get_logger().warn(f"No client for node: {node_name}")
            return
            
        client = self.axis_state_clients[node_name]
        if not client.service_is_ready():
            self.get_logger().debug(f"Service not ready for {node_name} - skipping")
            return
        
        # Add timeout and error handling
        try:
            request = AxisState.Request()
            request.axis_requested_state = state
            
            future = client.call_async(request)
            # Don't add callback if system is under stress
            if hasattr(self, '_service_calls_in_progress'):
                if self._service_calls_in_progress > 5:  # Limit concurrent calls
                    self.get_logger().warn(f"Too many service calls in progress, skipping {node_name}")
                    return
            else:
                self._service_calls_in_progress = 0
                
            self._service_calls_in_progress += 1
            future.add_done_callback(lambda f: self.service_response_callback(f, node_name))
            
        except Exception as e:
            self.get_logger().error(f"Failed to call service for {node_name}: {e}")

    def service_response_callback(self, future, node_name):
        try:
            response = future.result()
            self.get_logger().debug(f"Service call to {node_name} completed")  # Change to debug
        except Exception as e:
            self.get_logger().error(f"Service call to {node_name} failed: {e}")
        finally:
            if hasattr(self, '_service_calls_in_progress'):
                self._service_calls_in_progress -= 1

    def destroy_node(self):
        # Clean up any resources before shutting down
        self.get_logger().info("Destroying main control loop node")
        super().destroy_node()
        if self.odrive_timer:
            self.odrive_timer.cancel()
        if self.wheel_timer:
            self.wheel_timer.cancel()
        for client in self.axis_state_clients.values():
            client.destroy()


def main():
    rclpy.init()
    main_ctrl_loop = MainControlLoop()
    rclpy.spin(main_ctrl_loop)
    main_ctrl_loop.destroy_node()
    rclpy.shutdown()


def main_ctrl():
    pass