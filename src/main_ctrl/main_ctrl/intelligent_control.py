import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from interfaces.msg import WheelCommands
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
        super().__init__('intelligent_ctrl_node')
        qos_profile=rclpy.qos.QoSProfile(depth=10)

        # General control variables:
        self.safety_on = True
        self.prev_start_button = 0  # Track previous button state for debouncing
        self.dt = 0.1  # Control loop period in seconds (10ms)

        # Wheel control variables:
        self.max_wheel_vel = 20
        self.wheel_commands = WheelCommands()
        self.hip_input_mode = 5
        self.knee_input_mode = 1

        # Mechanism:
        self.knee_gear_ratio = 6.0*30.0/15.0  # Gear ratio for knee motors (6:1 planetary * 30:19 pulley)
        self.hip_gear_ratio = 6.0 # Gear ratio for hip motors (6:1 planetary)

        # ODrive control/status variables:
        self.des_hip_splay = 0.0
        self.max_knee_vel = 8.0

        #____________________________________________
        #____________________________________________
        # torque limit
        self.max_knee_torque = 0.7 # Nm
        #____________________________________________
        #____________________________________________

        self.max_hip_angle = 1.0  # radians
        self.min_hip_angle = -0.5  # radians
        self.max_hip_vel = 0.5  # radians per second

        self.fr_hip_pos = 0.0
        self.fr_hip_vel = 0.0
        self.fr_hip_torque = 0.0
        self.fr_knee_pos = 0.0
        self.fr_knee_vel = 0.0
        self.fr_knee_torque = 0.0
        self.fl_hip_pos = 0.0
        self.fl_hip_vel = 0.0
        self.fl_hip_torque = 0.0
        self.fl_knee_pos = 0.0
        self.fl_knee_vel = 0.0
        self.fl_knee_torque = 0.0
        self.rr_hip_pos = 0.0
        self.rr_hip_vel = 0.0
        self.rr_hip_torque = 0.0
        self.rr_knee_pos = 0.0
        self.rr_knee_vel = 0.0
        self.rr_knee_torque = 0.0
        self.rl_hip_pos = 0.0
        self.rl_hip_vel = 0.0
        self.rl_hip_torque = 0.0
        self.rl_knee_pos = 0.0
        self.rl_knee_vel = 0.0
        self.rl_knee_torque = 0.0

        # Initialize ODrive-related objects as None - create them later
        self.axis_state_clients = {}
        self.odrive_publishers = {}
        self.odrive_subscribers = {}
        self.odrive_messages = {}
        self.odrive_timer = None
        self.odrive_initialized = False
        

        # Create joystick subscriber:
        self.joystick_subscriber = self.create_subscription(msg_type = Joy, topic = 'joy', callback=self.joy_callback, qos_profile=qos_profile)
        self.wheel_publisher_ = self.create_publisher(WheelCommands, 'wheel_commands', qos_profile)

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
        odrive_nodes = ['fr_hip', 'fr_knee', 'fl_hip', 'fl_knee', 'rr_hip', 'rr_knee', 'rl_hip', 'rl_knee']
        for node_name in odrive_nodes:
            # Add error handling for service creation
            try:
                self.axis_state_clients[node_name] = self.create_client(AxisState, f'/{node_name}/request_axis_state')
                self.get_logger().info(f"Created service client for {node_name}")
            except Exception as e:
                self.get_logger().error(f"Failed to create service client for {node_name}: {e}")


        # ODrive control messages:
        #____________________________________________
        #____________________________________________
        self.fr_hip_msg = ControlMessage(control_mode = 3, input_mode = 5)
        self.fr_knee_msg = ControlMessage(control_mode = 1, input_mode = 1)

        self.fl_hip_msg = ControlMessage(control_mode = 3, input_mode = 5)
        self.fl_knee_msg = ControlMessage(control_mode = 1, input_mode = 1)
        
        self.rr_hip_msg = ControlMessage(control_mode = 3, input_mode = 5)
        self.rr_knee_msg = ControlMessage(control_mode = 1, input_mode = 1)
        
        self.rl_hip_msg = ControlMessage(control_mode = 3, input_mode = 5)
        self.rl_knee_msg = ControlMessage(control_mode = 1, input_mode = 1)
        #____________________________________________
        #____________________________________________

        self.get_logger().info("ODrive control messages initialized")



        # Create subscriber for controller status messages:
        subscriber_qos = rclpy.qos.QoSProfile(
            depth=1,  # Smaller queue
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,  # Less strict
            durability=rclpy.qos.DurabilityPolicy.VOLATILE  # Don't store messages
        )
        self.fr_hip_sub = self.create_subscription(
            msg_type=ControllerStatus,
            topic='/fr_hip/controller_status',
            callback=self.fr_hip_callback,
            qos_profile=subscriber_qos
        )
        time.sleep(0.1)  # Small delay to ensure subscribers are ready
        self.fr_knee_sub = self.create_subscription(
            msg_type=ControllerStatus,
            topic='/fr_knee/controller_status',
            callback=self.fr_knee_callback,
            qos_profile=subscriber_qos
        )
        time.sleep(0.1)  # Small delay to ensure subscribers are ready

        self.fl_hip_sub = self.create_subscription(
            msg_type=ControllerStatus,
            topic='/fl_hip/controller_status',
            callback=self.fl_hip_callback,
            qos_profile=subscriber_qos
        )
        time.sleep(0.1)  # Small delay to ensure subscribers are ready

        self.fl_knee_sub = self.create_subscription(
            msg_type=ControllerStatus,
            topic='/fl_knee/controller_status',
            callback=self.fl_knee_callback,
            qos_profile=subscriber_qos
        )
        time.sleep(0.1)  # Small delay to ensure subscribers are ready

        self.rr_hip_sub = self.create_subscription(
            msg_type=ControllerStatus,
            topic='/rr_hip/controller_status',
            callback=self.rr_hip_callback,
            qos_profile=subscriber_qos
        )
        time.sleep(0.1)  # Small delay to ensure subscribers are ready

        self.rr_knee_sub = self.create_subscription(
            msg_type=ControllerStatus,
            topic='/rr_knee/controller_status',
            callback=self.rr_knee_callback,
            qos_profile=subscriber_qos
        )
        time.sleep(0.1)  # Small delay to ensure subscribers are ready

        self.rl_hip_sub = self.create_subscription(
            msg_type=ControllerStatus,
            topic='/rl_hip/controller_status',
            callback=self.rl_hip_callback,
            qos_profile=subscriber_qos
        )
        time.sleep(0.1)  # Small delay to ensure subscribers are ready
        self.rl_knee_sub = self.create_subscription(
            msg_type=ControllerStatus,
            topic='/rl_knee/controller_status',
            callback=self.rl_knee_callback,
            qos_profile=subscriber_qos
        )
        time.sleep(0.1)  # Small delay to ensure subscribers are ready
        self.get_logger().info("ODrive status subscribers initialized")

        # ODrive control message publishers:
        self.fr_hip_pub = self.create_publisher(ControlMessage, '/fr_hip/control_message', qos_profile)
        self.fr_knee_pub = self.create_publisher(ControlMessage, '/fr_knee/control_message', qos_profile)
        self.fl_hip_pub = self.create_publisher(ControlMessage, '/fl_hip/control_message', qos_profile)
        self.fl_knee_pub = self.create_publisher(ControlMessage, '/fl_knee/control_message', qos_profile)
        self.rr_hip_pub = self.create_publisher(ControlMessage, '/rr_hip/control_message', qos_profile)
        self.rr_knee_pub = self.create_publisher(ControlMessage, '/rr_knee/control_message', qos_profile)
        self.rl_hip_pub = self.create_publisher(ControlMessage, '/rl_hip/control_message', qos_profile)
        self.rl_knee_pub = self.create_publisher(ControlMessage, '/rl_knee/control_message', qos_profile)
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

        # --------------- Joystick Control (ODRIVE) ---------------
        # IF SAFETY IS OFF, CONTROL:
        if not self.safety_on:

            # Read buttons:
            x_button = msg.buttons[0]  # X button
            a_button = msg.buttons[1]  # Y button
            b_button = msg.buttons[2]  # A button
            y_button = msg.buttons[3]  # B button
            left_bumper = msg.buttons[4]  # Left bumper
            right_bumper = msg.buttons[5]  # Right bumper



            # Read right joystick for knee control:
            right_stick_ud = msg.axes[3]
            right_stick_lr = msg.axes[2]

            # Map joystick inputs to knee velocities:
            right_knee_vel = self.max_knee_vel * right_stick_ud + self.max_knee_vel * right_stick_lr
            left_knee_vel = self.max_knee_vel * right_stick_ud - self.max_knee_vel * right_stick_lr
            
            # Ensure knee velocities are within limits:
            left_knee_vel = max(min(left_knee_vel, self.max_knee_vel), -self.max_knee_vel)
            right_knee_vel = max(min(right_knee_vel, self.max_knee_vel), -self.max_knee_vel)


            #____________________________________________
            #____________________________________________
            # Map joystick inputs to knee TORQUES:
            right_knee_torque = self.max_knee_torque * right_stick_ud + self.max_knee_torque * right_stick_lr
            left_knee_torque = self.max_knee_torque * right_stick_ud - self.max_knee_torque * right_stick_lr
            
            # Ensure knee TORQUES are within limits:
            left_knee_torque = max(min(left_knee_torque, self.max_knee_torque), -self.max_knee_torque)
            right_knee_torque = max(min(right_knee_torque, self.max_knee_torque), -self.max_knee_torque)
            #____________________________________________
            #____________________________________________



            fr_knee_des_pos = self.fr_knee_pos + right_knee_vel * self.dt  # Assuming 10ms control loop
            fl_knee_des_pos = self.fl_knee_pos - left_knee_vel * self.dt  # Assuming 10ms control loop
            rr_knee_des_pos = self.rr_knee_pos + right_knee_vel * self.dt # Assuming 10ms control loop
            rl_knee_des_pos = self.rl_knee_pos - left_knee_vel * self.dt  # Assuming 10ms control loop


            # Map dpad inputs to hip velocities:
            dpad_ud = msg.axes[5]
            dpad_lr = msg.axes[4]
            self.des_hip_splay = self.des_hip_splay + dpad_ud * self.max_hip_vel * self.dt  # Adjust splay angle based on dpad input
            self.des_hip_splay = max(min(self.des_hip_splay, self.max_hip_angle), self.min_hip_angle)  # Clamp splay angle
            # Change configuration based on button presses:
            if(a_button == 1):
                self.des_hip_splay = 0.0  # Reset splay angle
                fr_knee_des_pos = self.nearest_pi_knee(self.rr_knee_pos)  # Example usage of nearest_pi function
                fl_knee_des_pos = self.nearest_pi_knee(self.fl_knee_pos)  # Example usage of nearest_pi function
                rr_knee_des_pos = self.nearest_pi_knee(self.rr_knee_pos)  # Example usage of nearest_pi function
                rl_knee_des_pos = self.nearest_pi_knee(self.rl_knee_pos)  # Example usage of nearest_pi function
            if(x_button == 1):
                self.des_hip_splay = -0.5  # Reset splay angle
                fr_knee_des_pos = self.nearest_pi_knee(self.fr_knee_pos) - 1  
                fl_knee_des_pos = self.nearest_pi_knee(self.fl_knee_pos) + 1
                rr_knee_des_pos = self.nearest_pi_knee(self.rr_knee_pos) + 1
                rl_knee_des_pos = self.nearest_pi_knee(self.rl_knee_pos) - 1
            if(y_button == 1):
                self.des_hip_splay = 0.5  # Reset splay angle
                fr_knee_des_pos = self.nearest_pi_knee(self.fr_knee_pos) + self.fr_hip_pos
                fl_knee_des_pos = self.nearest_pi_knee(self.fl_knee_pos) + self.fl_hip_pos
                rr_knee_des_pos = self.nearest_pi_knee(self.rr_knee_pos) + self.rr_hip_pos
                rl_knee_des_pos = self.nearest_pi_knee(self.rl_knee_pos) + self.rl_hip_pos
                self.get_logger().info(f"Setting hip splay to {self.des_hip_splay} radians")
                self.get_logger().info(f"Setting knee positions to FR: {fr_knee_des_pos}, FL: {fl_knee_des_pos}, RR: {rr_knee_des_pos}, RL: {rl_knee_des_pos}")

            # ---------------- ODrive Control Messages ----------------

            # Construct ODrive control messages:
            # Front Right Knee:
            
            #____________________________________________
            # 
            # Knee torque mode
            #____________________________________________
            self.fr_knee_msg.control_mode = 1
            self.fr_knee_msg.input_mode = self.knee_input_mode
            self.fr_knee_msg.input_pos = 0.0  # Not used in velocity
            self.fr_knee_msg.input_vel = 0.0
            self.fr_knee_msg.input_torque = right_knee_torque  # Not used in velocity control
            # Front Left Knee:
            self.fl_knee_msg.control_mode = 1
            self.fl_knee_msg.input_mode = self.knee_input_mode
            self.fl_knee_msg.input_pos = 0.0  # Not used in velocity
            self.fl_knee_msg.input_vel = 0.0
            self.fl_knee_msg.input_torque = left_knee_torque  # Not used in velocity control
            # Back Right Knee:
            self.rr_knee_msg.control_mode = 1
            self.rr_knee_msg.input_mode = self.knee_input_mode
            self.rr_knee_msg.input_pos = 0.0 # Not used in velocity
            self.rr_knee_msg.input_vel = 0.0
            self.rr_knee_msg.input_torque = right_knee_torque  # Not used in velocity control
            # Back Left Knee:
            self.rl_knee_msg.control_mode = 1
            self.rl_knee_msg.input_mode = self.knee_input_mode
            self.rl_knee_msg.input_pos = 0.0  # Not used in velocity
            self.rl_knee_msg.input_vel = 0.0
            self.rl_knee_msg.input_torque = left_knee_torque  # Not used in velocity control

            #____________________________________________
            #____________________________________________
            
            # Front Right Hip:
            self.fr_hip_msg.control_mode = 3
            self.fr_hip_msg.input_mode = self.hip_input_mode
            self.fr_hip_msg.input_pos = -self.des_hip_splay  # Desired hip splay position
            self.fr_hip_msg.input_vel = 0.0  # Not used
            self.fr_hip_msg.input_torque = 0.0  # Not used in velocity control
            # Front Left Hip:
            self.fl_hip_msg.control_mode = 3
            self.fl_hip_msg.input_mode = self.hip_input_mode
            self.fl_hip_msg.input_pos = self.des_hip_splay  # Desired hip splay position
            self.fl_hip_msg.input_vel = 0.0  # Not used
            self.fl_hip_msg.input_torque = 0.0  # Not used in velocity control
            # Back Right Hip:
            self.rr_hip_msg.control_mode = 3
            self.rr_hip_msg.input_mode = self.hip_input_mode
            self.rr_hip_msg.input_pos = self.des_hip_splay  # Desired hip splay position
            self.rr_hip_msg.input_vel = 0.0  # Not used
            self.rr_hip_msg.input_torque = 0.0
            # Back Left Hip:
            self.rl_hip_msg.control_mode = 3
            self.rl_hip_msg.input_mode = self.hip_input_mode
            self.rl_hip_msg.input_pos = -self.des_hip_splay  # Desired hip splay position
            self.rl_hip_msg.input_vel = 0.0  # Not used
            self.rl_hip_msg.input_torque = 0.0  # Not used



            # ---------------- Joystick Control (Wheels) ---------------
            # Here we would process the joystick command and set wheel speeds accordingly.
            left_stick_ud = msg.axes[0]  # Left stick left/right
            left_stick_lr = msg.axes[1]  # Left stick up/down

            # Map joystick inputs to differential wheel speeds:
            left_wheel_speed = self.max_wheel_vel * left_stick_ud + self.max_wheel_vel * left_stick_lr
            right_wheel_speed = self.max_wheel_vel * left_stick_ud - self.max_wheel_vel * left_stick_lr

        # IF SAFETY IS ON: 
        else:
            left_wheel_speed = 0.0
            right_wheel_speed = 0.0

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


    # Nearest Pi function to calculate closest multiple of pi:
    def nearest_pi_knee(self, angle):
        value = 0.5*6*30/15
        near_pi = np.round(angle/value) * value
        return near_pi

    # Publish the wheel commands to the wheel command topic:
    def publish_wheel_commands(self):
        if not self.odrive_initialized:
            return
        self.wheel_publisher_.publish(self.wheel_commands)


    # Callbacks for ODrive status messages:
    # These will update the ACTUAL position and velocity of the hip/knee motors:
    def fr_hip_callback(self, msg):
        self.fr_hip_pos = msg.pos_estimate
        self.fr_hip_vel = msg.vel_estimate
        self.fr_hip_torque = msg.torque_estimate
    def fr_knee_callback(self, msg):
        self.fr_knee_pos = msg.pos_estimate
        self.fr_knee_vel = msg.vel_estimate
        self.fr_knee_torque = msg.torque_estimate
    def fl_hip_callback(self, msg):
        self.fl_hip_pos = msg.pos_estimate
        self.fl_hip_vel = msg.vel_estimate
        self.fl_hip_torque = msg.torque_estimate
    def fl_knee_callback(self, msg):
        self.fl_knee_pos = msg.pos_estimate
        self.fl_knee_vel = msg.vel_estimate
        self.fl_knee_torque = msg.torque_estimate
    def rr_hip_callback(self, msg):
        self.rr_hip_pos = msg.pos_estimate
        self.rr_hip_vel = msg.vel_estimate  
        self.rr_hip_torque = msg.torque_estimate
    def rr_knee_callback(self, msg):
        self.rr_knee_pos = msg.pos_estimate
        self.rr_knee_vel = msg.vel_estimate
        self.rr_knee_torque = msg.torque_estimate
    def rl_hip_callback(self, msg):
        self.rl_hip_pos = msg.pos_estimate
        self.rl_hip_vel = msg.vel_estimate
        self.rl_hip_torque = msg.torque_estimate
    def rl_knee_callback(self, msg):
        self.rl_knee_pos = msg.pos_estimate
        self.rl_knee_vel = msg.vel_estimate
        self.rl_knee_torque = msg.torque_estimate

    def publish_odrive_commands(self):
        self.fr_knee_pub.publish(self.fr_knee_msg)
        self.fr_hip_pub.publish(self.fr_hip_msg)
        self.fl_hip_pub.publish(self.fl_hip_msg)
        self.fl_knee_pub.publish(self.fl_knee_msg)
        self.rr_hip_pub.publish(self.rr_hip_msg)
        self.rr_knee_pub.publish(self.rr_knee_msg)
        self.rl_hip_pub.publish(self.rl_hip_msg)
        self.rl_knee_pub.publish(self.rl_knee_msg)



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
    intelligent_ctrl_loop = MainControlLoop()
    rclpy.spin(intelligent_ctrl_loop)
    intelligent_ctrl_loop.destroy_node()
    rclpy.shutdown()


def intelligent_ctrl():
    pass