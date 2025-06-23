from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments

    # Create nodes for each ODrive motor
    fr_hip = Node(
        package='odrive_can',
        executable='odrive_can_node',
        name = 'fr_hip',
        parameters = [{
            'node_id' : 0,
            'interface' : 'can1',
        }] 
    )
    fr_knee = Node(
        package='odrive_can',
        executable='odrive_can_node',
        name = 'fr_knee',
        parameters = [{
            'node_id' : 1,
            'interface' : 'can1',
        }] 
    )
    fl_hip = Node(
        package='odrive_can',
        executable='odrive_can_node',
        name = 'fl_hip',
        parameters = [{
            'node_id' : 2,
            'interface' : 'can1',
        }] 
    )
    fl_knee = Node(
        package='odrive_can',
        executable='odrive_can_node',
        name = 'fl_knee',
        parameters = [{
            'node_id' : 3,
            'interface' : 'can1',
        }]
    )
    rr_hip = Node(
        package='odrive_can',
        executable='odrive_can_node',
        name = 'rr_hip',
        parameters = [{
            'node_id' : 4,
            'interface' : 'can1',
        }] 
    )
    rr_knee = Node(
        package='odrive_can',
        executable='odrive_can_node',
        name = 'rr_knee',
        parameters = [{
            'node_id' : 5,
            'interface' : 'can1',
        }]
    )
    rl_hip = Node(
        package='odrive_can',
        executable='odrive_can_node',
        name = 'rl_hip',
        parameters = [{
            'node_id' : 6,
            'interface' : 'can1',
        }] 
    )
    rl_knee = Node(
        package='odrive_can',
        executable='odrive_can_node',
        name = 'rl_knee',
        parameters = [{
            'node_id' : 7,
            'interface' : 'can1',
        }] 
    )

    # Main control node 
    # This node handles the main control loop, reading joystick inputs and mappng them to hip/knee/wheel commands
    main_ctrl_node= Node(
        package='main_ctrl',
        executable='main_ctrl',
        name='main_ctrl_node',
        
    ),

    # Joystick reader
    # This node reads joystick inputs and publishes them to the 'joy' topic
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',

    ),

    # Wheel control node
    # This node subscribes to the 'wheel_commands' topic and sends commands to the wheel driver board
    wheel_ctrl_node = Node(
        package='wheel_control',
        executable='wheel_control',
        name='wheel_ctrl',
    )
    
    return LaunchDescription([
        joy_node,
        wheel_ctrl_node,
        main_ctrl_node,
    ])
