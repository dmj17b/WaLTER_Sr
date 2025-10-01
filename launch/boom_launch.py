from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    # Declare launch arguments

    # Create nodes for each ODrive motor
    knee = Node(
        package='odrive_can',
        executable='odrive_can_node',
        name = 'knee',
        namespace='knee',
        parameters = [{
            'node_id' : 1,
            'interface' : 'can0',
        }] 
    )
    hip = Node(
        package='odrive_can',
        executable='odrive_can_node',
        name = 'hip',
        namespace='hip',
        parameters = [{
            'node_id' : 0,
            'interface' : 'can0',
        }] 
    )
    wheels = Node(
        package = 'pi_wheel_ctrl',
        executable = 'pi_wheel_ctrl',
        name = 'wheel_ctrl',
    )
    # Main control node 
    # This node handles the main control loop, reading joystick inputs and mappng them to hip/knee/wheel commands
    main_ctrl_node= Node(
        package='main_ctrl',
        executable='main_ctrl',
        name='main_ctrl_node',
        
    )

    # Joystick reader
    # This node reads joystick inputs and publishes them to the 'joy' topic
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'deadzone': 0.1,
            'dev': '/dev/input/js0',
            'coalesce_interval': 0.05,
        }]

    )



    return LaunchDescription([
        hip,
        knee,
        wheels,
        joy_node,
        main_ctrl_node,


    ])
