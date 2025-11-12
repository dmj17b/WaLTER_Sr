from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch

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
            'axis_idle_on_shutdown': True,
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
            'axis_idle_on_shutdown': True,
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

    ros_bagger =  launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '/hip/controller_status', '/knee/controller_status', '/knee/odrive_status', '/hip/odrive_status'],
        )

    bag_output_arg = DeclareLaunchArgument(
        'rosbags',
        default_value='my_recorded_bag',
        description='Directory to save the recorded bag file'
    )

    # Declare a launch argument for the topics to record
    topics_to_record_arg = DeclareLaunchArgument(
        'topics_to_record',
        default_value=['/topic1', '/topic2'],
        description='List of topics to record'
    )


    return LaunchDescription([
        hip,
        knee,
        wheels,
        joy_node,
        main_ctrl_node,
        ros_bagger,
    ])
