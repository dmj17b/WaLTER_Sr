from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
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


    # Declare the experiment name argument
    save_location_arg = DeclareLaunchArgument(
        "save_location",
        default_value="default_experiment",
        description="Location to save the experiment data",
    )

    # Get the experiment name
    save_location = LaunchConfiguration("save_location")
    record_all_topics = [
        "ros2",
        "bag",
        "record",
        "-a",
        "-o",
        save_location,
        "--storage",
        "sqlite3",
    ]

    ros_bagger = ExecuteProcess(
        cmd=record_all_topics,
        shell=True,
        name="record_all_topics",
        output="screen",
        emulate_tty=False,
    )

    """
    Put your nodes here.
    """

    return LaunchDescription(
        [
            save_location_arg,
            ros_bagger,
            # Put you node names here
            hip,
            knee,
            wheels,
            joy_node,
            main_ctrl_node,
        ]
    )
