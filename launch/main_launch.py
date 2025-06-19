from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='main_ctrl',
            executable='main_ctrl',
            name='main_ctrl_node',
            
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',

        ),
        Node(
            package='wheel_control',
            executable='wheel_control',
            name='wheel_ctrl',
        )
    ])