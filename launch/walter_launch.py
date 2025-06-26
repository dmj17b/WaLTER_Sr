from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():
    
    # Minimal robot description with multiple CAN interfaces - POSITION CONTROL
    robot_description_content = """<?xml version="1.0"?>
<robot name="walter_motors">
  <ros2_control name="odrive_hardware" type="system">
    <hardware>
      <plugin>odrive_ros2_control_plugin/ODriveHardwareInterface</plugin>
      <param name="can">can1</param>
    </hardware>
    
    <!-- Start with just one joint for testing -->
    <joint name="fr_knee">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <param name="node_id">0</param>
    </joint>
    
  </ros2_control>
</robot>"""
    robot_description = {"robot_description": robot_description_content}
    
    # Controller configuration - POSITION CONTROLLERS
    controller_params = {
        "controller_manager": {
            "ros__parameters": {
                "update_rate": 100,
                "joint_state_broadcaster": {
                    "type": "joint_state_broadcaster/JointStateBroadcaster"
                },
                "position_controller": {
                    "type": "position_controllers/JointGroupPositionController",
                    "joints": [
                        "fr_knee", ]
                }
            }
        },
    }
    
    # Controller manager node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_params],
        output="both",
    )
    
    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    # Position controller spawner (changed from velocity_controller)
    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
    )
    
    # Your existing nodes
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
    )
    
    main_ctrl_node = Node(
        package='main_ctrl',
        executable='main_ctrl',
        name='main_ctrl_node',
    )
    
    wheel_ctrl_node = Node(
        package='wheel_control',
        executable='wheel_control',
        name='wheel_ctrl',
    )
    
    return LaunchDescription([
        control_node,
        joint_state_broadcaster_spawner,
        position_controller_spawner,  # Changed from velocity_controller_spawner
        # joy_node,
        # main_ctrl_node,
        # wheel_ctrl_node,
    ])