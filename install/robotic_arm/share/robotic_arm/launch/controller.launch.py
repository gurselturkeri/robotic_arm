import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from scripts import GazeboRosPaths

def generate_launch_description():
    package_share_dir = get_package_share_directory("robotic_arm")    # Add package name
    urdf_file = os.path.join(package_share_dir, "urdf", "robotic_arm.urdf")  # Add urdf name
    controller_file = os.path.join(package_share_dir, "config", "jtc.yaml")
    robot_description = {"robot_description": urdf_file}
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=["gazebo","-s","libgazebo_ros_factory.so",],
                output="screen",
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=["-entity","robotic_arm","-b","-file", urdf_file,],    # Add package name
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                arguments=[urdf_file],
            ),

            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[robot_description, controller_file],
                output="screen",
            ),
            Node(
            	package="controller_manager",
            	executable="spawner",
            	arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
            )
        ]
    )