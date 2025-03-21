from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    ld = LaunchDescription()

    urdf_file = os.path.join(
        os.path.dirname(__file__), '..', 'urdf', 'arac_Montaj1_urdf.urdf'
    )
    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )
    joint_state_publisher_node=Node(
        package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'
    )

    ydlidar_launch_file = FindPackageShare('ydlidar_ros2_driver').find('ydlidar_ros2_driver') + '/launch/ydlidar_launch.py'
    
    # Include the launch file from ydlidar_ros2_driver package
    include_launch = IncludeLaunchDescription(ydlidar_launch_file)
    ld.add_action(include_launch)

    

    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    
    return ld