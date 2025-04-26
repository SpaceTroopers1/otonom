from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config_path = PathJoinSubstitution([
        FindPackageShare("IMU_odom"),
        "config",
        "ekf.yaml"
    ])

    return LaunchDescription([
        # Static transform from base_footprint to imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'imu_link']
        ),

        # EKF node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_path]
        )
    ])

