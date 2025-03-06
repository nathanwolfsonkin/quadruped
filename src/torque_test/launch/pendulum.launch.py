import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_description = get_package_share_directory('torque_test')

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': PathJoinSubstitution([
                pkg_description, 'models', 'pendulum.sdf'
            ]),
            'verbose': 'true'  # Enable verbose output for Gazebo
        }.items(),
    )


    return LaunchDescription([
        gz_sim,
    ])
