import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('quadruped_bringup')
    pkg_project_gazebo = get_package_share_directory('quadruped_gazebo')
    pkg_project_description = get_package_share_directory('quadruped_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo, 'models', 'quadruped_world.sdf'
        ])}.items(),
    )

    # Load the SDF file from "description" package
    # sdf_file  =  os.path.join(pkg_project_description, 'models', 'quadruped', 'model.sdf')
    # with open(sdf_file, 'r') as infp:
    #     robot_desc = infp.read()

    # # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='both',
    #     parameters=[
    #         {'use_sim_time': True},
    #         {'robot_description': robot_desc},
    #     ]
    # )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_bridge_config.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        # robot_state_publisher,
    ])
