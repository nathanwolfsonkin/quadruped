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
    pkg_project_bringup = get_package_share_directory('quadruped_bringup')
    pkg_project_gazebo = get_package_share_directory('quadruped_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo, 'models', 'quadruped_world.sdf'
        ])}.items(),
    )

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
    
    initial_conditions = Node(
        package='gait_generation',
        executable='initial_conditions',
        name='initial_conditions',
        output='both'
    )
    
    gait_loader = Node(
        package='gait_generation',
        executable='analytical_gait_loader',
        name='analytical_gait_loader',
        output='both'
    )
    
    log_gazebo_gait = Node(
        package='energy_pkg',
        executable='log_gazebo_gait',
        name='log_gazebo_gait',
        output='both'
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        initial_conditions,
        gait_loader,
        log_gazebo_gait,
    ])
