import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node


def generate_launch_description():

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('quadruped_bringup')
    pkg_project_gazebo = get_package_share_directory('quadruped_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    
    ############################### SIMULATION NODES ##################################################3
    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': TextSubstitution(
                text=PathJoinSubstitution([
                pkg_project_gazebo, 'models', 'quadruped_world.sdf'
            ]).perform(None) + ' -r'
        )}.items(),
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
    
    ############################################# ACTUATION NODES ###################################################
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
    
    ############################################# LOGGING NODES ############################################################
    
    desired_data_logger = Node(
        package='energy_pkg',
        executable='desired_data_logger',
        name='desired_data_logger',
        output='both',
    )
    
    raw_data_logger = Node(
        package='energy_pkg',
        executable='raw_data_logger',
        name='raw_data_logger',
        output='both'
    )
        
    low_pass_filter = Node(
        package='energy_pkg',
        executable='low_pass_filter',
        name='low_pass_filter',
        output='both'
    )
        
    filtered_data_logger = Node(
        package='energy_pkg',
        executable='filtered_data_logger',
        name='filtered_data_logger',
        output='both'
    )

    return LaunchDescription([
        ######## SIMULATION NODES ########
        gz_sim,
        bridge,
        ######## ACTUATION NODES #########
        initial_conditions,
        gait_loader,
        ######## LOGGING NODES #########
        desired_data_logger,
        raw_data_logger,
        # low_pass_filter,
        # filtered_data_logger,
    ])
