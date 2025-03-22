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
    py_gait_loader = Node(
        package='gait_generation',
        executable='analytical_gait_loader',
        name='python_gait_loader',
        output='both'
    )
    
    cpp_gait_loader = Node(
        package='gait_generation_cpp',
        executable='analytical_gait_loader',
        name='cpp_gait_loader',
        output='both'
    )

    return LaunchDescription([
        ######## SIMULATION NODES ########
        gz_sim,
        bridge,
        ######## ACTUATION NODES #########
        py_gait_loader,
        cpp_gait_loader,
    ])
