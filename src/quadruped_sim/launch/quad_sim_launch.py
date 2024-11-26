import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

package_name = 'quadruped_sim'

def generate_launch_description():

    launch_description = LaunchDescription()

    # # Correct the environment variable setup
    # gazebo_resource_envi_path = SetEnvironmentVariable(
    #     'GAZEBO_MODEL_PATH', 
    #     os.path.join(get_package_share_directory(package_name),'models','environment')
    # )

    # # Add the environment variable setup action to the launch description
    # launch_description.add_action(gazebo_resource_envi_path)
    
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_project_gazebo = get_package_share_directory('quadruped_sim')

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'models',
            'quadruped_world.sdf'
        ])}.items(),
    )

    # gazebo_server = Node(
    #     package='ros_gz_sim',
    #     executable='gzserver',
    #     arguments={
    #         'world_sdf_file' : "/workspace/src/quadruped_sim/models/quadruped_world.sdf"
    #     }
    # )
    
    launch_description.add_action(gz_sim)

    return launch_description
