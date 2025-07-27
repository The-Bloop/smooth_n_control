import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    pkg_share = FindPackageShare(package="smooth_n_control").find("smooth_n_control")

    smooth_path_server_node = Node(
            package='smooth_n_control',  
            executable='smooth_path_server', 
            output='screen',
            name='smooth_path_server'
        )
    
    trajectory_generator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'create_trajectory.launch.py')
        )
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'controller.launch.py')
        )
    )

    ld = LaunchDescription()

    ld.add_action(smooth_path_server_node)
    ld.add_action(trajectory_generator_launch)
    ld.add_action(controller_launch)

    return ld
