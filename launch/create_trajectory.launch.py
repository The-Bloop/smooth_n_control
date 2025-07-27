from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='smooth_n_control',  
            executable='create_trajectory_server', 
            output='screen',
            name='create_trajectory_server'
        ),
        Node(
            package='smooth_n_control',  
            executable='create_trajectory_client', 
            output='screen',
            name='create_trajectory_client'
        )
    ])