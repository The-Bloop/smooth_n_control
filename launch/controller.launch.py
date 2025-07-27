from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='smooth_n_control',  
            executable='move_bot_server', 
            output='screen',
            name='move_bot_server'
        ),
        Node(
            package='smooth_n_control',  
            executable='move_bot_client', 
            output='screen',
            name='move_bot_client'
        )
    ])