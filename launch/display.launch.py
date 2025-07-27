import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = FindPackageShare(package='smooth_n_control').find("smooth_n_control")
    rviz_config_path = os.path.join(pkg_share, 'rviz/rviz_config.rviz')

    robot_name = 'WheelRobotv3'
    rob_description_package_name = "wheel_robot_v3"
    robot_pkg_share = FindPackageShare(package=rob_description_package_name).find(rob_description_package_name)
    urdf_model_path = os.path.join(robot_pkg_share, 'urdf/robot.urdf.xacro')

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world = os.path.join(
        pkg_share,
        'worlds',
        'empty.world'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    logger = LaunchConfiguration('log_level', default=["info"])
    x_pose = LaunchConfiguration('x_pose', default='-0.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.0')

    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_pkg_share, 'launch', 'robot_localization.launch.py')
        ),
    )

    robot_description = Command([FindExecutable(name='xacro'), ' ', urdf_model_path])

    logger_arg = DeclareLaunchArgument(
        name='log_level',
        default_value="info",
        description='Indicates logging level of Nodes'
    )

    sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value="True",
        description='On/Off Simulation time'
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[urdf_model_path],
        parameters=[{'use_sim_time': use_sim_time}],
        )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 
        'robot_description': robot_description}],
        arguments=['--ros-args','--log-level',logger]
    )
    
    empty_map_node = Node(
            package='smooth_n_control',  
            executable='empty_map_publisher', 
            output='screen',
            name='empty_map_publisher'
    )
    
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path ]  # no config, so starts with empty display
        )
    
    gazebo_ros_spawner_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', robot_name,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01',
            '--ros-args','--log-level',logger
        ],
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(logger_arg)
    ld.add_action(sim_time_arg)

    #Launch Files
    ld.add_action(gzserver_launch)
    ld.add_action(gzclient_launch)
    ld.add_action(robot_localization_launch)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(gazebo_ros_spawner_node)
    ld.add_action(rviz_node)
    ld.add_action(empty_map_node)
    

    return ld