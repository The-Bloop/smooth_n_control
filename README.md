# Smooth N Control
Given a 2D path, this package will make it smoother, generate trajectories, and control a robot to complete the path.

## Setup
1. Clone the following repositories into your ROS2 Humble workspace:
   * [wheel_robot_v3](https://github.com/The-Bloop/wheel_robot_v3)
   * [Current Repo](https://github.com/The-Bloop/smooth_n_control)
2. Install the following packages:
    ```
    sudo apt install -y ros-humble-robot-localization
    sudo apt install -y ros-humble-xacro
    sudo apt install -y ros-humble-gazebo-ros-pkgs
    sudo apt install -y ros-humble-desktop
3. Build the packages.

## Execute
Note: Execute the following command in all terminals:

    cd <workspace_path>
    source install/local_setup.bash

1. Launch the display.launch.py file to launch the robot in Gazebo and RViz:
   ```
   ros2 launch smooth_n_control display.launch.py
2. In a new terminal, run the following lines to publish the transform between the odom and map frames:
   ```
   ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 map odom
3. In a new terminal, launch the nav_bringup launch file:
   ```
   ros2 launch smooth_n_control nav_bringup.launch.py
4. In another new terminal, run the following lines to start a SmoothPath service client:
   ```
   # You can substitute the file path for the 2D path file.
   # Check how the 2D path file must be written in pathFiles/path1.txt
   ros2 run smooth_n_control smooth_path_client src/smooth_n_control/pathFiles/path1.txt
