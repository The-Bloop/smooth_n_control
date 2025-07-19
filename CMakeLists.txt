cmake_minimum_required(VERSION 3.8)
project(smooth_n_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(smooth_n_control REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED) 
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SmoothPath.srv"
  DEPENDENCIES 
  std_msgs
)
#rosidl_default_generators

add_executable(smooth_path_server src/SmoothPathServer.cpp)
ament_target_dependencies(smooth_path_server rclcpp std_msgs smooth_n_control)

add_executable(smooth_path_client src/PathSmoothing.cpp)
ament_target_dependencies(smooth_path_client rclcpp std_msgs visualization_msgs smooth_n_control geometry_msgs tf2 tf2_geometry_msgs nav_msgs)

add_executable(empty_map_publisher src/EmptyMap.cpp)
ament_target_dependencies(empty_map_publisher rclcpp nav_msgs)

install(TARGETS 
  smooth_path_server
  smooth_path_client
  empty_map_publisher
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY srv src include launch rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
