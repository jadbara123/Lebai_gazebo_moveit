cmake_minimum_required(VERSION 3.8)
project(lebai_lm3_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(controller_manager REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install urdf files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install rviz files (if exists)
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Install world files (if exists)
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Ensure the executable script is installed to the 'lib' directory
install(PROGRAMS
    src/target_robot.py  # Path to your Python script relative to CMakeLists.txt
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
