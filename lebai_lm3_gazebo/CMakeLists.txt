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
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED) 

rosidl_generate_interfaces(${PROJECT_NAME}
  srv/ExecuteWaypoints.srv
  DEPENDENCIES geometry_msgs
)

add_executable(simple_move src/simple_move.cpp)
ament_target_dependencies(simple_move
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_task_constructor_core
  moveit_ros_planning
  ament_index_cpp
  yaml-cpp
)

target_link_libraries(simple_move yaml-cpp)

target_compile_features(simple_move PUBLIC cxx_std_17)

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

rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

# Link that target to your executable so it waits for headers to exist
target_link_libraries(simple_move "${cpp_typesupport_target}")

install(TARGETS
  simple_move
  DESTINATION lib/${PROJECT_NAME}
)

# Ensure the executable script is installed to the 'lib' directory
install(PROGRAMS
    script/target_robot.py  # Path to your Python script relative to CMakeLists.txt
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
