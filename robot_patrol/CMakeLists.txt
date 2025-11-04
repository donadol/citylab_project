cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

add_executable(patrol_executable src/patrol.cpp)
ament_target_dependencies(patrol_executable
  rclcpp
  sensor_msgs
  geometry_msgs
)

add_executable(direction_service src/direction_service.cpp)
ament_target_dependencies(direction_service
  rclcpp
  sensor_msgs
  geometry_msgs
)

add_executable(test_service src/test_service.cpp)
ament_target_dependencies(test_service
  rclcpp
  sensor_msgs
)

add_executable(patrol_with_service_executable src/patrol_with_service.cpp)
ament_target_dependencies(patrol_with_service_executable
  rclcpp
  sensor_msgs
  geometry_msgs
)

add_executable(go_to_pose_action src/go_to_pose_action.cpp)
ament_target_dependencies(go_to_pose_action
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  patrol_executable
  direction_service
  test_service
  patrol_with_service_executable
  go_to_pose_action
	DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
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

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetDirection.srv"
  "action/GoToPose.action"
   DEPENDENCIES sensor_msgs geometry_msgs
)

ament_package()
