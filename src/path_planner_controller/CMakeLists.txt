cmake_minimum_required(VERSION 3.8)
project(path_planner_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(path_planner_node src/path_planner_node.cpp)
ament_target_dependencies(path_planner_node rclcpp nav_msgs geometry_msgs tf2)

add_executable(pure_pursuit_controller_node src/pure_pursuit_controller_node.cpp)
ament_target_dependencies(pure_pursuit_controller_node rclcpp nav_msgs geometry_msgs tf2 tf2_geometry_msgs)

install(TARGETS
  path_planner_node
  pure_pursuit_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
