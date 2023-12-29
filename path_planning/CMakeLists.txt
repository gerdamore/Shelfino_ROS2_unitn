cmake_minimum_required(VERSION 3.5)
project(path_planning)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(ntp src/publisher_member_function.cpp)
#add_executable(Follow_path src/Follow_path.cpp)
ament_target_dependencies(ntp rclcpp std_msgs geometry_msgs nav2_msgs rclcpp_action)
#ament_target_dependencies(Follow_path rclcpp std_msgs geometry_msgs nav2_msgs rclcpp_action)
install(TARGETS
  ntp
#  Follow_path
  DESTINATION lib/${PROJECT_NAME})

ament_package()