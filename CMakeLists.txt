cmake_minimum_required(VERSION 3.16)
project(tutorial_ros2_motor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  include
)

# 라이브러리: Controller, Kinematics
add_library(tutorial_ros2_motor_core
  src/controller.cpp
  src/kinematics.cpp
  src/serial_com.cpp
)
target_include_directories(tutorial_ros2_motor_core PUBLIC include Boost::system)
# core는 ROS에 직접 의존하진 않지만, include 경로 통일을 위해 ament deps는 생략 가능
ament_target_dependencies(tutorial_ros2_motor_core
  rclcpp
)

# 실행 노드
add_executable(tutorial_motor_node
  src/tutorial_motor_node.cpp
)
target_include_directories(tutorial_motor_node PUBLIC include)
target_link_libraries(tutorial_motor_node
  tutorial_ros2_motor_core
)
ament_target_dependencies(tutorial_motor_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
)

install(TARGETS
  tutorial_ros2_motor_core
  tutorial_motor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
