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

set(MUJOCO_HINT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/mujoco-3.3.0")

find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h
  HINTS
    $ENV{MUJOCO_DIR}/include
    ${MUJOCO_HINT_DIR}/include
    /usr/local/include
    /usr/include
)

find_library(MUJOCO_LIBRARY NAMES mujoco libmujoco
  HINTS
    $ENV{MUJOCO_DIR}/lib
    ${MUJOCO_HINT_DIR}/lib
    /usr/local/lib
    /usr/lib
)

if (MUJOCO_INCLUDE_DIR AND MUJOCO_LIBRARY)
  message(STATUS "MuJoCo found: ${MUJOCO_INCLUDE_DIR}, ${MUJOCO_LIBRARY}")
  include_directories(${MUJOCO_INCLUDE_DIR})

  add_library(mujoco_sim_obj OBJECT
    src/mujoco_sim.cpp
  )
  target_include_directories(mujoco_sim_obj PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

  add_executable(mujoco_sim_node
    src/mujoco_sim_node.cpp
    $<TARGET_OBJECTS:mujoco_sim_obj>
  )
  ament_target_dependencies(mujoco_sim_node
    rclcpp geometry_msgs nav_msgs sensor_msgs tf2_ros ament_index_cpp)

  # 런타임에 libmujoco.so를 찾게 하려면(선택) RPATH 또는 LD_LIBRARY_PATH 중 택1
  # 여기서는 RPATH도 설정해둡니다.
  target_link_libraries(mujoco_sim_node ${MUJOCO_LIBRARY})
  set_target_properties(mujoco_sim_node PROPERTIES
    BUILD_RPATH "${MUJOCO_HINT_DIR}/lib"
    INSTALL_RPATH "${MUJOCO_HINT_DIR}/lib"
  )

  install(TARGETS mujoco_sim_node
    DESTINATION lib/${PROJECT_NAME})

  install(FILES resource/mobile_robot.xml
    DESTINATION share/${PROJECT_NAME}/resource)

else()
  message(WARNING "MuJoCo NOT found. Skipping mujoco_sim_node build.")
endif()

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
