cmake_minimum_required(VERSION 3.16)
project(tutorial_mobile_robot)

# ─────────────────────────────────────────────────────────────────────────────
# 기본 컴파일 설정
# ─────────────────────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

# ─────────────────────────────────────────────────────────────────────────────
# ROS 2 / ament 의존
# ─────────────────────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ─────────────────────────────────────────────────────────────────────────────
# 경로(헤더/리소스는 하나의 assets 디렉터리만 사용)
# ─────────────────────────────────────────────────────────────────────────────
set(PROJ_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(PROJ_ASSETS  ${CMAKE_CURRENT_SOURCE_DIR}/assets)
include_directories(${PROJ_INCLUDE})

# ─────────────────────────────────────────────────────────────────────────────
# 코어 라이브러리 (ROS 비의존 로직)
# ─────────────────────────────────────────────────────────────────────────────
add_library(tutorial_mobile_robot_core
  src/controller.cpp
  src/kinematics.cpp
  src/serial_com.cpp
)
target_include_directories(tutorial_mobile_robot_core PUBLIC
  $<BUILD_INTERFACE:${PROJ_INCLUDE}>
  $<INSTALL_INTERFACE:include>
)
# 코어가 rclcpp를 직접 쓰지 않는다면 아래 줄은 제거 가능
ament_target_dependencies(tutorial_mobile_robot_core rclcpp)

# ─────────────────────────────────────────────────────────────────────────────
# 실행 노드: tutorial_motor_node
# ─────────────────────────────────────────────────────────────────────────────
add_executable(tutorial_motor_node
  src/tutorial_motor_node.cpp
)
target_include_directories(tutorial_motor_node PUBLIC
  $<BUILD_INTERFACE:${PROJ_INCLUDE}>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(tutorial_motor_node
  tutorial_mobile_robot_core
)
ament_target_dependencies(tutorial_motor_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
)

# ─────────────────────────────────────────────────────────────────────────────
# MuJoCo (수업 가정: 항상 HOME에 설치, 경로 고정 가능)
# ─────────────────────────────────────────────────────────────────────────────
# ── MuJoCo 탐색 (버전 파일까지 지원)
set(MUJOCO_DEFAULT_ROOT "$ENV{HOME}/mujoco-3.3.0")
set(MUJOCO_ROOT "${MUJOCO_DEFAULT_ROOT}" CACHE PATH "Path to MuJoCo root (contains include/ and lib/)")

find_path(MUJOCO_INCLUDE_DIR
  NAMES mujoco/mujoco.h mujoco.h
  HINTS
    ${MUJOCO_ROOT}
    ${MUJOCO_ROOT}/include
    /usr/local/include
    /usr/include
  PATH_SUFFIXES
    include
    include/mujoco
)

find_library(MUJOCO_LIBRARY
  NAMES
    mujoco             # libmujoco.so
    libmujoco.so.3     # SONAME 버전
    libmujoco.so.3.3.0 # 완전 버전
  HINTS
    ${MUJOCO_ROOT}
    ${MUJOCO_ROOT}/lib
    /usr/local/lib
    /usr/lib
  PATH_SUFFIXES lib
)

if(MUJOCO_INCLUDE_DIR AND MUJOCO_LIBRARY)
  message(STATUS "MuJoCo found:")
  message(STATUS "  include: ${MUJOCO_INCLUDE_DIR}")
  message(STATUS "  library: ${MUJOCO_LIBRARY}")

  add_library(mujoco_sim_obj OBJECT
    src/mujoco_sim.cpp
  )
  target_include_directories(mujoco_sim_obj PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${MUJOCO_INCLUDE_DIR}
  )

  add_executable(mujoco_sim_node
    src/mujoco_sim_node.cpp
    $<TARGET_OBJECTS:mujoco_sim_obj>
  )
  target_include_directories(mujoco_sim_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${MUJOCO_INCLUDE_DIR}
  )
  ament_target_dependencies(mujoco_sim_node
    rclcpp geometry_msgs nav_msgs sensor_msgs tf2_ros ament_index_cpp
  )
  # ★ plain 모드로 통일 (PRIVATE 키워드 사용 금지: ament_target_dependencies가 plain 모드 사용)
  target_link_libraries(mujoco_sim_node
    ${MUJOCO_LIBRARY}
  )

  # 런타임 탐색(RPATH)
  set_target_properties(mujoco_sim_node PROPERTIES
    BUILD_RPATH "${MUJOCO_ROOT}/lib"
    INSTALL_RPATH "\$ORIGIN;${MUJOCO_ROOT}/lib"
  )

  install(TARGETS mujoco_sim_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(WARNING "MuJoCo NOT found at ${MUJOCO_ROOT}.
  헤더는 include/(mujoco/)mujoco.h, 라이브러리는 lib/libmujoco.so* 경로에 있어야 합니다.
  실제 설치 경로를 -DMUJOCO_ROOT=/정확한/경로 로 지정하세요.")
endif()

# ─────────────────────────────────────────────────────────────────────────────
# (선택) /robot_description 토픽 퍼블리셔 — RViz Topic 모드용
# ─────────────────────────────────────────────────────────────────────────────
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/robot_description_topic_publisher.cpp")
  add_executable(robot_description_topic_publisher
    src/robot_description_topic_publisher.cpp
  )
  target_include_directories(robot_description_topic_publisher PUBLIC
    $<BUILD_INTERFACE:${PROJ_INCLUDE}>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(robot_description_topic_publisher rclcpp std_msgs)
endif()

# ─────────────────────────────────────────────────────────────────────────────
# 설치(install) — 리소스는 오직 assets 디렉터리만 사용
# ─────────────────────────────────────────────────────────────────────────────
install(TARGETS
  tutorial_mobile_robot_core
  tutorial_motor_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(TARGET mujoco_sim_node)
  install(TARGETS mujoco_sim_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(TARGET robot_description_topic_publisher)
  install(TARGETS robot_description_topic_publisher
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY include/
  DESTINATION include
)

# ★ 리소스 전부 assets로 통일
install(DIRECTORY assets/ DESTINATION share/${PROJECT_NAME}/assets)

# 런치 파일을 별도 디렉터리로 유지하는 경우
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
