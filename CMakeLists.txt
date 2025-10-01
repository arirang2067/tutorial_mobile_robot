cmake_minimum_required(VERSION 3.16)
project(tutorial_mobile_robot)

# ─────────────────────────────────────────────────────────────────────────────
# 기본 컴파일 설정
# ─────────────────────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
# GCC 10+ 에서 나오는 C++17/ABI 안내 note를 숨기고 싶다면 아래 주석 해제
# add_compile_options(-Wno-psabi)

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
# 경로(헤더/리소스는 assets 하나만 사용)
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
ament_target_dependencies(tutorial_mobile_robot_core rclcpp)

# ─────────────────────────────────────────────────────────────────────────────
# 실행 노드: mobile_robot_node
# ─────────────────────────────────────────────────────────────────────────────
add_executable(mobile_robot_node
  src/mobile_robot_node.cpp
)
target_include_directories(mobile_robot_node PUBLIC
  $<BUILD_INTERFACE:${PROJ_INCLUDE}>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mobile_robot_node
  tutorial_mobile_robot_core
)
ament_target_dependencies(mobile_robot_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
)

# ─────────────────────────────────────────────────────────────────────────────
# 시리얼 브리지 노드 (토픽→시리얼)
# ─────────────────────────────────────────────────────────────────────────────
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/serial_bridge_node.cpp")
  add_executable(serial_bridge_node
    src/serial_bridge_node.cpp
  )
  target_include_directories(serial_bridge_node PUBLIC
    $<BUILD_INTERFACE:${PROJ_INCLUDE}>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(serial_bridge_node
    tutorial_mobile_robot_core
  )
  ament_target_dependencies(serial_bridge_node
    rclcpp
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    tf2
    tf2_ros
  )
  install(TARGETS serial_bridge_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(WARNING "serial_bridge_node.cpp not found. Skipping serial_bridge_node build.")
endif()

# ─────────────────────────────────────────────────────────────────────────────
# MuJoCo: 패키지 내부 고정 — aarch64/arm64 에서는 기본적으로 빌드 생략
# ─────────────────────────────────────────────────────────────────────────────
# 사용자가 강제 켤 수 있게 옵션 제공 (기본 ON). 단, aarch64면 자동 OFF 처리.
option(ENABLE_MUJOCO "Build MuJoCo simulator targets" ON)

# 현재 아키텍처 확인
string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" _ARCH_LC)
if(_ARCH_LC MATCHES "aarch64" OR _ARCH_LC MATCHES "arm64")
  if(ENABLE_MUJOCO)
    message(WARNING "Detected ${CMAKE_SYSTEM_PROCESSOR}. For safety, MuJoCo build will be disabled on aarch64/arm64.")
  endif()
  set(ENABLE_MUJOCO OFF)
endif()

if(ENABLE_MUJOCO)
  # 소스 내 MuJoCo 루트
  set(MUJOCO_SRC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/mujoco-3.3.0")
  set(MUJOCO_INSTALL_ROOT "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/mujoco-3.3.0")

  # 헤더/라이브러리 탐색
  find_path(MUJOCO_INCLUDE_DIR
    NAMES mujoco/mujoco.h mujoco.h
    HINTS
      ${MUJOCO_SRC_ROOT}
      ${MUJOCO_SRC_ROOT}/include
    PATH_SUFFIXES
      include
      include/mujoco
  )
  find_library(MUJOCO_LIBRARY
    NAMES
      mujoco
      libmujoco.so.3
      libmujoco.so.3.3.0
    HINTS
      ${MUJOCO_SRC_ROOT}
      ${MUJOCO_SRC_ROOT}/lib
    PATH_SUFFIXES lib
  )

  if(MUJOCO_INCLUDE_DIR AND MUJOCO_LIBRARY)
    message(STATUS "MuJoCo found (in source tree):")
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
    target_link_libraries(mujoco_sim_node
      ${MUJOCO_LIBRARY}
    )
    set_target_properties(mujoco_sim_node PROPERTIES
      BUILD_RPATH "${MUJOCO_SRC_ROOT}/lib"
      INSTALL_RPATH "\$ORIGIN/../../share/${PROJECT_NAME}/mujoco-3.3.0/lib"
    )
    install(TARGETS mujoco_sim_node
      RUNTIME DESTINATION lib/${PROJECT_NAME}
    )

    # (선택) 패키지와 함께 MuJoCo 배포 — aarch64에서는 아예 생략되므로 여기서만 설치
    if(EXISTS "${MUJOCO_SRC_ROOT}")
      install(DIRECTORY ${MUJOCO_SRC_ROOT}/
        DESTINATION share/${PROJECT_NAME}/mujoco-3.3.0
      )
    endif()
  else()
    message(WARNING "MuJoCo NOT found in ${MUJOCO_SRC_ROOT}.
    패키지 내부에 mujoco-3.3.0/include 및 lib이 있는지 확인하세요.")
  endif()
else()
  message(STATUS "ENABLE_MUJOCO=OFF → MuJoCo targets are skipped on this platform (${CMAKE_SYSTEM_PROCESSOR}).")
endif()

# ─────────────────────────────────────────────────────────────────────────────
# /robot_description 토픽 퍼블리셔 — RViz Topic 모드용
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
  install(TARGETS robot_description_topic_publisher
    RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

# ─────────────────────────────────────────────────────────────────────────────
# 설치(install)
# ─────────────────────────────────────────────────────────────────────────────
install(TARGETS
  tutorial_mobile_robot_core
  mobile_robot_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY assets/
  DESTINATION share/${PROJECT_NAME}/assets
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
