cmake_minimum_required(VERSION 3.8)
project(auv_slam)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Python3 COMPONENTS Development REQUIRED)

# Find pybind11
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

# Python installation
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/gate_detector.py
  scripts/gate_navigator.py
  scripts/pwm_mapper.py
  scripts/safety_monitor.py
  scripts/serial_bridge.py
  scripts/ball_detector.py
  scripts/ball_follower.py
  scripts/heave_pid_controller.py
  scripts/cmd_mixer.py
  scripts/test.py
  scripts/ros_gst_udp.py
  scripts/test_2.py
  scripts/depth_maintain.py
  scripts/qr_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# 1. Create the target first
pybind11_add_module(heave_pid_rt_py
  pid_cpp/wrapper.cpp
  pid_cpp/heave_pid_rt.cpp
)

# 2. Then apply properties to it
target_compile_features(heave_pid_rt_py PUBLIC cxx_std_17)
target_include_directories(heave_pid_rt_py PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/pid_cpp>
)


install(TARGETS heave_pid_rt_py
  LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}/auv_slam
)

install(DIRECTORY
  pid_cpp/
  DESTINATION include/${PROJECT_NAME}/pid_cpp
  FILES_MATCHING PATTERN "*.h"
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()