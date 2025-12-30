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

# Find pybind11
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
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


install(DIRECTORY
  pid_cpp/
  DESTINATION include/${PROJECT_NAME}/pid_cpp
  FILES_MATCHING PATTERN "*.h"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()