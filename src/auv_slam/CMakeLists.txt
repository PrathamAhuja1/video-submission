cmake_minimum_required(VERSION 3.8)
project(auv_slam)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python package
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
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()