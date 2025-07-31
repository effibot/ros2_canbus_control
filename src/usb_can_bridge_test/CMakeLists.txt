cmake_minimum_required(VERSION 3.8)
project(usb_can_bridge_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# USB-CAN Driver Library
add_library(usb_can_driver src/usb_can_driver.cpp)
target_include_directories(usb_can_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(usb_can_driver rclcpp)

# SocketCAN Bridge Node
add_executable(usb_can_socketcan_bridge src/usb_can_socketcan_bridge.cpp)
target_link_libraries(usb_can_socketcan_bridge usb_can_driver)
ament_target_dependencies(usb_can_socketcan_bridge 
  rclcpp 
  std_msgs
  geometry_msgs)

# Virtual Motor Simulator (using second USB-CAN adapter)
add_executable(virtual_motor_simulator src/virtual_motor_simulator.cpp)
target_link_libraries(virtual_motor_simulator usb_can_driver)
ament_target_dependencies(virtual_motor_simulator 
  rclcpp 
  std_msgs
  geometry_msgs
  sensor_msgs)

# Turtle Bridge (converts CAN motor feedback to turtle movement)
add_executable(turtle_bridge src/turtle_bridge.cpp)
ament_target_dependencies(turtle_bridge 
  rclcpp 
  geometry_msgs
  sensor_msgs
  turtlesim
  tf2
  tf2_ros
  tf2_geometry_msgs)

# Simple test nodes
add_executable(can_frame_publisher src/can_frame_publisher.cpp)
target_link_libraries(can_frame_publisher usb_can_driver)
ament_target_dependencies(can_frame_publisher rclcpp std_msgs)

add_executable(can_frame_subscriber src/can_frame_subscriber.cpp)
target_link_libraries(can_frame_subscriber usb_can_driver)
ament_target_dependencies(can_frame_subscriber rclcpp std_msgs)

# Install targets
install(TARGETS
  usb_can_socketcan_bridge
  virtual_motor_simulator
  turtle_bridge
  can_frame_publisher
  can_frame_subscriber
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  usb_can_driver
  DESTINATION lib)

install(DIRECTORY
  launch config scripts
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
