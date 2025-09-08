cmake_minimum_required(VERSION 3.8)
project(ros2_waveshare)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(include)

# USB-CAN Driver Library
add_library(usb_can_driver 
  src/usb_can_driver.cpp
  src/fixed_size_frame.cpp
  src/variable_size_frame.cpp
  src/frame_factory.cpp)
target_include_directories(usb_can_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(usb_can_driver rclcpp)

# SocketCAN Bridge Node (commented out - source file missing)
# add_executable(usb_can_socketcan_bridge src/usb_can_socketcan_bridge.cpp)
# target_link_libraries(usb_can_socketcan_bridge usb_can_driver)
# ament_target_dependencies(usb_can_socketcan_bridge 
#   rclcpp 
#   std_msgs
#   geometry_msgs)

# Simple test nodes (commented out - source files are empty)
# add_executable(can_frame_publisher src/can_frame_publisher.cpp)
# target_link_libraries(can_frame_publisher usb_can_driver)
# ament_target_dependencies(can_frame_publisher rclcpp std_msgs)

# add_executable(can_frame_subscriber src/can_frame_subscriber.cpp)
# target_link_libraries(can_frame_subscriber usb_can_driver)
# ament_target_dependencies(can_frame_subscriber rclcpp std_msgs)


# Install targets
install(TARGETS
  # usb_can_socketcan_bridge  # commented out - source file missing
  # can_frame_publisher       # commented out - source file empty
  # can_frame_subscriber      # commented out - source file empty
  test_frames
  test_portability
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  usb_can_driver
  DESTINATION lib)

# install(DIRECTORY
  # launch config scripts
  # DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
