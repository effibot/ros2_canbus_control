cmake_minimum_required(VERSION 3.8)
project(ros2_waveshare_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  "msg/MotorFeedback.msg"
  "msg/MotorCommand.msg"
  "msg/MotorStatus.msg"
  "msg/PDOStatistics.msg"
  
  # Services
  "srv/SDORead.srv"
  "srv/SDOWrite.srv"
  "srv/SetOperationMode.srv"
  "srv/GetMotorInfo.srv"
  
  # Actions
  "action/EnableMotor.action"
  "action/ResetFault.action"
  "action/MoveToPosition.action"
  
  DEPENDENCIES
    std_msgs
    sensor_msgs
    diagnostic_msgs
    action_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
