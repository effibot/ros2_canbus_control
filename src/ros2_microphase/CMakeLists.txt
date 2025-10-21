cmake_minimum_required(VERSION 3.8)
project(ros2_microphase)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(canopen_core REQUIRED)
find_package(canopen_interfaces REQUIRED)
find_package(canopen_base_driver REQUIRED)
find_package(canopen_proxy_driver REQUIRED)
find_package(lely_core_libraries REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(dependencies
rclcpp
rclcpp_components
rclcpp_lifecycle
lifecycle_msgs
canopen_core
canopen_interfaces
canopen_base_driver
canopen_proxy_driver
lely_core_libraries
std_msgs
std_srvs
)

generate_dcf(can_chain)

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}
)

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
