cmake_minimum_required(VERSION 3.8)
project(ros2_microphase)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2 -Wno-unused-parameter)
endif()

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# Optional: Only if you're using ros2_canopen features in this package
# find_package(canopen_core REQUIRED)
# find_package(canopen_interfaces REQUIRED)
# find_package(canopen_base_driver REQUIRED)
# find_package(canopen_proxy_driver REQUIRED)
# find_package(lely_core_libraries REQUIRED)

# Include directories
include_directories(
  include
)

# Find waveshare_cpp library
# Option 1: If waveshare_cpp is in lib/ subdirectory as a CMake project
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/lib/waveshare_cpp/CMakeLists.txt)
  add_subdirectory(lib/waveshare_cpp)
  set(WAVESHARE_FOUND TRUE)
  set(WAVESHARE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/lib/waveshare_cpp/include)
  set(WAVESHARE_LIBRARIES waveshare_cpp)
# Option 2: If waveshare_cpp is installed as a system package
elseif(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/lib/include/waveshare.hpp)
  set(WAVESHARE_FOUND TRUE)
  set(WAVESHARE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/lib/include)
  # Build library from source files
  file(GLOB_RECURSE WAVESHARE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/lib/src/*.cpp)
  add_library(waveshare_cpp STATIC ${WAVESHARE_SOURCES})
  target_include_directories(waveshare_cpp PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/include
  )
  set(WAVESHARE_LIBRARIES waveshare_cpp)
else()
  message(FATAL_ERROR "waveshare_cpp library not found in lib/ directory")
endif()

include_directories(${WAVESHARE_INCLUDE_DIRS})

# Dependencies list
set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  diagnostic_updater
  lifecycle_msgs
)

# Build lifecycle node executable
add_executable(canopen_lifecycle_node
  src/canopen_lifecycle.cpp
  src/canopen_lifecycle_main.cpp
)

target_include_directories(canopen_lifecycle_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${WAVESHARE_INCLUDE_DIRS}
)

target_link_libraries(canopen_lifecycle_node
  ${WAVESHARE_LIBRARIES}
)

ament_target_dependencies(canopen_lifecycle_node
  ${dependencies}
)

# Install executable
install(TARGETS canopen_lifecycle_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY 
  launch 
  config 
  DESTINATION share/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()