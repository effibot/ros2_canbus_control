cmake_minimum_required(VERSION 3.8)
project(ros2_waveshare)

# ==============================================================================
# Compiler Settings
# ==============================================================================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2 -Wno-unused-parameter)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ==============================================================================
# Find Dependencies
# ==============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

# Dependencies list
set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  diagnostic_updater
  lifecycle_msgs
)

# ==============================================================================
# Waveshare Library
# ==============================================================================
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/lib/waveshare_cpp/CMakeLists.txt)
  add_subdirectory(lib/waveshare_cpp)
  set(WAVESHARE_FOUND TRUE)
  set(WAVESHARE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/lib/waveshare_cpp/include)
  set(WAVESHARE_LIBRARIES waveshare_cpp waveshare_canopen)
elseif(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/lib/include/waveshare.hpp)
  set(WAVESHARE_FOUND TRUE)
  set(WAVESHARE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/lib/include)
  file(GLOB_RECURSE WAVESHARE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/lib/src/*.cpp)
  add_library(waveshare_cpp STATIC ${WAVESHARE_SOURCES})
  target_include_directories(waveshare_cpp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/lib/include)
  set(WAVESHARE_LIBRARIES waveshare_cpp)
else()
  message(FATAL_ERROR "waveshare_cpp library not found in lib/ directory")
endif()

include_directories(
  include
  ${WAVESHARE_INCLUDE_DIRS}
)

# ==============================================================================
# Lifecycle Node Executable
# ==============================================================================
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

ament_target_dependencies(canopen_lifecycle_node ${dependencies})

# ==============================================================================
# Test Scripts (Optional)
# ==============================================================================
add_executable(test_motor_communication
  scripts/test_motor_communication.cpp
)

target_include_directories(test_motor_communication PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${WAVESHARE_INCLUDE_DIRS}
)

target_link_libraries(test_motor_communication
  ${WAVESHARE_LIBRARIES}
)

# ==============================================================================
# Testing (Modular with Catch2)
# ==============================================================================
if(BUILD_TESTING)
  add_subdirectory(test)
  
  # Ament linting
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ==============================================================================
# Installation
# ==============================================================================
install(TARGETS canopen_lifecycle_node test_motor_communication
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch 
  config 
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# ==============================================================================
# Ament Package Export
# ==============================================================================
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()