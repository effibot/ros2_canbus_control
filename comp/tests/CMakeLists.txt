cmake_minimum_required(VERSION 3.16)
project(usb_can_frames_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(FetchContent)
FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG v3.5.3 # update as needed
)
FetchContent_MakeAvailable(Catch2)

# Interface library exposing headers from parent directory
add_library(usb_can_frames INTERFACE)
target_include_directories(usb_can_frames INTERFACE ${CMAKE_CURRENT_LIST_DIR}/..)

add_executable(test_frames
  test_fixed_variable_frames.cpp
)

target_link_libraries(test_frames PRIVATE usb_can_frames Catch2::Catch2WithMain)

include(CTest)
include(Catch)
catch_discover_tests(test_frames)

message(STATUS "Configured usb_can_frames_tests with Catch2")