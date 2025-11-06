# Test Configuration for ros2_waveshare CANopen stack
# Uses Catch2 for unit and integration testing

include(CTest)

# Set CMake policy for linker search path behavior
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# Find or fetch Catch2
find_package(Catch2 3 QUIET)

if(NOT Catch2_FOUND)
    message(STATUS "Catch2 not found, fetching from GitHub...")
    include(FetchContent)
    FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG v3.5.0  # Use latest stable version
    )
    FetchContent_MakeAvailable(Catch2)
endif()

# Find all test source files
file(GLOB TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp")

# List to track test names for installation
set(TEST_NAMES "")

# Register each test with Catch2
include(Catch)
foreach(test_src ${TEST_SOURCES})
    get_filename_component(test_name ${test_src} NAME_WE)
    
    # Create test executable
    add_executable(${test_name} ${test_src})
    
    # Link with Catch2, canopen_core, and nlohmann_json
    target_link_libraries(${test_name} PRIVATE
        Catch2::Catch2WithMain
        canopen_core
        nlohmann_json::nlohmann_json
    )
    
    # Include directories
    target_include_directories(${test_name} PRIVATE
        ${PROJECT_SOURCE_DIR}/include
    )
    
    # Discover Catch2 tests automatically
    catch_discover_tests(${test_name})
    
    # Add to list for installation
    list(APPEND TEST_NAMES ${test_name})
    
    message(STATUS "Registered test: ${test_name}")
endforeach()

# Install test executables (for ros2 run)
if(TEST_NAMES)
    install(TARGETS ${TEST_NAMES}
        DESTINATION lib/${PROJECT_NAME}
    )
endif()

