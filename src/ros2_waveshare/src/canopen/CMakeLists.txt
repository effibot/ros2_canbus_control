# CANopen Core Library
# Collects all CANopen implementation files

file(GLOB CANOPEN_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

add_library(canopen_core SHARED ${CANOPEN_SOURCES})

target_include_directories(canopen_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(canopen_core
  nlohmann_json::nlohmann_json
)

# Install library
install(TARGETS canopen_core
  EXPORT canopen_coreTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
