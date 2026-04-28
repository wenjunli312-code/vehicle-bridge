cmake_minimum_required(VERSION 3.16)
project(vehicle-bridge VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(common-utils REQUIRED)

add_library(vehicle_bridge STATIC
    src/can_message.cpp
    src/vehicle_controller.cpp
    src/vehicle_state.cpp
)

target_include_directories(vehicle_bridge
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(vehicle_bridge
    PUBLIC common_utils::common_utils
)

# Install headers
install(
    DIRECTORY include/vehicle_bridge/
    DESTINATION include/vehicle_bridge
)

# Install library
install(
    TARGETS vehicle_bridge
    EXPORT vehicle_bridgeTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install CMake targets
install(
    EXPORT vehicle_bridgeTargets
    FILE vehicle_bridgeTargets.cmake
    NAMESPACE vehicle_bridge::
    DESTINATION lib/cmake/vehicle_bridge
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/vehicle_bridgeConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/vehicle_bridgeConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/vehicle_bridgeConfig.cmake"
    INSTALL_DESTINATION lib/cmake/vehicle_bridge
)

install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/vehicle_bridgeConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/vehicle_bridgeConfigVersion.cmake"
    DESTINATION lib/cmake/vehicle_bridge
)

# Tests
enable_testing()
add_subdirectory(tests)
