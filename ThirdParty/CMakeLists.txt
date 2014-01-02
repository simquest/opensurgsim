cmake_minimum_required(VERSION 2.8)
project(OpenSurgSimThirdParty)

# Install Directories
set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "")
set(INSTALL_LIB_DIR ${CMAKE_INSTALL_PREFIX}/lib)
set(INSTALL_BIN_DIR ${CMAKE_INSTALL_PREFIX}/bin)
set(INSTALL_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include)

# Use SurgSim CMake modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/../CMake")

# Compile with the same build flags as SurgSim
include(SurgSimBuildFlags)

set(gtest_force_shared_crt ON CACHE BOOL "Use shared CRT for Google Test [OpenSurgSim forces this to true!]" FORCE)
add_subdirectory(gmock)

add_subdirectory(yaml-cpp)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "Disable testing and parse tools" FORCE)