# This file is a part of the OpenSurgSim project.
# Copyright 2012-2013, SimQuest Solutions Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


cmake_minimum_required(VERSION 2.6)
project(OpenSurgSim)

set(OPENSURGSIM_MAJOR_VERSION 0)
set(OPENSURGSIM_MINOR_VERSION 0)
set(OPENSURGSIM_PATCH_VERSION 0)
set(OPENSURGSIM_VERSION
  ${OPENSURGSIM_MAJOR_VERSION}.${OPENSURGSIM_MINOR_VERSION}.${OPENSURGSIM_PATCH_VERSION})

# Install Directories
set(INSTALL_LIB_DIR ${CMAKE_INSTALL_PREFIX}/lib)
set(INSTALL_BIN_DIR ${CMAKE_INSTALL_PREFIX}/bin)
set(INSTALL_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include)
if(WIN32 AND NOT CYGWIN)
  set(INSTALL_CMAKE_DIR ${CMAKE_INSTALL_PREFIX}/CMake)
else()
  set(INSTALL_CMAKE_DIR ${CMAKE_INSTALL_PREFIX}/lib/cmake/${PROJECT_NAME})
endif()

set(SURGSIM_TOP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${SURGSIM_TOP_INCLUDE_DIR})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMake)
set(SURGSIM_TOOLS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Tools")
set(SURGSIM_THIRD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty")

include(SurgSimBuildFlags)
include(SurgSimUtilities)

### Find required libraries

#find_package(OpenSceneGraph COMPONENTS osg osgViewer osgText osgUtil osgDB osgGA osgAnimation REQUIRED)
#find_package(Qt4 COMPONENTS QtCore)
#find_package(OpenThreads)
find_package(OpenGL)
find_package(Eigen3 REQUIRED)

find_package(GLUT)  # only needed for visual device tests
find_package(GlutFromOsg)  # look for GLUT in the OSG tree if not found above

set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.50 COMPONENTS thread system chrono filesystem REQUIRED)
mark_as_advanced(Boost_DIR)

find_package(OpenHaptics)
find_package(SixenseSdk)


if(SURGSIM_TESTS_BUILD)
	set(gtest_force_shared_crt ON CACHE BOOL "Use shared CRT for Google Test [OpenSurgSim forces this to true!]" FORCE)
	add_subdirectory(ThirdParty/gtest)
endif()



SET(EXPORT_TARGETS CACHE INTERNAL "export targets")

add_subdirectory(SurgSim)
add_subdirectory(Documentation)

if(SURGSIM_EXAMPLES_BUILD)
	add_subdirectory(Examples)
endif()

export(TARGETS ${EXPORT_TARGETS} 
	FILE "${PROJECT_BINARY_DIR}/OpenSurgSimTargets.cmake")

#Register where to find this build-tree. On windows this in the registry, on
#unix this goes in ~/.cmake/packages/OpenSurgSim
export(PACKAGE OpenSurgSim)

#OpenSurgSimConfig For the Build Tree
set(CONFIG_INCLUDE_DIRS "${SURGSIM_TOP_INCLUDE_DIR}")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/CMake/OpenSurgSimConfig.cmake.in
	"${PROJECT_BINARY_DIR}/OpenSurgSimConfig.cmake" @ONLY)

#OpenSurgSimConfig For the Install Tree
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}" "${CMAKE_INSTALL_PREFIX}/include")
set(CONFIG_INCLUDE_DIRS "\${OPENSURGSIM_CMAKE_DIR}/${REL_INCLUDE_DIR}")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/CMake/OpenSurgSimConfig.cmake.in
	"${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/OpenSurgSimConfig.cmake" @ONLY)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/CMake/OpenSurgSimConfigVersion.cmake.in
	"${PROJECT_BINARY_DIR}/OpenSurgSimConfigVersion.cmake" @ONLY)

install(FILES
	"${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/OpenSurgSimConfig.cmake"
	"${PROJECT_BINARY_DIR}/OpenSurgSimConfigVersion.cmake"
	DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)
install(EXPORT ${PROJECT_NAME}Targets DESTINATION ${INSTALL_CMAKE_DIR} COMPONENT dev)

#install(EXPORT ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

#option(SURGSIM_EXAMPLES_BUILD "Build the example files" ON)
