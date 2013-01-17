# This file is a part of the OpenSurgSim project.
# Copyright 2012-2013, SimQuest LLC.
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

if(NOT SURGSIM_TOP_INCLUDE_DIR)
	set(SURGSIM_TOP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
	include_directories(${SURGSIM_TOP_INCLUDE_DIR})
	set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${SURGSIM_TOP_INCLUDE_DIR}/CMake)
endif(NOT SURGSIM_TOP_INCLUDE_DIR)

include(SurgSimBuildFlags)
include(SurgSimUtilities)

### Find required libraries

#find_package(OpenSceneGraph COMPONENTS osg osgViewer osgText osgUtil osgDB osgGA osgAnimation REQUIRED)
#find_package(OpenGL)
#find_package(Qt4 COMPONENTS QtCore)
#find_package(OpenThreads)
find_package(Eigen3 REQUIRED)


option(SURGSIM_TESTS_BUILD "Include the unit tests in the build" ON)
if(SURGSIM_TESTS_BUILD)
	add_subdirectory(ThirdParty/gtest)
	set(gtest_force_shared_crt ON CACHE BOOL "Use shared CRT for Google Test [OpenSurgSim forces this to true!]" FORCE)
endif()

add_subdirectory(SurgSim)

#option(SURGSIM_EXAMPLES_BUILD "Build the example files" ON)
