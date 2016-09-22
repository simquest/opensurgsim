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

cmake_minimum_required(VERSION 2.8)

##############################################################################
#
# Copy this script onto the dashboard machine and then customize the variables
# OSS_* below. Run the script using the command
#
#   ctest -VV -O <logfile> -S <script>
#
# or for generators that require a configuration to be chosen
#
#   ctest -VV -O <logfile> -S <script> -C <configuration>
#
# Run ctest --help for more options.
#
##############################################################################

set(OSS_SOURCE_NAME "OpenSurgSim")
set(OSS_BUILD_TYPE "Experimental") # Continuous, Experimental, or Nightly
set(OSS_BUILD_CONFIGURATION "Debug")
set(OSS_CMAKE_GENERATOR "Unix Makefiles")
set(OSS_DASHBOARD_ROOT "$ENV{HOME}/Dashboards/OpenSurgSim")
set(OSS_CXX_COMPILER "g++")
set(OSS_CXX_COMPILER_VERSION "4.8")
set(OSS_ARCHITECTURE "x86_64")
set(OSS_GIT_REPOSITORY "git://git.assembla.com/OpenSurgSim.git")
set(OSS_WITH_COVERAGE TRUE)
set(OSS_COVERAGE_EXTRA_FLAGS "-g -fprofile-arcs -ftest-coverage")
set(OSS_WITH_MEMCHECK TRUE)
set(OSS_INITIAL_CACHE "")

##############################################################################

set(CTEST_BUILD_CONFIGURATION "${OSS_BUILD_CONFIGURATION}")
set(CTEST_CMAKE_GENERATOR "${OSS_CMAKE_GENERATOR}")

set(CTEST_SOURCE_NAME "${OSS_SOURCE_NAME}")
set(CTEST_BINARY_NAME "${CTEST_SOURCE_NAME}-${CTEST_BUILD_CONFIGURATION}")

set(CTEST_DASHBOARD_ROOT "${OSS_DASHBOARD_ROOT}")
set(CTEST_SOURCE_DIRECTORY "${CTEST_DASHBOARD_ROOT}/${CTEST_SOURCE_NAME}")
set(CTEST_BINARY_DIRECTORY "${CTEST_DASHBOARD_ROOT}/${CTEST_BINARY_NAME}")

cmake_host_system_information(RESULT CTEST_SITE QUERY FQDN)
set(CTEST_BUILD_NAME "${CMAKE_SYSTEM}-${OSS_CXX_COMPILER}-${OSS_CXX_COMPILER_VERSION}-${OSS_ARCHITECTURE}-${CTEST_BUILD_CONFIGURATION}")

find_program(CTEST_GIT_COMMAND NAMES git)

if(NOT EXISTS "${CTEST_SOURCE_DIRECTORY}")
	set(CTEST_CHECKOUT_COMMAND "${CTEST_GIT_COMMAND} clone -b CTestCDash ${OSS_GIT_REPOSITORY} ${CTEST_SOURCE_DIRECTORY}")
endif()

set(CTEST_UPDATE_COMMAND "${CTEST_GIT_COMMAND}")

if(NOT "${CTEST_SOURCE_DIRECTORY}" STREQUAL "${CTEST_BINARY_DIRECTORY}")
	ctest_empty_binary_directory("${CTEST_BINARY_DIRECTORY}")
endif()

if(OSS_WITH_COVERAGE)
	find_program(CTEST_COVERAGE_COMMAND NAMES gcov llvm-cov)

	if(CTEST_COVERAGE_COMMAND)
		set(OSS_INITIAL_CACHE "
${OSS_INITIAL_CACHE}
CMAKE_CXX_FLAGS:STRING=${OSS_COVERAGE_EXTRA_FLAGS} ${CMAKE_CXX_FLAGS}
CMAKE_C_FLAGS:STRING=${OSS_COVERAGE_EXTRA_FLAGS} ${CMAKE_C_FLAGS}
")
	endif()
endif()

if(OSS_WITH_MEMCHECK)
	find_program(CTEST_MEMORYCHECK_COMMAND NAMES valgrind purify)

	if(CTEST_MEMORYCHECK_COMMAND)
		set(VALGRIND_COMMAND_OPTIONS "-q --tool=memcheck --leak-check=yes --num-callers=50")
	endif()
endif()

file(WRITE "${CTEST_BINARY_DIRECTORY}/CMakeCache.txt" "${OSS_INITIAL_CACHE}")
set(CTEST_NOTES_FILES "${CTEST_BINARY_DIRECTORY}/CMakeCache.txt")

ctest_start(${OSS_BUILD_TYPE})

message("-------------------- [ Update ] --------------------")
ctest_update()

message("-------------------- [ Configure ] --------------------")
ctest_configure()
ctest_submit(PARTS Update Configure)

message("-------------------- [ Build ] --------------------")
ctest_read_custom_files("${CTEST_BINARY_DIRECTORY}")
ctest_build(APPEND)
ctest_submit(PARTS Build)

message("-------------------- [ Test ] --------------------")
ctest_test(APPEND)
ctest_submit(PARTS Test)

if(OSS_WITH_COVERAGE AND CTEST_COVERAGE_COMMAND)
	message("-------------------- [ Coverage ] --------------------")
	ctest_coverage()
	ctest_submit(PARTS Coverage)
endif()

if(OSS_WITH_MEMCHECK AND CTEST_MEMORYCHECK_COMMAND)
	message("-------------------- [ MemCheck ] --------------------")
	set(ENV{GTEST_DEATH_TEST_USE_FORK} 1)
	ctest_memcheck()
	ctest_submit(PARTS MemCheck)
	unset(ENV{GTEST_DEATH_TEST_USE_FORK})
endif()

ctest_submit(PARTS Notes)
message("-------------------- [ Done ] --------------------")
