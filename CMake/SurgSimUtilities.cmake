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


## CMake support for unit test building and running during the build.

# options related to unit tests
#
option(SURGSIM_TESTS_BUILD "Include the unit tests in the build" ON)
option(SURGSIM_TESTS_RUN "Run the unit tests at the end of the build" ON)
option(SURGSIM_TESTS_ALL_IN_ONE
	"Build a single binary with all unit tests.  [Does not work yet!]" OFF)
mark_as_advanced(SURGSIM_TESTS_ALL_IN_ONE)  # hide it as long as it's broken

# Builds the unit test executable or library (unless disabled).
# Does not try to run the test.
# Uses UNIT_TEST_SOURCES, UNIT_TEST_HEADERS and LIBS.
#
# You probably want to use surgsim_add_unit_tests(TESTNAME) intead.
#
macro(surgsim_unit_test_build_only TESTNAME)
	if(SURGSIM_TESTS_ALL_IN_ONE)
		add_library(${TESTNAME} ${UNIT_TEST_SOURCES} ${UNIT_TEST_HEADERS})
		target_link_libraries(${TESTNAME} ${LIBS})
		# NB: There's currently no way to pick up these libs and combine them.
		#     So this option does not currently do anything useful...
	else()
		add_executable(${TESTNAME} ${UNIT_TEST_SOURCES} ${UNIT_TEST_HEADERS})
		target_link_libraries(${TESTNAME} gtest_main ${LIBS})
	endif()
endmacro()

# Run the unit test executable (unless disabled or built all-in-one).
# Expects that the unit test build has already been set up via
#   surgsim_unit_test_build_only(TESTNAME).
# Uses the SURGSIM_TESTS_BUILD and SURGSIM_TESTS_ALL_IN_ONE options.
#
# You probably want to use surgsim_add_unit_tests(TESTNAME) intead.
#
macro(surgsim_unit_test_run_only TESTNAME)
	if(NOT SURGSIM_TESTS_ALL_IN_ONE)
		add_custom_command(TARGET ${TESTNAME} POST_BUILD COMMAND ${TESTNAME})
	endif()
endmacro()

# Build the unit test executable or library.
# The optional arguments can be used to turn off building and running
#   tests even when enabled by the corresponding global options.  (If
#   the options are off, they *do not* turn things back on!)
# Uses UNIT_TEST_SOURCES, UNIT_TEST_HEADERS and LIBS for this unit
#   test, as well as the SURGSIM_TESTS_BUILD and ..._RUN options.
#
# Note the use of optional arguments:
#   surgsim_add_unit_tests(<testname> [BUILD <boolean>] [RUN <boolean>])
#
macro(surgsim_add_unit_tests TESTNAME)
	set(SURGSIM_TESTS_building ${SURGSIM_TESTS_BUILD})
	set(SURGSIM_TESTS_running ${SURGSIM_TESTS_RUN})

	# First, parse the optional arguments
	set(ARGS_TO_PROCESS ${ARGN})
	while(1)
		list(LENGTH ARGS_TO_PROCESS NUM_ARGS_TO_PROCESS)
		if(NUM_ARGS_TO_PROCESS EQUAL 0)
			break()   # we're done here
		endif()
		list(GET ARGS_TO_PROCESS 0 ARG_CURRENT)
		#message("SSAUT: processing extra arg '${ARG_CURRENT}' ...")

		if(ARG_CURRENT STREQUAL "BUILD")
			if(NUM_ARGS_TO_PROCESS LESS 2)
				message(SEND_ERROR
					"surgsim_add_unit_tests(...BUILD <bool>...) is missing the argument!")
				break()
			endif(NUM_ARGS_TO_PROCESS LESS 2)
			list(GET ARGS_TO_PROCESS 1 SURGSIM_TESTS_building)
			#message("SSAUT: BUILDING set to ${SURGSIM_TESTS_building}")
			list(REMOVE_AT ARGS_TO_PROCESS 0 1)

		elseif(ARG_CURRENT STREQUAL "RUN")
			if(NUM_ARGS_TO_PROCESS LESS 2)
				message(SEND_ERROR
					"surgsim_add_unit_tests(...RUN <bool>...) is missing the argument!")
				break()
			endif(NUM_ARGS_TO_PROCESS LESS 2)
			list(GET ARGS_TO_PROCESS 1 SURGSIM_TESTS_running)
			#message("SSAUT: RUNNING set to ${SURGSIM_TESTS_running}")
			list(REMOVE_AT ARGS_TO_PROCESS 0 1)

		else()
			message(SEND_ERROR
				"surgsim_add_unit_tests(... '${ARG_CURRENT}' ...): bad argument!")
			break()
		endif()
	endwhile()

	# OK, we're ready to set up building and running tests
	if(SURGSIM_TESTS_building)
		surgsim_unit_test_build_only("${TESTNAME}")
		if(SURGSIM_TESTS_running)
			surgsim_unit_test_run_only("${TESTNAME}")
		endif()
	endif()
endmacro()
