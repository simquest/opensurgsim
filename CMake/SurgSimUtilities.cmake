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


## Miscellaneous CMake utility functions.

# Build the unit test executable or library.
# Uses UNIT_TEST_SOURCES, UNIT_TEST_HEADERS and LIBS.
macro(surgsim_add_unit_tests TESTNAME)
	option(SURGSIM_TESTS_ALL_IN_ONE "Build a single binary with all unit tests" OFF)
	option(SURGSIM_TESTS_RUN "Run the unit tests at the end of the build" ON)

	if(SURGSIM_TESTS_BUILD)
		if(SURGSIM_TESTS_ALL_IN_ONE)
			add_library(${TESTNAME} ${UNIT_TEST_SOURCES} ${UNIT_TEST_HEADERS})
			target_link_libraries(${TESTNAME} ${LIBS})
		else()
			add_executable(${TESTNAME} ${UNIT_TEST_SOURCES} ${UNIT_TEST_HEADERS})
			target_link_libraries(${TESTNAME} gtest_main ${LIBS})
			if(SURGSIM_TESTS_RUN)
				add_custom_command(TARGET ${TESTNAME} POST_BUILD COMMAND ${TESTNAME})
			endif()
		endif()
	endif(SURGSIM_TESTS_BUILD)
endmacro()
