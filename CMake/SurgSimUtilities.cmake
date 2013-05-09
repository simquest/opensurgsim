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

# Do all the work to add a library to the system
# Works with the install system and detects wether the library is 
# header only or has source files, for header only the headers are copied into
# the appropriate directory. 
# Note that when calling this the parameters  should be quoted to separate lists
function(surgsim_add_library LIBRARY_NAME SOURCE_FILES HEADER_FILES HEADER_DIRECTORY)
	if (SOURCE_FILES)
		add_library(${LIBRARY_NAME} ${SOURCE_FILES} ${HEADER_FILES})

		set_target_properties(${LIBRARY_NAME} PROPERTIES PUBLIC_HEADER ${HEADER_FILES})
		install(TARGETS ${LIBRARY_NAME}
			EXPORT ${PROJECT_NAME}Targets
			RUNTIME DESTINATION "${INSTALL_BIN_DIR}"
			LIBRARY DESTINATION "${INSTALL_LIB_DIR}"
			ARCHIVE DESTINATION "${INSTALL_LIB_DIR}"
			PUBLIC_HEADER DESTINATION ${INSTALL_INCLUDE_DIR}${HEADER_DIRECTORY})
			
		set(EXPORT_TARGETS ${LIBRARY_NAME} ${EXPORT_TARGETS} CACHE INTERNAL "export targets")
	else()
		install(FILES ${HEADER_FILES} DESTINATION ${INSTALL_INCLUDE_DIR}${HEADER_DIRECTORY})
	endif()
endfunction()


## CMake support for running Google's cpplint over the C++ source.

# Should we bother with cpplint at all?
option(SURGSIM_CPPLINT "Include the unit tests in the build" ON)

# Running cpplint requires Python
if(SURGSIM_CPPLINT)
	find_package(PythonInterp)
	if(NOT PYTHON_EXECUTABLE)
		set(SURGSIM_CPPLINT OFF)
	endif(NOT PYTHON_EXECUTABLE)
endif(SURGSIM_CPPLINT)

# Extra flags to pass to cpplint
set(CPPLINT_DEFAULT_EXTRA_FLAGS)
if(MSVC)
	set(CPPLINT_DEFAULT_EXTRA_FLAGS --output=vs7)
endif(MSVC)

set(SURGSIM_CPPLINT_EXTRA_FLAGS
	${CPPLINT_DEFAULT_EXTRA_FLAGS}
  CACHE STRING "Extra flags to pass to cpplint."
)
mark_as_advanced(SURGSIM_CPPLINT_EXTRA_FLAGS)

# Default filter settings for cpplint
set(CPPLINT_DEFAULT_FILTER_LIST
	# our coding standards differ from Google's when it comes to whitespace:
	-whitespace/tab -whitespace/braces -whitespace/line_length
	-whitespace/ending_newline
	# these flag some useful stuff, but also currently produce a lot of noise:
	-whitespace/operators -whitespace/newline -whitespace/blank_line
	-whitespace/comma -whitespace/parens -whitespace/comments
	# this is just broken if a comment is preceded by a tab and ends in ":":
	-whitespace/labels
	# our preferred include guard format differs from Google's:
	-build/header_guard
	# potentially useful, but generates a lot of noise:
	-build/include_what_you_use
	# potentially useful, but generates some crazy false positives
	# (it claims <unordered_map> is a C header!?):
	-build/include_order
)
string(REPLACE ";" "," CPPLINT_DEFAULT_FILTERS
	"--filter=${CPPLINT_DEFAULT_FILTER_LIST}")

set(SURGSIM_CPPLINT_FILTERS
	"${CPPLINT_DEFAULT_FILTERS}"
  CACHE STRING "Filter settings to pass to cpplint."
)
mark_as_advanced(SURGSIM_CPPLINT_FILTERS)


# Run cpplint (from Google's coding standards project) on specified files.
#
# Note the use of optional arguments:
#   surgsim_run_cpplint(<testname> [<file>...])
#
macro(surgsim_run_cpplint TARGET)
	if(SURGSIM_CPPLINT AND PYTHON_EXECUTABLE)
		add_custom_target("${TARGET}"
			${PYTHON_EXECUTABLE}
			  ${SURGSIM_THIRD_PARTY_DIR}/google-style-lint/cpplint.py
				${SURGSIM_CPPLINT_EXTRA_FLAGS} ${SURGSIM_CPPLINT_FILTERS} ${ARGN}
			WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
			COMMENT "Checking C++ sources with cpplint" VERBATIM)
	endif(SURGSIM_CPPLINT AND PYTHON_EXECUTABLE)
endmacro()

# Run cpplint (from Google's coding standards project) on the source
# files in the current directory and all of its subdirectories.
macro(surgsim_cpplint_this_tree TARGET)
	if(SURGSIM_CPPLINT AND PYTHON_EXECUTABLE)
		file(GLOB_RECURSE ALL_CXX_SOURCE_FILES *.h *.cpp)
		surgsim_run_cpplint("${TARGET}" ${ALL_CXX_SOURCE_FILES})
	endif(SURGSIM_CPPLINT AND PYTHON_EXECUTABLE)
endmacro()
