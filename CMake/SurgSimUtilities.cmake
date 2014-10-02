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


## Options related to integration with Visual Studio and other IDEs.

# options related to source file organization
#
option(SURGSIM_IDE_SEPARATE_SOURCES_HEADERS
	"If turned on, separate header files from the source files in the IDE. " OFF)
if(SURGSIM_IDE_SEPARATE_SOURCES_HEADERS)
	set(SURGSIM_IDE_SOURCE_PREFIX "Source Files/")
	set(SURGSIM_IDE_HEADER_PREFIX "Header Files/")
else()
	set(SURGSIM_IDE_SOURCE_PREFIX "Files/")
	set(SURGSIM_IDE_HEADER_PREFIX "Files/")
endif()

# Sets up the source file organization, for Visual Studio etc.
# This will group the source and header files in folders by directory.
#
# Note the use of optional arguments:
#   surgsim_set_source_groups(PREFIX FILES...)
#
function(surgsim_source_hierarchy PREFIX)
	foreach(FILE ${ARGN})
		# Get the directory name.  Note that for files in the current
		# directory (and empty prefix), you will get "" rather than ".".
		get_filename_component(FILE_PREFIXED_DIR "${PREFIX}${FILE}" PATH)
		# Apparently, the hierarchical separator is "\" (per CMake docs).
		# Hopefully that works in Linux IDEs too...
		string(REGEX REPLACE "/" "\\\\" FILE_GROUP "${FILE_PREFIXED_DIR}")
		# Set the IDE source group.  Note that top level is "", not ".".
		source_group("${FILE_GROUP}" FILES "${FILE}")
		#message("SRC ${FILE}|${FILE_GROUP}")
	endforeach(FILE ${ARGN})
endfunction()

# Sets up the folder organization for sources and headers in Visual Studio etc.
function(surgsim_show_ide_folders SOURCES HEADERS)
	surgsim_source_hierarchy("${SURGSIM_IDE_SOURCE_PREFIX}" ${SOURCES})
	surgsim_source_hierarchy("${SURGSIM_IDE_HEADER_PREFIX}" ${HEADERS})
endfunction()

set(SURGSIM_COPY_WARNING_ONCE TRUE)

# Copy zero or more files to the location of a built target, after the
# target is built successfully, but only if the condition (which
# should be a add_custom_command "generator expression") evaluates to 1.
#
# Note the use of optional arguments:
#   surgsim_copy_to_target_directory_if(<condition> <target> [<file>...])
#
function(surgsim_copy_to_target_directory_if CONDITION TARGET)
	foreach(FILE ${ARGN})
		if(NOT "${FILE}" STREQUAL "")
			# After much experimentation (and cmake-gui crashes), I learned
			# that as of CMake 2.8.10, you can't have line breaks or
			# multiple tokens within the body of a $<{0,1}:...> generator
			# expression.  So to conditionalize the command "foo bar", you
			# can't do "$<COND:foo bar>"; you need "$<COND:foo> $<COND:bar>"...
			# And on top of that, we can't use VERBATIM or each of the
			# conditional tokens will become "" rather than being skipped
			# altogether.
			get_filename_component(FNAME "${FILE}" NAME)
			set(CMD_COPY copy_if_different "${FILE}" $<TARGET_FILE_DIR:${TARGET}>)
			set(CMD_SKIP echo "${TARGET}: Not copying ${FNAME} for $<CONFIGURATION>")
			# Build a list of properly conditionalized list elements.
			set(CONDITIONAL_COMMAND  ${CMAKE_COMMAND} -E)
			if(CMAKE_VERSION VERSION_LESS 2.8.10)
				if(SURGSIM_COPY_WARNING_ONCE)
					# Only show the message once *per directory*.
					set(SURGSIM_COPY_WARNING_ONCE FALSE)
					message("Will copy target libraries unconditionally.  Please consider upgrading to CMake 2.8.10 or later.")
				endif(SURGSIM_COPY_WARNING_ONCE)
				list(APPEND CONDITIONAL_COMMAND ${CMD_COPY})
			else()
				foreach(TOKEN ${CMD_COPY})
					list(APPEND CONDITIONAL_COMMAND $<${CONDITION}:${TOKEN}>)
				endforeach(TOKEN ${CMD_COPY})
				foreach(TOKEN ${CMD_SKIP})
					list(APPEND CONDITIONAL_COMMAND $<$<NOT:${CONDITION}>:${TOKEN}>)
				endforeach(TOKEN ${CMD_SKIP})
			endif()
			# Now use the conditional command we built.  (Can't use VERBATIM!)
			add_custom_command(TARGET ${TARGET} POST_BUILD
				COMMAND ${CONDITIONAL_COMMAND})
		endif(NOT "${FILE}" STREQUAL "")
	endforeach(FILE ${ARGN})
endfunction()

# Copy zero or more files to the location of a built target, after the
# target is built successfully.
#
# Note the use of optional arguments:
#   surgsim_copy_to_target_directory(<target> [<file>...])
#
macro(surgsim_copy_to_target_directory TARGET)
	surgsim_copy_to_target_directory_if(1 "${TARGET}" ${ARGN})
endmacro()

# Copy zero or more files to the location of a built *Debug* target,
# after the target is built successfully.
#
# Note the use of optional arguments:
#   surgsim_copy_to_target_directory_for_debug(<target> [<file>...])
#
macro(surgsim_copy_to_target_directory_for_debug TARGET)
	surgsim_copy_to_target_directory_if($<CONFIG:Debug> "${TARGET}" ${ARGN})
endmacro()

# Copy zero or more files to the location of a built *non-Debug*
# target, after the target is built successfully.
#
# Note the use of optional arguments:
#   surgsim_copy_to_target_directory_for_release(<target> [<file>...])
#
macro(surgsim_copy_to_target_directory_for_release TARGET)
	surgsim_copy_to_target_directory_if($<NOT:$<CONFIG:Debug>>
		"${TARGET}" ${ARGN})
endmacro()

option(SURGSIM_RUN_TEST_WITHIN_BUILD "This exectutes the tests directly from the chosen build system." OFF)
# Build the unit test executable.
# Uses UNIT_TEST_SOURCES, UNIT_TEST_HEADERS, LIBS, UNIT_TEST_SHARED_LIBS,
# UNIT_TEST_SHARED_RELEASE_LIBS and UNIT_TEST_SHARED_DEBUG_LIBS.
#
macro(surgsim_add_unit_tests TESTNAME)
	add_executable(${TESTNAME} ${UNIT_TEST_SOURCES} ${UNIT_TEST_HEADERS})
	target_link_libraries(${TESTNAME} SurgSimTesting gmock ${LIBS})
	if(SURGSIM_PRECOMPILED_HEADERS)
		target_link_libraries(${TESTNAME} PrecompiledHeader)
	endif(SURGSIM_PRECOMPILED_HEADERS)
	add_test(NAME ${TESTNAME} COMMAND ${TESTNAME} "--gtest_output=xml")
	
	if(SURGSIM_RUN_TEST_WITHIN_BUILD)
        add_custom_command(TARGET ${TESTNAME} POST_BUILD
            COMMAND ${SURGSIM_TEST_RUN_PREFIX} $<TARGET_FILE:${TESTNAME}>
                ${SURGSIM_TEST_RUN_SUFFIX}
            VERBATIM)
    endif()


	# copy all ${UNIT_TEST_SHARED..._LIBS} to the test executable directory:
	surgsim_copy_to_target_directory(${TESTNAME}
		${UNIT_TEST_SHARED_LIBS})
	surgsim_copy_to_target_directory_for_release(${TESTNAME}
		${UNIT_TEST_SHARED_RELEASE_LIBS})
	surgsim_copy_to_target_directory_for_debug(${TESTNAME}
		${UNIT_TEST_SHARED_DEBUG_LIBS})
	surgsim_show_ide_folders("${UNIT_TEST_SOURCES}" "${UNIT_TEST_HEADERS}")
endmacro()

# Do all the work to add a library to the system
# Works with the install system and detects whether the library is 
# header only or has source files, for header only the headers are copied into
# the appropriate directory. 
# Note that when calling this the parameters  should be quoted to separate lists
function(surgsim_add_library LIBRARY_NAME SOURCE_FILES HEADER_FILES HEADER_DIRECTORY)
	if (SOURCE_FILES)
		add_library(${LIBRARY_NAME} ${SOURCE_FILES} ${HEADER_FILES})

		if(SURGSIM_PRECOMPILED_HEADERS AND NOT ${LIBRARY_NAME} MATCHES "PrecompiledHeader")
			foreach(source ${SOURCE_FILES})
				# C++ precompiled headers cannot be used with C source files
				if(source MATCHES ".c$")
					set_source_files_properties(${source} PROPERTIES COMPILE_FLAGS "/Y-")
				endif(source MATCHES ".c$")
			endforeach(source)
			target_link_libraries(${LIBRARY_NAME} PrecompiledHeader)
		endif(SURGSIM_PRECOMPILED_HEADERS AND NOT ${LIBRARY_NAME} MATCHES "PrecompiledHeader")

		set_target_properties(${LIBRARY_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}")
		install(TARGETS ${LIBRARY_NAME}
			EXPORT ${PROJECT_NAME}Targets
			RUNTIME DESTINATION "${INSTALL_BIN_DIR}"
			LIBRARY DESTINATION "${INSTALL_LIB_DIR}"
			ARCHIVE DESTINATION "${INSTALL_LIB_DIR}"
			PUBLIC_HEADER DESTINATION ${INSTALL_INCLUDE_DIR}/${HEADER_DIRECTORY})
			
		set(EXPORT_TARGETS ${LIBRARY_NAME} ${EXPORT_TARGETS} CACHE INTERNAL "export targets")
		surgsim_show_ide_folders("${SOURCE_FILES}" "${HEADER_FILES}")
	else()
		install(FILES ${HEADER_FILES} DESTINATION ${INSTALL_INCLUDE_DIR}/${HEADER_DIRECTORY})
		surgsim_show_ide_folders("" "${HEADER_FILES}")
	endif()
endfunction()


## CMake support for running Google's cpplint over the C++ source.

# Should we bother with cpplint at all?
option(SURGSIM_CPPLINT "Include cpplint in the build" ON)

# Running cpplint requires Python
if(SURGSIM_CPPLINT)
	find_package(PythonInterp)
	if(NOT PYTHON_EXECUTABLE)
		set(SURGSIM_CPPLINT OFF)
	endif(NOT PYTHON_EXECUTABLE)
endif(SURGSIM_CPPLINT)

# Extra flags to pass to run-lint
set(RUNLINT_DEFAULT_EXTRA_FLAGS)
if(MSVC)
	set(RUNLINT_DEFAULT_EXTRA_FLAGS --vs)
endif()

set(SURGSIM_RUNLINT_EXTRA_FLAGS
	${RUNLINT_DEFAULT_EXTRA_FLAGS}
	CACHE STRING "Extra flags to pass to run-lint."
)
mark_as_advanced(SURGSIM_RUNLINT_EXTRA_FLAGS)

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
	# things disallowed by Google's coding standards, but not ours:
	-runtime/rtti -readability/streams
)
string(REPLACE ";" "," CPPLINT_DEFAULT_FILTERS
	"--filter=${CPPLINT_DEFAULT_FILTER_LIST}")

set(SURGSIM_CPPLINT_FILTERS
	"${CPPLINT_DEFAULT_FILTERS}"
	CACHE STRING "Filter settings to pass to cpplint via run-cpplint."
)
mark_as_advanced(SURGSIM_CPPLINT_FILTERS)


# Run cpplint (from Google's coding standards project) on specified files.
#
# Note the use of optional arguments:
#   surgsim_run_cpplint(<testname> [<file|arg>...])
#
macro(surgsim_run_cpplint TARGET)
	if(SURGSIM_CPPLINT AND PYTHON_EXECUTABLE)
		add_custom_target("${TARGET}"
			${PYTHON_EXECUTABLE}
				${SURGSIM_TOOLS_DIR}/run-lint.py
					--cpplint-script
						${SURGSIM_THIRD_PARTY_DIR}/google-style-lint/cpplint.py
					${SURGSIM_CPPLINT_FILTERS} ${SURGSIM_RUNLINT_EXTRA_FLAGS} ${ARGN}
			WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
			COMMENT "Checking C++ sources with cpplint" VERBATIM)
	endif(SURGSIM_CPPLINT AND PYTHON_EXECUTABLE)
endmacro()

# Run cpplint (from Google's coding standards project) on the source
# files in the current directory and all of its subdirectories.
macro(surgsim_cpplint_this_tree TARGET)
	if(SURGSIM_CPPLINT AND PYTHON_EXECUTABLE)
		surgsim_run_cpplint("${TARGET}" "--traverse" "${CMAKE_CURRENT_SOURCE_DIR}")
	endif(SURGSIM_CPPLINT AND PYTHON_EXECUTABLE)
endmacro()

