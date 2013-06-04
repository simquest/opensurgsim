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


# Make it easier to tell CMake where to find dot for doxygen.  You can
# set the environment variable DOXYGEN_DOT_PATH to a directory path to
# where the dot binary lives.  (You don't need this on Linux if you
# install Graphviz in a standard location.)
#
# Note that the Windows installer for Graphviz 2.30 doesn't seem to
# write the install directory into the registry.
find_program(DOXYGEN_DOT_EXECUTABLE
	NAMES dot
	PATHS "$ENV{DOXYGEN_DOT_PATH}" "${DOXYGEN_DOT_PATH}"
  DOC "Graphviz Dot tool for using Doxygen")

find_package(Doxygen)

option(SURGSIM_DOCUMENTATION
	"Generate HTML documentation using Doxygen" ${DOXYGEN_FOUND})

option(SURGSIM_DOCUMENTATION_AUTOBUILD
	"(Re)generate HTML documentation during build, and do it every time." OFF)

set(SURGSIM_DOCUMENTATION_EXCLUDE_LIST
	*/UnitTests/*
	CACHE STRING "Directories and files to ignore when generating documentation")
mark_as_advanced(SURGSIM_DOCUMENTATION_EXCLUDE_LIST)

option(SURGSIM_DOCUMENTATION_LOCAL_MATHJAX
	"Use a local copy of MathJax for the HTML documentation" OFF)
mark_as_advanced(SURGSIM_DOCUMENTATION_LOCAL_MATHJAX)

set(PROJECT_LOGO "${CMAKE_CURRENT_SOURCE_DIR}/OpenSurgSim-dox.png")
set(HTML_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/header.html")
set(HTML_FOOTER "${CMAKE_CURRENT_SOURCE_DIR}/footer.html")


if(SURGSIM_DOCUMENTATION)
  if (NOT DOXYGEN_FOUND)
    message(FATAL_ERROR
      "Doxygen is needed to generate the documentation, but was not found.")

  else(NOT DOXYGEN_FOUND)
		# Set DOT_PATH to DOXYGEN_DOT_EXECUTABLE, or empty if not found.
		# Eliminates whining from Doxygen.
		if(DOXYGEN_DOT_FOUND)
			set(DOT_PATH "${DOXYGEN_DOT_EXECUTABLE}")
		else(DOXYGEN_DOT_FOUND)
			set(DOT_PATH "")
		endif(DOXYGEN_DOT_FOUND)

		# Control whether the "documentation" target needs to be built
		# separately or whether it will be tied into the project build.
		if(SURGSIM_DOCUMENTATION_AUTOBUILD)
			set(DOCUMENTATION_TARGET_FLAGS ALL)
		else(SURGSIM_DOCUMENTATION_AUTOBUILD)
			set(DOCUMENTATION_TARGET_FLAGS )
		endif(SURGSIM_DOCUMENTATION_AUTOBUILD)

		# Control the output format so warnings pop up in the Visual Studio list
		if(MSVC)
			set(WARN_FORMAT "\"$file($line): warning DOC1: $text\"")
		else(MSVC)
			set(WARN_FORMAT "\"$file:$line: $text\"")
		endif(MSVC)

		if(SURGSIM_DOCUMENTATION_LOCAL_MATHJAX)
			# Copy MathJax directly into the generated documentation
			if(EXISTS "${SURGSIM_THIRD_PARTY_DIR}/mathjax" AND
					NOT EXISTS "${CMAKE_CURRENT_BINARY_DIR}/html/mathjax")
				message("Copying local MathJax (please be patient)...")
				file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/html/")
				file(COPY "${SURGSIM_THIRD_PARTY_DIR}/mathjax" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/html/")
				message("...done!")
			endif()
			set(MATHJAX_RELPATH ./mathjax)
		else()
			# Use MathJax directly from the web
			set(MATHJAX_RELPATH http://www.mathjax.org/mathjax)
		endif()

		# Generate Doxyfile from Doxyfile.in, with @VARIABLE@ substitution.
		configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in
			${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)

		add_custom_target(documentation ${DOCUMENTATION_TARGET_FLAGS}
			${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
			SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in
			WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
			COMMENT "Generating HTML documentation" VERBATIM)
		# Put the Doxyfile.in at the top of the VS project, not "Header Files":
		source_group("" FILES "${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in")

		if(UNIX)
			install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html/ 
				DESTINATION share/doc/${CMAKE_PROJECT_NAME}) 
		else(UNIX)
			install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html/ 
				DESTINATION documentation) 
		endif(UNIX)
	endif(NOT DOXYGEN_FOUND)
endif(SURGSIM_DOCUMENTATION)
