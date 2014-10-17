# This file is a part of the OpenSurgSim project.
# Copyright 2013, SimQuest Solutions Inc.
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

find_package(OptiTrack REQUIRED)

include_directories(
	"${CMAKE_CURRENT_SOURCE_DIR}"
	"${OPENSCENEGRAPH_INCLUDE_DIR}"
	"${OPTITRACK_INCLUDE_DIR}"
)

set(TRACKIR_DEVICE_SOURCES
	TrackIRDevice.cpp
	TrackIRThread.cpp
)

set(TRACKIR_DEVICE_HEADERS
	TrackIRDevice.h
	TrackIRScaffold.h
	TrackIRThread.h
)

set(TRACKIR_DEVICE_LINUX_SOURCES
	linux/TrackIRScaffold.cpp
)

set(TRACKIR_DEVICE_WIN_SOURCES
	win32/TrackIRScaffold.cpp
)

if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
	list(APPEND TRACKIR_DEVICE_SOURCES ${TRACKIR_DEVICE_LINUX_SOURCES})
else()
	list(APPEND TRACKIR_DEVICE_SOURCES ${TRACKIR_DEVICE_WIN_SOURCES})
endif()

surgsim_add_library(
	TrackIRDevice
	"${TRACKIR_DEVICE_SOURCES}"
	"${TRACKIR_DEVICE_HEADERS}"
)

if(BUILD_TESTING)
	# The unit tests will be built whenever the library is built.
	add_subdirectory(UnitTests)

	if(GLUT_FOUND)
		add_subdirectory(VisualTest)
	endif(GLUT_FOUND)
endif()

# Natural Point Camera SDK needs this preprocessor definition to work properly
add_definitions( -DCAMERALIBRARY_IMPORTS )

# Put TrackIRDevice into folder "Devices"
set_target_properties(TrackIRDevice PROPERTIES FOLDER "Devices")
