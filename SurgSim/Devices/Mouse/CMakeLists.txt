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

include_directories(
	"${CMAKE_CURRENT_SOURCE_DIR}"
	"${OPENSCENEGRAPH_INCLUDE_DIR}"
)

set(MOUSE_DEVICE_SOURCES
	MouseDevice.cpp
	MouseScaffold.cpp
	OsgMouseHandler.cpp
)

set(MOUSE_DEVICE_HEADERS
	MouseDevice.h
	MouseScaffold.h
	OsgMouseHandler.h
)

set(SURGSIM_DEVICES_HEADERS ${SURGSIM_DEVICES_HEADERS} ${MOUSE_DEVICE_HEADERS} PARENT_SCOPE)

surgsim_add_library(
	MouseDevice
	"${MOUSE_DEVICE_SOURCES}"
	"${MOUSE_DEVICE_HEADERS}"
)

set(LIBS
	SurgSimInput
	${OPENSCENEGRAPH_LIBRARIES}
)

target_link_libraries(MouseDevice ${LIBS}
)

if(BUILD_TESTING)
	# The unit tests will be built whenever the library is built.
	add_subdirectory(UnitTests)
	add_subdirectory(VisualTests)
endif()

# Put MouseDevice into folder "Devices"
set_target_properties(MouseDevice PROPERTIES FOLDER "Devices")
