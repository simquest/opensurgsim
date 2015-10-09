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

set(KEYBOARD_DEVICE_SOURCES
	KeyboardDevice.cpp
	KeyboardScaffold.cpp
	OsgKeyboardHandler.cpp
)

set(KEYBOARD_DEVICE_HEADERS
	KeyboardDevice.h
	KeyboardScaffold.h
	KeyCode.h
	OsgKeyboardHandler.h
)

set(DEVICE_HEADERS ${DEVICE_HEADERS}
	Keyboard/KeyboardDevice.h #NOLINT
	Keyboard/KeyCode.h #NOLINT
	PARENT_SCOPE)

surgsim_add_library(
	OSS
	KeyboardDevice
	"${KEYBOARD_DEVICE_SOURCES}"
	"${KEYBOARD_DEVICE_HEADERS}"
)

set(LIBS
	SurgSimInput
	${Boost_LIBRARIES}
	${OPENSCENEGRAPH_LIBRARIES}
)

target_link_libraries(KeyboardDevice ${LIBS}
)

if(BUILD_TESTING)
	# The unit tests will be built whenever the library is built.
	add_subdirectory(UnitTests)
	add_subdirectory(VisualTests)
endif()

# Put KeyboardDevice into folder "Devices"
set_target_properties(KeyboardDevice PROPERTIES FOLDER "Devices")
