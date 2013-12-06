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
	"${SURGSIM_TOP_INCLUDE_DIR}"
	"${CMAKE_CURRENT_SOURCE_DIR}"
	"${EIGEN3_INCLUDE_DIR}"
	"${OPENSCENEGRAPH_INCLUDE_DIR}"
)

set(KEYBOARD_DEVICE_SOURCES
	KeyboardDevice.cpp
	KeyboardHandler.cpp
	KeyboardScaffold.cpp
)

set(KEYBOARD_DEVICE_HEADERS
	KeyCode.h
	KeyboardDevice.h
	KeyboardHandler.h
	KeyboardScaffold.h
)

surgsim_add_library(
	KeyboardDevice
	"${KEYBOARD_DEVICE_SOURCES}" "${KEYBOARD_DEVICE_HEADERS}"
	SurgSim/Devices/Keyboard
)

if(SURGSIM_TESTS_BUILD)
	# The unit tests will be built whenever the library is built.
	add_subdirectory(UnitTests)
	add_subdirectory(VisualTests)
endif()

# Put KeyboardDevice into folder "Devices"
set_target_properties(KeyboardDevice PROPERTIES FOLDER "Devices")