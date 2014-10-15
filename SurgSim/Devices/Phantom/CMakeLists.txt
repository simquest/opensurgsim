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


find_package(OpenHaptics REQUIRED)

link_directories(${Boost_LIBRARY_DIRS})

set(LIBS
	${Boost_LIBRARIES}
	${OPENHAPTICS_LIBRARIES}
	SurgSimFramework
	SurgSimInput
)

include_directories(
	"${CMAKE_CURRENT_SOURCE_DIR}"
	"${OPENHAPTICS_INCLUDE_DIR}"
)

set(PHANTOM_DEVICE_SOURCES
	PhantomDevice.cpp
	PhantomScaffold.cpp
)

set(PHANTOM_DEVICE_HEADERS
	PhantomDevice.h
	PhantomScaffold.h
)

# TODO(advornik): the installation should NOT copy all the headers...
surgsim_add_library(
	PhantomDevice
	"${PHANTOM_DEVICE_SOURCES}"
	"${PHANTOM_DEVICE_HEADERS}"
)

target_link_libraries(PhantomDevice ${LIBS})

if(BUILD_TESTING)
	# The unit tests will be built whenever the library is built.
	add_subdirectory(UnitTests)

	if(GLUT_FOUND)
		add_subdirectory(VisualTest)
	endif(GLUT_FOUND)
endif()

# Put PhantomDevice into folder "Devices"
set_target_properties(PhantomDevice PROPERTIES FOLDER "Devices")
