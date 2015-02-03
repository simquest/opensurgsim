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


find_package(SixenseSdk REQUIRED)

link_directories(${Boost_LIBRARY_DIRS})

set(LIBS
	${Boost_LIBRARIES}
)

include_directories(
	"${CMAKE_CURRENT_SOURCE_DIR}"
	"${SIXENSESDK_INCLUDE_DIR}"
)

set(SIXENSE_DEVICE_SOURCES
	SixenseDevice.cpp
	SixenseScaffold.cpp
	SixenseThread.cpp
)

set(SIXENSE_DEVICE_HEADERS
	SixenseDevice.h
	SixenseScaffold.h
	SixenseThread.h
)

# TODO(advornik): the installation should NOT copy all the headers...
surgsim_add_library(
	SixenseDevice
	"${SIXENSE_DEVICE_SOURCES}"
	"${SIXENSE_DEVICE_HEADERS}"
)

target_link_libraries(SixenseDevice ${LIBS})

if(BUILD_TESTING)
	add_subdirectory(UnitTests)

	if(GLUT_FOUND)
		add_subdirectory(VisualTest)
	endif(GLUT_FOUND)
endif()

# Put SixenseDevice into folder "Devices"
set_target_properties(SixenseDevice PROPERTIES FOLDER "Devices")
