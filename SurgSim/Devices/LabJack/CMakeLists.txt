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

find_package(LabJack REQUIRED)

link_directories(${Boost_LIBRARY_DIRS})

set(LIBS
	${Boost_LIBRARIES}
	SurgSimFramework
	SurgSimInput
	${LABJACK_LIBRARY}
)

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}
	${LABJACK_INCLUDE_DIR}
)

set(LABJACK_DEVICE_SOURCES
	LabJackDevice.cpp
	LabJackThread.cpp
)

set(LABJACK_DEVICE_HEADERS
	LabJackDevice.h
	LabJackScaffold.h
	LabJackThread.h
)

set(SURGSIM_DEVICES_HEADERS ${SURGSIM_DEVICES_HEADERS} ${LABJACK_DEVICE_HEADERS} PARENT_SCOPE)

set(LABJACK_DEVICE_LINUX_SOURCES
	linux/LabJackChecksums.cpp
	linux/LabJackScaffold.cpp
	linux/LabJackTypeConverters.cpp
)

set(LABJACK_DEVICE_LINUX_HEADERS
	linux/LabJackChecksums.h
	linux/LabJackConstants.h
	linux/LabJackTypeConverters.h
)

set(LABJACK_DEVICE_WIN_SOURCES
	win32/LabJackScaffold.cpp
)

if(${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
	list(APPEND LABJACK_DEVICE_SOURCES ${LABJACK_DEVICE_WIN_SOURCES})
else()
	list(APPEND LABJACK_DEVICE_HEADERS ${LABJACK_DEVICE_LINUX_HEADERS})
	list(APPEND LABJACK_DEVICE_SOURCES ${LABJACK_DEVICE_LINUX_SOURCES})
endif()

# TODO(advornik): the installation should NOT copy all the headers...
surgsim_add_library(
	LabJackDevice
	"${LABJACK_DEVICE_SOURCES}"
	"${LABJACK_DEVICE_HEADERS}"
)

target_link_libraries(LabJackDevice ${LIBS})

if(BUILD_TESTING)
	# The unit tests will be built whenever the library is built.
	add_subdirectory(UnitTests)

	if(GLUT_FOUND)
		add_subdirectory(VisualTest)
	endif(GLUT_FOUND)
endif()


set_target_properties(LabJackDevice PROPERTIES FOLDER "Devices")
