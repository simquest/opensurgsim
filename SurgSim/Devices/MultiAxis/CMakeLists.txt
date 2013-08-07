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



link_directories(${Boost_LIBRARY_DIRS})
set(LIBS ${LIBS} ${Boost_LIBRARIES})

include_directories(
	"${SURGSIM_TOP_INCLUDE_DIR}"
	"${CMAKE_CURRENT_SOURCE_DIR}"
	"${EIGEN3_INCLUDE_DIR}"
	"${Boost_INCLUDE_DIR}"
)

set(MULTIAXIS_DEVICE_SOURCES
	RawMultiAxisDevice.cpp
	RawMultiAxisScaffold.cpp
	RawMultiAxisThread.cpp
	SystemInputDeviceHandle.cpp
)

set(MULTIAXIS_DEVICE_HEADERS
	CreateInputDeviceHandle.h
	GetSystemError.h
	RawMultiAxisDevice.h
	RawMultiAxisScaffold.h
	RawMultiAxisThread.h
	SystemInputDeviceHandle.h
)

set(MULTIAXIS_DEVICE_LINUX_SOURCES
	linux/CreateInputDeviceHandle.cpp
	linux/FileDescriptor.cpp
	linux/GetSystemError.cpp
	linux/InputDeviceHandle.cpp
)

set(MULTIAXIS_DEVICE_LINUX_HEADERS
	BitSetBuffer.h
	linux/FileDescriptor.h
	linux/InputDeviceHandle.h
)

set(MULTIAXIS_DEVICE_WDK_HID_SOURCES
	win32/CreateInputDeviceHandle.cpp
	win32/FileHandle.cpp
	win32/GetSystemError.cpp
	win32/WdkHidDeviceHandle.cpp
)

set(MULTIAXIS_DEVICE_WDK_HID_HEADERS
	win32/FileHandle.h
	win32/WdkHidDeviceHandle.h
)

if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
	add_definitions( -DSURGSIM_MULTIAXIS_USING_LINUX_INPUT )
	list(APPEND MULTIAXIS_DEVICE_SOURCES ${MULTIAXIS_DEVICE_LINUX_SOURCES})
	list(APPEND MULTIAXIS_DEVICE_HEADERS ${MULTIAXIS_DEVICE_LINUX_HEADERS})
elseif(WINDDK_FOUND)
	add_definitions( -DSURGSIM_MULTIAXIS_USING_WINDOWS_HID )
	list(APPEND MULTIAXIS_DEVICE_SOURCES ${MULTIAXIS_DEVICE_WDK_HID_SOURCES})
	list(APPEND MULTIAXIS_DEVICE_HEADERS ${MULTIAXIS_DEVICE_WDK_HID_HEADERS})
endif()

# TODO(advornik): the installation should NOT copy all the headers...
surgsim_add_library(
	MultiAxisDevice
	"${MULTIAXIS_DEVICE_SOURCES}" "${MULTIAXIS_DEVICE_HEADERS}"
	SurgSim/Devices/MultiAxis
)
if(SURGSIM_TESTS_BUILD)
	add_subdirectory(UnitTests)

	if(GLUT_FOUND)
		add_subdirectory(VisualTest)
	endif(GLUT_FOUND)
endif()
