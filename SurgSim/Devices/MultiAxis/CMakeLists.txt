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

if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
else(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
	find_package(WDK REQUIRED)
endif(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")

include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

if(WDK_FOUND)
	if(WDK_CAN_BUILD_DIRECTLY)
		#message(STATUS "WDK does not need additional include paths; see the try_compile in FindWDK.cmake")
	else(WDK_CAN_BUILD_DIRECTLY)
		#message(STATUS "WDK was found, and needs the include directory '${WDK_INCLUDE_DIR}'.")
		include_directories(
				# This is a hack needed to get the Visual Studio includes into the
				# include path ahead of the WDK ones, which is needed for the WDK
				# 7.1.0 not to interfere with includes provided by Visual Studio.
				# CMake will prefix this entry by the current path, but Visual
				# Studio will split the path at the semicolon; the net effect is to
				# add $(IncludePath) to the Visual Studio include directories.
				# In cmake 2.8.11+, this hack generates a Policy CMP0021 warning due to relative path.
				".;$(IncludePath)"
				# Now it's safe to add ${WDK_INCLUDE_DIR}.
				"${WDK_INCLUDE_DIR}"
				)
	endif(WDK_CAN_BUILD_DIRECTLY)
endif(WDK_FOUND)

set(MULTIAXIS_DEVICE_SOURCES
	MultiAxisDevice.cpp
	RawMultiAxisDevice.cpp
	RawMultiAxisScaffold.cpp
	RawMultiAxisThread.cpp
	SystemInputDeviceHandle.cpp
)

set(MULTIAXIS_DEVICE_HEADERS
	CreateInputDeviceHandle.h
	GetSystemError.h
	MultiAxisDevice.h
	RawMultiAxisDevice.h
	RawMultiAxisScaffold.h
	RawMultiAxisThread.h
	SystemInputDeviceHandle.h
)

set(SURGSIM_DEVICES_HEADERS ${SURGSIM_DEVICES_HEADERS}
	MultiAxisDevice.h
	RawMultiAxisDevice.h
	PARENT_SCOPE)

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
	list(APPEND MULTIAXIS_DEVICE_SOURCES ${MULTIAXIS_DEVICE_LINUX_SOURCES})
	list(APPEND MULTIAXIS_DEVICE_HEADERS ${MULTIAXIS_DEVICE_LINUX_HEADERS})
elseif(WDK_FOUND)
	list(APPEND MULTIAXIS_DEVICE_SOURCES ${MULTIAXIS_DEVICE_WDK_HID_SOURCES})
	list(APPEND MULTIAXIS_DEVICE_HEADERS ${MULTIAXIS_DEVICE_WDK_HID_HEADERS})
endif()

# TODO(advornik): the installation should NOT copy all the headers...
surgsim_add_library(
	MultiAxisDevice
	"${MULTIAXIS_DEVICE_SOURCES}"
	"${MULTIAXIS_DEVICE_HEADERS}"
)

set(LIBS  
	SurgSimDeviceFilters
	SurgSimInput 
	SurgSimFramework 
)

if(WDK_FOUND)
	list(APPEND LIBS ${WDK_LIBRARIES})
endif()

target_link_libraries(MultiAxisDevice ${LIBS}
)

if(BUILD_TESTING)
	add_subdirectory(UnitTests)

	if(GLUT_FOUND)
		add_subdirectory(VisualTest)
	endif(GLUT_FOUND)
endif()

# Put MultiAxisDevice into folder "Devices"
set_target_properties(MultiAxisDevice PROPERTIES FOLDER "Devices")
