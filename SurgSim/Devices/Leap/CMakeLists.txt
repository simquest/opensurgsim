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

find_package(LeapSdk REQUIRED)

include_directories(SYSTEM
	${LEAPSDK_INCLUDE_DIR}
)

set(LIBS
	SurgSimDataStructures
	SurgSimFramework
	SurgSimInput
	${LEAPSDK_LIBRARY}
)

set(LEAP_DEVICE_SOURCES
	LeapDevice.cpp
	LeapScaffold.cpp
)

set(LEAP_DEVICE_HEADERS
	LeapDevice.h
	LeapScaffold.h
)

set(SURGSIM_DEVICES_HEADERS ${SURGSIM_DEVICES_HEADERS} Leap/LeapDevice.h PARENT_SCOPE)

surgsim_add_library(
   OSS
   LeapDevice
   "${LEAP_DEVICE_SOURCES}"
   "${LEAP_DEVICE_HEADERS}"
)
target_link_libraries(LeapDevice ${LIBS})

if(BUILD_TESTING)
	add_subdirectory(UnitTests)

   if(GLUT_FOUND)
	   add_subdirectory(VisualTest)
   endif(GLUT_FOUND)
endif()

set_target_properties(LeapDevice PROPERTIES FOLDER "Devices")

