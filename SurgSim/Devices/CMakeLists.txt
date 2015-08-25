# This file is a part of the OpenSurgSim project.
# Copyright 2015, SimQuest Solutions Inc.
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

add_subdirectory(DeviceFilters)
add_subdirectory(IdentityPoseDevice)
add_subdirectory(Keyboard)
add_subdirectory(Mouse)

set(OPTIONAL_DEVICES
	LabJack
	Leap
	MultiAxis
	Nimble
	Novint
	Oculus
	OpenNI
	Phantom
	Sixense
	TrackIR
)

set(DEVICE_LIBRARIES "SurgSimDeviceFilters;IdentityPoseDevice;KeyboardDevice;MouseDevice")
set(DEVICE_DOCUMENTATION devices.dox)
foreach(DEVICE ${OPTIONAL_DEVICES})
	string(TOUPPER ${DEVICE} DEVICE_UPPER_CASE)
	option(BUILD_DEVICE_${DEVICE_UPPER_CASE} "Build ${DEVICE} device." OFF)
	if(BUILD_DEVICE_${DEVICE_UPPER_CASE})
		add_subdirectory(${DEVICE})
		list(APPEND DEVICE_DOCUMENTATION ${DEVICE}/${DEVICE}.dox)
		set(DEVICE_LIBRARIES "${DEVICE_LIBRARIES};${DEVICE}Device")
	endif(BUILD_DEVICE_${DEVICE_UPPER_CASE})
endforeach(DEVICE)

set(SURGSIM_DEVICES_SOURCES
	DeviceUtilities.cpp
)

set(SURGSIM_DEVICES_HEADERS
	DeviceUtilities.h
)

surgsim_add_library(
	SurgSimDevices
	"${SURGSIM_DEVICES_SOURCES}"
	"${SURGSIM_DEVICES_HEADERS}"
)

target_link_libraries(SurgSimDevices SurgSimMath ${DEVICE_LIBRARIES})

surgsim_create_library_header(Devices.h "${SURGSIM_DEVICES_HEADERS};${DEVICE_HEADERS}")

add_custom_target(SurgSimDevicesDocumentation SOURCES ${DEVICE_DOCUMENTATION})
set_target_properties(SurgSimDevicesDocumentation PROPERTIES FOLDER "Devices")
