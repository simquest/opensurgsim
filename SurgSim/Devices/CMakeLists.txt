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

set(DEVICES
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

set(SURGSIM_DEVICES_DOCUMENTATION
	devices.dox
)

foreach(DEVICE ${DEVICES})
	string(TOUPPER ${DEVICE} DEVICE_UPPER_CASE)
	option(BUILD_DEVICE_${DEVICE_UPPER_CASE} "Build ${DEVICE} device." OFF)
	if(BUILD_DEVICE_${DEVICE_UPPER_CASE})
		add_subdirectory(${DEVICE})
		list(APPEND SURGSIM_DEVICES_DOCUMENTATION ${DEVICE}/${DEVICE}.dox)
	endif(BUILD_DEVICE_${DEVICE_UPPER_CASE})
endforeach(DEVICE)

surgsim_create_library_header(Devices.h "${SURGSIM_DEVICES_HEADERS}")

add_custom_target(SurgSimDevicesDocumentation SOURCES ${SURGSIM_DEVICES_DOCUMENTATION})
set_target_properties(SurgSimDevicesDocumentation PROPERTIES FOLDER "Devices")
