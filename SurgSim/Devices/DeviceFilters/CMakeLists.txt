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

set(LIBS
	${Boost_LIBRARIES}
	SurgSimDataStructures
	SurgSimFramework
	SurgSimInput
	SurgSimMath
)

set(DEVICE_FILTERS_SOURCES
	BoolToScalar.cpp
	DeviceFilter.cpp
	FilteredDevice.cpp
	ForceScale.cpp
	PoseIntegrator.cpp
	PoseTransform.cpp
	RecordPose.cpp
)

set(DEVICE_FILTERS_HEADERS
	BoolToScalar.h
	DeviceFilter.h
	FilteredDevice.h
	ForceScale.h
	PoseIntegrator.h
	PoseTransform.h
	RecordPose.h
)

set(DEVICE_HEADERS ${DEVICE_HEADERS}
	DeviceFilters/DeviceFilter.h #NOLINT
	DeviceFilters/FilteredDevice.h #NOLINT
	DeviceFilters/ForceScale.h #NOLINT
	DeviceFilters/PoseIntegrator.h #NOLINT
	DeviceFilters/PoseTransform.h #NOLINT
	DeviceFilters/RecordPose.h #NOLINT
	PARENT_SCOPE)

surgsim_add_library(
	SurgSimDeviceFilters
	"${DEVICE_FILTERS_SOURCES}"
	"${DEVICE_FILTERS_HEADERS}"
)

target_link_libraries(SurgSimDeviceFilters ${LIBS})

if(BUILD_TESTING)
	# The unit tests will be built whenever the library is built.
	add_subdirectory(UnitTests)
endif()

set_target_properties(SurgSimDeviceFilters PROPERTIES FOLDER "Devices")
