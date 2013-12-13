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
	"${CMAKE_CURRENT_SOURCE_DIR}"
	"${Boost_INCLUDE_DIR}"
	"${EIGEN3_INCLUDE_DIR}"
	"${OPENSCENEGRAPH_INCLUDE_DIR}"
	"${OPTITRACK_INCLUDE_DIR}"
	"${SURGSIM_TOP_INCLUDE_DIR}"
)

set(TRACKIR_DEVICE_SOURCES
	TrackIRDevice.cpp
	TrackIRScaffold.cpp
	TrackIRThread.cpp
)

set(TRACKIR_DEVICE_HEADERS
	TrackIRDevice.h
	TrackIRScaffold.h
	TrackIRThread.h
)

surgsim_add_library(
	TrackIRDevice
	"${TRACKIR_DEVICE_SOURCES}" "${TRACKIR_DEVICE_HEADERS}"
	SurgSim/Devices/TrackIR
)
if(SURGSIM_TESTS_BUILD)
	# The unit tests will be built whenever the library is built.
	add_subdirectory(UnitTests)
	add_subdirectory(VisualTest)
endif()

# Put TrackIRDevice into folder "Devices"
set_target_properties(TrackIRDevice PROPERTIES FOLDER "Devices")