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
set(LIBS ${LIBS} ${Boost_LIBRARIES} ${OPENHAPTICS_LIBRARIES})

include_directories(
	"${SURGSIM_TOP_INCLUDE_DIR}"
	"${CMAKE_CURRENT_SOURCE_DIR}"
	"${EIGEN3_INCLUDE_DIR}"
	"${Boost_INCLUDE_DIR}"
)

set(IDENTITY_POSE_DEVICE_SOURCES
	IdentityPoseDevice.cpp
)

set(IDENTITY_POSE_DEVICE_HEADERS
	IdentityPoseDevice.h
)

add_library(IdentityPoseDevice
	${IDENTITY_POSE_DEVICE_SOURCES} ${IDENTITY_POSE_DEVICE_HEADERS})

if(SURGSIM_TESTS_BUILD)
	add_subdirectory(UnitTests)
endif()
