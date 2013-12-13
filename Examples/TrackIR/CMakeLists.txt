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

link_directories(
	${Boost_LIBRARY_DIRS}
	${OSG_INCLUDE_DIR}
)

include_directories(
	"${SURGSIM_TOP_INCLUDE_DIR}"
	"${CMAKE_CURRENT_SOURCE_DIR}"
	"${EIGEN3_INCLUDE_DIR}"
	"${Boost_INCLUDE_DIR}"
	"${OSG_INCLUDE_DIR}"
	"${OPENTHREADS_INCLUDE_DIR}"
	"${OPTITRACK_INCLUDE_DIR}"
	"${YAML_CPP_SOURCE_DIR}/include"
)

set(TRACKIREXAMPLE_SOURCES
	TrackIRExample.cpp
)

set(TRACKIREXAMPLE_HEADERS
)

add_executable(ExampleTrackIR
	${TRACKIREXAMPLE_SOURCES} ${TRACKIREXAMPLE_HEADERS})

set(LIBS ${LIBS}
	TrackIRDevice
	SurgSimBlocks
	SurgSimDataStructures
	SurgSimGraphics
	SurgSimMath
	yaml-cpp
	${OPTITRACK_LIBRARY}	
)

	
# Copy the device DLLs into the build directory
surgsim_copy_to_target_directory_for_release(ExampleTrackIR
	${OPTITRACK_SHARED_RELEASE}
)
surgsim_copy_to_target_directory_for_debug(ExampleTrackIR
	${OPTITRACK_SHARED_DEBUG}
)

target_link_libraries(ExampleTrackIR ${LIBS}
)

# Put ExampleTrackIR into folder "Examples"
set_target_properties(ExampleTrackIR PROPERTIES FOLDER "Examples") 