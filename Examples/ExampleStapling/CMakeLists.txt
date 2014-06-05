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
	${SURGSIM_TOP_INCLUDE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}
	${EIGEN3_INCLUDE_DIR}
	${Boost_INCLUDE_DIR}
	${OSG_INCLUDE_DIR}
	${YAML_CPP_INCLUDE_DIR}
)

set(EXAMPLESTAPLING_SOURCES
	StapleElement.cpp
	StaplerBehavior.cpp
	StaplingExample.cpp
)

set(EXAMPLESTAPLING_HEADERS
	StapleElement.h
	StaplerBehavior.h
)

add_executable(ExampleStapling
	${EXAMPLESTAPLING_SOURCES} ${EXAMPLESTAPLING_HEADERS}
)

surgsim_show_ide_folders(
	"${EXAMPLESTAPLING_SOURCES}" "${EXAMPLESTAPLING_HEADERS}")

set(LIBS 
	${LIBS}
	IdentityPoseDevice
	MultiAxisDevice
	SurgSimBlocks
	SurgSimFramework
	SurgSimGraphics
	${YAML_CPP_LIBRARIES}
)

file(COPY ${CMAKE_SOURCE_DIR}/Examples/Data/Stapling DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Data)

# Configure the path for the data files
configure_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/config.txt.in"
	"${CMAKE_CURRENT_BINARY_DIR}/config.txt"
)

target_link_libraries(ExampleStapling ${LIBS})

# Put ExampleStapling into folder "Examples"
set_target_properties(ExampleStapling PROPERTIES FOLDER "Examples") 
