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
	${CMAKE_CURRENT_SOURCE_DIR}
	${OSG_INCLUDE_DIR}
)

set(SOURCES
	StapleElement.cpp
	StaplerBehavior.cpp
)

set(HEADERS
	StapleElement.h
	StaplerBehavior.h
)

add_executable(Stapling
	Stapling.cpp
	${SOURCES}
	${HEADERS}
)

add_executable(SerializedStapling
	SerializedStapling.cpp
	${SOURCES}
	${HEADERS}
)

surgsim_show_ide_folders("Main.cpp SerializedMain.cpp ${SOURCES}" "${HEADERS}")

set(LIBS 
	IdentityPoseDevice
	MultiAxisDevice
	SurgSimBlocks
	SurgSimFramework
	SurgSimGraphics
	${YAML_CPP_LIBRARIES}
)
target_link_libraries(Stapling ${LIBS})
target_link_libraries(SerializedStapling ${LIBS})

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/Data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# Configure the path for the data files
configure_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/config.txt.in"
	"${CMAKE_CURRENT_BINARY_DIR}/config.txt"
)

set_target_properties(Stapling PROPERTIES FOLDER "Examples") 
set_target_properties(SerializedStapling PROPERTIES FOLDER "Examples") 
