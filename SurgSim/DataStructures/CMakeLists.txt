# This file is a part of the OpenSurgSim project.
# Copyright 2012-2013, SimQuest Solutions Inc.
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
	${EIGEN3_INCLUDE_DIR}
)

set(SURGSIM_DATA_STRUCTURES_SOURCES
	DataGroupBuilder.cpp
	IndexDirectory.cpp
)

set(SURGSIM_DATA_STRUCTURES_HEADERS
	DataGroup.h
	DataGroup-inl.h
	DataGroupBuilder.h
	IndexDirectory.h
	Mesh.h
	MeshElement.h
	MeshVertex.h
	NamedData.h
	NamedData-inl.h
	NamedDataBuilder.h
	TriangleMesh.h
)

add_library(SurgSimDataStructures
	${SURGSIM_DATA_STRUCTURES_SOURCES} ${SURGSIM_DATA_STRUCTURES_HEADERS}
)

install(FILES ${SURGSIM_DATA_STRUCTURES_HEADERS} DESTINATION include/SurgSim/DataStructures)

set_target_properties(SurgSimDataStructures PROPERTIES 
	PUBLIC_HEADER "${SURGSIM_DATA_STRUCTURES_HEADERS}")

install(TARGETS SurgSimDataStructures
	EXPORT ${PROJECT_NAME}Targets
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	PUBLIC_HEADER DESTINATION include/SurgSim/DataStructures)

if(SURGSIM_TESTS_BUILD)
	add_subdirectory(UnitTests)
endif()
