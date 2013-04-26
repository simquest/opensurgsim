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

set(SURGSIM_DATA_STRUCTURES_MESHES_SOURCES
	Mesh.cpp
	
	MeshVertex.cpp
	MeshVertexData.cpp
)

set(SURGSIM_DATA_STRUCTURES_MESHES_HEADERS
	Mesh.h
	MeshElement.h
	MeshElement-inl.h
	MeshElementData.h
	MeshElementData-inl.h
	MeshVertex.h
	MeshVertexData.h
)

add_library(SurgSimDataStructuresMeshes
	${SURGSIM_DATA_STRUCTURES_MESHES_SOURCES} ${SURGSIM_DATA_STRUCTURES_MESHES_HEADERS}
)

install(FILES ${SURGSIM_DATA_STRUCTURES_MESHES_HEADERS} DESTINATION include/SurgSim/DataStructures/Meshes)

install(TARGETS SurgSimDataStructuresMeshes
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib)

if(SURGSIM_TESTS_BUILD)
	add_subdirectory(UnitTests)
endif()
