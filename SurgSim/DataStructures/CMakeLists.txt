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
	AabbTree.cpp
	AabbTreeData.cpp
	AabbTreeIntersectionVisitor.cpp
	AabbTreeNode.cpp
	DataGroupBuilder.cpp
	DataGroupCopier.cpp
	IndexDirectory.cpp
	OctreeNode.cpp
	ply.c
	PlyReader.cpp
	Tree.cpp
	TreeData.cpp
	TreeNode.cpp
	TriangleMesh.cpp
	TriangleMeshBase.cpp
	TriangleMeshPlyReaderDelegate.cpp
)

set(SURGSIM_DATA_STRUCTURES_HEADERS
	AabbTree.h
	AabbTreeData.h
	AabbTreeIntersectionVisitor.h
	AabbTreeNode.h
	BufferedValue.h
	BufferedValue-inl.h
	DataGroup.h
	DataGroup-inl.h
	DataGroupBuilder.h
	DataGroupCopier.h
	EmptyData.h
	IndexDirectory.h
	MeshElement.h
	NamedData.h
	NamedData-inl.h
	NamedDataBuilder.h
	NamedVariantData.h
	NamedVariantData-inl.h
	OctreeNode.h
	OctreeNode-inl.h
	OptionalValue.h
	ply.h
	PlyReader.h
	PlyReaderDelegate.h
	TetrahedronMesh.h
	TetrahedronMesh-inl.h
	Tree.h
	TreeData.h
	TreeNode.h
	TreeVisitor.h
	TriangleMesh.h
	TriangleMesh-inl.h
	TriangleMeshBase.h
	TriangleMeshBase-inl.h
	TriangleMeshPlyReaderDelegate.h
	Vertex.h
	Vertices.h
)

surgsim_add_library(
	SurgSimDataStructures
	"${SURGSIM_DATA_STRUCTURES_SOURCES}"
	"${SURGSIM_DATA_STRUCTURES_HEADERS}"
	"SurgSim/DataStructures"
)

set(LIBS 
	SurgSimMath 
	SurgSimFramework
	${Boost_LIBRARIES}
)

target_link_libraries(SurgSimDataStructures ${LIBS}
)

if(BUILD_TESTING)
	add_subdirectory(UnitTests)
endif()

set_target_properties(SurgSimDataStructures PROPERTIES FOLDER "DataStructures")
