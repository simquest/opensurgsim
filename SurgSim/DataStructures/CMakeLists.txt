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


set(SURGSIM_DATA_STRUCTURES_SOURCES
	AabbTree.cpp
	AabbTreeData.cpp
	AabbTreeIntersectionVisitor.cpp
	AabbTreeNode.cpp
	DataGroup.cpp
	DataGroupBuilder.cpp
	DataGroupCopier.cpp
	IndexDirectory.cpp
	IndexedLocalCoordinate.cpp
	OctreeNode.cpp
	OctreeNodePlyReaderDelegate.cpp
	ply.c
	PlyReader.cpp
	Tree.cpp
	TreeData.cpp
	TreeNode.cpp
	TriangleMesh.cpp
	TriangleMeshUtilities.cpp
)

set(SURGSIM_DATA_STRUCTURES_HEADERS
	AabbTree.h
	AabbTreeData.h
	AabbTreeIntersectionVisitor.h
	AabbTreeNode.h
	BufferedValue.h
	BufferedValue-inl.h
	DataGroup.h
	DataGroupBuilder.h
	DataGroupCopier.h
	DataStructuresConvert.h
	DataStructuresConvert-inl.h
	EmptyData.h
	Grid.h
	Grid-inl.h
	Groups.h
	Groups-inl.h
	Image.h
	Image-inl.h
	IndexDirectory.h
	IndexedLocalCoordinate.h
	Location.h
	MeshElement.h
	NamedData.h
	NamedData-inl.h
	NamedDataBuilder.h
	NamedVariantData.h
	NamedVariantData-inl.h
	OctreeNode.h
	OctreeNode-inl.h
	OctreeNodePlyReaderDelegate.h
	OctreeNodePlyReaderDelegate-inl.h
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
	TriangleMeshPlyReaderDelegate-inl.h
	TriangleMeshUtilities.h
	TriangleMeshUtilities-inl.h
	Vertex.h
	Vertices.h
)
surgsim_create_library_header(DataStructures.h "${SURGSIM_DATA_STRUCTURES_HEADERS}")

surgsim_add_library(
	SurgSimDataStructures
	"${SURGSIM_DATA_STRUCTURES_SOURCES}"
	"${SURGSIM_DATA_STRUCTURES_HEADERS}"
)

set(LIBS 
	SurgSimFramework
	${Boost_LIBRARIES}
)

target_link_libraries(SurgSimDataStructures ${LIBS}
)

if(BUILD_TESTING)
	add_subdirectory(UnitTests)

	if(BUILD_PERFORMANCE_TESTING)
		add_subdirectory(PerformanceTests)
	endif()
endif()

set_target_properties(SurgSimDataStructures PROPERTIES FOLDER "DataStructures")
