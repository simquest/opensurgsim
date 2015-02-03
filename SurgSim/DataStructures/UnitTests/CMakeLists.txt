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

include_directories (
	${gtest_SOURCE_DIR}/include
	${OSG_INCLUDE_DIR}
)

set(UNIT_TEST_SOURCES
	AabbTreeDataTests.cpp
	AabbTreeNodeTests.cpp
	AabbTreeTests.cpp
	BufferedValueTests.cpp
	DataGroupTests.cpp
	DataStructuresConvertTests.cpp
	Grid1DTests.cpp
	Grid2DTests.cpp
	Grid3DTests.cpp
	GridTests.cpp
	GroupsTests.cpp
	ImageTest.cpp
	IndexDirectoryTests.cpp
	IndexedLocalCoordinateTest.cpp
	LocationTests.cpp
	MeshElementTest.cpp
	MeshTest.cpp
	MeshVertexTest.cpp
	NamedDataTests.cpp
	NamedVariantDataTests.cpp
	OctreeNodeTests.cpp
	OptionalValueTests.cpp
	PlyReaderTests.cpp
	TetrahedronMeshTest.cpp
	TriangleMeshBaseTest.cpp
	TriangleMeshTest.cpp
)

set(UNIT_TEST_HEADERS
	GridTests.h
	MockObjects.h
)

set(LIBS
	SurgSimDataStructures
	SurgSimGraphics
	SurgSimMath
)

file(COPY Data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY ${SURGSIM_SOURCE_DIR}/SurgSim/Testing/TriangleMeshBaseTests DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY ${SURGSIM_SOURCE_DIR}/SurgSim/Testing/MeshShapeData DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Data)
file(COPY ${SURGSIM_SOURCE_DIR}/SurgSim/Testing/OctreeShapeData DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Data)

configure_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/config.txt.in"
	"${CMAKE_CURRENT_BINARY_DIR}/config.txt"
)

surgsim_add_unit_tests(SurgSimDataStructuresTest)

set_target_properties(SurgSimDataStructuresTest PROPERTIES FOLDER "DataStructures")
