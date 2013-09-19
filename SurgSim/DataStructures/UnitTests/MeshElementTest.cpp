// This file is a part of the OpenSurgSim project.
// Copyright 2012-2013, SimQuest Solutions Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \file
/// Tests for the MeshElement class.
/// Edges and Triangles are used as example elements for testing.

#include "gtest/gtest.h"

#include "SurgSim/DataStructures/MeshElement.h"
#include "SurgSim/DataStructures/UnitTests/MockObjects.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::Math::Vector3d;

/// Edge element with ID data.
typedef SurgSim::DataStructures::MeshElement<2, MockEdgeData> MockEdge;
/// Triangle element with ID and edge ID data.
typedef SurgSim::DataStructures::MeshElement<3, MockTriangleData> MockTriangle;
/// Tetrahedron element with no data
typedef SurgSim::DataStructures::MeshElement<4, void> MockTetrahedron;

TEST(MeshElementTest, InitTest)
{
	std::array<unsigned int, 2> edgeVertices = {{0, 1}};
	MockEdgeData edgeData(0);
	ASSERT_NO_THROW({MockEdge edge(edgeVertices, edgeData);});

	std::array<unsigned int, 3> triangleVertices = {{0, 1, 2}};
	std::array<unsigned int, 3> triangleEdges = {{0, 1, 2}};
	MockTriangleData triangleData(0, triangleEdges);
	ASSERT_NO_THROW({MockTriangle triangle(triangleVertices, triangleData);});

	std::array<unsigned int, 4> tetrahedronVertices = {{0, 1, 2, 3}};
	ASSERT_NO_THROW({MockTetrahedron triangle(tetrahedronVertices);});
}

TEST(MeshElementTest, EdgeTest)
{
	std::array<unsigned int, 2> edgeVertices = {{2, 10}};
	MockEdgeData edgeData(5);
	MockEdge edge(edgeVertices, edgeData);

	EXPECT_EQ(edgeVertices, edge.verticesId);
	EXPECT_EQ(edgeData, edge.data);

	{
		const MockEdgeData& data = edge.data;
		EXPECT_EQ(5u, data.getId());
	}

	/// Check comparisons

	MockEdge sameEdge(edgeVertices, edgeData);
	EXPECT_TRUE(edge == sameEdge);
	EXPECT_FALSE(edge != sameEdge);

	std::array<unsigned int, 2> differentEdgeVertices = {{10, 5}};
	MockEdgeData differentEdgeData(7);

	MockEdge edgeWithDifferentVertices(differentEdgeVertices, edgeData);
	EXPECT_FALSE(edge == edgeWithDifferentVertices);
	EXPECT_TRUE(edge != edgeWithDifferentVertices);

	MockEdge edgeWithDifferentData(edgeVertices, differentEdgeData);
	EXPECT_FALSE(edge == edgeWithDifferentData);
	EXPECT_TRUE(edge != edgeWithDifferentData);

	MockEdge edgeWithDifferentVerticesAndData(differentEdgeVertices, differentEdgeData);
	EXPECT_FALSE(edge == edgeWithDifferentVerticesAndData);
	EXPECT_TRUE(edge != edgeWithDifferentVerticesAndData);
}

TEST(MeshElementTest, TriangleTest)
{
	std::array<unsigned int, 3> triangleVertices = {{5, 2, 10}};
	std::array<unsigned int, 3> triangleEdges = {{0, 1, 2}};
	MockTriangleData triangleData(4, triangleEdges);
	MockTriangle triangle(triangleVertices, triangleData);

	EXPECT_EQ(triangleVertices, triangle.verticesId);
	EXPECT_EQ(triangleData, triangle.data);

	{
		const MockTriangleData& data = triangle.data;
		EXPECT_EQ(4u, data.getId());
		EXPECT_EQ(triangleEdges, data.getEdges());
	}

	/// Check comparisons

	MockTriangle sameTriangle(triangleVertices, triangleData);
	EXPECT_TRUE(triangle == sameTriangle);
	EXPECT_FALSE(triangle != sameTriangle);

	std::array<unsigned int, 3> differentTriangleVertices = {{10, 5, 7}};
	std::array<unsigned int, 3> differentTriangleEdges = {{2, 1, 3}};
	MockTriangleData differentTriangleData(4, differentTriangleEdges);

	MockTriangle triangleWithDifferentVertices(differentTriangleVertices, triangleData);
	EXPECT_FALSE(triangle == triangleWithDifferentVertices);
	EXPECT_TRUE(triangle != triangleWithDifferentVertices);

	MockTriangle triangleWithDifferentData(triangleVertices, differentTriangleData);
	EXPECT_FALSE(triangle == triangleWithDifferentData);
	EXPECT_TRUE(triangle != triangleWithDifferentData);

	MockTriangle triangleWithDifferentVerticesAndData(differentTriangleVertices, differentTriangleData);
	EXPECT_FALSE(triangle == triangleWithDifferentVerticesAndData);
	EXPECT_TRUE(triangle != triangleWithDifferentVerticesAndData);
}

TEST(MeshElementTest, TetrahedronTest)
{
	std::array<unsigned int, 4> tetrahedronVertices = {{5, 2, 10, 6}};
	MockTetrahedron tetrahedron(tetrahedronVertices);

	EXPECT_EQ(tetrahedronVertices, tetrahedron.verticesId);

	/// Check comparisons

	MockTetrahedron sameTetrahedron(tetrahedronVertices);
	EXPECT_TRUE(tetrahedron == sameTetrahedron);
	EXPECT_FALSE(tetrahedron != sameTetrahedron);

	std::array<unsigned int, 4> differentTetrahedronVertices = {{10, 5, 7, 3}};

	MockTetrahedron tetrahedronWithDifferentVertices(differentTetrahedronVertices);
	EXPECT_FALSE(tetrahedron == tetrahedronWithDifferentVertices);
	EXPECT_TRUE(tetrahedron != tetrahedronWithDifferentVertices);
}