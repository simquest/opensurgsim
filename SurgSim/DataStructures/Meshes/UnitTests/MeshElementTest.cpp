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

/** @file
 * Tests for the MeshElement class.
 */

#include "gtest/gtest.h"

#include "SurgSim/DataStructures/Meshes/MeshElement.h"
#include "SurgSim/DataStructures/Meshes/MeshElementData.h"
#include "SurgSim/DataStructures/Meshes/UnitTests/MockObjects.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::Math::Vector3d;

/// Edge element
typedef SurgSim::DataStructures::MeshElement<2> Edge;
/// Triangle element
typedef SurgSim::DataStructures::MeshElement<3> Triangle;

TEST(MeshElementTest, InitTest)
{
	std::array<unsigned int, 2> edgeVertices = {{0, 1}};
	ASSERT_NO_THROW({Edge edge(edgeVertices);});

	std::shared_ptr<MockEdgeData> edgeData = std::make_shared<MockEdgeData>(0);
	ASSERT_NO_THROW({Edge edge(edgeVertices, edgeData);});

	std::array<unsigned int, 3> triangleVertices = {{0, 1, 2}};
	ASSERT_NO_THROW({Triangle triangle(triangleVertices);});

	std::array<unsigned int, 3> triangleEdges = {{0, 1, 2}};
	std::shared_ptr<MockTriangleData> triangleData = std::make_shared<MockTriangleData>(0, triangleEdges);
	ASSERT_NO_THROW({Triangle triangle(triangleVertices, triangleData);});
}

TEST(MeshElementTest, EdgeTest)
{
	std::array<unsigned int, 2> edgeVertices = {{0, 1}};
	Edge edge(edgeVertices);

	EXPECT_EQ(edgeVertices, edge.vertices);
	EXPECT_EQ(nullptr, edge.data);

	/// Check comparisons

	Edge sameEdge(edgeVertices);
	EXPECT_TRUE(edge == sameEdge);
	EXPECT_FALSE(edge != sameEdge);

	std::array<unsigned int, 2> differentEdgeVertices = {{1, 2}};
	Edge differentEdge(differentEdgeVertices);
	EXPECT_FALSE(edge == differentEdge);
	EXPECT_TRUE(edge != differentEdge);
}

TEST(MeshElementTest, EdgeWithDataTest)
{
	std::array<unsigned int, 2> edgeVertices = {{2, 10}};
	std::shared_ptr<MockEdgeData> edgeData = std::make_shared<MockEdgeData>(5);
	Edge edge(edgeVertices, edgeData);

	EXPECT_EQ(edgeVertices, edge.vertices);
	EXPECT_NE(nullptr, edge.data);

	{
		std::shared_ptr<MockEdgeData> data = std::dynamic_pointer_cast<MockEdgeData>(edge.data);
		EXPECT_EQ(5, data->getId());
	}
}

TEST(MeshElementTest, TriangleTest)
{
	std::array<unsigned int, 3> triangleVertices = {{5, 2, 10}};
	Triangle triangle(triangleVertices);

	EXPECT_EQ(triangleVertices, triangle.vertices);
	EXPECT_EQ(nullptr, triangle.data);

	/// Check comparisons

	Triangle sameTriangle(triangleVertices);
	EXPECT_TRUE(triangle == sameTriangle);
	EXPECT_FALSE(triangle != sameTriangle);

	std::array<unsigned int, 3> differentTriangleVertices = {{10, 5, 7}};
	Triangle differentTriangle(differentTriangleVertices);
	EXPECT_FALSE(triangle == differentTriangle);
	EXPECT_TRUE(triangle != differentTriangle);
}

TEST(MeshElementTest, TriangleWithDataTest)
{
	std::array<unsigned int, 3> triangleVertices = {{5, 2, 10}};
	std::array<unsigned int, 3> triangleEdges = {{0, 1, 2}};
	std::shared_ptr<MockTriangleData> triangleData = std::make_shared<MockTriangleData>(4, triangleEdges);
	Triangle triangle(triangleVertices, triangleData);

	EXPECT_EQ(triangleVertices, triangle.vertices);
	EXPECT_NE(nullptr, triangle.data);

	{
		std::shared_ptr<MockTriangleData> data = std::dynamic_pointer_cast<MockTriangleData>(triangle.data);
		EXPECT_EQ(4, data->getId());
		EXPECT_EQ(triangleEdges, data->getEdges());
	}

	/// Check comparisons

	Triangle sameTriangle(triangleVertices, triangleData);
	EXPECT_TRUE(triangle == sameTriangle);
	EXPECT_FALSE(triangle != sameTriangle);

	std::array<unsigned int, 3> differentTriangleVertices = {{10, 5, 7}};
	std::array<unsigned int, 3> differentTriangleEdges = {{2, 1, 3}};
	std::shared_ptr<MockTriangleData> differentTriangleData = std::make_shared<MockTriangleData>(4, differentTriangleEdges);

	Triangle triangleWithDifferentVertices(differentTriangleVertices, triangleData);
	EXPECT_FALSE(triangle == triangleWithDifferentVertices);
	EXPECT_TRUE(triangle != triangleWithDifferentVertices);

	Triangle triangleWithDifferentData(triangleVertices, differentTriangleData);
	EXPECT_FALSE(triangle == triangleWithDifferentData);
	EXPECT_TRUE(triangle != triangleWithDifferentData);

	Triangle triangleWithDifferentVerticesAndData(differentTriangleVertices, differentTriangleData);
	EXPECT_FALSE(triangle == triangleWithDifferentVerticesAndData);
	EXPECT_TRUE(triangle != triangleWithDifferentVerticesAndData);

	Triangle triangleWithNoData(triangleVertices);
	EXPECT_FALSE(triangle == triangleVertices);
	EXPECT_TRUE(triangle != triangleVertices);

	Triangle differentTriangleWithNoData(differentTriangleVertices);
	EXPECT_FALSE(triangle == differentTriangleVertices);
	EXPECT_TRUE(triangle != differentTriangleVertices);
}
