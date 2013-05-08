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
/// Tests for the Mesh class.

#include "gtest/gtest.h"

#include "SurgSim/DataStructures/MeshVertex.h"
#include "SurgSim/DataStructures/MeshElement.h"
#include "SurgSim/DataStructures/UnitTests/MockObjects.h"

#include <random>

using SurgSim::DataStructures::MeshVertex;
using SurgSim::Math::Vector3d;

class MeshTest : public ::testing::Test
{
public:
	void SetUp()
	{
		// Set to true to print the test positions.
		bool printPositions = false;
		// Set to true to print the test normals.
		bool printNormals = false;
		// Set the number of test vertices
		unsigned int numVertices = 10;

		std::default_random_engine generator;
		std::uniform_real_distribution<double> positionDistribution(-10.0, 10.0);
		std::uniform_real_distribution<double> normalDistribution(-1.0, 1.0);

		if (printPositions)
		{
			printf("Test Positions:\n");
		}

		/// Generate random positions for each vertex
		for (unsigned int i = 0; i < numVertices; ++i)
		{
			Vector3d position(positionDistribution(generator), positionDistribution(generator),
				positionDistribution(generator));
			testPositions.push_back(position);

			if (printPositions)
			{
				printf("\t%d: (%g, %g, %g)\n", i, position.x(), position.y(), position.z());
			}
		}

		if (printNormals)
		{
			printf("Test Normals:\n");
		}

		/// Generate random normals for each vertex
		for (unsigned int i = 0; i < numVertices; ++i)
		{
			Vector3d normal(normalDistribution(generator), normalDistribution(generator),
				normalDistribution(generator));
			normal.normalize();
			testNormals.push_back(normal);

			if (printNormals)
			{
				printf("\t%d: (%g, %g, %g)\n", i, normal.x(), normal.y(), normal.z());
			}
		}

	}

	void TearDown()
	{

	}

	/// Positions of test vertices
	std::vector<Vector3d> testPositions;
	/// Normals of test vertices
	std::vector<Vector3d> testNormals;
};


TEST_F(MeshTest, InitTest)
{
	ASSERT_NO_THROW({MockMesh mesh;});
}

TEST_F(MeshTest, CreateVerticesTest)
{
	MockMesh mesh;

	EXPECT_EQ(0, mesh.getNumVertices());
	EXPECT_EQ(0, mesh.getVertices().size());

	/// Create the test vertices
	for (unsigned int i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createVertex(testPositions[i], testNormals[i]));
		EXPECT_EQ(i + 1, mesh.getNumVertices());

		const std::vector<MockMesh::Vertex>& vertices = mesh.getVertices();
		EXPECT_EQ(i + 1, vertices.size());

		/// Make sure each vertex is set properly
		for (unsigned int j = 0; j < mesh.getNumVertices(); ++j)
		{
			EXPECT_EQ(testPositions[j], vertices[j].position);

			const MockVertexData& data = vertices[j].data;
			EXPECT_EQ(j, data.getId());
			EXPECT_EQ(testNormals[j], data.getNormal());
		}
	}
}

TEST_F(MeshTest, SetVertexPositionsTest)
{
	MockMesh mesh;

	/// Create vertices with test normals, but all positions at (0,0,0)
	for (unsigned int i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createVertex(Vector3d(0.0, 0.0, 0.0), testNormals[i]));
		EXPECT_EQ(i + 1, mesh.getNumVertices());
	}

	mesh.setVertexPositions(testPositions);

	EXPECT_EQ(1, mesh.getNumUpdates());
	EXPECT_EQ(testPositions.size(), mesh.getNumVertices());

	const std::vector<MockMesh::Vertex>& vertices = mesh.getVertices();
	EXPECT_EQ(testPositions.size(), vertices.size());

	/// Make sure each vertex is set properly
	for (unsigned int i = 0; i < mesh.getNumVertices(); ++i)
	{
		EXPECT_EQ(testPositions[i], vertices[i].position);

		const MockVertexData& data = vertices[i].data;
		EXPECT_EQ(testNormals[i], data.getNormal());
	}

	mesh.setVertexPositions(testPositions, false);

	EXPECT_EQ(1, mesh.getNumUpdates());
	EXPECT_EQ(testPositions.size(), mesh.getNumVertices());
	EXPECT_EQ(testPositions.size(), mesh.getVertices().size());

	mesh.setVertexPositions(testPositions, true);

	EXPECT_EQ(2, mesh.getNumUpdates());
	EXPECT_EQ(testPositions.size(), mesh.getNumVertices());
	EXPECT_EQ(testPositions.size(), mesh.getVertices().size());

	/// Test the individual set/get methods
	mesh.setVertexPosition(5, Vector3d(0.0, 0.0, 0.0));

	/// Make sure each vertex is set properly
	for (unsigned int i = 0; i < mesh.getNumVertices(); ++i)
	{
		if (i == 5)
		{
			EXPECT_EQ(Vector3d(0.0, 0.0, 0.0), mesh.getVertexPosition(i));
			EXPECT_EQ(testNormals[i],  mesh.getVertexNormal(i));
		}
		else
		{
			EXPECT_EQ(testPositions[i], mesh.getVertexPosition(i));
			EXPECT_EQ(testNormals[i],  mesh.getVertexNormal(i));
		}
	}

	/// Try setting with wrong number of vertices
	mesh.createVertex(Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0)); // create one more vertex

	EXPECT_ANY_THROW(mesh.setVertexPositions(testPositions));
}

TEST_F(MeshTest, ClearTest)
{
	MockMesh mesh;

	EXPECT_EQ(0, mesh.getNumUpdates());
	EXPECT_EQ(0, mesh.getNumVertices());
	EXPECT_EQ(0, mesh.getVertices().size());

	/// Create vertices
	for (unsigned int i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createVertex(Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0)));
		EXPECT_EQ(i + 1, mesh.getNumVertices());
	}

	EXPECT_EQ(testPositions.size(), mesh.getNumVertices());
	EXPECT_EQ(testPositions.size(), mesh.getVertices().size());

	/// Clear mesh
	mesh.clear();

	EXPECT_EQ(0, mesh.getNumVertices());
	EXPECT_EQ(0, mesh.getVertices().size());
}

TEST_F(MeshTest, UpdateTest)
{
	MockMesh mesh;

	EXPECT_EQ(0, mesh.getNumUpdates());

	for (int i = 0; i < 10; ++i)
	{
		mesh.update();
		EXPECT_EQ(i + 1, mesh.getNumUpdates());
	}
}

TEST_F(MeshTest, ComparisonTest)
{
	MockMesh mesh;

	/// Create vertices using test positions and normals
	for (unsigned int i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createVertex(testPositions[i], testNormals[i]));
	}

	MockMesh sameMesh;

	/// Create same mesh again
	for (unsigned int i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, sameMesh.createVertex(testPositions[i], testNormals[i]));
	}

	MockMesh differentMesh;

	/// Create vertices, each with position and normal of (0,0,0)
	for (unsigned int i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, differentMesh.createVertex(Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0)));
	}

	/// Test comparisons
	EXPECT_TRUE(mesh == sameMesh);
	EXPECT_FALSE(mesh != sameMesh);

	EXPECT_FALSE(mesh == differentMesh);
	EXPECT_TRUE(mesh != differentMesh);
}