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

#include "SurgSim/DataStructures/Vertex.h"
#include "SurgSim/DataStructures/MeshElement.h"
#include "SurgSim/DataStructures/UnitTests/MockObjects.h"

#include <random>

using SurgSim::DataStructures::TriangleMesh;
using SurgSim::Math::Vector3d;

class TriangleMeshTest : public ::testing::Test
{
public:
	void SetUp()
	{
		// Set to true to print the test positions.
		bool printPositions = false;
		// Set to true to print the test normals.
		bool printNormals = false;
		// Set to true to print the test triangles.
		bool printTriangles = false;
		// Set to true to print the test edges.
		bool printEdges = false;
		// Set the number of test vertices
		unsigned int numVertices = 10;
		// Set the number of test triangles
		unsigned int numTriangles = 20;

		std::default_random_engine generator;
		std::uniform_real_distribution<double> positionDistribution(-10.0, 10.0);
		std::uniform_real_distribution<double> normalDistribution(-1.0, 1.0);
		std::uniform_int_distribution<unsigned int> vertexIdDistribution(0, numVertices);

		if (printPositions)
		{
			printf("Test Vertex Positions:\n");
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
			printf("Test Vertex Normals:\n");
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

		if (printTriangles)
		{
			printf("Test Triangles:\n");
		}

		/// Generate random vertex IDs within [0, numVertices) in triplets for mesh triangles
		for (unsigned int i = 0; i < numTriangles; ++i)
		{
			std::array<unsigned int, 3> triangleVertices = {{ vertexIdDistribution(generator),
				vertexIdDistribution(generator), vertexIdDistribution(generator) }};
			testTriangleVertices.push_back(triangleVertices);

			/// Create 3 vertex ID pairs for each triangle edge (not worrying about duplicates for these tests)
			std::array<unsigned int, 3> triangleEdges;
			for (int j = 0; j < 3; ++j)
			{
				std::array<unsigned int, 2> edgeVertices = {{ triangleVertices[0], triangleVertices[1] }};
				testEdgeVertices.push_back(edgeVertices);

				triangleEdges[j] = testEdgeVertices.size() - 1;
			}
			testTriangleEdges.push_back(triangleEdges);

			if (printTriangles)
			{
				printf("\t%d: Vertices (%d, %d, %d), Edges (%d, %d, %d)\n", i,
					triangleVertices[0], triangleVertices[1], triangleVertices[2],
					triangleEdges[0], triangleEdges[1], triangleEdges[2]);
			}
		}

		if (printEdges)
		{
			printf("Test Edges:\n");

			for (unsigned int i = 0; i < testEdgeVertices.size(); ++i)
			{
				const std::array<unsigned int, 2>& edgeVertices = testEdgeVertices[i];
				printf("\t%d: (%d, %d)\n", i, edgeVertices[0], edgeVertices[1]);
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

	/// Vertices of test edges
	std::vector<std::array<unsigned int, 2>> testEdgeVertices;

	/// Vertices of test triangles
	std::vector<std::array<unsigned int, 3>> testTriangleVertices;
	/// Edges of test triangles
	std::vector<std::array<unsigned int, 3>> testTriangleEdges;
};


TEST_F(TriangleMeshTest, InitTest)
{
	ASSERT_NO_THROW({MockTriangleMesh mesh;});

	/// Make sure we can create triangle meshes with each possible combination of void data.
	typedef TriangleMesh<void, MockEdgeData, MockTriangleData> TriangleMeshNoVertexData;
	typedef TriangleMesh<MockVertexData, void, MockTriangleData> TriangleMeshNoEdgeData;
	typedef TriangleMesh<MockVertexData, MockEdgeData, void> TriangleMeshNoTriangleData;

	typedef TriangleMesh<MockVertexData, void, void> TriangleMeshNoEdgeOrTriangleData;
	typedef TriangleMesh<void, MockEdgeData, void> TriangleMeshNoVertexOrTriangleData;
	typedef TriangleMesh<void, void, MockTriangleData> TriangleMeshNoVertexOrEdgeData;

	typedef TriangleMesh<void, void, void> TriangleMeshNoData;

	ASSERT_NO_THROW({TriangleMeshNoVertexData mesh;});
	ASSERT_NO_THROW({TriangleMeshNoEdgeData mesh;});
	ASSERT_NO_THROW({TriangleMeshNoTriangleData mesh;});

	ASSERT_NO_THROW({TriangleMeshNoEdgeOrTriangleData mesh;});
	ASSERT_NO_THROW({TriangleMeshNoVertexOrTriangleData mesh;});
	ASSERT_NO_THROW({TriangleMeshNoVertexOrEdgeData mesh;});

	ASSERT_NO_THROW({TriangleMeshNoData mesh;});
}

TEST_F(TriangleMeshTest, CreateVerticesTest)
{
	MockTriangleMesh mesh;

	EXPECT_EQ(0u, mesh.getNumVertices());
	EXPECT_EQ(0u, mesh.getVertices().size());

	/// Create the test vertices
	for (unsigned int i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createVertex(testPositions[i], testNormals[i]));
		EXPECT_EQ(i + 1, mesh.getNumVertices());

		const std::vector<MockTriangleMesh::Vertex>& vertices = mesh.getVertices();
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

	/// Create the test edges
	for (unsigned int i = 0; i < testEdgeVertices.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createEdge(testEdgeVertices[i]));
		EXPECT_EQ(i + 1, mesh.getNumEdges());

		const std::vector<MockTriangleMesh::Edge>& edges = mesh.getEdges();
		EXPECT_EQ(i + 1, edges.size());

		/// Make sure each vertex is set properly
		for (unsigned int j = 0; j < mesh.getNumEdges(); ++j)
		{
			EXPECT_EQ(testEdgeVertices[j], edges[j].vertices);

			const MockEdgeData& data = edges[j].data;
			EXPECT_EQ(j, data.getId());
		}
	}

	/// Create the test triangles
	for (unsigned int i = 0; i < testTriangleVertices.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createTriangle(testTriangleVertices[i], testTriangleEdges[i]));
		EXPECT_EQ(i + 1, mesh.getNumTriangles());

		const std::vector<MockTriangleMesh::Triangle>& triangles = mesh.getTriangles();
		EXPECT_EQ(i + 1, triangles.size());

		/// Make sure each vertex is set properly
		for (unsigned int j = 0; j < mesh.getNumTriangles(); ++j)
		{
			EXPECT_EQ(testTriangleVertices[j], triangles[j].vertices);

			const MockTriangleData& data = triangles[j].data;
			EXPECT_EQ(j, data.getId());
			EXPECT_EQ(testTriangleEdges[j], data.getEdges());
		}
	}
}

TEST_F(TriangleMeshTest, SetVertexPositionsTest)
{
	MockTriangleMesh mesh;

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

TEST_F(TriangleMeshTest, ClearTest)
{
	MockTriangleMesh mesh;

	EXPECT_EQ(0, mesh.getNumUpdates());

	EXPECT_EQ(0u, mesh.getNumVertices());
	EXPECT_EQ(0u, mesh.getVertices().size());

	EXPECT_EQ(0u, mesh.getNumEdges());
	EXPECT_EQ(0u, mesh.getEdges().size());

	EXPECT_EQ(0u, mesh.getNumTriangles());
	EXPECT_EQ(0u, mesh.getTriangles().size());

	/// Create mesh using test data
	for (unsigned int i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createVertex(Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0)));
		EXPECT_EQ(i + 1, mesh.getNumVertices());
	}
	for (unsigned int i = 0; i < testEdgeVertices.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createEdge(testEdgeVertices[i]));
		EXPECT_EQ(i + 1, mesh.getNumEdges());
	}
	for (unsigned int i = 0; i < testTriangleVertices.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createTriangle(testTriangleVertices[i], testTriangleEdges[i]));
		EXPECT_EQ(i + 1, mesh.getNumTriangles());
	}

	EXPECT_EQ(testPositions.size(), mesh.getNumVertices());
	EXPECT_EQ(testPositions.size(), mesh.getVertices().size());

	EXPECT_EQ(testEdgeVertices.size(), mesh.getNumEdges());
	EXPECT_EQ(testEdgeVertices.size(), mesh.getEdges().size());

	EXPECT_EQ(testTriangleVertices.size(), mesh.getNumTriangles());
	EXPECT_EQ(testTriangleVertices.size(), mesh.getTriangles().size());

	/// Clear mesh
	mesh.clear();

	EXPECT_EQ(0u, mesh.getNumVertices());
	EXPECT_EQ(0u, mesh.getVertices().size());

	EXPECT_EQ(0u, mesh.getNumEdges());
	EXPECT_EQ(0u, mesh.getEdges().size());

	EXPECT_EQ(0u, mesh.getNumTriangles());
	EXPECT_EQ(0u, mesh.getTriangles().size());
}

TEST_F(TriangleMeshTest, UpdateTest)
{
	MockTriangleMesh mesh;

	EXPECT_EQ(0, mesh.getNumUpdates());

	for (int i = 0; i < 10; ++i)
	{
		mesh.update();
		EXPECT_EQ(i + 1, mesh.getNumUpdates());
	}
}

TEST_F(TriangleMeshTest, ComparisonTest)
{
	/// Create mesh using test data
	MockTriangleMesh mesh;
	for (unsigned int i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createVertex(testPositions[i], testNormals[i]));
	}
	for (unsigned int i = 0; i < testEdgeVertices.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createEdge(testEdgeVertices[i]));
	}
	for (unsigned int i = 0; i < testTriangleVertices.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createTriangle(testTriangleVertices[i], testTriangleEdges[i]));
	}

	/// Create same mesh again
	MockTriangleMesh sameMesh;

	for (unsigned int i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, sameMesh.createVertex(testPositions[i], testNormals[i]));
	}
	for (unsigned int i = 0; i < testEdgeVertices.size(); ++i)
	{
		EXPECT_EQ(i, sameMesh.createEdge(testEdgeVertices[i]));
	}
	for (unsigned int i = 0; i < testTriangleVertices.size(); ++i)
	{
		EXPECT_EQ(i, sameMesh.createTriangle(testTriangleVertices[i], testTriangleEdges[i]));
	}

	/// Create mesh with test data, but each vertex has position and normal of (0,0,0) to make them different
	MockTriangleMesh meshWithDifferentVertices;
	for (unsigned int i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, meshWithDifferentVertices.createVertex(Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0)));
	}
	for (unsigned int i = 0; i < testEdgeVertices.size(); ++i)
	{
		EXPECT_EQ(i, meshWithDifferentVertices.createEdge(testEdgeVertices[i]));
	}
	for (unsigned int i = 0; i < testTriangleVertices.size(); ++i)
	{
		EXPECT_EQ(i, meshWithDifferentVertices.createTriangle(testTriangleVertices[i], testTriangleEdges[i]));
	}

	/// Create mesh with test data, but reverse each edge's vertex order to make them different
	MockTriangleMesh meshWithDifferentEdges;
	for (unsigned int i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, meshWithDifferentEdges.createVertex(testPositions[i], testNormals[i]));
	}
	for (unsigned int i = 0; i < testEdgeVertices.size(); ++i)
	{
		std::array<unsigned int, 2> edge = {{ testEdgeVertices[i][1], testEdgeVertices[i][0] }};
		EXPECT_EQ(i, meshWithDifferentEdges.createEdge(edge));
	}
	for (unsigned int i = 0; i < testTriangleVertices.size(); ++i)
	{
		EXPECT_EQ(i, meshWithDifferentEdges.createTriangle(testTriangleVertices[i], testTriangleEdges[i]));
	}

	/// Create mesh with test data, but only create half of the triangles to make the list different.
	MockTriangleMesh meshWithDifferentTriangles;
	for (unsigned int i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, meshWithDifferentTriangles.createVertex(testPositions[i], testNormals[i]));
	}
	for (unsigned int i = 0; i < testEdgeVertices.size(); ++i)
	{
		EXPECT_EQ(i, meshWithDifferentTriangles.createEdge(testEdgeVertices[i]));
	}
	for (unsigned int i = 0; i < testTriangleVertices.size() / 2; ++i)
	{
		EXPECT_EQ(i, meshWithDifferentTriangles.createTriangle(testTriangleVertices[i], testTriangleEdges[i]));
	}

	/// Test comparisons
	EXPECT_TRUE(mesh == sameMesh);
	EXPECT_FALSE(mesh != sameMesh);

	EXPECT_FALSE(mesh == meshWithDifferentVertices);
	EXPECT_TRUE(mesh != meshWithDifferentVertices);

	EXPECT_FALSE(mesh == meshWithDifferentEdges);
	EXPECT_TRUE(mesh != meshWithDifferentEdges);

	EXPECT_FALSE(mesh == meshWithDifferentTriangles);
	EXPECT_TRUE(mesh != meshWithDifferentTriangles);
}