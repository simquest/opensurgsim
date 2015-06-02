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
/// Tests for the Tetrahedron class.

#include "gtest/gtest.h"

#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/Vertex.h"
#include "SurgSim/DataStructures/MeshElement.h"
#include "SurgSim/DataStructures/UnitTests/MockObjects.h"

#include <random>

using SurgSim::DataStructures::EmptyData;
using SurgSim::DataStructures::TetrahedronMesh;
using SurgSim::Math::Vector3d;

typedef TetrahedronMesh<EmptyData, EmptyData, EmptyData, EmptyData> TetrahedronMeshNoData;

class TetrahedronMeshTest : public ::testing::Test
{
public:
	void SetUp()
	{
		// Set to true to print the test positions.
		bool printPositions = false;
		// Set to true to print the test normals.
		bool printNormals = false;
		// Set to true to print the test edges.
		bool printEdges = false;
		// Set to true to print the test triangles.
		bool printTriangles = false;
		// Set to true to print the test tetrahedrons.
		bool printTetrahedrons = false;
		// Set the number of test vertices
		size_t numVertices = 10;
		// Set the number of test tetrahedrons
		size_t numTetrahedrons = 15;

		std::default_random_engine generator;
		std::uniform_real_distribution<double> positionDistribution(-10.0, 10.0);
		std::uniform_real_distribution<double> normalDistribution(-1.0, 1.0);
		std::uniform_int_distribution<size_t> vertexIdDistribution(0, numVertices-1);

		if (printPositions)
		{
			std::cout << "Test Vertex Positions:\n";
		}

		/// Generate random positions for each vertex
		for (size_t i = 0; i < numVertices; ++i)
		{
			Vector3d position(positionDistribution(generator), positionDistribution(generator),
				positionDistribution(generator));
			testPositions.push_back(position);

			if (printPositions)
			{
				std::cout << "\t" << i << ": (" << position.x() << ", " << position.y() << ", " << position.z()
						  << ")\n";
			}
		}

		if (printNormals)
		{
			std::cout << "Test Vertex Normals:\n";
		}

		/// Generate random normals for each vertex
		for (size_t i = 0; i < numVertices; ++i)
		{
			Vector3d normal(normalDistribution(generator), normalDistribution(generator),
				normalDistribution(generator));
			normal.normalize();
			testNormals.push_back(normal);

			if (printNormals)
			{
				std::cout << "\t" << i << ": (" << normal.x() << ", " << normal.y() << ", " << normal.z() << ")\n";
			}
		}

		if (printTetrahedrons)
		{
			std::cout << "Test Tetrahedrons:\n";
		}

		/// Generate random vertex IDs within [0, numVertices) in quadruplets for mesh tetrahedrons
		for (size_t i = 0; i < numTetrahedrons; ++i)
		{
			std::array<size_t, 4> tetrahedronVertices = {{ vertexIdDistribution(generator),
				vertexIdDistribution(generator), vertexIdDistribution(generator), vertexIdDistribution(generator) }};
			testTetrahedronsVerticesId.push_back(tetrahedronVertices);

			/// Create 6 vertex ID pairs for each tetrahedron edge (not worrying about duplicates for these tests)
			std::array<size_t, 6> tetrahedronEdges;
			int edgeIDs[6][2] = { {0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3} };
			for (int j = 0; j < 6; ++j)
			{
				std::array<size_t, 2> edgeVertices =
				{
					{
						tetrahedronVertices[edgeIDs[j][0]],
						tetrahedronVertices[edgeIDs[j][1]]
					}
				};
				testEdgesVerticesId.push_back(edgeVertices);

				tetrahedronEdges[j] = testEdgesVerticesId.size() - 1;
			}
			testTetrahedronsEdgesId.push_back(tetrahedronEdges);

			/// Create 4 vertex ID pairs for each tetrahedron triangle (not worrying about duplicates for these tests)
			int vertexIDs[4][3] = { {0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3} };
			int tetTriangleEdgeIds[4][3] = { {0, 1, 3}, {0, 2, 4}, {1, 2, 5}, {3, 4, 5} };
			std::array<size_t, 4> tetrahedronTriangles;
			for (int j = 0; j < 4; ++j)
			{
				std::array<size_t, 3> triangleVertices =
				{
					{
						tetrahedronVertices[vertexIDs[j][0]],
						tetrahedronVertices[vertexIDs[j][1]],
						tetrahedronVertices[vertexIDs[j][2]]
					}
				};
				testTrianglesVerticesId.push_back(triangleVertices);

				tetrahedronTriangles[j] = testTrianglesVerticesId.size() - 1;

				std::array<size_t, 3> triangleEdges;
				for (int k = 0; k < 3; k++)
				{
					triangleEdges[k] = tetrahedronEdges[tetTriangleEdgeIds[j][k]];
				}
				testTrianglesEdgesId.push_back(triangleEdges);
			}
			testTetrahedronsTrianglesId.push_back(tetrahedronTriangles);

			if (printTetrahedrons)
			{
				std::cout << "\t" << i
						  << ": Vertices (" << formatIterator(tetrahedronVertices, ", ")
						  << "), Edges (" << formatIterator(tetrahedronEdges, ", ")
						  << "), Triangles (" << formatIterator(tetrahedronTriangles, ", ") << ")\n";
			}
		}

		if (printTriangles)
		{
			std::cout << "Test Triangles:\n";

			for (size_t i = 0; i < testTrianglesVerticesId.size(); ++i)
			{
				const std::array<size_t, 3>& triangleVertices = testTrianglesVerticesId[i];
				const std::array<size_t, 3>& triangleEdges = testTrianglesEdgesId[i];

				std::cout << "\t" << i << ": Vertices (" << formatIterator(triangleVertices, ", ")
						  << ") - Edges (" << formatIterator(triangleEdges, ", ") << ")\n";
			}
		}

		if (printEdges)
		{
			std::cout << "Test Edges:\n";

			for (size_t i = 0; i < testEdgesVerticesId.size(); ++i)
			{
				const std::array<size_t, 2>& edgeVertices = testEdgesVerticesId[i];
				std::cout << "\t" << i << ": (" << formatIterator(edgeVertices, ", ") << ")\n";
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

	/// Vertices Id for all edges
	std::vector<std::array<size_t, 2>> testEdgesVerticesId;

	/// Vertices Id for all triangles
	std::vector<std::array<size_t, 3>> testTrianglesVerticesId;
	/// Edges Id for all triangles
	std::vector<std::array<size_t, 3>> testTrianglesEdgesId;

	/// Vertices Id for all tetrahedrons
	std::vector<std::array<size_t, 4>> testTetrahedronsVerticesId;
	/// Edges Id for all tetrahedrons
	std::vector<std::array<size_t, 6>> testTetrahedronsEdgesId;
	/// Triangles Id for all tetrahedrons
	std::vector<std::array<size_t, 4>> testTetrahedronsTrianglesId;
};


TEST_F(TetrahedronMeshTest, InitTest)
{
	ASSERT_NO_THROW({MockTetrahedronMesh mesh;});
	ASSERT_NO_THROW({TetrahedronMeshNoData mesh;});
}

TEST_F(TetrahedronMeshTest, CreateVerticesTest)
{
	MockTetrahedronMesh mesh;

	EXPECT_EQ(0u, mesh.getNumVertices());
	EXPECT_EQ(0u, mesh.getVertices().size());
	EXPECT_EQ(0u, mesh.getNumEdges());
	EXPECT_EQ(0u, mesh.getEdges().size());
	EXPECT_EQ(0u, mesh.getNumTriangles());
	EXPECT_EQ(0u, mesh.getTriangles().size());
	EXPECT_EQ(0u, mesh.getNumTetrahedrons());
	EXPECT_EQ(0u, mesh.getTetrahedrons().size());

	EXPECT_EQ(0, mesh.getNumUpdates());

	/// Create the test vertices
	for (size_t i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createVertex(testPositions[i], testNormals[i]));
		EXPECT_EQ(i + 1, mesh.getNumVertices());

		const std::vector<MockTetrahedronMesh::VertexType>& vertices = mesh.getVertices();
		EXPECT_EQ(i + 1, vertices.size());

		/// Make sure each vertex is set properly
		for (size_t j = 0; j < mesh.getNumVertices(); ++j)
		{
			EXPECT_EQ(testPositions[j], vertices[j].position);

			const MockVertexData& data = vertices[j].data;
			EXPECT_EQ(j, data.getId());
			EXPECT_EQ(testNormals[j], data.getNormal());
		}
	}

	/// Create the test edges
	for (size_t i = 0; i < testEdgesVerticesId.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createEdge(testEdgesVerticesId[i]));
		EXPECT_EQ(i + 1, mesh.getNumEdges());

		const std::vector<MockTetrahedronMesh::EdgeType>& edges = mesh.getEdges();
		EXPECT_EQ(i + 1, edges.size());

		/// Make sure each vertex is set properly
		for (size_t j = 0; j < mesh.getNumEdges(); ++j)
		{
			EXPECT_EQ(testEdgesVerticesId[j], edges[j].verticesId);

			const MockEdgeData& data = edges[j].data;
			EXPECT_EQ(j, data.getId());
		}
	}

	/// Create the test triangles
	for (size_t i = 0; i < testTrianglesVerticesId.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createTriangle(testTrianglesVerticesId[i], testTrianglesEdgesId[i]));
		EXPECT_EQ(i + 1, mesh.getNumTriangles());

		const std::vector<MockTetrahedronMesh::TriangleType>& triangles = mesh.getTriangles();
		EXPECT_EQ(i + 1, triangles.size());

		/// Make sure each vertex is set properly
		for (size_t j = 0; j < mesh.getNumTriangles(); ++j)
		{
			EXPECT_EQ(testTrianglesVerticesId[j], triangles[j].verticesId);

			const MockTriangleData& data = triangles[j].data;
			EXPECT_EQ(j, data.getId());
			EXPECT_EQ(testTrianglesEdgesId[j], data.getEdges());
		}
	}

	/// Create the test tetrahedrons
	for (size_t i = 0; i < testTetrahedronsVerticesId.size(); ++i)
	{
		 EXPECT_EQ(i, mesh.createTetrahedron(testTetrahedronsVerticesId[i], \
			testTetrahedronsEdgesId[i], testTetrahedronsTrianglesId[i]));
		EXPECT_EQ(i + 1, mesh.getNumTetrahedrons());

		const std::vector<MockTetrahedronMesh::TetrahedronType>& tetrahedrons = mesh.getTetrahedrons();
		EXPECT_EQ(i + 1, tetrahedrons.size());

		/// Make sure each tetrahedron is set properly
		for (size_t j = 0; j < mesh.getNumTetrahedrons(); ++j)
		{
			EXPECT_EQ(testTetrahedronsVerticesId[j], tetrahedrons[j].verticesId);

			const MockTetrahedronData& data = tetrahedrons[j].data;
			EXPECT_EQ(j, data.getId());
			EXPECT_EQ(testTetrahedronsEdgesId[j], data.getEdges());
			EXPECT_EQ(testTetrahedronsTrianglesId[j], data.getTriangles());
		}
	}
}

TEST_F(TetrahedronMeshTest, isValidTest)
{
	MockTetrahedronMesh mesh;

	EXPECT_TRUE(mesh.isValid());

	/// Create the edges (no vertices yet => the mesh is NOT valid)
	for (size_t i = 0; i < testEdgesVerticesId.size(); ++i)
	{
		mesh.createEdge(testEdgesVerticesId[i]);
	}

	EXPECT_FALSE(mesh.isValid());

	/// Create the triangles (no vertices yet => the mesh is NOT valid)
	for (size_t i = 0; i < testTrianglesVerticesId.size(); ++i)
	{
		mesh.createTriangle(testTrianglesVerticesId[i], testTrianglesEdgesId[i]);
	}

	EXPECT_FALSE(mesh.isValid());

	/// Create the tetrahedrons (no vertices yet => the mesh is NOT valid)
	for (size_t i = 0; i < testTetrahedronsVerticesId.size(); ++i)
	{
		mesh.createTetrahedron(testTetrahedronsVerticesId[i], testTetrahedronsEdgesId[i], \
			testTetrahedronsTrianglesId[i]);
	}

	EXPECT_FALSE(mesh.isValid());

	/// Create the vertices
	for (size_t i = 0; i < testPositions.size(); ++i)
	{
		mesh.createVertex(testPositions[i], testNormals[i]);
	}

	EXPECT_TRUE(mesh.isValid());
}

TEST_F(TetrahedronMeshTest, SetVertexPositionsTest)
{
	MockTetrahedronMesh mesh;

	/// Create vertices with test normals, but all positions at (0,0,0)
	for (size_t i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createVertex(Vector3d(0.0, 0.0, 0.0), testNormals[i]));
		EXPECT_EQ(i + 1, mesh.getNumVertices());
	}

	mesh.setVertexPositions(testPositions);

	EXPECT_EQ(1, mesh.getNumUpdates());
	EXPECT_EQ(testPositions.size(), mesh.getNumVertices());

	const std::vector<MockTetrahedronMesh::VertexType>& vertices = mesh.getVertices();
	EXPECT_EQ(testPositions.size(), vertices.size());

	/// Make sure each vertex is set properly
	for (size_t i = 0; i < mesh.getNumVertices(); ++i)
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
	for (size_t i = 0; i < mesh.getNumVertices(); ++i)
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

TEST_F(TetrahedronMeshTest, ClearTest)
{
	MockTetrahedronMesh mesh;

	EXPECT_EQ(0, mesh.getNumUpdates());

	EXPECT_EQ(0u, mesh.getNumVertices());
	EXPECT_EQ(0u, mesh.getVertices().size());

	EXPECT_EQ(0u, mesh.getNumEdges());
	EXPECT_EQ(0u, mesh.getEdges().size());

	EXPECT_EQ(0u, mesh.getNumTriangles());
	EXPECT_EQ(0u, mesh.getTriangles().size());

	EXPECT_EQ(0u, mesh.getNumTetrahedrons());
	EXPECT_EQ(0u, mesh.getTetrahedrons().size());

	/// Create mesh using test data
	for (size_t i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createVertex(Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0)));
		EXPECT_EQ(i + 1, mesh.getNumVertices());
	}
	for (size_t i = 0; i < testEdgesVerticesId.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createEdge(testEdgesVerticesId[i]));
		EXPECT_EQ(i + 1, mesh.getNumEdges());
	}
	for (size_t i = 0; i < testTrianglesVerticesId.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createTriangle(testTrianglesVerticesId[i], testTrianglesEdgesId[i]));
		EXPECT_EQ(i + 1, mesh.getNumTriangles());
	}
	for (size_t i = 0; i < testTetrahedronsVerticesId.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createTetrahedron(testTetrahedronsVerticesId[i], \
			testTetrahedronsEdgesId[i], testTetrahedronsTrianglesId[i]));
		EXPECT_EQ(i + 1, mesh.getNumTetrahedrons());
	}

	EXPECT_EQ(testPositions.size(), mesh.getNumVertices());
	EXPECT_EQ(testPositions.size(), mesh.getVertices().size());

	EXPECT_EQ(testEdgesVerticesId.size(), mesh.getNumEdges());
	EXPECT_EQ(testEdgesVerticesId.size(), mesh.getEdges().size());

	EXPECT_EQ(testTrianglesVerticesId.size(), mesh.getNumTriangles());
	EXPECT_EQ(testTrianglesVerticesId.size(), mesh.getTriangles().size());

	EXPECT_EQ(testTetrahedronsVerticesId.size(), mesh.getNumTetrahedrons());
	EXPECT_EQ(testTetrahedronsVerticesId.size(), mesh.getTetrahedrons().size());

	/// Clear mesh
	mesh.clear();

	EXPECT_EQ(0u, mesh.getNumVertices());
	EXPECT_EQ(0u, mesh.getVertices().size());

	EXPECT_EQ(0u, mesh.getNumEdges());
	EXPECT_EQ(0u, mesh.getEdges().size());

	EXPECT_EQ(0u, mesh.getNumTriangles());
	EXPECT_EQ(0u, mesh.getTriangles().size());

	EXPECT_EQ(0u, mesh.getNumTetrahedrons());
	EXPECT_EQ(0u, mesh.getTetrahedrons().size());
}

TEST_F(TetrahedronMeshTest, UpdateTest)
{
	MockTetrahedronMesh mesh;

	EXPECT_EQ(0, mesh.getNumUpdates());

	for (int i = 0; i < 10; ++i)
	{
		mesh.update();
		EXPECT_EQ(i + 1, mesh.getNumUpdates());
	}
}

TEST_F(TetrahedronMeshTest, ComparisonTest)
{
	/// Create mesh using test data
	MockTetrahedronMesh mesh;
	for (size_t i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createVertex(testPositions[i], testNormals[i]));
	}
	for (size_t i = 0; i < testEdgesVerticesId.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createEdge(testEdgesVerticesId[i]));
	}
	for (size_t i = 0; i < testTrianglesVerticesId.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createTriangle(testTrianglesVerticesId[i], testTrianglesEdgesId[i]));
	}
	for (size_t i = 0; i < testTetrahedronsVerticesId.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createTetrahedron(testTetrahedronsVerticesId[i], \
			testTetrahedronsEdgesId[i], testTetrahedronsTrianglesId[i]));
	}

	/// Create same mesh again
	MockTetrahedronMesh sameMesh;

	for (size_t i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, sameMesh.createVertex(testPositions[i], testNormals[i]));
	}
	for (size_t i = 0; i < testEdgesVerticesId.size(); ++i)
	{
		EXPECT_EQ(i, sameMesh.createEdge(testEdgesVerticesId[i]));
	}
	for (size_t i = 0; i < testTrianglesVerticesId.size(); ++i)
	{
		EXPECT_EQ(i, sameMesh.createTriangle(testTrianglesVerticesId[i], testTrianglesEdgesId[i]));
	}
	for (size_t i = 0; i < testTetrahedronsVerticesId.size(); ++i)
	{
		EXPECT_EQ(i, sameMesh.createTetrahedron(testTetrahedronsVerticesId[i], \
			testTetrahedronsEdgesId[i], testTetrahedronsTrianglesId[i]));
	}

	/// Create mesh with test data, but each vertex has position and normal of (0,0,0) to make them different
	MockTetrahedronMesh meshWithDifferentVertices;
	for (size_t i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, meshWithDifferentVertices.createVertex(Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0)));
	}
	for (size_t i = 0; i < testEdgesVerticesId.size(); ++i)
	{
		EXPECT_EQ(i, meshWithDifferentVertices.createEdge(testEdgesVerticesId[i]));
	}
	for (size_t i = 0; i < testTrianglesVerticesId.size(); ++i)
	{
		EXPECT_EQ(i, meshWithDifferentVertices.createTriangle(testTrianglesVerticesId[i], testTrianglesEdgesId[i]));
	}
	for (size_t i = 0; i < testTetrahedronsVerticesId.size(); ++i)
	{
		EXPECT_EQ(i, meshWithDifferentVertices.createTetrahedron(testTetrahedronsVerticesId[i], \
			testTetrahedronsEdgesId[i], testTetrahedronsTrianglesId[i]));
	}

	/// Create mesh with test data, but reverse each edge's vertex order to make them different
	MockTetrahedronMesh meshWithDifferentEdges;
	for (size_t i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, meshWithDifferentEdges.createVertex(testPositions[i], testNormals[i]));
	}
	for (size_t i = 0; i < testEdgesVerticesId.size(); ++i)
	{
		std::array<size_t, 2> edge = {{ testEdgesVerticesId[i][1], testEdgesVerticesId[i][0] }};
		EXPECT_EQ(i, meshWithDifferentEdges.createEdge(edge));
	}
	for (size_t i = 0; i < testTrianglesVerticesId.size(); ++i)
	{
		EXPECT_EQ(i, meshWithDifferentEdges.createTriangle(testTrianglesVerticesId[i], testTrianglesEdgesId[i]));
	}
	for (size_t i = 0; i < testTetrahedronsVerticesId.size(); ++i)
	{
		EXPECT_EQ(i, meshWithDifferentEdges.createTetrahedron(testTetrahedronsVerticesId[i], \
			testTetrahedronsEdgesId[i], testTetrahedronsTrianglesId[i]));
	}

	/// Create mesh with test data, but only create half of the triangles to make the list different.
	MockTetrahedronMesh meshWithDifferentTriangles;
	for (size_t i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, meshWithDifferentTriangles.createVertex(testPositions[i], testNormals[i]));
	}
	for (size_t i = 0; i < testEdgesVerticesId.size(); ++i)
	{
		EXPECT_EQ(i, meshWithDifferentTriangles.createEdge(testEdgesVerticesId[i]));
	}
	for (size_t i = 0; i < testTrianglesVerticesId.size() / 2; ++i)
	{
		EXPECT_EQ(i, meshWithDifferentTriangles.createTriangle(testTrianglesVerticesId[i], testTrianglesEdgesId[i]));
	}
	for (size_t i = 0; i < testTetrahedronsVerticesId.size(); ++i)
	{
		EXPECT_EQ(i, meshWithDifferentTriangles.createTetrahedron(testTetrahedronsVerticesId[i], \
			testTetrahedronsEdgesId[i], testTetrahedronsTrianglesId[i]));
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
