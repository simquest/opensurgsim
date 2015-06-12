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

#include <gtest/gtest.h>
#include <random>

#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/MeshElement.h"
#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/DataStructures/UnitTests/MockObjects.h"
#include "SurgSim/DataStructures/Vertex.h"
#include "SurgSim/Framework/Runtime.h"


using SurgSim::DataStructures::EmptyData;
using SurgSim::DataStructures::TriangleMesh;
using SurgSim::DataStructures::TriangleMeshPlain;
using SurgSim::Math::Vector3d;


/// Triangle Mesh for testing using MockVertexData, MockEdgeData, and MockTriangleData
class MockTriangleMesh : public SurgSim::DataStructures::TriangleMesh<MockVertexData, MockEdgeData, MockTriangleData>
{
public:
	/// Vertex type for convenience
	typedef TriangleMesh<MockVertexData, MockEdgeData, MockTriangleData>::VertexType VertexType;
	/// Edge type for convenience
	typedef TriangleMesh<MockVertexData, MockEdgeData, MockTriangleData>::EdgeType EdgeType;
	/// Triangle type for convenience
	typedef TriangleMesh<MockVertexData, MockEdgeData, MockTriangleData>::TriangleType TriangleType;

	/// Constructor. Start out with no vertices and 0 updates
	MockTriangleMesh() :
		SurgSim::DataStructures::TriangleMesh<MockVertexData, MockEdgeData, MockTriangleData>(),
		m_numUpdates(0)
	{
	}

	SURGSIM_CLASSNAME(MockTriangleMesh);

	/// Destructor
	virtual ~MockTriangleMesh()
	{
	}

	/// Create a new vertex in the mesh
	/// \param	position	Position of the vertex
	/// \param	normal	Normal of the vertex
	/// \return	Unique ID of vertex in the mesh
	size_t createVertex(const SurgSim::Math::Vector3d& position, const SurgSim::Math::Vector3d& normal)
	{
		VertexType vertex(position, MockVertexData(getNumVertices(), normal));

		return addVertex(vertex);
	}

	/// Create a new edge in the mesh
	/// \param	vertices	Edge vertices
	/// \return	Unique ID of vertex in the mesh
	size_t createEdge(const std::array<size_t, 2>& vertices)
	{
		EdgeType edge(vertices, MockEdgeData(getNumEdges()));

		return addEdge(edge);
	}

	/// Create a new triangle in the mesh
	/// \param	vertices	The triangle vertices
	/// \param	edges	The triangle edges
	/// \return	Unique ID of vertex in the mesh
	size_t createTriangle(const std::array<size_t, 3>& vertices, const std::array<size_t, 3>& edges)
	{
		TriangleType triangle(vertices, MockTriangleData(getNumTriangles(), edges));

		return addTriangle(triangle);
	}

	/// Returns the normal of a vertex
	const SurgSim::Math::Vector3d& getVertexNormal(size_t id) const
	{
		return getVertex(id).data.getNormal();
	}

	/// Returns the number of updates performed on the mesh
	int getNumUpdates() const
	{
		return m_numUpdates;
	}

private:
	/// Provides update functionality, which just increments the number of updates
	bool doUpdate() override
	{
		++m_numUpdates;
		return true;
	}

	/// Number of updates performed on the mesh
	int m_numUpdates;
};

template<>
std::string SurgSim::DataStructures::TriangleMesh<MockVertexData, MockEdgeData, MockTriangleData>
::m_className = "MockTriangleMesh";

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
		size_t numVertices = 10;
		// Set the number of test triangles
		size_t numTriangles = 20;

		std::default_random_engine generator;
		std::uniform_real_distribution<double> positionDistribution(-10.0, 10.0);
		std::uniform_real_distribution<double> normalDistribution(-1.0, 1.0);
		std::uniform_int_distribution<size_t> vertexIdDistribution(0, numVertices - 1);

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

		if (printTriangles)
		{
			std::cout << "Test Triangles:\n";
		}

		/// Generate random vertex IDs within [0, numVertices) in triplets for mesh triangles
		for (size_t i = 0; i < numTriangles; ++i)
		{
			std::array<size_t, 3> triangleVertices = {{
					vertexIdDistribution(generator),
					vertexIdDistribution(generator), vertexIdDistribution(generator)
				}
			};
			testTriangleVertices.push_back(triangleVertices);

			/// Create 3 vertex ID pairs for each triangle edge (not worrying about duplicates for these tests)
			std::array<size_t, 3> triangleEdges;
			for (int j = 0; j < 3; ++j)
			{
				std::array<size_t, 2> edgeVertices = {{ triangleVertices[0], triangleVertices[1] }};
				testEdgeVertices.push_back(edgeVertices);

				triangleEdges[j] = testEdgeVertices.size() - 1;
			}
			testTriangleEdges.push_back(triangleEdges);

			if (printTriangles)
			{
				std::cout << "\t" << i << ": Vertices (" << formatIterator(triangleVertices, ", ")
						  << "), Edges (" << formatIterator(triangleEdges, ", ") << ")\n";
			}
		}

		if (printEdges)
		{
			std::cout << "Test Edges:\n";

			for (size_t i = 0; i < testEdgeVertices.size(); ++i)
			{
				const std::array<size_t, 2>& edgeVertices = testEdgeVertices[i];
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

	/// Vertices of test edges
	std::vector<std::array<size_t, 2>> testEdgeVertices;

	/// Vertices of test triangles
	std::vector<std::array<size_t, 3>> testTriangleVertices;
	/// Edges of test triangles
	std::vector<std::array<size_t, 3>> testTriangleEdges;
};


TEST_F(TriangleMeshTest, InitTest)
{
	ASSERT_NO_THROW({MockTriangleMesh mesh;});
	ASSERT_NO_THROW({TriangleMeshPlain mesh;});
}

TEST_F(TriangleMeshTest, CreateVerticesTest)
{
	MockTriangleMesh mesh;

	EXPECT_EQ(0u, mesh.getNumVertices());
	EXPECT_EQ(0u, mesh.getVertices().size());

	/// Create the test vertices
	for (size_t i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createVertex(testPositions[i], testNormals[i]));
		EXPECT_EQ(i + 1, mesh.getNumVertices());

		const std::vector<MockTriangleMesh::VertexType>& vertices = mesh.getVertices();
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
	for (size_t i = 0; i < testEdgeVertices.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createEdge(testEdgeVertices[i]));
		EXPECT_EQ(i + 1, mesh.getNumEdges());

		const std::vector<MockTriangleMesh::EdgeType>& edges = mesh.getEdges();
		EXPECT_EQ(i + 1, edges.size());

		/// Make sure each vertex is set properly
		for (size_t j = 0; j < mesh.getNumEdges(); ++j)
		{
			EXPECT_EQ(testEdgeVertices[j], edges[j].verticesId);

			const MockEdgeData& data = edges[j].data;
			EXPECT_EQ(j, data.getId());
		}
	}

	/// Create the test triangles
	for (size_t i = 0; i < testTriangleVertices.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createTriangle(testTriangleVertices[i], testTriangleEdges[i]));
		EXPECT_EQ(i + 1, mesh.getNumTriangles());

		const std::vector<MockTriangleMesh::TriangleType>& triangles = mesh.getTriangles();
		EXPECT_EQ(i + 1, triangles.size());

		/// Make sure each vertex is set properly
		for (size_t j = 0; j < mesh.getNumTriangles(); ++j)
		{
			EXPECT_EQ(testTriangleVertices[j], triangles[j].verticesId);

			const MockTriangleData& data = triangles[j].data;
			EXPECT_EQ(j, data.getId());
			EXPECT_EQ(testTriangleEdges[j], data.getEdges());
		}
	}
}

TEST_F(TriangleMeshTest, isValidTest)
{
	MockTriangleMesh mesh;

	EXPECT_TRUE(mesh.isValid());

	/// Create the edges (no vertices yet => the mesh is NOT valid)
	for (size_t i = 0; i < testEdgeVertices.size(); ++i)
	{
		mesh.createEdge(testEdgeVertices[i]);
	}

	EXPECT_FALSE(mesh.isValid());

	/// Create the triangles (no vertices yet => the mesh is NOT valid)
	for (size_t i = 0; i < testTriangleVertices.size(); ++i)
	{
		mesh.createTriangle(testTriangleVertices[i], testTriangleEdges[i]);
	}

	EXPECT_FALSE(mesh.isValid());

	/// Create the vertices
	for (size_t i = 0; i < testPositions.size(); ++i)
	{
		mesh.createVertex(testPositions[i], testNormals[i]);
	}

	EXPECT_TRUE(mesh.isValid());
}

TEST_F(TriangleMeshTest, SetVertexPositionsTest)
{
	MockTriangleMesh mesh;

	/// Create vertices with test normals, but all positions at (0,0,0)
	for (size_t i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createVertex(Vector3d(0.0, 0.0, 0.0), testNormals[i]));
		EXPECT_EQ(i + 1, mesh.getNumVertices());
	}

	mesh.setVertexPositions(testPositions);

	EXPECT_EQ(1, mesh.getNumUpdates());
	EXPECT_EQ(testPositions.size(), mesh.getNumVertices());

	const std::vector<MockMesh::VertexType>& vertices = mesh.getVertices();
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

	EXPECT_THROW(mesh.setVertexPositions(testPositions), SurgSim::Framework::AssertionFailure);
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
	for (size_t i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createVertex(Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0)));
		EXPECT_EQ(i + 1, mesh.getNumVertices());
	}
	for (size_t i = 0; i < testEdgeVertices.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createEdge(testEdgeVertices[i]));
		EXPECT_EQ(i + 1, mesh.getNumEdges());
	}
	for (size_t i = 0; i < testTriangleVertices.size(); ++i)
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
	for (size_t i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createVertex(testPositions[i], testNormals[i]));
	}
	for (size_t i = 0; i < testEdgeVertices.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createEdge(testEdgeVertices[i]));
	}
	for (size_t i = 0; i < testTriangleVertices.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createTriangle(testTriangleVertices[i], testTriangleEdges[i]));
	}

	/// Create same mesh again
	MockTriangleMesh sameMesh;

	for (size_t i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, sameMesh.createVertex(testPositions[i], testNormals[i]));
	}
	for (size_t i = 0; i < testEdgeVertices.size(); ++i)
	{
		EXPECT_EQ(i, sameMesh.createEdge(testEdgeVertices[i]));
	}
	for (size_t i = 0; i < testTriangleVertices.size(); ++i)
	{
		EXPECT_EQ(i, sameMesh.createTriangle(testTriangleVertices[i], testTriangleEdges[i]));
	}

	/// Create mesh with test data, but each vertex has position and normal of (0,0,0) to make them different
	MockTriangleMesh meshWithDifferentVertices;
	for (size_t i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, meshWithDifferentVertices.createVertex(Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0)));
	}
	for (size_t i = 0; i < testEdgeVertices.size(); ++i)
	{
		EXPECT_EQ(i, meshWithDifferentVertices.createEdge(testEdgeVertices[i]));
	}
	for (size_t i = 0; i < testTriangleVertices.size(); ++i)
	{
		EXPECT_EQ(i, meshWithDifferentVertices.createTriangle(testTriangleVertices[i], testTriangleEdges[i]));
	}

	/// Create mesh with test data, but reverse each edge's vertex order to make them different
	MockTriangleMesh meshWithDifferentEdges;
	for (size_t i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, meshWithDifferentEdges.createVertex(testPositions[i], testNormals[i]));
	}
	for (size_t i = 0; i < testEdgeVertices.size(); ++i)
	{
		std::array<size_t, 2> edge = {{ testEdgeVertices[i][1], testEdgeVertices[i][0] }};
		EXPECT_EQ(i, meshWithDifferentEdges.createEdge(edge));
	}
	for (size_t i = 0; i < testTriangleVertices.size(); ++i)
	{
		EXPECT_EQ(i, meshWithDifferentEdges.createTriangle(testTriangleVertices[i], testTriangleEdges[i]));
	}

	/// Create mesh with test data, but only create half of the triangles to make the list different.
	MockTriangleMesh meshWithDifferentTriangles;
	for (size_t i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, meshWithDifferentTriangles.createVertex(testPositions[i], testNormals[i]));
	}
	for (size_t i = 0; i < testEdgeVertices.size(); ++i)
	{
		EXPECT_EQ(i, meshWithDifferentTriangles.createEdge(testEdgeVertices[i]));
	}
	for (size_t i = 0; i < testTriangleVertices.size() / 2; ++i)
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


TEST_F(TriangleMeshTest, CopyConstructorTest)
{
	MockTriangleMesh mesh;

	/// Create mesh using test data
	for (size_t i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createVertex(Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0)));
		EXPECT_EQ(i + 1, mesh.getNumVertices());
	}
	for (size_t i = 0; i < testEdgeVertices.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createEdge(testEdgeVertices[i]));
		EXPECT_EQ(i + 1, mesh.getNumEdges());
	}
	for (size_t i = 0; i < testTriangleVertices.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createTriangle(testTriangleVertices[i], testTriangleEdges[i]));
		EXPECT_EQ(i + 1, mesh.getNumTriangles());
	}

	SurgSim::DataStructures::TriangleMeshPlain mesh2(mesh);

	for (size_t i = 0; i < mesh.getNumVertices(); ++i)
	{
		EXPECT_EQ(mesh.getVertexPosition(i), mesh2.getVertexPosition(i));
	}
	for (size_t i = 0; i < mesh.getNumEdges(); ++i)
	{
		EXPECT_EQ(mesh.getEdge(i).verticesId, mesh2.getEdge(i).verticesId);
	}
	for (size_t i = 0; i < mesh.getNumTriangles(); ++i)
	{
		EXPECT_EQ(mesh.getTriangle(i).verticesId, mesh2.getTriangle(i).verticesId);
	}
}

TEST_F(TriangleMeshTest, LoadTest)
{
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");

	{
		SCOPED_TRACE("Load nonexistent file should throw");
		auto mesh = std::make_shared<SurgSim::DataStructures::TriangleMeshPlain>();
		EXPECT_THROW(mesh->load("Nonexistent file"), SurgSim::Framework::AssertionFailure);
	}

	{
		SCOPED_TRACE("Load existent file which contains invalid mesh should throw");
		auto mesh = std::make_shared<SurgSim::DataStructures::TriangleMeshPlain>();
		EXPECT_THROW(mesh->load("Geometry/InvalidMesh.ply"), SurgSim::Framework::AssertionFailure);
	}

	{
		SCOPED_TRACE("Load existent file which contains valid mesh should not throw");
		auto mesh = std::make_shared<SurgSim::DataStructures::TriangleMeshPlain>();
		EXPECT_NO_THROW(mesh->load("Geometry/staple_collision.ply"));
	}

	{
		SCOPED_TRACE("Load Cube.ply and check mesh");
		auto mesh = std::make_shared<SurgSim::DataStructures::TriangleMeshPlain>();
		mesh->load("Geometry/Cube.ply");
		ASSERT_NO_THROW(mesh->load("Geometry/Cube.ply"));

		EXPECT_EQ(26u, mesh->getNumVertices());
		EXPECT_EQ(12u, mesh->getNumTriangles());

		// The first and last vertices from the file
		Vector3d vertex0(1.0, 1.0, -1.0);
		Vector3d vertex25(-1.0, -1.0, 1.0);

		EXPECT_TRUE(vertex0.isApprox(mesh->getVertex(0).position));
		EXPECT_TRUE(vertex25.isApprox(mesh->getVertex(25).position));

		std::array<size_t, 3> triangle0 = {0, 1, 2};
		std::array<size_t, 3> triangle11 = {10, 25, 11};

		EXPECT_EQ(triangle0, mesh->getTriangle(0).verticesId);
		EXPECT_EQ(triangle11, mesh->getTriangle(11).verticesId);
	}
}


TEST_F(TriangleMeshTest, GetTrianglePositions)
{
	MockTriangleMesh mesh;

	// Initialization
	auto normal = testNormals.begin();
	for (auto position = testPositions.begin(); position != testPositions.end(); ++position)
	{
		mesh.createVertex(*position, *normal++);
	}

	auto edges = testTriangleEdges.begin();
	for (auto vertices = testTriangleVertices.begin(); vertices != testTriangleVertices.end(); ++vertices)
	{
		mesh.createTriangle(*vertices, *edges++);
	}

	// Testing
	for (size_t id = 0; id < mesh.getTriangles().size(); ++id)
	{
		auto verticesPositions = mesh.getTrianglePositions(id);

		auto& vertexIds = mesh.getTriangle(id).verticesId;

		EXPECT_TRUE(mesh.getVertex(vertexIds[0]).position.isApprox(verticesPositions[0]));
		EXPECT_TRUE(mesh.getVertex(vertexIds[1]).position.isApprox(verticesPositions[1]));
		EXPECT_TRUE(mesh.getVertex(vertexIds[2]).position.isApprox(verticesPositions[2]));
	}
}

TEST_F(TriangleMeshTest, TriangleDeletionTest)
{
	typedef TriangleMeshPlain::VertexType VertexType;
	typedef TriangleMeshPlain::TriangleType TriangleType;

	TriangleMeshPlain mesh;

	mesh.addVertex(VertexType(testPositions[0]));
	mesh.addVertex(VertexType(testPositions[0]));
	mesh.addVertex(VertexType(testPositions[0]));

	TriangleType::IdType ids = {0, 1, 2};
	mesh.addTriangle(TriangleType(ids));
	mesh.addTriangle(TriangleType(ids));
	mesh.addTriangle(TriangleType(ids));

	EXPECT_TRUE(mesh.isValid());

	EXPECT_NO_THROW(mesh.removeTriangle(1));
	EXPECT_TRUE(mesh.isValid());

	// Basic checks
	EXPECT_EQ(2u, mesh.getNumTriangles());
	EXPECT_THROW(mesh.getTriangle(1), SurgSim::Framework::AssertionFailure);
	EXPECT_EQ(3u, mesh.getTriangles().size());

	// Should be able to remove the same triangle twice
	EXPECT_NO_THROW(mesh.removeTriangle(1));

	// Remove all other triangles to check boundary conditions
	mesh.removeTriangle(0);
	EXPECT_EQ(1u, mesh.getNumTriangles());

	mesh.removeTriangle(2);
	EXPECT_EQ(0u, mesh.getNumTriangles());

	EXPECT_EQ(3u, mesh.getTriangles().size());

	// Adding a new triangle we should get an id from the old ids
	EXPECT_GT(3u, mesh.addTriangle(TriangleType(ids)));
	EXPECT_EQ(1u, mesh.getNumTriangles());

	// The array size should not have been change
	EXPECT_EQ(3u, mesh.getTriangles().size());

	EXPECT_GT(3u, mesh.addTriangle(TriangleType(ids)));
	EXPECT_GT(3u, mesh.addTriangle(TriangleType(ids)));

	// That is a new triangle
	EXPECT_EQ(3u, mesh.addTriangle(TriangleType(ids)));
	EXPECT_EQ(4u, mesh.getNumTriangles());
	EXPECT_EQ(4u, mesh.getTriangles().size());

	// Test clear with deleted triangles
	mesh.removeTriangle(3);
	EXPECT_NO_THROW(mesh.clear());
	EXPECT_EQ(0u, mesh.getNumTriangles());
	EXPECT_EQ(0u, mesh.addTriangle(TriangleType(ids)));
	EXPECT_EQ(1u, mesh.addTriangle(TriangleType(ids)));

}
