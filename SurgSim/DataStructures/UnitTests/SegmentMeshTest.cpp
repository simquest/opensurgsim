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
#include <boost/filesystem.hpp>

#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/MeshElement.h"
#include "SurgSim/DataStructures/SegmentMesh.h"
#include "SurgSim/DataStructures/UnitTests/MockObjects.h"
#include "SurgSim/DataStructures/Vertex.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/ApplicationData.h"


using SurgSim::DataStructures::EmptyData;
using SurgSim::DataStructures::SegmentMesh;
using SurgSim::DataStructures::SegmentMeshPlain;
using SurgSim::Math::Vector3d;


/// Triangle Mesh for testing using MockVertexData, MockEdgeData, and MockTriangleData
class MockSegmentMesh : public SurgSim::DataStructures::SegmentMesh<MockVertexData, MockEdgeData>
{
public:
	/// Vertex type for convenience
	typedef SegmentMesh<MockVertexData, MockEdgeData>::VertexType VertexType;
	/// Edge type for convenience
	typedef SegmentMesh<MockVertexData, MockEdgeData>::EdgeType EdgeType;
	/// Triangle type for convenience
	typedef SegmentMesh<MockVertexData, MockEdgeData>::TriangleType TriangleType;

	/// Constructor. Start out with no vertices and 0 updates
	MockSegmentMesh() :
		SurgSim::DataStructures::SegmentMesh<MockVertexData, MockEdgeData>(),
		m_numUpdates(0)
	{
	}

	SURGSIM_CLASSNAME(MockSegmentMesh);

	/// Destructor
	virtual ~MockSegmentMesh()
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
std::string SurgSim::DataStructures::SegmentMesh<MockVertexData, MockEdgeData>
::TriangleMesh::m_className = "MockSegmentMesh";

class SegmentMeshTest : public ::testing::Test
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
		// Set the number of test vertices
		size_t numVertices = 10;

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

		if (printEdges)
		{
			std::cout << "Test Edges:\n";
		}

		for (size_t i = 0; i < numVertices - 1; ++i)
		{
			std::array<size_t, 2> edgeVertices = {{ i, i + 1 }};
			testEdgeVertices.push_back(edgeVertices);
			if (printEdges)
			{
				std::cout << "\t" << i << ": (" << formatIterator(edgeVertices, ", ") << ")\n";
			}
		}
	}

	void TearDown()
	{
		boost::filesystem::remove("export.ply");
	}

	/// Positions of test vertices
	std::vector<Vector3d> testPositions;
	/// Normals of test vertices
	std::vector<Vector3d> testNormals;

	/// Vertices of test edges
	std::vector<std::array<size_t, 2>> testEdgeVertices;
};


TEST_F(SegmentMeshTest, InitTest)
{
	ASSERT_NO_THROW({MockSegmentMesh mesh;});
	ASSERT_NO_THROW({SegmentMeshPlain mesh;});
}

TEST_F(SegmentMeshTest, ClassNameTest)
{
	MockSegmentMesh mesh;
	EXPECT_EQ("MockSegmentMesh", mesh.getClassName());
	SegmentMeshPlain meshPlain;
	EXPECT_EQ("SurgSim::DataStructures::SegmentMeshPlain", meshPlain.getClassName());
}

TEST_F(SegmentMeshTest, CreateVerticesTest)
{
	MockSegmentMesh mesh;

	EXPECT_EQ(0u, mesh.getNumVertices());
	EXPECT_EQ(0u, mesh.getVertices().size());

	/// Create the test vertices
	for (size_t i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createVertex(testPositions[i], testNormals[i]));
		EXPECT_EQ(i + 1, mesh.getNumVertices());

		const std::vector<MockSegmentMesh::VertexType>& vertices = mesh.getVertices();
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

		const std::vector<MockSegmentMesh::EdgeType>& edges = mesh.getEdges();
		EXPECT_EQ(i + 1, edges.size());

		/// Make sure each vertex is set properly
		for (size_t j = 0; j < mesh.getNumEdges(); ++j)
		{
			EXPECT_EQ(testEdgeVertices[j], edges[j].verticesId);

			const MockEdgeData& data = edges[j].data;
			EXPECT_EQ(j, data.getId());
		}
	}
}

TEST_F(SegmentMeshTest, isValidTest)
{
	MockSegmentMesh mesh;

	EXPECT_TRUE(mesh.isValid());

	/// Create the edges (no vertices yet => the mesh is NOT valid)
	for (size_t i = 0; i < testEdgeVertices.size(); ++i)
	{
		mesh.createEdge(testEdgeVertices[i]);
	}

	EXPECT_FALSE(mesh.isValid());

	/// Create the vertices
	for (size_t i = 0; i < testPositions.size(); ++i)
	{
		mesh.createVertex(testPositions[i], testNormals[i]);
	}

	EXPECT_TRUE(mesh.isValid());
}

TEST_F(SegmentMeshTest, SetVertexPositionsTest)
{
	MockSegmentMesh mesh;

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

TEST_F(SegmentMeshTest, ClearTest)
{
	MockSegmentMesh mesh;

	EXPECT_EQ(0, mesh.getNumUpdates());

	EXPECT_EQ(0u, mesh.getNumVertices());
	EXPECT_EQ(0u, mesh.getVertices().size());

	EXPECT_EQ(0u, mesh.getNumEdges());
	EXPECT_EQ(0u, mesh.getEdges().size());

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

	EXPECT_EQ(testPositions.size(), mesh.getNumVertices());
	EXPECT_EQ(testPositions.size(), mesh.getVertices().size());

	EXPECT_EQ(testEdgeVertices.size(), mesh.getNumEdges());
	EXPECT_EQ(testEdgeVertices.size(), mesh.getEdges().size());

	/// Clear mesh
	mesh.clear();

	EXPECT_EQ(0u, mesh.getNumVertices());
	EXPECT_EQ(0u, mesh.getVertices().size());

	EXPECT_EQ(0u, mesh.getNumEdges());
	EXPECT_EQ(0u, mesh.getEdges().size());
}

TEST_F(SegmentMeshTest, UpdateTest)
{
	MockSegmentMesh mesh;

	EXPECT_EQ(0, mesh.getNumUpdates());

	for (int i = 0; i < 10; ++i)
	{
		mesh.update();
		EXPECT_EQ(i + 1, mesh.getNumUpdates());
	}
}

TEST_F(SegmentMeshTest, ComparisonTest)
{
	/// Create mesh using test data
	MockSegmentMesh mesh;
	for (size_t i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createVertex(testPositions[i], testNormals[i]));
	}
	for (size_t i = 0; i < testEdgeVertices.size(); ++i)
	{
		EXPECT_EQ(i, mesh.createEdge(testEdgeVertices[i]));
	}

	/// Create same mesh again
	MockSegmentMesh sameMesh;

	for (size_t i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, sameMesh.createVertex(testPositions[i], testNormals[i]));
	}
	for (size_t i = 0; i < testEdgeVertices.size(); ++i)
	{
		EXPECT_EQ(i, sameMesh.createEdge(testEdgeVertices[i]));
	}

	/// Create mesh with test data, but each vertex has position and normal of (0,0,0) to make them different
	MockSegmentMesh meshWithDifferentVertices;
	for (size_t i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, meshWithDifferentVertices.createVertex(Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0)));
	}
	for (size_t i = 0; i < testEdgeVertices.size(); ++i)
	{
		EXPECT_EQ(i, meshWithDifferentVertices.createEdge(testEdgeVertices[i]));
	}

	/// Create mesh with test data, but reverse each edge's vertex order to make them different
	MockSegmentMesh meshWithDifferentEdges;
	for (size_t i = 0; i < testPositions.size(); ++i)
	{
		EXPECT_EQ(i, meshWithDifferentEdges.createVertex(testPositions[i], testNormals[i]));
	}
	for (size_t i = 0; i < testEdgeVertices.size(); ++i)
	{
		std::array<size_t, 2> edge = {{ testEdgeVertices[i][1], testEdgeVertices[i][0] }};
		EXPECT_EQ(i, meshWithDifferentEdges.createEdge(edge));
	}

	/// Test comparisons
	EXPECT_TRUE(mesh == sameMesh);
	EXPECT_FALSE(mesh != sameMesh);

	EXPECT_FALSE(mesh == meshWithDifferentVertices);
	EXPECT_TRUE(mesh != meshWithDifferentVertices);

	EXPECT_FALSE(mesh == meshWithDifferentEdges);
	EXPECT_TRUE(mesh != meshWithDifferentEdges);
}


TEST_F(SegmentMeshTest, CopyConstructorTest)
{
	MockSegmentMesh mesh;

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

	SurgSim::DataStructures::SegmentMeshPlain mesh2(mesh);

	for (size_t i = 0; i < mesh.getNumVertices(); ++i)
	{
		EXPECT_EQ(mesh.getVertexPosition(i), mesh2.getVertexPosition(i));
	}
	for (size_t i = 0; i < mesh.getNumEdges(); ++i)
	{
		EXPECT_EQ(mesh.getEdge(i).verticesId, mesh2.getEdge(i).verticesId);
	}
}

TEST_F(SegmentMeshTest, GetEdgePositions)
{
	MockSegmentMesh mesh;

	// Initialization
	auto normal = testNormals.begin();
	for (auto position = testPositions.begin(); position != testPositions.end(); ++position)
	{
		mesh.createVertex(*position, *normal++);
	}

	for (auto vertices = testEdgeVertices.begin(); vertices != testEdgeVertices.end(); ++vertices)
	{
		mesh.createEdge(*vertices);
	}

	// Testing
	for (size_t id = 0; id < mesh.getEdges().size(); ++id)
	{
		auto verticesPositions = mesh.getEdgePositions(id);

		auto& vertexIds = mesh.getEdge(id).verticesId;

		EXPECT_TRUE(mesh.getVertex(vertexIds[0]).position.isApprox(verticesPositions[0]));
		EXPECT_TRUE(mesh.getVertex(vertexIds[1]).position.isApprox(verticesPositions[1]));
	}
}


TEST_F(SegmentMeshTest, AssertingFunctions)
{
	MockSegmentMesh mesh;

	// Initialization
	auto normal = testNormals.begin();
	for (auto position = testPositions.begin(); position != testPositions.end(); ++position)
	{
		mesh.createVertex(*position, *normal++);
	}

	for (auto vertices = testEdgeVertices.begin(); vertices != testEdgeVertices.end(); ++vertices)
	{
		mesh.createEdge(*vertices);
	}

	// Testing
	std::array<size_t, 3> triangleIds = {{0, 1, 2}};
	MockSegmentMesh::TriangleType triangle(triangleIds);
	EXPECT_THROW(mesh.addTriangle(triangle), SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(mesh.getNumTriangles(), SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(mesh.getTriangles(), SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(mesh.getTriangle(0), SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(mesh.removeTriangle(0), SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(mesh.getTrianglePositions(0), SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(mesh.doClearTriangles(), SurgSim::Framework::AssertionFailure);
}

TEST_F(SegmentMeshTest, CreateDefaultedges)
{
	MockSegmentMesh mesh;
	for (size_t i = 0; i < 10; ++i)
	{
		mesh.createVertex(Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0));
	}

	mesh.createDefaultEdges();
	EXPECT_EQ(9, mesh.getNumEdges());
	for (size_t i = 0; i < mesh.getNumEdges(); ++i)
	{
		EXPECT_EQ(i, mesh.getEdge(i).verticesId[0]);
		EXPECT_EQ(i + 1, mesh.getEdge(i).verticesId[1]);
	}
}

TEST_F(SegmentMeshTest, LoadMesh)
{
	auto mesh = std::make_shared<SegmentMeshPlain>();
	SurgSim::Framework::ApplicationData data("config.txt");

	mesh->load("SegmentMeshTest/segmentmesh.ply", data);

	ASSERT_EQ(4u, mesh->getNumVertices());
	ASSERT_EQ(3u, mesh->getNumEdges());

	auto edge = mesh->getEdge(0);
	ASSERT_EQ(0u, edge.verticesId[0]);
	ASSERT_EQ(1u, edge.verticesId[1]);

	edge = mesh->getEdge(1);
	ASSERT_EQ(1u, edge.verticesId[0]);
	ASSERT_EQ(2u, edge.verticesId[1]);

	edge = mesh->getEdge(2);
	ASSERT_EQ(2u, edge.verticesId[0]);
	ASSERT_EQ(3u, edge.verticesId[1]);
}



TEST_F(SegmentMeshTest, WriteMesh)
{
	auto mesh = std::make_shared<SegmentMeshPlain>();
	SurgSim::Framework::ApplicationData data("config.txt");

	mesh->load("SegmentMeshTest/segmentmesh.ply", data);

	EXPECT_NO_THROW(mesh->save("export.ply"));

	std::vector<std::string> paths(1, ".");
	SurgSim::Framework::ApplicationData localPath(paths);

	auto loaded = std::make_shared<SegmentMeshPlain>();
	EXPECT_NO_THROW(loaded->load("export.ply", localPath));

	ASSERT_EQ(4u, mesh->getNumVertices());
	ASSERT_EQ(3u, mesh->getNumEdges());

	auto edge = mesh->getEdge(0);
	ASSERT_EQ(0u, edge.verticesId[0]);
	ASSERT_EQ(1u, edge.verticesId[1]);

	edge = mesh->getEdge(1);
	ASSERT_EQ(1u, edge.verticesId[0]);
	ASSERT_EQ(2u, edge.verticesId[1]);

	edge = mesh->getEdge(2);
	ASSERT_EQ(2u, edge.verticesId[0]);
	ASSERT_EQ(3u, edge.verticesId[1]);
}
