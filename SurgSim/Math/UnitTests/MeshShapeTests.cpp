// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest Solutions Inc.
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

#include <time.h>

#include <gtest/gtest.h>

#include "SurgSim/DataStructures/AabbTree.h"
#include "SurgSim/DataStructures/AabbTreeNode.h"
#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/BoxShape.h"
#include "SurgSim/Math/MathConvert.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::DataStructures::EmptyData;
using SurgSim::DataStructures::TriangleMeshPlain;

namespace
{
const double epsilon = 1e-8;
}

namespace SurgSim
{
namespace Math
{

// CUBE
//     3*----------*2
//     /          /|
//   7*----------* |
//    |         6| |
//    | *0       | *1
//    |          |/
//   4*----------*5

static const int cubeNumPoints = 8;
static const Vector3d cubePoints[8] =
{
	Vector3d(-1.0 / 2.0, -1.0 / 2.0, -1.0 / 2.0),
	Vector3d(1.0 / 2.0, -1.0 / 2.0, -1.0 / 2.0),
	Vector3d(1.0 / 2.0,  1.0 / 2.0, -1.0 / 2.0),
	Vector3d(-1.0 / 2.0,  1.0 / 2.0, -1.0 / 2.0),

	Vector3d(-1.0 / 2.0, -1.0 / 2.0,  1.0 / 2.0),
	Vector3d(1.0 / 2.0, -1.0 / 2.0,  1.0 / 2.0),
	Vector3d(1.0 / 2.0,  1.0 / 2.0,  1.0 / 2.0),
	Vector3d(-1.0 / 2.0,  1.0 / 2.0,  1.0 / 2.0)
};

static const int cubeNumEdges = 12;
static const int cubeEdges[12][2] =
{
	{0, 1}, {3, 2}, {4, 5}, {7, 6}, // +X
	{0, 3}, {1, 2}, {4, 7}, {5 , 6}, // +Y
	{0, 4}, {1, 5}, {2, 6}, {3, 7}  // +Z
};

static const int cubeNumTriangles = 12;
static const int cubeTrianglesCCW[12][3] =
{
	{6, 2, 3}, {6, 3, 7}, // Top    ( 0  1  0) [6237]
	{0, 1, 5}, {0, 5, 4}, // Bottom ( 0 -1  0) [0154]
	{4, 5, 6}, {4, 6, 7}, // Front  ( 0  0  1) [4567]
	{0, 3, 2}, {0, 2, 1}, // Back   ( 0  0 -1) [0321]
	{1, 2, 6}, {1, 6, 5}, // Right  ( 1  0  0) [1265]
	{0, 4, 7}, {0, 7, 3}  // Left   (-1  0  0) [0473]
};

class MeshShapeTest : public ::testing::Test
{
public:
	void SetUp()
	{
		m_numIterations = 100;
		m_runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
		srand((unsigned int)time(nullptr));
	}

	/// Number of iterations to test
	int m_numIterations;
	std::shared_ptr<SurgSim::Framework::Runtime> m_runtime;
};

TEST_F(MeshShapeTest, InvalidMeshCubeTest)
{
	// Cube
	std::shared_ptr<TriangleMeshPlain> invalidTriMesh = std::make_shared<TriangleMeshPlain>();
	for (int i = 0; i < cubeNumPoints; i++)
	{
		Vector3d p;
		p[0] = cubePoints[i][0];
		p[1] = cubePoints[i][1];
		p[2] = cubePoints[i][2];
		TriangleMeshPlain::VertexType v(p);
		invalidTriMesh->addVertex(v);
	}
	for (int i = 0; i < cubeNumEdges; i++)
	{
		std::array<size_t, 2> edgePoints;
		for (int j = 0; j < 2; j++)
		{
			edgePoints[j] = cubeEdges[i][j];
		}
		TriangleMeshPlain::EdgeType e(edgePoints);
		invalidTriMesh->addEdge(e);
	}
	for (int i = 0; i < cubeNumTriangles; i++)
	{
		std::array<size_t, 3> trianglePoints;
		for (int j = 0; j < 3; j++)
		{
			// Add an offset of 3 to the indices (=> some of them will be invalid)
			trianglePoints[j] = cubeTrianglesCCW[i][j] + 3;
		}
		TriangleMeshPlain::TriangleType t(trianglePoints);
		invalidTriMesh->addTriangle(t);
	}

	EXPECT_THROW(MeshShape invalidMeshShape(*invalidTriMesh), SurgSim::Framework::AssertionFailure);
}

TEST_F(MeshShapeTest, EmptyMeshTest)
{
	std::shared_ptr<TriangleMeshPlain> emptyMesh = std::make_shared<TriangleMeshPlain>();

	EXPECT_NO_THROW(MeshShape meshShape(*emptyMesh));

	MeshShape meshShape(*emptyMesh);
	EXPECT_NEAR(0.0, meshShape.getVolume(), epsilon);
	EXPECT_TRUE(meshShape.getCenter().isZero());
	EXPECT_TRUE(meshShape.getSecondMomentOfVolume().isZero());
	EXPECT_TRUE(meshShape.isValid()); // An empty mesh is regarded as valid.

	MeshShape emptyMeshShape;
	EXPECT_TRUE(emptyMeshShape.isValid());
}

TEST_F(MeshShapeTest, ValidMeshTest)
{
	{
		SCOPED_TRACE("Invalid Mesh");
		auto emptyMesh = std::make_shared<TriangleMeshPlain>();
		auto meshShape = std::make_shared<MeshShape>(*emptyMesh);

		for (int i = 0; i < cubeNumEdges; ++i)
		{
			std::array<size_t, 2> edgePoints;
			for (int j = 0; j < 2; j++)
			{
				edgePoints[j] = cubeEdges[i][j];
			}
			TriangleMeshPlain::EdgeType e(edgePoints);
			meshShape->addEdge(e);
		}

		EXPECT_FALSE(meshShape->isValid());
	}

	{
		SCOPED_TRACE("Valid Mesh");
		auto emptyMesh = std::make_shared<TriangleMeshPlain>();
		auto meshShape = std::make_shared<MeshShape>(*emptyMesh);

		std::shared_ptr<TriangleMeshPlain> invalidTriMesh = std::make_shared<TriangleMeshPlain>();
		for (int i = 0; i < cubeNumPoints; i++)
		{
			Vector3d point(cubePoints[i][0], cubePoints[i][1], cubePoints[i][2]);
			TriangleMeshPlain::VertexType vertex(point);
			meshShape->addVertex(vertex);
		}

		EXPECT_TRUE(meshShape->isValid());
	}
}

TEST_F(MeshShapeTest, MeshCubeVSBoxTest)
{
	for (int iterationID = 0; iterationID < m_numIterations; iterationID++)
	{
		double lx = 10.0 * (1.0 / static_cast<double>(iterationID + 1));
		double ly = 10.0 * (2.0 / static_cast<double>(iterationID + 1));
		double lz = 10.0 * (static_cast<double>(iterationID + 1) / 3.0);

		// Cube
		std::shared_ptr<TriangleMeshPlain> mesh = std::make_shared<TriangleMeshPlain>();
		for (int i = 0; i < cubeNumPoints; i++)
		{
			Vector3d p;
			p[0] = cubePoints[i][0] * lx;
			p[1] = cubePoints[i][1] * ly;
			p[2] = cubePoints[i][2] * lz;
			TriangleMeshPlain::VertexType v(p);
			mesh->addVertex(v);
		}
		for (int i = 0; i < cubeNumEdges; i++)
		{
			std::array<size_t, 2> edgePoints;
			for (int j = 0; j < 2; j++)
			{
				edgePoints[j] = cubeEdges[i][j];
			}
			TriangleMeshPlain::EdgeType e(edgePoints);
			mesh->addEdge(e);
		}
		for (int i = 0; i < cubeNumTriangles; i++)
		{
			std::array<size_t, 3> trianglePoints;
			for (int j = 0; j < 3; j++)
			{
				trianglePoints[j] = cubeTrianglesCCW[i][j];
			}
			TriangleMeshPlain::TriangleType t(trianglePoints);
			mesh->addTriangle(t);
		}

		MeshShape boxMesh(*mesh);

		BoxShape boxShape(lx, ly, lz);

		EXPECT_NEAR(boxShape.getVolume(), boxMesh.getVolume(), epsilon);
		EXPECT_TRUE((boxShape.getCenter() - boxMesh.getCenter()).isZero());
		EXPECT_TRUE(boxShape.getSecondMomentOfVolume().isApprox(boxMesh.getSecondMomentOfVolume(), epsilon));
	}
}

TEST_F(MeshShapeTest, SerializationTest)
{
	const std::string fileName = "MeshShapeData/staple_collision.ply";
	auto meshShape = std::make_shared<MeshShape>();
	EXPECT_NO_THROW(meshShape->load(fileName));
	EXPECT_TRUE(meshShape->isValid());

	// We chose to let YAML serialization only works with base class pointer.
	// i.e. We need to serialize 'meshShape' via a SurgSim::Math::Shape pointer.
	// The usage YAML::Node node = meshShape; will not compile.
	std::shared_ptr<Shape> shape = meshShape;

	YAML::Node node;
	ASSERT_NO_THROW(node = shape); // YAML::convert<std::shared_ptr<SurgSim::Math::Shape>> will be called.
	EXPECT_TRUE(node.IsMap());
	EXPECT_EQ(1u, node.size());

	std::shared_ptr<MeshShape> newMeshShape;
	ASSERT_NO_THROW(newMeshShape = std::dynamic_pointer_cast<MeshShape>(node.as<std::shared_ptr<Shape>>()));

	EXPECT_EQ("SurgSim::Math::MeshShape", newMeshShape->getClassName());

	EXPECT_EQ(meshShape->getNumVertices(), newMeshShape->getNumVertices());
	EXPECT_EQ(meshShape->getNumEdges(), newMeshShape->getNumEdges());
	EXPECT_EQ(meshShape->getNumTriangles(), newMeshShape->getNumTriangles());
	EXPECT_TRUE(newMeshShape->isValid());
}

TEST_F(MeshShapeTest, CreateAabbTreeTest)
{
	const std::string fileName = "MeshShapeData/staple_collision.ply";
	auto meshShape = std::make_shared<MeshShape>();
	EXPECT_NO_THROW(meshShape->load(fileName));

	auto tree = meshShape->getAabbTree();

	auto triangles = meshShape->getTriangles();
	auto vertices = meshShape->getVertices();

	EXPECT_EQ(224u, triangles.size());
	EXPECT_EQ(504u, vertices.size());

	for (auto it = triangles.begin(); it != triangles.end(); ++it)
	{
		auto ids = it->verticesId;
		EXPECT_TRUE(tree->getAabb().contains(makeAabb(
				vertices[ids[0]].position, vertices[ids[1]].position, vertices[ids[2]].position)));
	}
}

TEST_F(MeshShapeTest, SetPoseTest)
{
	const std::string fileName = "MeshShapeData/staple_collision.ply";
	auto mesh = std::make_shared<SurgSim::DataStructures::TriangleMeshPlain>();
	ASSERT_NO_THROW(mesh->load(fileName));

	auto originalMesh = std::make_shared<MeshShape>(*mesh);
	auto actualMesh = std::make_shared<MeshShape>(*mesh);

	RigidTransform3d transform = makeRigidTransform(Vector3d(4.3, 2.1, 6.5), Vector3d(-1.5, 7.5, -2.5),
			Vector3d(8.7, -4.7, -3.1));
	actualMesh->setPose(transform);

	Vector3d expectedPosition;
	ASSERT_EQ(originalMesh->getNumVertices(), actualMesh->getNumVertices());
	for (size_t i = 0; i < actualMesh->getNumVertices(); i++)
	{
		Vector3d expectedPosition = transform * originalMesh->getVertexPosition(i);
		ASSERT_TRUE(actualMesh->getVertexPosition(i).isApprox(expectedPosition, epsilon)) <<
			"Vertex #" << i << " is not as expected, remaining vertices may also be incorrect.";
	}

	Vector3d expectedNormal;
	ASSERT_EQ(originalMesh->getNumTriangles(), actualMesh->getNumTriangles());
	for (size_t i = 0; i < actualMesh->getNumTriangles(); i++)
	{
		expectedNormal = transform.linear() * originalMesh->getTriangle(i).data.normal;
		ASSERT_TRUE(expectedNormal.isApprox(actualMesh->getTriangle(i).data.normal, epsilon)) <<
			"Normal #" << i << " is not as expected, remaining normals may also be incorrect.";
	}
}

TEST_F(MeshShapeTest, NormalTest)
{
	auto mesh = std::make_shared<TriangleMeshPlain>();

	// Add vertex
	TriangleMeshPlain::VertexType v0(Vector3d(-1.0, -1.0, -1.0));
	TriangleMeshPlain::VertexType v1(Vector3d(1.0, -1.0, -1.0));
	TriangleMeshPlain::VertexType v2(Vector3d(-1.0,  1.0, -1.0));

	mesh->addVertex(v0);
	mesh->addVertex(v1);
	mesh->addVertex(v2);

	// Add edges
	std::array<size_t, 2> edgePoints01 = {0, 1};
	std::array<size_t, 2> edgePoints12 = {1, 2};
	std::array<size_t, 2> edgePoints20 = {2, 0};

	TriangleMeshPlain::EdgeType e01(edgePoints01);
	TriangleMeshPlain::EdgeType e12(edgePoints12);
	TriangleMeshPlain::EdgeType e20(edgePoints20);

	mesh->addEdge(e01);
	mesh->addEdge(e12);
	mesh->addEdge(e20);

	// Add triangle
	std::array<size_t, 3> trianglePoints = {0, 1, 2};

	TriangleMeshPlain::TriangleType t(trianglePoints);
	mesh->addTriangle(t);

	std::shared_ptr<MeshShape> meshWithNormal = std::make_shared<MeshShape>(*mesh);

	Vector3d expectedZNormal(0.0, 0.0, 1.0);
	EXPECT_EQ(expectedZNormal, meshWithNormal->getNormal(0));

	// Update new vertex location of v2 to v3
	MeshShape::VertexType v3(Vector3d(-1.0, -1.0, 1.0));
	SurgSim::DataStructures::Vertex<SurgSim::DataStructures::EmptyData>& v2p = meshWithNormal->getVertex(2);
	v2p = v3;

	// Recompute normals for meshWithNormal
	meshWithNormal->calculateNormals();
	Vector3d expectedXNormal(0.0, -1.0, 0.0);
	EXPECT_EQ(expectedXNormal, meshWithNormal->getNormal(0));
}


TEST_F(MeshShapeTest, DoLoadTest)
{
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	{
		SCOPED_TRACE("Normal load should succeed");
		auto meshShape = std::make_shared<MeshShape>();
		EXPECT_NO_THROW(meshShape->load("MeshShapeData/staple_collision.ply"));
		EXPECT_TRUE(meshShape->isValid());
	}

	{
		SCOPED_TRACE("Load through parameter should succeed");
		auto fileName = std::string("MeshShapeData/staple_collision.ply");
		auto meshShape = std::make_shared<MeshShape>();

		EXPECT_NO_THROW(meshShape->setValue("FileName", fileName));
		EXPECT_TRUE(meshShape->isValid());
	}

	{
		SCOPED_TRACE("Load of invalid mesh should throw");
		auto meshShape = std::make_shared<MeshShape>();
		EXPECT_THROW(meshShape->load("MeshShapeData/InvalidMesh.ply"), SurgSim::Framework::AssertionFailure);
	}

	{
		SCOPED_TRACE("Load of non existant file should throw");
		auto fileName = std::string("Nonexistent file");
		auto meshShape = std::make_shared<MeshShape>();
		EXPECT_THROW(meshShape->load("Nonexistent file"), SurgSim::Framework::AssertionFailure);
	}
}

};
};
