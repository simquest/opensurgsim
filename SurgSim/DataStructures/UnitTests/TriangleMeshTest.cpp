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

#include <gtest/gtest.h>

#include "SurgSim/DataStructures/MeshElement.h"
#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/DataStructures/TriangleMeshUtilities.h"
#include "SurgSim/DataStructures/UnitTests/MockObjects.h"
#include "SurgSim/DataStructures/Vertex.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"

#include <random>

using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace DataStructures
{

TEST(TriangleMeshTest, NormalTest)
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
	std::array<size_t, 2> edgePoints01;
	std::array<size_t, 2> edgePoints02;
	std::array<size_t, 2> edgePoints12;

	TriangleMeshPlain::EdgeType e01(edgePoints01);
	TriangleMeshPlain::EdgeType e02(edgePoints02);
	TriangleMeshPlain::EdgeType e12(edgePoints12);

	mesh->addEdge(e01);
	mesh->addEdge(e02);
	mesh->addEdge(e12);

	// Add triangle
	std::array<size_t, 3> trianglePoints = {0, 1, 2};

	TriangleMeshPlain::TriangleType t(trianglePoints);
	mesh->addTriangle(t);

	std::shared_ptr<TriangleMesh> meshWithNormal = std::make_shared<TriangleMesh>(*mesh);

	Vector3d expectedZNormal(0.0, 0.0, 1.0);
	EXPECT_EQ(expectedZNormal, meshWithNormal->getNormal(0));

	// Update new vertex location of v2 to v3
	TriangleMeshPlain::VertexType v3(Vector3d(-1.0, -1.0, 1.0));
	SurgSim::DataStructures::Vertex<SurgSim::DataStructures::EmptyData>& v2p = meshWithNormal->getVertex(2);
	v2p = v3;

	// Recompute normals for meshWithNormal
	meshWithNormal->calculateNormals();
	Vector3d expectedXNormal(0.0, -1.0, 0.0);
	EXPECT_EQ(expectedXNormal, meshWithNormal->getNormal(0));
}

TEST(TriangleMeshTest, CopyWithTransformTest)
{
	const std::string fileName = "MeshShapeData/staple_collision.ply";
	auto originalMesh = std::make_shared<SurgSim::DataStructures::TriangleMesh>(*loadTriangleMesh(fileName));
	auto expectedMesh = std::make_shared<SurgSim::DataStructures::TriangleMesh>(*originalMesh);
	auto actualMesh = std::make_shared<SurgSim::DataStructures::TriangleMesh>(*originalMesh);

	RigidTransform3d transform = SurgSim::Math::makeRigidTransform(
									 Vector3d(4.3, 2.1, 6.5), Vector3d(-1.5, 7.5, -2.5), Vector3d(8.7, -4.7, -3.1));

	for (auto it = expectedMesh->getVertices().begin(); it != expectedMesh->getVertices().end(); ++it)
	{
		it->position = transform * it->position;
	}

	for (auto it = expectedMesh->getTriangles().begin(); it != expectedMesh->getTriangles().end(); ++it)
	{
		it->data.normal = transform.linear() * it->data.normal;
	}

	actualMesh->copyWithTransform(transform, *originalMesh);

	EXPECT_EQ(expectedMesh->getVertices(), actualMesh->getVertices());
	EXPECT_EQ(expectedMesh->getTriangles(), actualMesh->getTriangles());
}

TEST(TriangleMeshTest, DoLoadTest)
{
	SurgSim::Framework::ApplicationData appData("config.txt");

	{
		SCOPED_TRACE("Load nonexistent file should throw");
		// Nonexistent file
		const std::string fileName = "Nonexistent file";
		auto mesh = std::make_shared<SurgSim::DataStructures::TriangleMesh>();
		EXPECT_ANY_THROW(mesh->load(fileName, appData));
	}

	{
		SCOPED_TRACE("Load existent file which contains invalid mesh should throw");
		// File exists, but contains an invalid Mesh
		const std::string fileName = "MeshShapeData/InvalidMesh.ply";
		auto mesh = std::make_shared<SurgSim::DataStructures::TriangleMesh>();
		EXPECT_ANY_THROW(mesh->load(fileName, appData));
	}

	{
		SCOPED_TRACE("Load existent file which contains valid mesh should not throw");
		// File exists, and contains a valid Mesh
		const std::string fileName = "MeshShapeData/staple_collision.ply";
		auto mesh = std::make_shared<SurgSim::DataStructures::TriangleMesh>();
		EXPECT_NO_THROW(mesh->load(fileName, appData));
	}
}

};
};
