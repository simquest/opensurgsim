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

#include <gtest/gtest.h>

#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/DataStructures/TriangleMeshBase.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::DataStructures::TriangleMesh;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace DataStructures
{

TEST(TriangleMeshTest, NormalTest)
{
	typedef SurgSim::DataStructures::TriangleMeshBase<void, void, void> TriangleMeshBase;
	typedef SurgSim::DataStructures::MeshElement<2, void> EdgeElement;
	typedef SurgSim::DataStructures::MeshElement<3, void> TriangleElement;

	std::shared_ptr<TriangleMeshBase> mesh = std::make_shared<TriangleMeshBase>();

	// Add vertex
	TriangleMeshBase::VertexType v0(Vector3d(-1.0, -1.0, -1.0));
	TriangleMeshBase::VertexType v1(Vector3d( 1.0, -1.0, -1.0));
	TriangleMeshBase::VertexType v2(Vector3d(-1.0,  1.0, -1.0));

	mesh->addVertex(v0);
	mesh->addVertex(v1);
	mesh->addVertex(v2);

	// Add edges
	std::array<unsigned int, 2> edgePoints01;
	std::array<unsigned int, 2> edgePoints02;
	std::array<unsigned int, 2> edgePoints12;

	EdgeElement element01(edgePoints01);
	EdgeElement element02(edgePoints02);
	EdgeElement element12(edgePoints12);

	TriangleMeshBase::EdgeType e01(element01);
	TriangleMeshBase::EdgeType e02(element02);
	TriangleMeshBase::EdgeType e12(element12);

	mesh->addEdge(e01);
	mesh->addEdge(e02);
	mesh->addEdge(e12);

	// Add triangle
	std::array<unsigned int, 3> trianglePoints = {0, 1, 2};

	TriangleElement triangleElement(trianglePoints);
	TriangleMeshBase::TriangleType t(triangleElement);
	mesh->addTriangle(t);

	std::shared_ptr<TriangleMesh> meshWithNormal = std::make_shared<TriangleMesh>(*mesh);

	Vector3d expectedZNormal(0.0, 0.0, 1.0);
	EXPECT_EQ(expectedZNormal, meshWithNormal->getNormal(0));

	// Update new vertex location of v2 to v3
	Vector3d v3(-1.0, -1.0, 1.0);
	SurgSim::DataStructures::Vertex<SurgSim::DataStructures::EmptyData>& v2p = meshWithNormal->getVertex(2);
	v2p = v3;

	// Recompute normals for meshWithNormal
	meshWithNormal->calculateNormals();
	Vector3d expectedXNormal(0.0, -1.0, 0.0);
	EXPECT_EQ(expectedXNormal, meshWithNormal->getNormal(0));
}

};
};