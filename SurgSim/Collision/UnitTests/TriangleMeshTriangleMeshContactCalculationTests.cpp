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

#include "SurgSim/Collision/UnitTests/ContactCalculationTestsCommon.h"
#include "SurgSim/Collision/TriangleMeshTriangleMeshDcdContact.h"
#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/TriangleMeshBase.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::DataStructures::EmptyData;
using SurgSim::DataStructures::TriangleMeshBase;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Collision
{

/*	CUBE
	  3*----------*2
	  /          /|
	7*----------* |
	 |         6| |
	 | *0       | *1
	 |          |/
	4*----------*5
*/
static const int cubeNumPoints = 8;
static const Vector3d cubePoints[8] =
{
	Vector3d(-1.0 / 2.0, -1.0 / 2.0, -1.0 / 2.0),
	Vector3d( 1.0 / 2.0, -1.0 / 2.0, -1.0 / 2.0),
	Vector3d( 1.0 / 2.0,  1.0 / 2.0, -1.0 / 2.0),
	Vector3d(-1.0 / 2.0,  1.0 / 2.0, -1.0 / 2.0),
	Vector3d(-1.0 / 2.0, -1.0 / 2.0,  1.0 / 2.0),
	Vector3d( 1.0 / 2.0, -1.0 / 2.0,  1.0 / 2.0),
	Vector3d( 1.0 / 2.0,  1.0 / 2.0,  1.0 / 2.0),
	Vector3d(-1.0 / 2.0,  1.0 / 2.0,  1.0 / 2.0)
};

static const int cubeNumEdges = 12;
static const int cubeEdges[12][2] =
{
	{0, 1}, {3, 2}, {4, 5}, {7, 6}, // +X
	{0, 3}, {1, 2}, {4, 7}, {5 ,6}, // +Y
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

void doTriangleMeshTriangleMeshTest(std::shared_ptr<MeshShape> meshA,
									const RigidTransform3d& meshATransform,
									std::shared_ptr<MeshShape> meshB,
									const RigidTransform3d& meshBTransform,
									const std::list<std::shared_ptr<Contact>> expectedContacts)
{
	std::shared_ptr<Representation> meshARep = std::make_shared<ShapeCollisionRepresentation>(
												   "Collision Mesh 0",
												   meshA,
												   meshATransform);
	std::shared_ptr<Representation> meshBRep = std::make_shared<ShapeCollisionRepresentation>(
												   "Collision Mesh 1",
												   meshB,
												   meshBTransform);

	// Perform collision detection.
	TriangleMeshTriangleMeshDcdContact calcContact;
	std::shared_ptr<CollisionPair> pair = std::make_shared<CollisionPair>(meshARep, meshBRep);
	calcContact.calculateContact(pair);

	contactsInfoEqualityTest(expectedContacts, pair->getContacts());
}

TEST(TriangleMeshTriangleMeshContactCalculationTests, NonintersectionTest)
{
	using SurgSim::Math::makeRigidTransform;
	using SurgSim::Math::makeRotationQuaternion;

	typedef SurgSim::DataStructures::TriangleMeshBase<EmptyData, EmptyData, EmptyData> TriangleMeshBase;
	typedef SurgSim::DataStructures::MeshElement<2, EmptyData> EdgeElement;
	typedef SurgSim::DataStructures::MeshElement<3, EmptyData> TriangleElement;

	// Create a Mesh Cube
	std::shared_ptr<TriangleMeshBase> mesh = std::make_shared<TriangleMeshBase>();
	for (int i = 0; i < cubeNumPoints; ++i)
	{
		Vector3d p;
		p[0] = cubePoints[i][0];
		p[1] = cubePoints[i][1];
		p[2] = cubePoints[i][2];
		TriangleMeshBase::VertexType v(p);
		mesh->addVertex(v);
	}
	for (int i = 0; i < cubeNumEdges; ++i)
	{
		std::array<unsigned int, 2> edgePoints;
		for (int j = 0; j < 2; j++)
		{
			edgePoints[j] = cubeEdges[i][j];
		}
		EdgeElement edgeElement(edgePoints);
		TriangleMeshBase::EdgeType e(edgeElement);
		mesh->addEdge(e);
	}
	for (int i = 0; i < cubeNumTriangles; ++i)
	{
		std::array<unsigned int, 3> trianglePoints;
		for (int j = 0; j < 3; j++)
		{
			trianglePoints[j] = cubeTrianglesCCW[i][j];
		}
		TriangleElement triangleElement(trianglePoints);
		TriangleMeshBase::TriangleType t(triangleElement);
		mesh->addTriangle(t);
	}

	std::shared_ptr<SurgSim::Math::MeshShape> cubeMeshA = std::make_shared<SurgSim::Math::MeshShape>(*mesh);
	std::shared_ptr<SurgSim::Math::MeshShape> cubeMeshB = std::make_shared<SurgSim::Math::MeshShape>(*mesh);

	SurgSim::Math::RigidTransform3d cubeMeshATransform;
	SurgSim::Math::RigidTransform3d cubeMeshBTransform;
	SurgSim::Math::RigidTransform3d globalTransform;
	const std::list<std::shared_ptr<Contact>> emptyContacts;

	double cubeSize = 1.0;
	double epsilonTrans = 0.001;

	{
		SCOPED_TRACE("No intersection, boxB above boxA");
		cubeMeshATransform.setIdentity();
		cubeMeshBTransform = Eigen::Translation<double, 3>(0.0, cubeSize + epsilonTrans, 0.0);
		globalTransform = makeRigidTransform(makeRotationQuaternion(1.234, Vector3d(1.2, 3.4, 5.6).normalized()),
											 Vector3d(34.4, 567.6, 234.5));
		cubeMeshATransform = globalTransform * cubeMeshATransform;
		cubeMeshBTransform = globalTransform * cubeMeshBTransform;

		doTriangleMeshTriangleMeshTest(cubeMeshA, cubeMeshATransform, cubeMeshB, cubeMeshBTransform, emptyContacts);
	}

	{
		SCOPED_TRACE("No intersection, boxB below boxA");
		cubeMeshATransform.setIdentity();
		cubeMeshBTransform = Eigen::Translation<double, 3>(0.0, -(cubeSize + epsilonTrans), 0.0);
		globalTransform = makeRigidTransform(makeRotationQuaternion(1.4, Vector3d(10.2, 34.4, 15.6).normalized()),
											 Vector3d(3.4, 6.6, 2.5));
		cubeMeshATransform = globalTransform * cubeMeshATransform;
		cubeMeshBTransform = globalTransform * cubeMeshBTransform;

		doTriangleMeshTriangleMeshTest(cubeMeshA, cubeMeshATransform, cubeMeshB, cubeMeshBTransform, emptyContacts);
	}

	{
		SCOPED_TRACE("No intersection, boxB to the left of boxA");
		cubeMeshATransform.setIdentity();
		cubeMeshBTransform = Eigen::Translation<double, 3>(-(cubeSize + epsilonTrans), 0.0, 0.0);
		globalTransform = makeRigidTransform(makeRotationQuaternion(1.4, Vector3d(1.2, 3.4, 5.6).normalized()),
											 Vector3d(340.4, 567.6, 234.5));
		cubeMeshATransform = globalTransform * cubeMeshATransform;
		cubeMeshBTransform = globalTransform * cubeMeshBTransform;

		doTriangleMeshTriangleMeshTest(cubeMeshA, cubeMeshATransform, cubeMeshB, cubeMeshBTransform, emptyContacts);
	}

	{
		SCOPED_TRACE("No intersection, boxB to the right of boxA");
		cubeMeshATransform.setIdentity();
		cubeMeshBTransform = Eigen::Translation<double, 3>((cubeSize + epsilonTrans), 0.0, 0.0);
		globalTransform = makeRigidTransform(makeRotationQuaternion(1.2, Vector3d(11.2, 13.4, 15.6).normalized()),
											 Vector3d(3.4, 5.6, 2.5));
		cubeMeshATransform = globalTransform * cubeMeshATransform;
		cubeMeshBTransform = globalTransform * cubeMeshBTransform;

		doTriangleMeshTriangleMeshTest(cubeMeshA, cubeMeshATransform, cubeMeshB, cubeMeshBTransform, emptyContacts);
	}

	{
		SCOPED_TRACE("No intersection, boxB in front of boxA");
		cubeMeshATransform.setIdentity();
		cubeMeshBTransform = Eigen::Translation<double, 3>(0.0, 0.0, (cubeSize + epsilonTrans));
		globalTransform = makeRigidTransform(makeRotationQuaternion(2.234, Vector3d(10.2, 30.4, 50.6).normalized()),
											 Vector3d(84.4, 56.6, 24.5));
		cubeMeshATransform = globalTransform * cubeMeshATransform;
		cubeMeshBTransform = globalTransform * cubeMeshBTransform;

		doTriangleMeshTriangleMeshTest(cubeMeshA, cubeMeshATransform, cubeMeshB, cubeMeshBTransform, emptyContacts);
	}

	{
		SCOPED_TRACE("No intersection, boxB behind boxA");
		cubeMeshATransform.setIdentity();
		cubeMeshBTransform = Eigen::Translation<double, 3>(0.0, 0.0, -(cubeSize + epsilonTrans));
		globalTransform = makeRigidTransform(makeRotationQuaternion(1.24, Vector3d(9.2, 7.4, 5.6).normalized()),
											 Vector3d(39.4, 67.6, 34.5));
		cubeMeshATransform = globalTransform * cubeMeshATransform;
		cubeMeshBTransform = globalTransform * cubeMeshBTransform;

		doTriangleMeshTriangleMeshTest(cubeMeshA, cubeMeshATransform, cubeMeshB, cubeMeshBTransform, emptyContacts);
	}
}


void addNewTriangle(std::shared_ptr<SurgSim::DataStructures::TriangleMeshBase<EmptyData, EmptyData, EmptyData>> mesh,
					SurgSim::Math::Vector3d point0, SurgSim::Math::Vector3d point1, SurgSim::Math::Vector3d point2)
{
	typedef SurgSim::DataStructures::TriangleMeshBase<EmptyData, EmptyData, EmptyData> TriangleMesh;

	// Add vertices
	TriangleMesh::VertexType vertexMesh(Vector3d::Zero());

	vertexMesh = TriangleMesh::VertexType(point0);
	unsigned int index0 = mesh->addVertex(vertexMesh);

	vertexMesh = TriangleMesh::VertexType(point1);
	unsigned int index1 = mesh->addVertex(vertexMesh);

	vertexMesh = TriangleMesh::VertexType(point2);
	unsigned int index2 = mesh->addVertex(vertexMesh);

	// Add edges
	std::array<unsigned int, 2> edge;
	TriangleMesh::EdgeType meshEdge(edge);

	edge[0] = index0;
	edge[1] = index1;
	meshEdge = TriangleMesh::EdgeType(edge);
	mesh->addEdge(meshEdge);

	edge[0] = index1;
	edge[1] = index2;
	meshEdge = TriangleMesh::EdgeType(edge);
	mesh->addEdge(meshEdge);

	edge[0] = index2;
	edge[1] = index0;
	meshEdge = TriangleMesh::EdgeType(edge);
	mesh->addEdge(meshEdge);

	// Add triangle
	std::array<unsigned int, 3> triangle = {index0, index1, index2};
	TriangleMesh::TriangleType meshTriangle(triangle);
	mesh->addTriangle(meshTriangle);
}

TEST(TriangleMeshTriangleMeshContactCalculationTests, IntersectionTest)
{
	using SurgSim::Math::MeshShape;
	using SurgSim::Math::Vector3d;
	using SurgSim::Math::RigidTransform3d;

	RigidTransform3d pose;
	pose = SurgSim::Math::makeRigidTransform(
			SurgSim::Math::makeRotationQuaternion(0.3468, Vector3d(0.2577, 0.8245, 1.0532).normalized()),
			Vector3d(120.34, 567.23, -832.84));

	typedef SurgSim::DataStructures::TriangleMeshBase<EmptyData, EmptyData, EmptyData> TriangleMesh;

	{
		auto baseTriangles = std::make_shared<TriangleMesh>();
		static const int numTriangles = 100;

		std::list<std::shared_ptr<Contact>> expectedContacts;
		double expectedDepth;
		std::pair<Location, Location> expectedPenetrationPoints;
		Vector3d expectedPoint0, expectedPoint1;
		Vector3d expectedNormal, expectedContact;
		for (int i = 0; i < numTriangles; i++)
		{
			addNewTriangle(baseTriangles,
						   Vector3d(0.5, 0.5, static_cast<double>(i) / numTriangles),
						   Vector3d(-0.5, 0.5, static_cast<double>(i) / numTriangles),
						   Vector3d(0.0, -0.5, static_cast<double>(i) / numTriangles));
			expectedDepth = static_cast<double>(i) / numTriangles;
			if (expectedDepth > 0.5)
			{
				expectedDepth = 0.5;
				expectedNormal = pose.linear() * Vector3d(0,1,0);
				expectedPoint0 = pose * Vector3d(0,-0.5,static_cast<double>(i) / numTriangles);
				expectedPoint1 = pose * Vector3d(0,0,static_cast<double>(i) / numTriangles);
			}
			else
			{
				expectedNormal = pose.linear() * Vector3d(0,0,-1);
				expectedPoint0 = pose * Vector3d(0,0,static_cast<double>(i) / numTriangles);
				expectedPoint1 = pose * Vector3d(0,0,0);
			}
			if (expectedDepth > 0.0)
			{
				expectedPenetrationPoints.first.globalPosition.setValue(expectedPoint0);
				expectedPenetrationPoints.second.globalPosition.setValue(expectedPoint1);
				expectedPenetrationPoints.first.triangleId.setValue(i);
				expectedPenetrationPoints.second.triangleId.setValue(0);
				auto contact = std::make_shared<Contact>(expectedDepth, expectedContact, expectedNormal,
														 expectedPenetrationPoints);
				expectedContacts.push_back(contact);
			}
		}
		auto baseMesh = std::make_shared<MeshShape>(*baseTriangles);

		auto intersectingTriangle = std::make_shared<TriangleMesh>();
		addNewTriangle(intersectingTriangle,
					   Vector3d(0.0, 0.0, 0.0),
					   Vector3d(0.0, 0.0, 1.0),
					   Vector3d(1.0, 0.0, 0.5));
		auto triangleMesh = std::make_shared<MeshShape>(*intersectingTriangle);

		// Looking in -y, triangle A points in +y, +z is left, +x is down
		//                     |-------| => k
		//     * * * * * * * * * * * * *  -----
		//       *             *     *      |
		//         *           *   *        |   => 2*k on right, 2*(1-k) on left
		//           *         * *          |
		//             *       *          -----
		//               *   *
		//                 *
		//
		// Looking in -z, triangle B points in +z, +y is up, +x is right
		//    (1) When triangle A sticks sufficiently out of triangle B
		//        i.e. k >= 1/8 && k <= 7/8
		//
		//                 *             -----
		//               *   *             |   => 1/2
		//             *       *           |
		//           *     * * * *       -----
		//         *               *       |   => 1/2
		//       *                   *     |
		//     * * * * * * * * * * * * * -----
		//                 |-----| => 1/4
		//
		//    (2) When triangle A only partially sticks out of triangle B
		//        i.e. k > 7/8 || k < 1/8
		//                 *             -----
		//               *   *             |   => 1/2
		//             *       *           |
		//           *     *     *       -----
		//         *               *       |   => 1/2
		//       *                   *     |
		//     * * * * * * * * * * * * * -----
		//                 |--| => 2k
		//

		doTriangleMeshTriangleMeshTest(baseMesh, pose, triangleMesh,
									   pose, expectedContacts);
	}
}


}; // namespace Collision
}; // namespace Surgsim