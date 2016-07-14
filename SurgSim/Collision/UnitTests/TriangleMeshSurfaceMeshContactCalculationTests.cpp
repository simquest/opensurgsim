// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
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

#include "SurgSim/Collision/TriangleMeshSurfaceMeshContact.h"
#include "SurgSim/Collision/UnitTests/ContactCalculationTestsCommon.h"
#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/IndexedLocalCoordinate.h"
#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::DataStructures::TriangleMeshPlain;
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

void doTriangleMeshSurfaceMeshTest(std::shared_ptr<MeshShape> meshA,
									const RigidTransform3d& meshATransform,
									std::shared_ptr<Math::SurfaceMeshShape> meshB,
									const RigidTransform3d& meshBTransform,
									const std::list<std::shared_ptr<Contact>> expectedContacts)
{
	std::shared_ptr<ShapeCollisionRepresentation> meshARep =
		std::make_shared<ShapeCollisionRepresentation>("Collision Mesh 0");
	meshARep->setShape(meshA);
	meshARep->setLocalPose(meshATransform);

	std::shared_ptr<ShapeCollisionRepresentation> meshBRep =
		std::make_shared<ShapeCollisionRepresentation>("Collision Mesh 1");
	meshBRep->setShape(meshB);
	meshBRep->setLocalPose(meshBTransform);

	// Perform collision detection.
	TriangleMeshSurfaceMeshContact calcContact;
	std::shared_ptr<CollisionPair> pair = std::make_shared<CollisionPair>(meshARep, meshBRep);
	calcContact.calculateContact(pair);

	contactsInfoEqualityTest(expectedContacts, pair->getContacts(), true);
}

TEST(TriangleMeshSurfaceMeshContactCalculationTests, NonintersectionTest)
{
	using SurgSim::Math::makeRigidTransform;
	using SurgSim::Math::makeRotationQuaternion;

	// Create a Mesh Cube
	std::shared_ptr<TriangleMeshPlain> meshA = std::make_shared<TriangleMeshPlain>();
	std::shared_ptr<TriangleMeshPlain> meshB = std::make_shared<TriangleMeshPlain>();
	for (int i = 0; i < cubeNumPoints; ++i)
	{
		Vector3d p;
		p[0] = cubePoints[i][0];
		p[1] = cubePoints[i][1];
		p[2] = cubePoints[i][2];
		TriangleMeshPlain::VertexType v(p);
		meshA->addVertex(v);
		meshB->addVertex(v);
	}
	for (int i = 0; i < cubeNumEdges; ++i)
	{
		std::array<size_t, 2> edgePoints;
		for (int j = 0; j < 2; j++)
		{
			edgePoints[j] = cubeEdges[i][j];
		}
		TriangleMeshPlain::EdgeType e(edgePoints);
		meshA->addEdge(e);
		meshB->addEdge(e);
	}
	for (int i = 0; i < cubeNumTriangles; ++i)
	{
		std::array<size_t, 3> trianglePoints;
		for (int j = 0; j < 3; j++)
		{
			trianglePoints[j] = cubeTrianglesCCW[i][j];
		}
		TriangleMeshPlain::TriangleType t(trianglePoints);
		meshA->addTriangle(t);
		if (i < cubeNumTriangles - 2)
		{
			meshB->addTriangle(t);
		}
	}

	std::shared_ptr<SurgSim::Math::MeshShape> cubeMeshA = std::make_shared<SurgSim::Math::MeshShape>(*meshA);
	std::shared_ptr<Math::SurfaceMeshShape> cubeMeshB = std::make_shared<Math::SurfaceMeshShape>(*meshB);

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

		doTriangleMeshSurfaceMeshTest(cubeMeshA, cubeMeshATransform, cubeMeshB, cubeMeshBTransform, emptyContacts);
	}

	{
		SCOPED_TRACE("No intersection, boxB below boxA");
		cubeMeshATransform.setIdentity();
		cubeMeshBTransform = Eigen::Translation<double, 3>(0.0, -(cubeSize + epsilonTrans), 0.0);
		globalTransform = makeRigidTransform(makeRotationQuaternion(1.4, Vector3d(10.2, 34.4, 15.6).normalized()),
											 Vector3d(3.4, 6.6, 2.5));
		cubeMeshATransform = globalTransform * cubeMeshATransform;
		cubeMeshBTransform = globalTransform * cubeMeshBTransform;

		doTriangleMeshSurfaceMeshTest(cubeMeshA, cubeMeshATransform, cubeMeshB, cubeMeshBTransform, emptyContacts);
	}

	{
		SCOPED_TRACE("No intersection, boxB to the left of boxA");
		cubeMeshATransform.setIdentity();
		cubeMeshBTransform = Eigen::Translation<double, 3>(-(cubeSize + epsilonTrans), 0.0, 0.0);
		globalTransform = makeRigidTransform(makeRotationQuaternion(1.4, Vector3d(1.2, 3.4, 5.6).normalized()),
											 Vector3d(340.4, 567.6, 234.5));
		cubeMeshATransform = globalTransform * cubeMeshATransform;
		cubeMeshBTransform = globalTransform * cubeMeshBTransform;

		doTriangleMeshSurfaceMeshTest(cubeMeshA, cubeMeshATransform, cubeMeshB, cubeMeshBTransform, emptyContacts);
	}

	{
		SCOPED_TRACE("No intersection, boxB to the right of boxA");
		cubeMeshATransform.setIdentity();
		cubeMeshBTransform = Eigen::Translation<double, 3>((cubeSize + epsilonTrans), 0.0, 0.0);
		globalTransform = makeRigidTransform(makeRotationQuaternion(1.2, Vector3d(11.2, 13.4, 15.6).normalized()),
											 Vector3d(3.4, 5.6, 2.5));
		cubeMeshATransform = globalTransform * cubeMeshATransform;
		cubeMeshBTransform = globalTransform * cubeMeshBTransform;

		doTriangleMeshSurfaceMeshTest(cubeMeshA, cubeMeshATransform, cubeMeshB, cubeMeshBTransform, emptyContacts);
	}

	{
		SCOPED_TRACE("No intersection, boxB in front of boxA");
		cubeMeshATransform.setIdentity();
		cubeMeshBTransform = Eigen::Translation<double, 3>(0.0, 0.0, (cubeSize + epsilonTrans));
		globalTransform = makeRigidTransform(makeRotationQuaternion(2.234, Vector3d(10.2, 30.4, 50.6).normalized()),
											 Vector3d(84.4, 56.6, 24.5));
		cubeMeshATransform = globalTransform * cubeMeshATransform;
		cubeMeshBTransform = globalTransform * cubeMeshBTransform;

		doTriangleMeshSurfaceMeshTest(cubeMeshA, cubeMeshATransform, cubeMeshB, cubeMeshBTransform, emptyContacts);
	}

	{
		SCOPED_TRACE("No intersection, boxB behind boxA");
		cubeMeshATransform.setIdentity();
		cubeMeshBTransform = Eigen::Translation<double, 3>(0.0, 0.0, -(cubeSize + epsilonTrans));
		globalTransform = makeRigidTransform(makeRotationQuaternion(1.24, Vector3d(9.2, 7.4, 5.6).normalized()),
											 Vector3d(39.4, 67.6, 34.5));
		cubeMeshATransform = globalTransform * cubeMeshATransform;
		cubeMeshBTransform = globalTransform * cubeMeshBTransform;

		doTriangleMeshSurfaceMeshTest(cubeMeshA, cubeMeshATransform, cubeMeshB, cubeMeshBTransform, emptyContacts);
	}
}

namespace
{
void addNewTriangle(std::shared_ptr<TriangleMeshPlain> mesh,
					SurgSim::Math::Vector3d point0, SurgSim::Math::Vector3d point1, SurgSim::Math::Vector3d point2)
{

	// Add vertices
	TriangleMeshPlain::VertexType vertexMesh(Vector3d::Zero());

	vertexMesh = TriangleMeshPlain::VertexType(point0);
	size_t index0 = mesh->addVertex(vertexMesh);

	vertexMesh = TriangleMeshPlain::VertexType(point1);
	size_t index1 = mesh->addVertex(vertexMesh);

	vertexMesh = TriangleMeshPlain::VertexType(point2);
	size_t index2 = mesh->addVertex(vertexMesh);

	// Add edges
	std::array<size_t, 2> edge;
	TriangleMeshPlain::EdgeType meshEdge(edge);

	edge[0] = index0;
	edge[1] = index1;
	meshEdge = TriangleMeshPlain::EdgeType(edge);
	mesh->addEdge(meshEdge);

	edge[0] = index1;
	edge[1] = index2;
	meshEdge = TriangleMeshPlain::EdgeType(edge);
	mesh->addEdge(meshEdge);

	edge[0] = index2;
	edge[1] = index0;
	meshEdge = TriangleMeshPlain::EdgeType(edge);
	mesh->addEdge(meshEdge);

	// Add triangle
	std::array<size_t, 3> triangle = {index0, index1, index2};
	TriangleMeshPlain::TriangleType meshTriangle(triangle);
	mesh->addTriangle(meshTriangle);
}
}

TEST(TriangleMeshSurfaceMeshContactCalculationTests, IntersectionTest)
{
	using SurgSim::Math::MeshShape;
	using SurgSim::Math::Vector3d;
	using SurgSim::Math::RigidTransform3d;

	RigidTransform3d pose;
	pose = SurgSim::Math::makeRigidTransform(
			   SurgSim::Math::makeRotationQuaternion(0.3468, Vector3d(0.2577, 0.8245, 1.0532).normalized()),
			   Vector3d(120.34, 567.23, -832.84));
	{
		// The first mesh.
		auto intersectingTriangle = std::make_shared<TriangleMeshPlain>();
		addNewTriangle(intersectingTriangle,
					   Vector3d(0.0, 0.0, 0.0),
					   Vector3d(0.0, 0.0, 1.0),
					   Vector3d(1.0, 0.0, 0.5));
		auto triangleMesh = std::make_shared<MeshShape>(*intersectingTriangle);

		// The first mesh.
		auto baseTriangles = std::make_shared<TriangleMeshPlain>();

		static const int numTriangles = 100;

		std::list<std::shared_ptr<Contact>> expectedContacts;
		double expectedDepth;
		double zIncrement = 1.0 / (numTriangles + 1);
		double zValue;
		std::pair<Location, Location> expectedPenetrationPoints;
		Vector3d expectedPoint0, expectedPoint1;
		Vector3d expectedNormal, expectedContact(0, 0, 0);
		for (int i = 0; i < numTriangles; i++)
		{
			zValue = static_cast<double>(i + 1) * zIncrement;
			addNewTriangle(baseTriangles,
						   Vector3d(50, 50, zValue),
						   Vector3d(-50, 50, zValue),
						   Vector3d(0, -50, zValue));
			expectedDepth = zValue;
			if (expectedDepth >= 0.5)
			{
				expectedDepth = 1.0 - expectedDepth;
				expectedNormal = pose.linear() * Vector3d(0, 0, -1);
				expectedPoint0 = Vector3d(0, 0, 1.0);
				expectedPoint1 = Vector3d(0, 0, zValue);
			}
			else
			{
				expectedNormal = pose.linear() * Vector3d(0, 0, 1);
				expectedPoint0 = Vector3d(0, 0, 0.0);
				expectedPoint1 = Vector3d(0, 0, zValue);
			}
			if (expectedDepth > 0.0)
			{
				expectedPenetrationPoints.first.rigidLocalPosition.setValue(expectedPoint0);
				expectedPenetrationPoints.second.rigidLocalPosition.setValue(expectedPoint1);
				SurgSim::DataStructures::IndexedLocalCoordinate triangleLocalPosition;
				triangleLocalPosition.index = 0;
				expectedPenetrationPoints.first.triangleMeshLocalCoordinate.setValue(triangleLocalPosition);
				triangleLocalPosition.index = i;
				expectedPenetrationPoints.second.triangleMeshLocalCoordinate.setValue(triangleLocalPosition);
				auto contact = std::make_shared<TriangleContact>(
								   COLLISION_DETECTION_TYPE_DISCRETE, expectedDepth,
								   1.0, expectedContact, expectedNormal, expectedPenetrationPoints);
				contact->firstVertices = intersectingTriangle->getTrianglePositions(0);
				contact->secondVertices = baseTriangles->getTrianglePositions(baseTriangles->getNumTriangles() - 1);
				expectedContacts.push_back(contact);
			}
		}
		auto baseMesh = std::make_shared<Math::SurfaceMeshShape>(*baseTriangles);

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

		doTriangleMeshSurfaceMeshTest(triangleMesh, pose, baseMesh, pose, expectedContacts);
	}
}


}; // namespace Collision
}; // namespace Surgsim
