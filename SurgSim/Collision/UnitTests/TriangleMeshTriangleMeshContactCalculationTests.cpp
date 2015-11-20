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

#include "SurgSim/Collision/TriangleMeshTriangleMeshContact.h"
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

void doTriangleMeshTriangleMeshTest(std::shared_ptr<MeshShape> meshA,
									const RigidTransform3d& meshATransform,
									std::shared_ptr<MeshShape> meshB,
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
	TriangleMeshTriangleMeshDcdContact calcContact;
	std::shared_ptr<CollisionPair> pair = std::make_shared<CollisionPair>(meshARep, meshBRep);
	calcContact.calculateContact(pair);

	contactsInfoEqualityTest(expectedContacts, pair->getContacts(), true);
}

TEST(TriangleMeshTriangleMeshContactCalculationTests, NonintersectionTest)
{
	using SurgSim::Math::makeRigidTransform;
	using SurgSim::Math::makeRotationQuaternion;

	// Create a Mesh Cube
	std::shared_ptr<TriangleMeshPlain> mesh = std::make_shared<TriangleMeshPlain>();
	for (int i = 0; i < cubeNumPoints; ++i)
	{
		Vector3d p;
		p[0] = cubePoints[i][0];
		p[1] = cubePoints[i][1];
		p[2] = cubePoints[i][2];
		TriangleMeshPlain::VertexType v(p);
		mesh->addVertex(v);
	}
	for (int i = 0; i < cubeNumEdges; ++i)
	{
		std::array<size_t, 2> edgePoints;
		for (int j = 0; j < 2; j++)
		{
			edgePoints[j] = cubeEdges[i][j];
		}
		TriangleMeshPlain::EdgeType e(edgePoints);
		mesh->addEdge(e);
	}
	for (int i = 0; i < cubeNumTriangles; ++i)
	{
		std::array<size_t, 3> trianglePoints;
		for (int j = 0; j < 3; j++)
		{
			trianglePoints[j] = cubeTrianglesCCW[i][j];
		}
		TriangleMeshPlain::TriangleType t(trianglePoints);
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

TEST(TriangleMeshTriangleMeshContactCalculationTests, IntersectionTest)
{
	using SurgSim::Math::MeshShape;
	using SurgSim::Math::Vector3d;
	using SurgSim::Math::RigidTransform3d;

	RigidTransform3d pose;
	pose = SurgSim::Math::makeRigidTransform(
			   SurgSim::Math::makeRotationQuaternion(0.3468, Vector3d(0.2577, 0.8245, 1.0532).normalized()),
			   Vector3d(120.34, 567.23, -832.84));
	{
		// The second mesh.
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
		double zOffset = 0.5 / numTriangles;
		double zValue;
		std::pair<Location, Location> expectedPenetrationPoints;
		Vector3d expectedPoint0, expectedPoint1;
		Vector3d expectedNormal, expectedContact(0, 0, 0);
		for (int i = 0; i < numTriangles - 1; i++)
		{
			zValue = static_cast<double>(i) / numTriangles + zOffset;
			addNewTriangle(baseTriangles,
						   Vector3d(0.5, 0.5, zValue),
						   Vector3d(-0.5, 0.5, zValue),
						   Vector3d(0.0, -0.5, zValue));
			expectedDepth = zValue;
			if (expectedDepth >= 0.5)
			{
				expectedDepth = 0.5;
				expectedNormal = pose.linear() * Vector3d(0, 1, 0);
				expectedPoint0 = Vector3d(0, -0.5, zValue);
				expectedPoint1 = Vector3d(0, 0, zValue);
			}
			else
			{
				expectedNormal = pose.linear() * Vector3d(0, 0, -1);
				expectedPoint0 = Vector3d(0, 0, zValue);
				expectedPoint1 = Vector3d(0, 0, 0);
			}
			if (expectedDepth > 0.0)
			{
				expectedPenetrationPoints.first.rigidLocalPosition.setValue(expectedPoint0);
				expectedPenetrationPoints.second.rigidLocalPosition.setValue(expectedPoint1);
				SurgSim::DataStructures::IndexedLocalCoordinate triangleLocalPosition;
				triangleLocalPosition.index = i;
				expectedPenetrationPoints.first.triangleMeshLocalCoordinate.setValue(triangleLocalPosition);
				triangleLocalPosition.index = 0;
				expectedPenetrationPoints.second.triangleMeshLocalCoordinate.setValue(triangleLocalPosition);
				auto contact = std::make_shared<TriangleContact>(
								   COLLISION_DETECTION_TYPE_DISCRETE, expectedDepth,
								   1.0, expectedContact, expectedNormal, expectedPenetrationPoints);
				contact->firstVertices = baseTriangles->getTrianglePositions(baseTriangles->getNumTriangles() - 1);
				contact->secondVertices = intersectingTriangle->getTrianglePositions(0);
				for (size_t i = 0; i < 3; ++i)
				{
					contact->firstVertices[i] = contact->firstVertices[i];
					contact->secondVertices[i] = contact->secondVertices[i];
				}
				expectedContacts.push_back(contact);
			}
		}
		auto baseMesh = std::make_shared<MeshShape>(*baseTriangles);

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

TEST(TriangleMeshTriangleMeshContactCalculationTests, IntersectionTestAtIdenticalDepth)
{
	using SurgSim::Math::MeshShape;
	using SurgSim::Math::Vector3d;
	using SurgSim::Math::RigidTransform3d;

	RigidTransform3d pose;
	pose = SurgSim::Math::makeRigidTransform(
			   SurgSim::Math::makeRotationQuaternion(0.4638, Vector3d(0.5727, 0.2485, 1.532).normalized()),
			   Vector3d(210.34, 675.23, -283.84));
	pose = SurgSim::Math::makeRigidTransform(
			   SurgSim::Math::makeRotationQuaternion(0.0, Vector3d(0.5727, 0.2485, 1.532).normalized()),
			   Vector3d(0, 0, 0));

	{
		// The second mesh.
		const double e = 8e-11;
		auto intersectingTriangle = std::make_shared<TriangleMeshPlain>();
		addNewTriangle(intersectingTriangle,
					   Vector3d(e, 0.0, 0.0),
					   Vector3d(-0.5, 0.0, 1.0),
					   Vector3d(e, 0.0, 1.0));
		auto triangleMesh = std::make_shared<MeshShape>(*intersectingTriangle);

		// The first mesh.
		auto baseTriangles = std::make_shared<TriangleMeshPlain>();

		static const size_t numTriangles = 100;
		SurgSim::DataStructures::IndexedLocalCoordinate triangleLocalPosition;

		std::list<std::shared_ptr<Contact>> expectedContacts;
		double expectedDepth;
		double expectedTime;
		double interval = 1.0 / static_cast<double>(numTriangles + 1);
		double coordinate;
		std::pair<Location, Location> expectedPenetrationPoints;
		Vector3d expectedPoint0, expectedPoint1;
		Vector3d expectedNormal, expectedContact(0, 0, 0);
		for (size_t i = 0; i < numTriangles; i++)
		{
			coordinate = interval * static_cast<double>(i + 1);

			addNewTriangle(baseTriangles,
						   Vector3d(-e, -coordinate, coordinate),
						   Vector3d(0.5, 1.0 - coordinate, coordinate),
						   Vector3d(-e, 1.0 - coordinate, coordinate));
			expectedDepth = coordinate;
			expectedTime = 1.0;
			{
				expectedPenetrationPoints.first.rigidLocalPosition.setValue(Vector3d(0, 0, coordinate));
				expectedPenetrationPoints.second.rigidLocalPosition.setValue(Vector3d(0, 0, 0));
				triangleLocalPosition.index = i;
				expectedPenetrationPoints.first.triangleMeshLocalCoordinate.setValue(triangleLocalPosition);
				triangleLocalPosition.index = 0;
				expectedPenetrationPoints.second.triangleMeshLocalCoordinate.setValue(triangleLocalPosition);
				auto contact = std::make_shared<TriangleContact>(
								   COLLISION_DETECTION_TYPE_DISCRETE,
								   expectedDepth, expectedTime, expectedContact,
								   pose.linear() * Vector3d(0, 0, -1),
								   expectedPenetrationPoints);
				contact->firstVertices = baseTriangles->getTrianglePositions(baseTriangles->getNumTriangles() - 1);
				contact->secondVertices = intersectingTriangle->getTrianglePositions(0);
				for (size_t i = 0; i < 3; ++i)
				{
					contact->firstVertices[i] = contact->firstVertices[i];
					contact->secondVertices[i] = contact->secondVertices[i];
				}
				expectedContacts.push_back(contact);
			}
			{
				expectedPenetrationPoints.first.rigidLocalPosition.setValue(Vector3d(0, -coordinate, coordinate));
				expectedPenetrationPoints.second.rigidLocalPosition.setValue(Vector3d(0, 0, coordinate));
				triangleLocalPosition.index = i;
				expectedPenetrationPoints.first.triangleMeshLocalCoordinate.setValue(triangleLocalPosition);
				triangleLocalPosition.index = 0;
				expectedPenetrationPoints.second.triangleMeshLocalCoordinate.setValue(triangleLocalPosition);
				auto contact = std::make_shared<TriangleContact>(
								   COLLISION_DETECTION_TYPE_DISCRETE,
								   expectedDepth, expectedTime, expectedContact,
								   pose.linear() * Vector3d(0, 1, 0),
								   expectedPenetrationPoints);
				contact->firstVertices = baseTriangles->getTrianglePositions(baseTriangles->getNumTriangles() - 1);
				contact->secondVertices = intersectingTriangle->getTrianglePositions(0);
				for (size_t i = 0; i < 3; ++i)
				{
					contact->firstVertices[i] = contact->firstVertices[i];
					contact->secondVertices[i] = contact->secondVertices[i];
				}
				expectedContacts.push_back(contact);
			}
		}
		auto baseMesh = std::make_shared<MeshShape>(*baseTriangles);

		std::shared_ptr<ShapeCollisionRepresentation> meshARep =
			std::make_shared<ShapeCollisionRepresentation>("Collision Mesh 0");
		meshARep->setShape(baseMesh);
		meshARep->setLocalPose(pose);

		std::shared_ptr<ShapeCollisionRepresentation> meshBRep =
			std::make_shared<ShapeCollisionRepresentation>("Collision Mesh 1");
		meshBRep->setShape(triangleMesh);
		meshBRep->setLocalPose(pose);

		// Perform collision detection.
		TriangleMeshTriangleMeshDcdContact calcContact;
		std::shared_ptr<CollisionPair> pair = std::make_shared<CollisionPair>(meshARep, meshBRep);
		calcContact.calculateContact(pair);

		auto& calculatedContacts = pair->getContacts();
		EXPECT_EQ(numTriangles, calculatedContacts.size());

		for (auto it = calculatedContacts.begin(); it != calculatedContacts.end(); ++it)
		{
			EXPECT_TRUE(isContactPresentInList(*it, expectedContacts, false));
		}
	}
}


}; // namespace Collision
}; // namespace Surgsim
