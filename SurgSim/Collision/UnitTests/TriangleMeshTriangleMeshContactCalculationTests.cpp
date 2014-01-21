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
#include "SurgSim/DataStructures/TriangleMesh.h"

using SurgSim::DataStructures::TriangleMesh;

namespace SurgSim
{
namespace Collision
{

/* CUBE
3*----------*2
/          /|
7*----------* |
|         6| |
| *0       | *1
|          |/
4*----------*5
*/
static const int cubeNumPoints = 8;
static const SurgSim::Math::Vector3d cubePoints[8] =
{
	SurgSim::Math::Vector3d(-1.0/2.0, -1.0/2.0, -1.0/2.0),
	SurgSim::Math::Vector3d(1.0/2.0, -1.0/2.0, -1.0/2.0),
	SurgSim::Math::Vector3d(1.0/2.0,  1.0/2.0, -1.0/2.0),
	SurgSim::Math::Vector3d(-1.0/2.0,  1.0/2.0, -1.0/2.0),

	SurgSim::Math::Vector3d(-1.0/2.0, -1.0/2.0,  1.0/2.0),
	SurgSim::Math::Vector3d(1.0/2.0, -1.0/2.0,  1.0/2.0),
	SurgSim::Math::Vector3d(1.0/2.0,  1.0/2.0,  1.0/2.0),
	SurgSim::Math::Vector3d(-1.0/2.0,  1.0/2.0,  1.0/2.0)
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

class EmptyData
{
public:
	bool operator ==(const EmptyData& e) const
	{
		return true;
	}
};

static Vector3d calculateTriangleMeshVertex(const int i,
											const Quaterniond& quat,
											const Vector3d& trans)
{
	return (quat * Vector3d(cubePoints[i][0], cubePoints[i][1], cubePoints[i][2])) +
		   trans;
}

void doTriangleMeshTriangleMeshTest(std::shared_ptr<MeshShape> meshA,
									const RigidTransform3d& meshATransform,
									std::shared_ptr<MeshShape> meshB,
									const RigidTransform3d& meshBTransform,
									std::list<std::shared_ptr<Contact>> expectedContacts)
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

	if (expectedContacts.size() != 0)
	{
		EXPECT_TRUE(pair->hasContacts());

		if (pair->hasContacts())
		{
			contactsInfoEqualityTest(expectedContacts, pair->getContacts());
		}
	}
}


TEST(TriangleMeshTriangleMeshContactCalculationTests, UnitTests)
{
	using SurgSim::Math::makeRigidTransform;
	using SurgSim::Math::makeRotationQuaternion;

	typedef SurgSim::DataStructures::TriangleMesh<void,void,void> TriangleMesh;
	typedef SurgSim::DataStructures::MeshElement<2,void> EdgeElement;
	typedef SurgSim::DataStructures::MeshElement<3,void> TriangleElement;

	// Create a Mesh Cube
	std::shared_ptr<TriangleMesh> mesh = std::make_shared<TriangleMesh>();
	for (int i = 0; i < cubeNumPoints; i++)
	{
		SurgSim::Math::Vector3d p;
		p[0] = cubePoints[i][0];
		p[1] = cubePoints[i][1];
		p[2] = cubePoints[i][2];
		TriangleMesh::VertexType v(p);
		mesh->addVertex(v);
	}
	for (int i = 0; i < cubeNumEdges; i++)
	{
		std::array<unsigned int,2> edgePoints;
		for (int j = 0; j < 2; j++)
		{
			edgePoints[j] = cubeEdges[i][j];
		}
		EdgeElement edgeElement(edgePoints);
		TriangleMesh::EdgeType e(edgeElement);
		mesh->addEdge(e);
	}
	for (int i = 0; i < cubeNumTriangles; i++)
	{
		std::array<unsigned int,3> trianglePoints;
		for (int j = 0; j < 3; j++)
		{
			trianglePoints[j] = cubeTrianglesCCW[i][j];
		}
		TriangleElement triangleElement(trianglePoints);
		TriangleMesh::TriangleType t(triangleElement);
		mesh->addTriangle(t);
	}

	std::shared_ptr<SurgSim::Math::MeshShape> cubeMeshA =
			std::make_shared<SurgSim::Math::MeshShape>(mesh);
	std::shared_ptr<SurgSim::Math::MeshShape> cubeMeshB =
			std::make_shared<SurgSim::Math::MeshShape>(mesh);

	SurgSim::Math::RigidTransform3d cubeMeshATransform;
	SurgSim::Math::RigidTransform3d cubeMeshBTransform;
	SurgSim::Math::RigidTransform3d globalTransform;

	double cubeSize = 1.0;
	double epsilonTrans = 0.001;

	{
		SCOPED_TRACE("No intersection, boxB above boxA");
		cubeMeshATransform = makeRigidTransform(
								 Quaterniond::Identity(),
								 Vector3d(0.0,0.0,0.0));
		cubeMeshBTransform = makeRigidTransform(
								 Quaterniond::Identity(),
								 Vector3d(0.0, (cubeSize + epsilonTrans), 0.0));
		globalTransform = makeRigidTransform(
							  makeRotationQuaternion(1.234, Vector3d(1.2, 3.4, 5.6)),
							  Vector3d(34.4, 567.6, 234.5));
		cubeMeshATransform = globalTransform * cubeMeshATransform;
		cubeMeshBTransform = globalTransform * cubeMeshBTransform;
		std::list<std::shared_ptr<Contact>> expectedContacts;
		doTriangleMeshTriangleMeshTest(cubeMeshA, cubeMeshATransform, cubeMeshB, cubeMeshBTransform, expectedContacts);
	}

	{
		SCOPED_TRACE("No intersection, boxB below boxA");
		cubeMeshATransform = makeRigidTransform(
								 Quaterniond::Identity(),
								 Vector3d(0.0,0.0,0.0));
		cubeMeshBTransform = makeRigidTransform(
								 Quaterniond::Identity(),
								 Vector3d(0.0, -(cubeSize + epsilonTrans), 0.0));
		globalTransform = makeRigidTransform(
							  makeRotationQuaternion(1.4, Vector3d(10.2, 34.4, 15.6)),
							  Vector3d(3.4, 6.6, 2.5));
		cubeMeshATransform = globalTransform * cubeMeshATransform;
		cubeMeshBTransform = globalTransform * cubeMeshBTransform;
		std::list<std::shared_ptr<Contact>> expectedContacts;
		doTriangleMeshTriangleMeshTest(cubeMeshA, cubeMeshATransform, cubeMeshB, cubeMeshBTransform, expectedContacts);
	}

	{
		SCOPED_TRACE("No intersection, boxB to the left of boxA");
		cubeMeshATransform = makeRigidTransform(
								 Quaterniond::Identity(),
								 Vector3d(0.0,0.0,0.0));
		cubeMeshBTransform = makeRigidTransform(
								 Quaterniond::Identity(),
								 Vector3d(-(cubeSize + epsilonTrans), 0.0, 0.0));
		globalTransform = makeRigidTransform(
							  makeRotationQuaternion(1.4, Vector3d(1.2, 3.4, 5.6)),
							  Vector3d(340.4, 567.6, 234.5));
		cubeMeshATransform = globalTransform * cubeMeshATransform;
		cubeMeshBTransform = globalTransform * cubeMeshBTransform;
		std::list<std::shared_ptr<Contact>> expectedContacts;
		doTriangleMeshTriangleMeshTest(cubeMeshA, cubeMeshATransform, cubeMeshB, cubeMeshBTransform, expectedContacts);
	}

	{
		SCOPED_TRACE("No intersection, boxB to the right of boxA");
		cubeMeshATransform = makeRigidTransform(
								 Quaterniond::Identity(),
								 Vector3d(0.0,0.0,0.0));
		cubeMeshBTransform = makeRigidTransform(
								 Quaterniond::Identity(),
								 Vector3d((cubeSize + epsilonTrans), 0.0, 0.0));
		globalTransform = makeRigidTransform(
							  makeRotationQuaternion(1.2, Vector3d(11.2, 13.4, 15.6)),
							  Vector3d(3.4, 5.6, 2.5));
		cubeMeshATransform = globalTransform * cubeMeshATransform;
		cubeMeshBTransform = globalTransform * cubeMeshBTransform;
		std::list<std::shared_ptr<Contact>> expectedContacts;
		doTriangleMeshTriangleMeshTest(cubeMeshA, cubeMeshATransform, cubeMeshB, cubeMeshBTransform, expectedContacts);
	}

	{
		SCOPED_TRACE("No intersection, boxB in front of boxA");
		cubeMeshATransform = makeRigidTransform(
								 Quaterniond::Identity(),
								 Vector3d(0.0,0.0,0.0));
		cubeMeshBTransform = makeRigidTransform(
								 Quaterniond::Identity(),
								 Vector3d(0.0, 0.0, (cubeSize + epsilonTrans)));
		globalTransform = makeRigidTransform(
							  makeRotationQuaternion(2.234, Vector3d(10.2, 30.4, 50.6)),
							  Vector3d(84.4, 56.6, 24.5));
		cubeMeshATransform = globalTransform * cubeMeshATransform;
		cubeMeshBTransform = globalTransform * cubeMeshBTransform;
		std::list<std::shared_ptr<Contact>> expectedContacts;
		doTriangleMeshTriangleMeshTest(cubeMeshA, cubeMeshATransform, cubeMeshB, cubeMeshBTransform, expectedContacts);
	}

	{
		SCOPED_TRACE("No intersection, boxB behind boxA");
		cubeMeshATransform = makeRigidTransform(
								 Quaterniond::Identity(),
								 Vector3d(0.0,0.0,0.0));
		cubeMeshBTransform = makeRigidTransform(
								 Quaterniond::Identity(),
								 Vector3d(0.0, 0.0, -(cubeSize + epsilonTrans)));
		globalTransform = makeRigidTransform(
							  makeRotationQuaternion(1.24, Vector3d(9.2, 7.4, 5.6)),
							  Vector3d(39.4, 67.6, 34.5));
		cubeMeshATransform = globalTransform * cubeMeshATransform;
		cubeMeshBTransform = globalTransform * cubeMeshBTransform;
		std::list<std::shared_ptr<Contact>> expectedContacts;
		doTriangleMeshTriangleMeshTest(cubeMeshA, cubeMeshATransform, cubeMeshB, cubeMeshBTransform, expectedContacts);
	}

	epsilonTrans = -1e-10;
	double expectedDepth = 1.0;
	std::pair<Location, Location> penetrationPoint;
	penetrationPoint.first.globalPosition.setValue(Vector3d::Zero());
	penetrationPoint.second.globalPosition.setValue(Vector3d::Zero());

	{
		SCOPED_TRACE("Just touching, boxB above boxA");
		cubeMeshATransform = makeRigidTransform(
								 Quaterniond::Identity(),
								 Vector3d(0.0,0.0,0.0));
		cubeMeshBTransform = makeRigidTransform(
								 Quaterniond::Identity(),
								 Vector3d(0.0, (cubeSize + epsilonTrans), 0.0));
		globalTransform = makeRigidTransform(
							  makeRotationQuaternion(2.4, Vector3d(-1.2, 3.4, 5.6)),
							  Vector3d(34.4, -67.6, 34.5));
		cubeMeshATransform = globalTransform * cubeMeshATransform;
		cubeMeshBTransform = globalTransform * cubeMeshBTransform;
		std::shared_ptr<Contact> expectedContact = std::make_shared<Contact>(expectedDepth,
												   Vector3d::Zero(),
												   globalTransform.rotation() * -Vector3d::UnitY(),
												   penetrationPoint);
		std::list<std::shared_ptr<Contact>> expectedContacts;
		doTriangleMeshTriangleMeshTest(cubeMeshA, cubeMeshATransform, cubeMeshB, cubeMeshBTransform, expectedContacts);
	}

	{
		SCOPED_TRACE("Just touching, boxB below boxA");
		cubeMeshATransform = makeRigidTransform(
								 Quaterniond::Identity(),
								 Vector3d(0.0,0.0,0.0));
		cubeMeshBTransform = makeRigidTransform(
								 Quaterniond::Identity(),
								 Vector3d(0.0, -(cubeSize + epsilonTrans), 0.0));
		globalTransform = makeRigidTransform(
							  makeRotationQuaternion(-1.234, Vector3d(1.2, -3.4, -5.6)),
							  Vector3d(-34.4, -67.6, 34.5));
		cubeMeshATransform = globalTransform * cubeMeshATransform;
		cubeMeshBTransform = globalTransform * cubeMeshBTransform;
		std::shared_ptr<Contact> expectedContact = std::make_shared<Contact>(expectedDepth,
												   Vector3d::Zero(),
												   globalTransform.rotation() * Vector3d::UnitY(),
												   penetrationPoint);
		std::list<std::shared_ptr<Contact>> expectedContacts;
		doTriangleMeshTriangleMeshTest(cubeMeshA, cubeMeshATransform, cubeMeshB, cubeMeshBTransform, expectedContacts);
	}

	{
		SCOPED_TRACE("Just touching, boxB to the left of boxA");
		cubeMeshATransform = makeRigidTransform(
								 Quaterniond::Identity(),
								 Vector3d(0.0,0.0,0.0));
		cubeMeshBTransform = makeRigidTransform(
								 Quaterniond::Identity(),
								 Vector3d(-(cubeSize + epsilonTrans), 0.0, 0.0));
		globalTransform = makeRigidTransform(
							  makeRotationQuaternion(0.973, Vector3d(2.5, 3.5, 8.2)),
							  Vector3d(23.98, 95.37, 68.93));
		cubeMeshATransform = globalTransform * cubeMeshATransform;
		cubeMeshBTransform = globalTransform * cubeMeshBTransform;
		std::shared_ptr<Contact> expectedContact = std::make_shared<Contact>(expectedDepth,
												   Vector3d::Zero(),
												   globalTransform.rotation() * Vector3d::UnitX(),
												   penetrationPoint);
		std::list<std::shared_ptr<Contact>> expectedContacts;
		doTriangleMeshTriangleMeshTest(cubeMeshA, cubeMeshATransform, cubeMeshB, cubeMeshBTransform, expectedContacts);
	}

	{
		SCOPED_TRACE("Just touching, boxB to the right of boxA");
		cubeMeshATransform = makeRigidTransform(
								 Quaterniond::Identity(),
								 Vector3d(0.0,0.0,0.0));
		cubeMeshBTransform = makeRigidTransform(
								 Quaterniond::Identity(),
								 Vector3d((cubeSize + epsilonTrans), 0.0, 0.0));
		globalTransform = makeRigidTransform(
							  makeRotationQuaternion(3.272, Vector3d(7.06, 2.74, 8.36)),
							  Vector3d(4.38, 8.36, 6.87));
		cubeMeshATransform = globalTransform * cubeMeshATransform;
		cubeMeshBTransform = globalTransform * cubeMeshBTransform;
		std::shared_ptr<Contact> expectedContact = std::make_shared<Contact>(expectedDepth,
												   Vector3d::Zero(),
												   globalTransform.rotation() * -Vector3d::UnitX(),
												   penetrationPoint);
		std::list<std::shared_ptr<Contact>> expectedContacts;
		doTriangleMeshTriangleMeshTest(cubeMeshA, cubeMeshATransform, cubeMeshB, cubeMeshBTransform, expectedContacts);
	}

	{
		SCOPED_TRACE("Just touching, boxB in front of boxA");
		cubeMeshATransform = makeRigidTransform(
								 Quaterniond::Identity(),
								 Vector3d(0.0,0.0,0.0));
		cubeMeshBTransform = makeRigidTransform(
								 Quaterniond::Identity(),
								 Vector3d(0.0, 0.0, (cubeSize + epsilonTrans)));
		globalTransform = makeRigidTransform(
							  makeRotationQuaternion(-0.538, Vector3d(94.3, 65.3, -92.4)),
							  Vector3d(-84.24, 39.38, 85.63));
		cubeMeshATransform = globalTransform * cubeMeshATransform;
		cubeMeshBTransform = globalTransform * cubeMeshBTransform;
		std::shared_ptr<Contact> expectedContact = std::make_shared<Contact>(expectedDepth,
												   Vector3d::Zero(),
												   globalTransform.rotation() * -Vector3d::UnitZ(),
												   penetrationPoint);
		std::list<std::shared_ptr<Contact>> expectedContacts;
		doTriangleMeshTriangleMeshTest(cubeMeshA, cubeMeshATransform, cubeMeshB, cubeMeshBTransform, expectedContacts);
	}

	{
		SCOPED_TRACE("Just touching, boxB behind boxA");
		cubeMeshATransform = makeRigidTransform(
								 Quaterniond::Identity(),
								 Vector3d(0.0,0.0,0.0));
		cubeMeshBTransform = makeRigidTransform(
								 Quaterniond::Identity(),
								 Vector3d(0.0, 0.0, -(cubeSize + epsilonTrans)));
		globalTransform = makeRigidTransform(
							  makeRotationQuaternion(0.463, Vector3d(54.3, 5.23, 83.2)),
							  Vector3d(-84.35, 73.65, 32.47));
		cubeMeshATransform = globalTransform * cubeMeshATransform;
		cubeMeshBTransform = globalTransform * cubeMeshBTransform;
		std::shared_ptr<Contact> expectedContact = std::make_shared<Contact>(expectedDepth,
												   Vector3d::Zero(),
												   globalTransform.rotation() * Vector3d::UnitZ(),
												   penetrationPoint);
		std::list<std::shared_ptr<Contact>> expectedContacts;
		doTriangleMeshTriangleMeshTest(cubeMeshA, cubeMeshATransform, cubeMeshB, cubeMeshBTransform, expectedContacts);
	}

	epsilonTrans = 0.0;
	expectedDepth = 1.0;

	{
		SCOPED_TRACE("Vertex into face, boxB above boxA");
		cubeMeshATransform = makeRigidTransform(
								 Quaterniond::Identity(),
								 Vector3d(0.0,0.0,0.0));
		cubeMeshBTransform = makeRigidTransform(
								 makeRotationQuaternion(M_PI_4 * 1.00012, Vector3d(1.0, 0.0, 0.0)) *
								 makeRotationQuaternion(M_PI_4 * 1.00034, Vector3d(0.0, 1.0, 0.0)),
								 Vector3d(0.0, (cubeSize + epsilonTrans), 0.0));
		globalTransform = makeRigidTransform(
							  makeRotationQuaternion(6.372, Vector3d(-1.2, 3.4, 5.6)),
							  Vector3d(74.4, 67.6, 64.5));
		cubeMeshATransform = globalTransform * cubeMeshATransform;
		cubeMeshBTransform = globalTransform * cubeMeshBTransform;
		std::shared_ptr<Contact> expectedContact = std::make_shared<Contact>(expectedDepth,
												   Vector3d::Zero(),
												   globalTransform.rotation() * -Vector3d::UnitY(),
												   penetrationPoint);
		std::list<std::shared_ptr<Contact>> expectedContacts;
		doTriangleMeshTriangleMeshTest(cubeMeshA, cubeMeshATransform, cubeMeshB, cubeMeshBTransform, expectedContacts);
	}

	{
		SCOPED_TRACE("Vertex into face, boxB below boxA");
		cubeMeshATransform = makeRigidTransform(
								 Quaterniond::Identity(),
								 Vector3d(0.0,0.0,0.0));
		cubeMeshBTransform = makeRigidTransform(
								 makeRotationQuaternion(M_PI_4 * 1.0001, Vector3d(1.0, 0.0, 0.0)) *
								 makeRotationQuaternion(M_PI_4 * 1.0003, Vector3d(0.0, 1.0, 0.0)),
								 Vector3d(0.0, -(cubeSize + epsilonTrans), 0.0));
		globalTransform = makeRigidTransform(
							  makeRotationQuaternion(0.34, Vector3d(31.2, 43.4, 25.6)),
							  Vector3d(4.4, 7.6, 2.5));
		cubeMeshATransform = globalTransform * cubeMeshATransform;
		cubeMeshBTransform = globalTransform * cubeMeshBTransform;
		std::shared_ptr<Contact> expectedContact = std::make_shared<Contact>(expectedDepth,
												   Vector3d::Zero(),
												   globalTransform.rotation() * Vector3d::UnitY(),
												   penetrationPoint);
		std::list<std::shared_ptr<Contact>> expectedContacts;
		doTriangleMeshTriangleMeshTest(cubeMeshA, cubeMeshATransform, cubeMeshB, cubeMeshBTransform, expectedContacts);
	}

	{
		SCOPED_TRACE("Vertex into face, boxB to the left of boxA");
		cubeMeshATransform = makeRigidTransform(
								 Quaterniond::Identity(),
								 Vector3d(0.0,0.0,0.0));
		cubeMeshBTransform = makeRigidTransform(
								 makeRotationQuaternion(M_PI_4 * 1.0002, Vector3d(0.0, 0.0, 1.0)) *
								 makeRotationQuaternion(M_PI_4 * 1.0002, Vector3d(0.0, 1.0, 0.0)),
								 Vector3d(-(cubeSize + epsilonTrans), 0.0, 0.0));
		globalTransform = makeRigidTransform(
							  makeRotationQuaternion(-1.234, Vector3d(-1.2, -3.4, -5.6)),
							  Vector3d(-34.4, -567.6, -234.5));
		cubeMeshATransform = globalTransform * cubeMeshATransform;
		cubeMeshBTransform = globalTransform * cubeMeshBTransform;
		std::shared_ptr<Contact> expectedContact = std::make_shared<Contact>(expectedDepth,
												   Vector3d::Zero(),
												   globalTransform.rotation() * Vector3d::UnitX(),
												   penetrationPoint);
		std::list<std::shared_ptr<Contact>> expectedContacts;
		doTriangleMeshTriangleMeshTest(cubeMeshA, cubeMeshATransform, cubeMeshB, cubeMeshBTransform, expectedContacts);
	}

	{
		SCOPED_TRACE("Vertex into face, boxB to the right of boxA");
		cubeMeshATransform = makeRigidTransform(
								 Quaterniond::Identity(),
								 Vector3d(0.0,0.0,0.0));
		cubeMeshBTransform = makeRigidTransform(
								 makeRotationQuaternion(M_PI_4 * 1.00025, Vector3d(0.0, 0.0, 1.0)) *
								 makeRotationQuaternion(M_PI_4 * 1.00014, Vector3d(0.0, 1.0, 0.0)),
								 Vector3d((cubeSize + epsilonTrans), 0.0, 0.0));
		globalTransform = makeRigidTransform(
							  makeRotationQuaternion(0.34, Vector3d(1.2, -3.4, 5.6)),
							  Vector3d(-7.3, -2.5, -1.7));
		cubeMeshATransform = globalTransform * cubeMeshATransform;
		cubeMeshBTransform = globalTransform * cubeMeshBTransform;
		std::shared_ptr<Contact> expectedContact = std::make_shared<Contact>(expectedDepth,
												   Vector3d::Zero(),
												   globalTransform.rotation() * -Vector3d::UnitX(),
												   penetrationPoint);
		std::list<std::shared_ptr<Contact>> expectedContacts;
		doTriangleMeshTriangleMeshTest(cubeMeshA, cubeMeshATransform, cubeMeshB, cubeMeshBTransform, expectedContacts);
	}

	{
		SCOPED_TRACE("Vertex into face, boxB in front of boxA");
		cubeMeshATransform = makeRigidTransform(
								 Quaterniond::Identity(),
								 Vector3d(0.0,0.0,0.0));
		cubeMeshBTransform = makeRigidTransform(
								 makeRotationQuaternion(M_PI_4 * 1.00009, Vector3d(1.0, 0.0, 0.0)) *
								 makeRotationQuaternion(M_PI_4 * 1.00011, Vector3d(0.0, 1.0, 0.0)),
								 Vector3d(0.0, 0.0, (cubeSize + epsilonTrans)));
		globalTransform = makeRigidTransform(
							  makeRotationQuaternion(-0.736, Vector3d(0.7357, 0.7257, 0.3642)),
							  Vector3d(0.2846, 0.9774, 0.2974));
		cubeMeshATransform = globalTransform * cubeMeshATransform;
		cubeMeshBTransform = globalTransform * cubeMeshBTransform;
		std::shared_ptr<Contact> expectedContact = std::make_shared<Contact>(expectedDepth,
												   Vector3d::Zero(),
												   globalTransform.rotation() * -Vector3d::UnitZ(),
												   penetrationPoint);
		std::list<std::shared_ptr<Contact>> expectedContacts;
		doTriangleMeshTriangleMeshTest(cubeMeshA, cubeMeshATransform, cubeMeshB, cubeMeshBTransform, expectedContacts);
	}

	{
		SCOPED_TRACE("Vertex into face, boxB behind boxA");
		cubeMeshATransform = makeRigidTransform(
								 Quaterniond::Identity(),
								 Vector3d(0.0,0.0,0.0));
		cubeMeshBTransform = makeRigidTransform(
								 makeRotationQuaternion(M_PI_4 * 1.00011, Vector3d(1.0, 0.0, 0.0)) *
								 makeRotationQuaternion(M_PI_4 * 1.00033, Vector3d(0.0, 1.0, 0.0)),
								 Vector3d(0.0, 0.0, -(cubeSize + epsilonTrans)));
		globalTransform = makeRigidTransform(
							  makeRotationQuaternion(0.864, Vector3d(18.66, 28.64, 75.28)),
							  Vector3d(64.39, 85.27, 74.36));
		cubeMeshATransform = globalTransform * cubeMeshATransform;
		cubeMeshBTransform = globalTransform * cubeMeshBTransform;
		std::shared_ptr<Contact> expectedContact = std::make_shared<Contact>(expectedDepth,
												   Vector3d::Zero(),
												   globalTransform.rotation() * Vector3d::UnitZ(),
												   penetrationPoint);
		std::list<std::shared_ptr<Contact>> expectedContacts;
		doTriangleMeshTriangleMeshTest(cubeMeshA, cubeMeshATransform, cubeMeshB, cubeMeshBTransform, expectedContacts);
	}
}

}; // namespace Collision
}; // namespace Surgsim