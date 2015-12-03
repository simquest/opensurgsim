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

#include "SurgSim/Collision/TriangleMeshPlaneContact.h"
#include "SurgSim/Collision/UnitTests/ContactCalculationTestsCommon.h"
#include "SurgSim/DataStructures/AabbTree.h"
#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::DataStructures::TriangleMeshPlain;
using SurgSim::Math::Geometry::DistanceEpsilon;
using SurgSim::Math::Matrix33d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Collision
{

namespace
{
// CUBE
//   3*----------*2
//   /          /|
// 7*----------* |
//  |         6| |
//  | *0       | *1
//  |          |/
// 4*----------*5

static const int cubeNumPoints = 8;
static const SurgSim::Math::Vector3d cubePoints[8] =
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

Vector3d calculateTriangleMeshVertex(const int i,
									 const Quaterniond& quat,
									 const Vector3d& trans)
{

	return (quat * Vector3d(cubePoints[i][0], cubePoints[i][1], cubePoints[i][2])) + trans;
}

void generateTriangleMeshPlaneContact(std::list<std::shared_ptr<Contact>>* expectedContacts,
									  const int expectedNumberOfContacts, const int* expectedMeshIndicesInContacts,
									  const Vector3d& meshTrans, const Quaterniond& meshQuat,
									  const Vector3d& planeNormal, const double planeD,
									  const Vector3d& planeTrans, const Quaterniond& planeQuat)
{
	Vector3d vertex;
	Vector3d boxLocalVertex, planeLocalVertex;
	Vector3d planeNormalGlobal = planeQuat * planeNormal;
	Vector3d pointOnPlane = planeTrans + (planeNormalGlobal * planeD);
	double depth = 0.0;
	Vector3d collisionNormal = planeNormalGlobal;

	for (int i = 0; i < expectedNumberOfContacts; ++i)
	{
		vertex = calculateTriangleMeshVertex(expectedMeshIndicesInContacts[i], meshQuat, meshTrans);
		depth = -planeNormalGlobal.dot(vertex - pointOnPlane);

		boxLocalVertex = calculateTriangleMeshVertex(expectedMeshIndicesInContacts[i],
						 Quaterniond::Identity(), Vector3d::Zero());
		planeLocalVertex = vertex + planeNormalGlobal * depth;
		planeLocalVertex = planeQuat.inverse() * (planeLocalVertex - planeTrans);

		std::pair<Location, Location> penetrationPoint;
		penetrationPoint.first.rigidLocalPosition.setValue(boxLocalVertex);
		penetrationPoint.second.rigidLocalPosition.setValue(planeLocalVertex);
		expectedContacts->push_back(std::make_shared<Contact>(
										COLLISION_DETECTION_TYPE_DISCRETE, depth, 1.0,
										Vector3d::Zero(), collisionNormal, penetrationPoint));
	}
}

void doTriangleMeshPlaneTest(std::shared_ptr<SurgSim::Math::MeshShape> mesh,
							 const Quaterniond& meshQuat,
							 const Vector3d& meshTrans,
							 std::shared_ptr<PlaneShape> plane,
							 const Quaterniond& planeQuat,
							 const Vector3d& planeTrans,
							 const int expectedNumberOfContacts,
							 const int* expectedMeshIndicesInContacts)
{
	std::shared_ptr<ShapeCollisionRepresentation> meshRep =
		std::make_shared<ShapeCollisionRepresentation>("Collision Mesh 0");
	meshRep->setShape(mesh);
	meshRep->setLocalPose(SurgSim::Math::makeRigidTransform(meshQuat, meshTrans));

	std::shared_ptr<ShapeCollisionRepresentation> planeRep =
		std::make_shared<ShapeCollisionRepresentation>("Collision Plane 0");
	planeRep->setShape(plane);
	planeRep->setLocalPose(SurgSim::Math::makeRigidTransform(planeQuat, planeTrans));

	// First calculate the expected contact info.
	std::list<std::shared_ptr<Contact>> expectedContacts;
	if (expectedNumberOfContacts > 0)
	{
		generateTriangleMeshPlaneContact(&expectedContacts, expectedNumberOfContacts, expectedMeshIndicesInContacts,
										 meshTrans, meshQuat, plane->getNormal(), plane->getD(),
										 planeTrans, planeQuat);
	}

	// Perform collision detection.
	TriangleMeshPlaneContact calcContact;
	std::shared_ptr<CollisionPair> pair = std::make_shared<CollisionPair>(meshRep, planeRep);
	calcContact.calculateContact(pair);

	const Vector3d globalPlaneNormal = planeRep->getPose().linear() * plane->getNormal();
	// update the AABB tree
	mesh->update();
	const double maxRadius = mesh->getAabbTree()->getAabb().diagonal().norm() / 2.0;
	const Vector3d planeToMesh = mesh->getCenter() - planeTrans;
	Vector3d nearestPointOnPlane;
	const double distanceMeshPlane = SurgSim::Math::distancePointPlane(planeToMesh, globalPlaneNormal,
									 plane->getD(), &nearestPointOnPlane);

	const double minDepth = -distanceMeshPlane - maxRadius;
	const double maxDepth = -distanceMeshPlane + maxRadius;

	for (auto contact : pair->getContacts())
	{
		EXPECT_LT(-DistanceEpsilon, contact->depth);
		EXPECT_LT(minDepth - DistanceEpsilon, contact->depth);
		EXPECT_GT(maxDepth + DistanceEpsilon, contact->depth);
		EXPECT_TRUE(eigenEqual(globalPlaneNormal, contact->normal));
	}

	// Compare the contact info.
	contactsInfoEqualityTest(expectedContacts, pair->getContacts());
}


TEST(TriangleMeshPlaneContactCalculationTests, UnitTests)
{
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
		for (int j = 0; j < 2; ++j)
		{
			edgePoints[j] = cubeEdges[i][j];
		}
		TriangleMeshPlain::EdgeType e(edgePoints);
		mesh->addEdge(e);
	}
	for (int i = 0; i < cubeNumTriangles; ++i)
	{
		std::array<size_t, 3> trianglePoints;
		for (int j = 0; j < 3; ++j)
		{
			trianglePoints[j] = cubeTrianglesCCW[i][j];
		}
		TriangleMeshPlain::TriangleType t(trianglePoints);
		mesh->addTriangle(t);
	}

	std::shared_ptr<SurgSim::Math::MeshShape> cubeMesh = std::make_shared<SurgSim::Math::MeshShape>(*mesh);

	std::shared_ptr<PlaneShape> plane = std::make_shared<PlaneShape>();
	Quaterniond meshQuat;
	Vector3d meshTrans;
	Quaterniond planeQuat;
	Vector3d planeTrans;
	Quaterniond globalQuat;
	Vector3d planNormal;
	Matrix33d mRotation;

	const double epsilonTrans = 0.1;
	const double cubeSize = 1.0;

	{
		SCOPED_TRACE("No intersection, box in front of plane, no rotation");
		meshQuat = Quaterniond::Identity();
		meshTrans = Vector3d::Zero();
		planeQuat = Quaterniond::Identity();
		planeTrans = Vector3d(0.0, -(cubeSize / 2.0 + epsilonTrans), 0.0);
		int expectedNumberOfContacts = 0;
		int expectedBoxIndicesInContacts[] = {0};
		doTriangleMeshPlaneTest(cubeMesh, meshQuat, meshTrans,
								plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
		SCOPED_TRACE("Intersection, 04 contacts, no rotation");
		meshQuat = Quaterniond::Identity();
		meshTrans = Vector3d::Zero();
		planeQuat = Quaterniond::Identity();
		planeTrans = Vector3d::Zero();
		int expectedNumberOfContacts = 4;
		int expectedBoxIndicesInContacts[] = {0, 1, 4, 5};
		doTriangleMeshPlaneTest(cubeMesh, meshQuat, meshTrans,
								plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
		SCOPED_TRACE("intersection, 08 contacts, no rotation");
		meshQuat = Quaterniond::Identity();
		meshTrans = Vector3d::Zero();
		planeQuat = Quaterniond::Identity();
		planeTrans = Vector3d(0.0, (cubeSize / 2.0 + epsilonTrans), 0.0);
		int expectedNumberOfContacts = 8;
		int expectedBoxIndicesInContacts[] = {0, 1, 2, 3, 4, 5, 6, 7};
		doTriangleMeshPlaneTest(cubeMesh, meshQuat, meshTrans,
								plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
		SCOPED_TRACE("intersection, 02 contacts, plane rotate(Z, -45)");
		meshQuat = Quaterniond::Identity();
		meshTrans = Vector3d::Zero();
		mRotation = Eigen::AngleAxisd(-M_PI_4, Vector3d::UnitZ());
		planeQuat = Quaterniond(mRotation);
		planeTrans = Vector3d(-1.0, -1.0, 0.0) * (cubeSize / 2.0 - epsilonTrans);
		int expectedNumberOfContacts = 2;
		int expectedBoxIndicesInContacts[] = {0, 4};
		doTriangleMeshPlaneTest(cubeMesh, meshQuat, meshTrans,
								plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
		SCOPED_TRACE("intersection, 06 contacts, plane rotate(Z, 45)");
		meshQuat = Quaterniond::Identity();
		meshTrans = Vector3d::Zero();
		mRotation = Eigen::AngleAxisd(-M_PI_4, Vector3d::UnitZ());
		planeQuat = Quaterniond(mRotation);
		planNormal = mRotation * Vector3d::UnitY();
		planeTrans = planNormal * (cubeSize / 2 - epsilonTrans);
		int expectedNumberOfContacts = 6;
		int expectedBoxIndicesInContacts[] = {0, 4, 1, 3, 7, 5};
		doTriangleMeshPlaneTest(cubeMesh, meshQuat, meshTrans,
								plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	// Test cases to cover all collisions between each corner of the cube with the plane
	{
		SCOPED_TRACE("intersection, 01 contact, plane rotate(XZ, -45)");
		meshQuat = Quaterniond::Identity();
		meshTrans = Vector3d::Zero();
		mRotation = Eigen::AngleAxisd(-M_PI_4, Vector3d(1.0, 0.0, 1.0).normalized());
		planeQuat = Quaterniond(mRotation);
		planNormal = Vector3d::UnitY();
		planeTrans = -1.0 * planNormal * (sqrt(3.0) * cubeSize / 2.0 - epsilonTrans);
		int expectedNumberOfContacts = 1;
		int expectedBoxIndicesInContacts[] = {4};
		doTriangleMeshPlaneTest(cubeMesh, meshQuat, meshTrans,
								plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
		SCOPED_TRACE("intersection, 01 contacts, plane rotate(+XZ,-M_PI/2-45)");
		meshQuat = Quaterniond::Identity();
		meshTrans = Vector3d::Zero();
		mRotation = Eigen::AngleAxisd(-M_PI_2 - M_PI_4, Vector3d(1.0, 0.0, 1.0).normalized());
		planeQuat = Quaterniond(mRotation);
		planNormal = mRotation * Vector3d::UnitY();
		planeTrans = -1.0 * planNormal * (sqrt(3.0) * cubeSize / 2.0 - epsilonTrans);
		int expectedNumberOfContacts = 1;
		int expectedBoxIndicesInContacts[] = {7};
		doTriangleMeshPlaneTest(cubeMesh, meshQuat, meshTrans,
								plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
		SCOPED_TRACE("intersection, 01 contacts, plane rotate(+X-Z, +45)");
		meshQuat = Quaterniond::Identity();
		meshTrans = Vector3d::Zero();
		mRotation = Eigen::AngleAxisd(M_PI_4, Vector3d(1.0, 0.0, -1.0).normalized());
		planeQuat = Quaterniond(mRotation);
		planNormal = mRotation * Vector3d::UnitY();
		planeTrans = -1.0 * planNormal * (sqrt(3.0) * cubeSize / 2.0 - epsilonTrans);
		int expectedNumberOfContacts = 1;
		int expectedBoxIndicesInContacts[] = {0};
		doTriangleMeshPlaneTest(cubeMesh, meshQuat, meshTrans,
								plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
		SCOPED_TRACE("intersection, 01 contacts, plane rotate(+X-Z, M_PI/2+45)");
		meshQuat = Quaterniond::Identity();
		meshTrans = Vector3d::Zero();
		mRotation = Eigen::AngleAxisd(0.75 * M_PI, Vector3d(1.0, 0.0, -1.0).normalized());
		planeQuat = Quaterniond(mRotation);
		planNormal = mRotation * Vector3d::UnitY();
		planeTrans = -1.0 * planNormal * (sqrt(3.0) * cubeSize / 2.0 - epsilonTrans);
		int expectedNumberOfContacts = 1;
		int expectedBoxIndicesInContacts[] = {3};
		doTriangleMeshPlaneTest(cubeMesh, meshQuat, meshTrans,
								plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
		SCOPED_TRACE("intersection, 01 contacts, plane rotate(+X-Z, -45)");
		meshQuat = Quaterniond::Identity();
		meshTrans = Vector3d::Zero();
		mRotation = Eigen::AngleAxisd(-M_PI_4, Vector3d(1.0, 0.0, -1.0).normalized());
		planeQuat = Quaterniond(mRotation);
		planNormal = mRotation * Vector3d::UnitY();
		planeTrans = -1.0 * planNormal * (sqrt(3.0) * cubeSize / 2.0 - epsilonTrans);
		int expectedNumberOfContacts = 1;
		int expectedBoxIndicesInContacts[] = {5};
		doTriangleMeshPlaneTest(cubeMesh, meshQuat, meshTrans,
								plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
		SCOPED_TRACE("intersection, 01 contacts, plane rotate(+X-Z,-M_PI/2-45)");
		meshQuat = Quaterniond::Identity();
		meshTrans = Vector3d::Zero();
		mRotation = Eigen::AngleAxisd(-M_PI_2 - M_PI_4, Vector3d(1.0, 0.0, -1.0).normalized());
		planeQuat = Quaterniond(mRotation);
		planNormal = mRotation * Vector3d::UnitY();
		planeTrans = -1.0 * planNormal * (sqrt(3.0) * cubeSize / 2.0 - epsilonTrans);
		int expectedNumberOfContacts = 1;
		int expectedBoxIndicesInContacts[] = {6};
		doTriangleMeshPlaneTest(cubeMesh, meshQuat, meshTrans,
								plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
		SCOPED_TRACE("intersection, 01 contacts, plane rotate(+X+Z, +45)");
		meshQuat = Quaterniond::Identity();
		meshTrans = Vector3d::Zero();
		mRotation = Eigen::AngleAxisd(M_PI_4, Vector3d(1.0, 0.0, 1.0).normalized());
		planeQuat = Quaterniond(mRotation);
		planNormal = mRotation * Vector3d::UnitY();
		planeTrans = -1.0 * planNormal * (sqrt(3.0) * cubeSize / 2.0 - epsilonTrans);
		int expectedNumberOfContacts = 1;
		int expectedBoxIndicesInContacts[] = {1};
		doTriangleMeshPlaneTest(cubeMesh, meshQuat, meshTrans,
								plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
		SCOPED_TRACE("intersection, 01 contacts, plane rotate(+X+Z, M_PI/2+45)");
		meshQuat = Quaterniond::Identity();
		meshTrans = Vector3d::Zero();
		mRotation = Eigen::AngleAxisd(M_PI_2 + M_PI_4, Vector3d(1.0, 0.0, 1.0).normalized());
		planeQuat = Quaterniond(mRotation);
		planNormal = mRotation * Vector3d::UnitY();
		planeTrans = -1.0 * planNormal * (sqrt(3.0) * cubeSize / 2.0 - epsilonTrans);
		int expectedNumberOfContacts = 1;
		int expectedBoxIndicesInContacts[] = {2};
		doTriangleMeshPlaneTest(cubeMesh, meshQuat, meshTrans,
								plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
		SCOPED_TRACE("intersection, 01 contacts, plane & cube -0.8*M_PI*rotate(Z, -M_PI/4)");
		globalQuat = SurgSim::Math::makeRotationQuaternion<double, Vector3d::Options>(-1.318, Vector3d::UnitZ());
		meshQuat = globalQuat * Quaterniond::Identity();
		meshTrans = Vector3d::Zero();
		planeQuat = globalQuat * Quaterniond(Eigen::AngleAxisd(-M_PI_4, Vector3d::UnitZ()));
		planeTrans = Vector3d(-1.0, -1.0, 0.0) * (cubeSize / 2.0 - epsilonTrans);
		int expectedNumberOfContacts = 2;
		int expectedBoxIndicesInContacts[] = {0, 4};
		doTriangleMeshPlaneTest(cubeMesh, meshQuat, meshTrans,
								plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}
}


}

}; // Physics
}; // Surgsim
