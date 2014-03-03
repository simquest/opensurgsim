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
#include "SurgSim/Collision/TriangleMeshPlaneDcdContact.h"
#include "SurgSim/DataStructures/EmptyData.h"

using SurgSim::DataStructures::EmptyData;

using SurgSim::DataStructures::TriangleMesh;

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
static const SurgSim::Math::Vector3d cubePoints[8]=
{
	SurgSim::Math::Vector3d(-1.0/2.0, -1.0/2.0, -1.0/2.0),
	SurgSim::Math::Vector3d( 1.0/2.0, -1.0/2.0, -1.0/2.0),
	SurgSim::Math::Vector3d( 1.0/2.0,  1.0/2.0, -1.0/2.0),
	SurgSim::Math::Vector3d(-1.0/2.0,  1.0/2.0, -1.0/2.0),

	SurgSim::Math::Vector3d(-1.0/2.0, -1.0/2.0,  1.0/2.0),
	SurgSim::Math::Vector3d( 1.0/2.0, -1.0/2.0,  1.0/2.0),
	SurgSim::Math::Vector3d( 1.0/2.0,  1.0/2.0,  1.0/2.0),
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

Vector3d calculateTriangleMeshVertex(const int i,
							const Quaterniond& quat,
							const Vector3d& trans)
{

	return (quat * Vector3d(cubePoints[i][0], cubePoints[i][1], cubePoints[i][2])) +
		trans;
}

void generateTriangleMeshPlaneContact(std::list<std::shared_ptr<Contact>>* expectedContacts,
	const int expectedNumberOfContacts, const int* expectedMeshIndicesInContacts,
	const Vector3d& meshTrans, const Quaterniond& meshQuat,
	const Vector3d& planeNormal, const double planeD,
	const Vector3d& planeTrans, const Quaterniond& planeQuat)
{
	Vector3d vertex;
	Vector3d planeNormalGlobal = planeQuat * planeNormal;
	Vector3d pointOnPlane = planeTrans + (planeNormalGlobal * planeD);
	double depth = 0.0;
	Vector3d collisionNormal = planeNormalGlobal;

	for (int i = 0; i < expectedNumberOfContacts; ++i)
	{
		vertex = calculateTriangleMeshVertex(expectedMeshIndicesInContacts[i], meshQuat, meshTrans);
		std::pair<Location, Location> penetrationPoint;
		penetrationPoint.first.globalPosition.setValue(vertex);
		depth = planeNormalGlobal.dot(vertex - pointOnPlane);
		penetrationPoint.second.globalPosition.setValue(vertex - planeNormalGlobal * depth);
		expectedContacts->push_back(std::make_shared<Contact>(depth, Vector3d::Zero(),
			collisionNormal, penetrationPoint));
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
	std::shared_ptr<Representation> meshRep = std::make_shared<ShapeCollisionRepresentation>(
		"Collision Mesh 0", mesh, SurgSim::Math::makeRigidTransform(meshQuat, meshTrans));
	std::shared_ptr<Representation> planeRep = std::make_shared<ShapeCollisionRepresentation>(
		"Collision Plane 0", plane, SurgSim::Math::makeRigidTransform(planeQuat, planeTrans));

	// First calculate the expected contact info.
	std::list<std::shared_ptr<Contact>> expectedContacts;
	if (expectedNumberOfContacts > 0)
	{
		generateTriangleMeshPlaneContact(&expectedContacts, expectedNumberOfContacts, expectedMeshIndicesInContacts,
			meshTrans, meshQuat, plane->getNormal(), plane->getD(), planeTrans,
			planeQuat);
	}

	// Perform collision detection.
	TriangleMeshPlaneDcdContact calcContact;
	std::shared_ptr<CollisionPair> pair = std::make_shared<CollisionPair>(meshRep, planeRep);
	calcContact.calculateContact(pair);

	// Compare the contact info.
	contactsInfoEqualityTest(expectedContacts, pair->getContacts());
}


TEST(TriangleMeshPlaneContactCalculationTests, UnitTests)
{
	typedef SurgSim::DataStructures::TriangleMesh<EmptyData,EmptyData,EmptyData> TriangleMesh;
	typedef SurgSim::DataStructures::MeshElement<2,EmptyData> EdgeElement;
	typedef SurgSim::DataStructures::MeshElement<3,EmptyData> TriangleElement;

	// Create a Mesh Cube
	std::shared_ptr<TriangleMesh> mesh = std::make_shared<TriangleMesh>();
	for (int i = 0; i < cubeNumPoints; i++)
	{
		EmptyData emptyData;
		SurgSim::Math::Vector3d p;
		p[0] = cubePoints[i][0];
		p[1] = cubePoints[i][1];
		p[2] = cubePoints[i][2];
		TriangleMesh::VertexType v(p, emptyData);
		mesh->addVertex(v);
	}
	for (int i = 0; i < cubeNumEdges; i++)
	{
		EmptyData emptyData;
		std::array<unsigned int,2> edgePoints;
		for (int j = 0; j < 2; j++)
		{
			edgePoints[j] = cubeEdges[i][j];
		}
		EdgeElement edgeElement(edgePoints, emptyData);
		TriangleMesh::EdgeType e(edgeElement);
		mesh->addEdge(e);
	}
	for (int i = 0; i < cubeNumTriangles; i++)
	{
		EmptyData emptyData;
		std::array<unsigned int,3> trianglePoints;
		for (int j = 0; j < 3; j++)
		{
			trianglePoints[j] = cubeTrianglesCCW[i][j];
		}
		TriangleElement triangleElement(trianglePoints, emptyData);
		TriangleMesh::TriangleType t(triangleElement);
		mesh->addTriangle(t);
	}

	std::shared_ptr<SurgSim::Math::MeshShape> cubeMesh =
		std::make_shared<SurgSim::Math::MeshShape>(mesh);

	std::shared_ptr<PlaneShape> plane = std::make_shared<PlaneShape>();
	SurgSim::Math::Quaterniond meshQuat;
	SurgSim::Math::Vector3d meshTrans;
	SurgSim::Math::Quaterniond planeQuat;
	SurgSim::Math::Vector3d planeTrans;
	SurgSim::Math::Quaterniond globalQuat;
	SurgSim::Math::Vector3d planNormal;
	SurgSim::Math::Matrix33d mRotation;

	const double epsilonTrans = 0.1;
	const double cubeSize = 1;

	{
		SCOPED_TRACE("No intersection, box in front of plane, no rotation");
		meshQuat = Quaterniond::Identity();
		meshTrans = Vector3d(0.0,0.0,0.0);
		planeQuat = Quaterniond::Identity();
		planeTrans = Vector3d(0.0, -(cubeSize / 2 + epsilonTrans), 0.0);
		int expectedNumberOfContacts = 0;
		int expectedBoxIndicesInContacts[] = {0};
		doTriangleMeshPlaneTest(cubeMesh, meshQuat, meshTrans,
				plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
		SCOPED_TRACE("Intersection, 04 contacts, no rotation");
		meshQuat = Quaterniond::Identity();
		meshTrans = Vector3d(0.0,0.0,0.0);
		planeQuat = Quaterniond::Identity();
		planeTrans = Vector3d(0.0,0.0,0.0);
		int expectedNumberOfContacts = 4;
		int expectedBoxIndicesInContacts[] = {0, 1, 4, 5};
		doTriangleMeshPlaneTest(cubeMesh, meshQuat, meshTrans,
				plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
		SCOPED_TRACE("intersection, 08 contacts, no rotation");
		meshQuat = Quaterniond::Identity();
		meshTrans = Vector3d(0.0,0.0,0.0);
		planeQuat = Quaterniond::Identity();
		planeTrans = Vector3d(0.0,(cubeSize / 2 + epsilonTrans),0.0);
		int expectedNumberOfContacts = 8;
		int expectedBoxIndicesInContacts[] = {0, 1, 2, 3, 4, 5, 6, 7};
		doTriangleMeshPlaneTest(cubeMesh, meshQuat, meshTrans,
				plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
		SCOPED_TRACE("intersection, 02 contacts, plane rotate(Z, -45)");
		meshQuat = Quaterniond::Identity();
		meshTrans = Vector3d(0.0,0.0,0.0);
		mRotation = Eigen::AngleAxisd(-0.25 * M_PI, SurgSim::Math::Vector3d(0, 0, 1).normalized());
		planeQuat = SurgSim::Math::Quaterniond(mRotation);
		planeTrans = Vector3d(-1,-1,0.0) * (cubeSize / 2 - epsilonTrans);
		int expectedNumberOfContacts = 2;
		int expectedBoxIndicesInContacts[] = {0, 4};
		doTriangleMeshPlaneTest(cubeMesh, meshQuat, meshTrans,
				plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
		SCOPED_TRACE("intersection, 06 contacts, plane rotate(Z, 45)");
		meshQuat = Quaterniond::Identity();
		meshTrans = Vector3d(0.0,0.0,0.0);
		mRotation = Eigen::AngleAxisd(-0.25 * M_PI, SurgSim::Math::Vector3d(0, 0, 1).normalized());
		planeQuat = SurgSim::Math::Quaterniond(mRotation);
		planNormal = mRotation * SurgSim::Math::Vector3d(0, 1, 0);
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
		meshTrans = Vector3d(0.0,0.0,0.0);
		mRotation = Eigen::AngleAxisd(-0.25 * M_PI, SurgSim::Math::Vector3d(1, 0, 1).normalized());
		planeQuat = SurgSim::Math::Quaterniond(mRotation);
		planNormal = SurgSim::Math::Vector3d(0, 1, 0);
		planeTrans = -1 * planNormal * (sqrt(3.0) * cubeSize / 2 - epsilonTrans);
		int expectedNumberOfContacts = 1;
		int expectedBoxIndicesInContacts[] = {4};
		doTriangleMeshPlaneTest(cubeMesh, meshQuat, meshTrans,
				plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
		SCOPED_TRACE("intersection, 01 contacts, plane rotate(+XZ,-M_PI/2-45)");
		meshQuat = Quaterniond::Identity();
		meshTrans = Vector3d(0.0,0.0,0.0);
		mRotation = Eigen::AngleAxisd(-M_PI / 2 - M_PI / 4, SurgSim::Math::Vector3d(1, 0, 1).normalized());
		planeQuat = SurgSim::Math::Quaterniond(mRotation);
		planNormal = mRotation * SurgSim::Math::Vector3d(0, 1, 0);
		planeTrans = -1 * planNormal * (sqrt(3.0) * cubeSize / 2 - epsilonTrans);
		int expectedNumberOfContacts = 1;
		int expectedBoxIndicesInContacts[] = {7};
		doTriangleMeshPlaneTest(cubeMesh, meshQuat, meshTrans,
				plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
		SCOPED_TRACE("intersection, 01 contacts, plane rotate(+X-Z, +45)");
		meshQuat = Quaterniond::Identity();
		meshTrans = Vector3d(0.0,0.0,0.0);
		mRotation = Eigen::AngleAxisd(0.25 * M_PI, SurgSim::Math::Vector3d(1, 0, -1).normalized());
		planeQuat = SurgSim::Math::Quaterniond(mRotation);
		planNormal = mRotation * SurgSim::Math::Vector3d(0, 1, 0);
		planeTrans = -1 * planNormal * (sqrt(3.0) * cubeSize / 2 - epsilonTrans);
		int expectedNumberOfContacts = 1;
		int expectedBoxIndicesInContacts[] = {0};
		doTriangleMeshPlaneTest(cubeMesh, meshQuat, meshTrans,
				plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
		SCOPED_TRACE("intersection, 01 contacts, plane rotate(+X-Z, M_PI/2+45)");
		meshQuat = Quaterniond::Identity();
		meshTrans = Vector3d(0.0,0.0,0.0);
		mRotation = Eigen::AngleAxisd(M_PI / 2 + M_PI / 4, SurgSim::Math::Vector3d(1, 0, -1).normalized());
		planeQuat = SurgSim::Math::Quaterniond(mRotation);
		planNormal = mRotation * SurgSim::Math::Vector3d(0, 1, 0);
		planeTrans = -1 * planNormal * (sqrt(3.0) * cubeSize / 2 - epsilonTrans);
		int expectedNumberOfContacts = 1;
		int expectedBoxIndicesInContacts[] = {3};
		doTriangleMeshPlaneTest(cubeMesh, meshQuat, meshTrans,
				plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
		SCOPED_TRACE("intersection, 01 contacts, plane rotate(+X-Z, -45)");
		meshQuat = Quaterniond::Identity();
		meshTrans = Vector3d(0.0,0.0,0.0);
		mRotation = Eigen::AngleAxisd(-0.25 * M_PI, SurgSim::Math::Vector3d(1, 0, -1).normalized());
		planeQuat = SurgSim::Math::Quaterniond(mRotation);
		planNormal = mRotation * SurgSim::Math::Vector3d(0, 1, 0);
		planeTrans = -1 * planNormal * (sqrt(3.0) * cubeSize / 2 - epsilonTrans);
		int expectedNumberOfContacts = 1;
		int expectedBoxIndicesInContacts[] = {5};
		doTriangleMeshPlaneTest(cubeMesh, meshQuat, meshTrans,
				plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
		SCOPED_TRACE("intersection, 01 contacts, plane rotate(+X-Z,-M_PI/2-45)");
		meshQuat = Quaterniond::Identity();
		meshTrans = Vector3d(0.0,0.0,0.0);
		mRotation = Eigen::AngleAxisd(-M_PI / 2 - M_PI / 4, SurgSim::Math::Vector3d(1, 0, -1).normalized());
		planeQuat = SurgSim::Math::Quaterniond(mRotation);
		planNormal = mRotation * SurgSim::Math::Vector3d(0, 1, 0);
		planeTrans = -1 * planNormal * (sqrt(3.0) * cubeSize / 2 - epsilonTrans);
		int expectedNumberOfContacts = 1;
		int expectedBoxIndicesInContacts[] = {6};
		doTriangleMeshPlaneTest(cubeMesh, meshQuat, meshTrans,
				plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
		SCOPED_TRACE("intersection, 01 contacts, plane rotate(+X+Z, +45)");
		meshQuat = Quaterniond::Identity();
		meshTrans = Vector3d(0.0,0.0,0.0);
		mRotation = Eigen::AngleAxisd(0.25 * M_PI, SurgSim::Math::Vector3d(1, 0, 1).normalized());
		planeQuat = SurgSim::Math::Quaterniond(mRotation);
		planNormal = mRotation * SurgSim::Math::Vector3d(0, 1, 0);
		planeTrans = -1 * planNormal * (sqrt(3.0) * cubeSize / 2 - epsilonTrans);
		int expectedNumberOfContacts = 1;
		int expectedBoxIndicesInContacts[] = {1};
		doTriangleMeshPlaneTest(cubeMesh, meshQuat, meshTrans,
				plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
		SCOPED_TRACE("intersection, 01 contacts, plane rotate(+X+Z, M_PI/2+45)");
		meshQuat = Quaterniond::Identity();
		meshTrans = Vector3d(0.0,0.0,0.0);
		mRotation = Eigen::AngleAxisd(M_PI / 2 + M_PI / 4, SurgSim::Math::Vector3d(1, 0, 1).normalized());
		planeQuat = SurgSim::Math::Quaterniond(mRotation);
		planNormal = mRotation * SurgSim::Math::Vector3d(0, 1, 0);
		planeTrans = -1 * planNormal * (sqrt(3.0) * cubeSize / 2 - epsilonTrans);
		int expectedNumberOfContacts = 1;
		int expectedBoxIndicesInContacts[] = {2};
		doTriangleMeshPlaneTest(cubeMesh, meshQuat, meshTrans,
				plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
		SCOPED_TRACE("intersection, 01 contacts, plane & cube -0.8*M_PI*rotate(Z, -M_PI/4)");
		globalQuat = SurgSim::Math::makeRotationQuaternion(-1.318, Vector3d(0,0,1).normalized());
		meshQuat = globalQuat * Quaterniond::Identity();
		meshTrans = Vector3d(0.0,0.0,0.0);
		double angle = -0.25 * M_PI;
		SurgSim::Math::Vector3d axis(0, 0, 1);
		planeQuat = globalQuat*SurgSim::Math::Quaterniond(Eigen::AngleAxisd(angle, axis.normalized()));
		planeTrans = Vector3d(-1,-1,0.0) * (cubeSize / 2 - epsilonTrans);
		int expectedNumberOfContacts = 2;
		int expectedBoxIndicesInContacts[] = {0, 4};
		doTriangleMeshPlaneTest(cubeMesh, meshQuat, meshTrans,
				plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}
}


}

}; // Physics
}; // Surgsim
