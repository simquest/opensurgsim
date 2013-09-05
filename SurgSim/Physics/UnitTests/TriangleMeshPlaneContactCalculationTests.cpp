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
#include <SurgSim/Physics/UnitTests/RepresentationUtilities.h>
#include <SurgSim/Physics/UnitTests/MockCollisionRepresentation.h>
#include <memory>

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>

#include <SurgSim/Physics/RigidRepresentationState.h>
#include <SurgSim/Physics/RigidShape.h>
#include <SurgSim/Physics/SphereShape.h>
#include <SurgSim/Physics/MeshShape.h>
#include <SurgSim/Physics/CollisionRepresentation.h>
#include <SurgSim/Physics/ContactCalculation.h>
#include <SurgSim/Physics/CollisionPair.h>

#include <SurgSim/Math/Geometry.h>
#include <SurgSim/Physics/RigidShapeCollisionRepresentation.h>
#include <SurgSim/DataStructures/TriangleMesh.h>

#include <SurgSim/Physics/TriangleMeshPlaneDcdContact.h>

using SurgSim::Math::Vector3d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::DataStructures::TriangleMesh;

namespace
{
double epsilon = 1e-10;
}


namespace SurgSim
{
namespace Physics
{

namespace
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

class EmptyData
{
public:
	bool operator ==(const EmptyData& e) const
	{
		return true;
	}
};

::testing::AssertionResult eigenEqual(const Vector3d& left, const Vector3d& right, double epsilon)
{
	double dist = (left - right).norm();
	if (std::abs(dist) < epsilon)
	{
		return ::testing::AssertionSuccess();
	}
	else
	{
		return ::testing::AssertionFailure() << std::endl << "Vectors not close, expected: " << left.transpose() <<
			   std::endl << " result: " << right.transpose() << std::endl;
	}
}

::testing::AssertionResult isContactPresentInList(std::shared_ptr<Contact> expected,
                                                  const std::list<std::shared_ptr<Contact>>& contactsList)
{
    using SurgSim::Math::Geometry::ScalarEpsilon;

    bool contactPresent = false;
    for (auto it = contactsList.begin(); it != contactsList.end() && !contactPresent; ++it)
    {
        // Compare the normals.
        contactPresent = eigenEqual(expected->normal, it->get()->normal, ScalarEpsilon);
        // Compare the global position of first object.
        contactPresent &= eigenEqual(expected->penetrationPoints.first.globalPosition.getValue(),
                                     it->get()->penetrationPoints.first.globalPosition.getValue(), ScalarEpsilon);
        // Compare the global position of second object.
        contactPresent &= eigenEqual(expected->penetrationPoints.second.globalPosition.getValue(),
                                     it->get()->penetrationPoints.second.globalPosition.getValue(),
                                     ScalarEpsilon);
        // Compare the depth.
        contactPresent &= std::abs(expected->depth - it->get()->depth) <= ScalarEpsilon;
    }

    if (contactPresent)
    {
        return ::testing::AssertionSuccess();
    }
    else
    {
        return ::testing::AssertionFailure() << "Expected contact not found in calculated contacts list:\n" <<
               "Normal: " << expected->normal << "\n" <<
               "First objects' contact point: " << expected->penetrationPoints.first.globalPosition.getValue()
               << "\n" <<
               "Second objects' contact point: " << expected->penetrationPoints.second.globalPosition.getValue()
               << "\n" <<
               "Depth of penetration: " << expected->depth << "\n";
    }
}

void contactsInfoEqualityTest(const std::list<std::shared_ptr<Contact>>& expectedContacts,
                              const std::list<std::shared_ptr<Contact>>& calculatedContacts)
{
    SCOPED_TRACE("Comparing the contact info.");

    EXPECT_EQ(expectedContacts.size(), calculatedContacts.size());

    for (auto it = expectedContacts.begin(); it != expectedContacts.end(); ++it)
    {
        EXPECT_TRUE(isContactPresentInList(*it, calculatedContacts));
    }
}

Vector3d calculateMeshVertex(const int i,
							const Quaterniond& quat,
							const Vector3d& trans)
{
	
	return (quat * Vector3d(cubePoints[i][0], cubePoints[i][1], cubePoints[i][2])) +
		trans;
}

void generateTriangleMeshPlaneContact(std::list<std::shared_ptr<Contact>>& expectedContacts,
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
		vertex = calculateMeshVertex(expectedMeshIndicesInContacts[i], meshQuat, meshTrans);
		std::pair<Location, Location> penetrationPoint;
		penetrationPoint.first.globalPosition.setValue(vertex);
		depth = planeNormalGlobal.dot(vertex - pointOnPlane);
		penetrationPoint.second.globalPosition.setValue(vertex - planeNormalGlobal * depth);
		expectedContacts.push_back(std::make_shared<Contact>(depth, Vector3d::Zero(),
			collisionNormal, penetrationPoint));
	}
}

template <class VertexType, class EdgeType, class TriangleType>
void doTriangleMeshPlaneTest(std::shared_ptr<MeshShape<VertexType, EdgeType, TriangleType>> mesh,
					const Quaterniond& meshQuat,
					const Vector3d& meshTrans,
					std::shared_ptr<PlaneShape> plane,
					const Quaterniond& planeQuat,
					const Vector3d& planeTrans,
					const int expectedNumberOfContacts,
					const int* expectedMeshIndicesInContacts)
{
	std::shared_ptr<CollisionRepresentation> meshRep = std::make_shared<MockCollisionRepresentation>(
		"Collision Mesh 0",
		mesh,
		meshQuat,
		meshTrans);
	std::shared_ptr<CollisionRepresentation> planeRep = std::make_shared<MockCollisionRepresentation>(
		"Collision Plane 0",
		plane,
		planeQuat,
		planeTrans);

	// First calculate the expected contact info.
	std::list<std::shared_ptr<Contact>> expectedContacts;
	if (expectedNumberOfContacts > 0)
	{
		unsigned int totalMeshVertices = mesh->getMesh()->getNumVertices();
		const std::vector<SurgSim::DataStructures::Vertex<VertexType>> Vertices = mesh->getMesh()->getVertices();

		generateTriangleMeshPlaneContact(expectedContacts, expectedNumberOfContacts, expectedMeshIndicesInContacts,
			meshTrans, meshQuat, plane->getNormal(), plane->getD(), planeTrans,
			planeQuat);
	}

	// Perform collision detection.
	TriangleMeshPlaneDcdContact<VertexType, EdgeType, TriangleType> calcContact;
	std::shared_ptr<CollisionPair> pair = std::make_shared<CollisionPair>(meshRep, planeRep);
	calcContact.calculateContact(pair);

	// Compare the contact info.
	contactsInfoEqualityTest(expectedContacts, pair->getContacts());
}


TEST(ContactCalculationTests, TriangleMeshPlaneCalculation)
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

	std::shared_ptr<SurgSim::Physics::MeshShape<EmptyData, EmptyData, EmptyData>> cubeMesh =  
		std::make_shared<SurgSim::Physics::MeshShape<EmptyData, EmptyData, EmptyData>>(mesh);

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
		planeTrans = Vector3d(0.0,-(cubeSize/2+epsilonTrans),0.0);
        int expectedNumberOfContacts = 0;
        int expectedBoxIndicesInContacts[] = {0};
		doTriangleMeshPlaneTest<EmptyData, EmptyData, EmptyData> (cubeMesh, meshQuat, meshTrans, 
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
		doTriangleMeshPlaneTest<EmptyData, EmptyData, EmptyData> (cubeMesh, meshQuat, meshTrans, 
			plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
        SCOPED_TRACE("intersection, 08 contacts, no rotation");
        meshQuat = Quaterniond::Identity();
        meshTrans = Vector3d(0.0,0.0,0.0);
		planeQuat = Quaterniond::Identity(); 
		planeTrans = Vector3d(0.0,(cubeSize/2+epsilonTrans),0.0);
        int expectedNumberOfContacts = 8;
        int expectedBoxIndicesInContacts[] = {0, 1, 2, 3, 4, 5, 6, 7};
		doTriangleMeshPlaneTest<EmptyData, EmptyData, EmptyData> (cubeMesh, meshQuat, meshTrans, 
			plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
        SCOPED_TRACE("intersection, 02 contacts, plane rotate(Z, -45)");
        meshQuat = Quaterniond::Identity();
        meshTrans = Vector3d(0.0,0.0,0.0);
		mRotation = Eigen::AngleAxisd(-0.25*M_PI, SurgSim::Math::Vector3d(0, 0, 1).normalized());
		planeQuat = SurgSim::Math::Quaterniond(mRotation); 
		planeTrans = Vector3d(-1,-1,0.0)*(cubeSize/2-epsilonTrans);
        int expectedNumberOfContacts = 2;
        int expectedBoxIndicesInContacts[] = {0, 4};
		doTriangleMeshPlaneTest<EmptyData, EmptyData, EmptyData> (cubeMesh, meshQuat, meshTrans, 
			plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
        SCOPED_TRACE("intersection, 06 contacts, plane rotate(Z, 45)");
        meshQuat = Quaterniond::Identity();
        meshTrans = Vector3d(0.0,0.0,0.0);
		mRotation = Eigen::AngleAxisd(-0.25*M_PI, SurgSim::Math::Vector3d(0, 0, 1).normalized());
		planeQuat = SurgSim::Math::Quaterniond(mRotation); 
		planNormal = mRotation*SurgSim::Math::Vector3d(0, 1, 0);
		planeTrans = planNormal*(cubeSize/2-epsilonTrans);
        int expectedNumberOfContacts = 6;
        int expectedBoxIndicesInContacts[] = {0, 4, 1, 3, 7, 5};
		doTriangleMeshPlaneTest<EmptyData, EmptyData, EmptyData> (cubeMesh, meshQuat, meshTrans, 
			plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	// Test cases to cover all collisions between each corner of the cube with the plane 
	{
		SCOPED_TRACE("intersection, 01 contact, plane rotate(XZ, -45)");
        meshQuat = Quaterniond::Identity();
        meshTrans = Vector3d(0.0,0.0,0.0);
		mRotation = Eigen::AngleAxisd(-0.25*M_PI, SurgSim::Math::Vector3d(1, 0, 1).normalized());
		planeQuat = SurgSim::Math::Quaterniond(mRotation); 
		planNormal = SurgSim::Math::Vector3d(0, 1, 0);
		planeTrans = -1*planNormal*(sqrt(3)*cubeSize/2 - epsilonTrans);
        int expectedNumberOfContacts = 1;
        int expectedBoxIndicesInContacts[] = {4};
		doTriangleMeshPlaneTest<EmptyData, EmptyData, EmptyData> (cubeMesh, meshQuat, meshTrans, 
			plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
        SCOPED_TRACE("intersection, 01 contacts, plane rotate(+XZ,-M_PI/2-45)");
        meshQuat = Quaterniond::Identity();
        meshTrans = Vector3d(0.0,0.0,0.0);
		mRotation = Eigen::AngleAxisd(-M_PI/2-M_PI/4, SurgSim::Math::Vector3d(1, 0, 1).normalized());
		planeQuat = SurgSim::Math::Quaterniond(mRotation); 
		planNormal = mRotation*SurgSim::Math::Vector3d(0, 1, 0);
		planeTrans = -1*planNormal*(sqrt(3)*cubeSize/2 - epsilonTrans);
        int expectedNumberOfContacts = 1;
        int expectedBoxIndicesInContacts[] = {7};
		doTriangleMeshPlaneTest<EmptyData, EmptyData, EmptyData> (cubeMesh, meshQuat, meshTrans, 
			plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
        SCOPED_TRACE("intersection, 01 contacts, plane rotate(+X-Z, +45)");
        meshQuat = Quaterniond::Identity();
        meshTrans = Vector3d(0.0,0.0,0.0);
		mRotation = Eigen::AngleAxisd(0.25*M_PI, SurgSim::Math::Vector3d(1, 0, -1).normalized());
		planeQuat = SurgSim::Math::Quaterniond(mRotation); 
		planNormal = mRotation*SurgSim::Math::Vector3d(0, 1, 0);
		planeTrans = -1*planNormal*(sqrt(3)*cubeSize/2 - epsilonTrans);
        int expectedNumberOfContacts = 1;
        int expectedBoxIndicesInContacts[] = {0};
		doTriangleMeshPlaneTest<EmptyData, EmptyData, EmptyData> (cubeMesh, meshQuat, meshTrans, 
			plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
        SCOPED_TRACE("intersection, 01 contacts, plane rotate(+X-Z, M_PI/2+45)");
        meshQuat = Quaterniond::Identity();
        meshTrans = Vector3d(0.0,0.0,0.0);
		mRotation = Eigen::AngleAxisd(M_PI/2+M_PI/4, SurgSim::Math::Vector3d(1, 0, -1).normalized());
		planeQuat = SurgSim::Math::Quaterniond(mRotation); 
		planNormal = mRotation*SurgSim::Math::Vector3d(0, 1, 0);
		planeTrans = -1*planNormal*(sqrt(3)*cubeSize/2 - epsilonTrans);
        int expectedNumberOfContacts = 1;
        int expectedBoxIndicesInContacts[] = {3};
		doTriangleMeshPlaneTest<EmptyData, EmptyData, EmptyData> (cubeMesh, meshQuat, meshTrans, 
			plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
        SCOPED_TRACE("intersection, 01 contacts, plane rotate(+X-Z, -45)");
        meshQuat = Quaterniond::Identity();
        meshTrans = Vector3d(0.0,0.0,0.0);
		mRotation = Eigen::AngleAxisd(-0.25*M_PI, SurgSim::Math::Vector3d(1, 0, -1).normalized());
		planeQuat = SurgSim::Math::Quaterniond(mRotation); 
		planNormal = mRotation*SurgSim::Math::Vector3d(0, 1, 0);
		planeTrans = -1*planNormal*(sqrt(3)*cubeSize/2 - epsilonTrans);
        int expectedNumberOfContacts = 1;
        int expectedBoxIndicesInContacts[] = {5};
		doTriangleMeshPlaneTest<EmptyData, EmptyData, EmptyData> (cubeMesh, meshQuat, meshTrans, 
			plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
        SCOPED_TRACE("intersection, 01 contacts, plane rotate(+X-Z,-M_PI/2-45)");
        meshQuat = Quaterniond::Identity();
        meshTrans = Vector3d(0.0,0.0,0.0);
		mRotation = Eigen::AngleAxisd(-M_PI/2-M_PI/4, SurgSim::Math::Vector3d(1, 0, -1).normalized());
		planeQuat = SurgSim::Math::Quaterniond(mRotation); 
		planNormal = mRotation*SurgSim::Math::Vector3d(0, 1, 0);
		planeTrans = -1*planNormal*(sqrt(3)*cubeSize/2 - epsilonTrans);
        int expectedNumberOfContacts = 1;
        int expectedBoxIndicesInContacts[] = {6};
		doTriangleMeshPlaneTest<EmptyData, EmptyData, EmptyData> (cubeMesh, meshQuat, meshTrans, 
			plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
        SCOPED_TRACE("intersection, 01 contacts, plane rotate(+X+Z, +45)");
        meshQuat = Quaterniond::Identity();
        meshTrans = Vector3d(0.0,0.0,0.0);
		mRotation = Eigen::AngleAxisd(0.25*M_PI, SurgSim::Math::Vector3d(1, 0, 1).normalized());
		planeQuat = SurgSim::Math::Quaterniond(mRotation); 
		planNormal = mRotation*SurgSim::Math::Vector3d(0, 1, 0);
		planeTrans = -1*planNormal*(sqrt(3)*cubeSize/2 - epsilonTrans);
        int expectedNumberOfContacts = 1;
        int expectedBoxIndicesInContacts[] = {1};
		doTriangleMeshPlaneTest<EmptyData, EmptyData, EmptyData> (cubeMesh, meshQuat, meshTrans, 
			plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
        SCOPED_TRACE("intersection, 01 contacts, plane rotate(+X+Z, M_PI/2+45)");
        meshQuat = Quaterniond::Identity();
        meshTrans = Vector3d(0.0,0.0,0.0);
		mRotation = Eigen::AngleAxisd(M_PI/2 +M_PI/4, SurgSim::Math::Vector3d(1, 0, 1).normalized());
		planeQuat = SurgSim::Math::Quaterniond(mRotation); 
		planNormal = mRotation*SurgSim::Math::Vector3d(0, 1, 0);
		planeTrans = -1*planNormal*(sqrt(3)*cubeSize/2 - epsilonTrans);
        int expectedNumberOfContacts = 1;
        int expectedBoxIndicesInContacts[] = {2};
		doTriangleMeshPlaneTest<EmptyData, EmptyData, EmptyData> (cubeMesh, meshQuat, meshTrans, 
			plane, planeQuat, planeTrans, expectedNumberOfContacts, expectedBoxIndicesInContacts);
	}

	{
        SCOPED_TRACE("intersection, 01 contacts, plane & cube -0.8*M_PI*rotate(Z, -M_PI/4)");
		globalQuat = SurgSim::Math::makeRotationQuaternion(-1.318, Vector3d(0,0,1).normalized());
        meshQuat = globalQuat*Quaterniond::Identity();
        meshTrans = Vector3d(0.0,0.0,0.0);
		double angle = -0.25*M_PI;
		SurgSim::Math::Vector3d axis(0, 0, 1);
		planeQuat = globalQuat*SurgSim::Math::Quaterniond(Eigen::AngleAxisd(angle, axis.normalized())); 
		planeTrans = Vector3d(-1,-1,0.0)*(cubeSize/2-epsilonTrans);
        int expectedNumberOfContacts = 2;
        int expectedBoxIndicesInContacts[] = {0, 4};
		doTriangleMeshPlaneTest<EmptyData, EmptyData, EmptyData> (cubeMesh, meshQuat, meshTrans, plane, planeQuat, planeTrans, expectedNumberOfContacts,
					   expectedBoxIndicesInContacts);
	}

}


}

}; // Physics
}; // Surgsim
