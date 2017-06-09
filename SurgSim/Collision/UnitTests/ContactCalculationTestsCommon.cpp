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

#include "SurgSim/Collision/UnitTests/ContactCalculationTestsCommon.h"

using SurgSim::DataStructures::IndexedLocalCoordinate;


namespace SurgSim
{
namespace Collision
{

::testing::AssertionResult eigenEqual(const Vector3d& left, const Vector3d& right)
{
	if (std::abs((left - right).norm()) < SurgSim::Math::Geometry::DistanceEpsilon)
	{
		return ::testing::AssertionSuccess();
	}
	else
	{
		return ::testing::AssertionFailure() << std::endl << "Vectors not close, expected: " << left.transpose() <<
			   std::endl << " result: " << right.transpose() << std::endl;
	}
}

void checkContactInfo(std::shared_ptr<Contact> contact, CollisionDetectionType expectedType,
					  double expectedDepth, double expectedTime,
					  const Vector3d& expectedNormal, const Vector3d& expectedPenetrationPointFirst,
					  const Vector3d& expectedPenetrationPointSecond)
{
	EXPECT_EQ(expectedType, contact->type);
	EXPECT_NEAR(expectedDepth, contact->depth, SurgSim::Math::Geometry::DistanceEpsilon);
	EXPECT_NEAR(expectedTime, contact->time, SurgSim::Math::Geometry::DistanceEpsilon);
	EXPECT_TRUE(eigenEqual(expectedNormal, contact->normal));
	EXPECT_TRUE(contact->penetrationPoints.first.rigidLocalPosition.hasValue());
	EXPECT_TRUE(contact->penetrationPoints.second.rigidLocalPosition.hasValue());
	EXPECT_TRUE(eigenEqual(expectedPenetrationPointFirst,
						   contact->penetrationPoints.first.rigidLocalPosition.getValue()));
	EXPECT_TRUE(eigenEqual(expectedPenetrationPointSecond,
						   contact->penetrationPoints.second.rigidLocalPosition.getValue()));
}

bool checkMeshLocalCoordinate(
	const SurgSim::DataStructures::OptionalValue<IndexedLocalCoordinate>& actualLocalCoordinate,
	const std::array<SurgSim::Math::Vector3d, 3>& vertices,
	const SurgSim::DataStructures::OptionalValue<IndexedLocalCoordinate>& expectedLocalCoordinate,
	const SurgSim::Math::Vector3d& expectedLocalPosition)
{
	bool isEqual = true;
	EXPECT_EQ(expectedLocalCoordinate.hasValue(), actualLocalCoordinate.hasValue());
	if (expectedLocalCoordinate.hasValue() && actualLocalCoordinate.hasValue())
	{
		isEqual &=
			expectedLocalCoordinate.getValue().index == actualLocalCoordinate.getValue().index;
		Vector3d barycentricCoordinates = actualLocalCoordinate.getValue().coordinate;
		isEqual &= eigenEqual(expectedLocalPosition,
							  barycentricCoordinates[0] * vertices[0] +
							  barycentricCoordinates[1] * vertices[1] +
							  barycentricCoordinates[2] * vertices[2]);
	}
	return isEqual;
}

::testing::AssertionResult isContactPresentInList(std::shared_ptr<Contact> expected,
		const std::list<std::shared_ptr<Contact>>& contactsList,
		bool expectedHasTriangleContactObject)
{
	using SurgSim::Math::Geometry::ScalarEpsilon;

	bool contactPresent = false;
	for (auto it = contactsList.begin(); it != contactsList.end() && !contactPresent; ++it)
	{
		// Compare the normals.
		contactPresent = eigenEqual(expected->normal, it->get()->normal);
		// Compare the global position of first object.
		contactPresent &= eigenEqual(expected->penetrationPoints.first.rigidLocalPosition.getValue(),
									 it->get()->penetrationPoints.first.rigidLocalPosition.getValue());
		// Compare the global position of second object.
		contactPresent &= eigenEqual(expected->penetrationPoints.second.rigidLocalPosition.getValue(),
									 it->get()->penetrationPoints.second.rigidLocalPosition.getValue());
		// Compare the depth.
		contactPresent &= std::abs(expected->depth - it->get()->depth) <= ScalarEpsilon;
		// Compare the time.
		contactPresent &= std::abs(expected->time - it->get()->time) <= ScalarEpsilon;
		// Compare the contact types.
		contactPresent &= (expected->type == it->get()->type);
		// Check if the optional 'meshLocalCoordinate' are the same.
		std::shared_ptr<SurgSim::Collision::TriangleContact> triangleContact;
		std::shared_ptr<SurgSim::Collision::Contact> contact;
		if (expectedHasTriangleContactObject)
		{
			triangleContact = std::static_pointer_cast<SurgSim::Collision::TriangleContact>(expected);
			contact = *it;
		}
		else
		{
			triangleContact = std::static_pointer_cast<SurgSim::Collision::TriangleContact>(*it);
			contact = expected;
		}
		contactPresent &= checkMeshLocalCoordinate(
							  contact->penetrationPoints.first.triangleMeshLocalCoordinate,
							  triangleContact->firstVertices,
							  triangleContact->penetrationPoints.first.triangleMeshLocalCoordinate,
							  expected->penetrationPoints.first.rigidLocalPosition.getValue());
		contactPresent &= checkMeshLocalCoordinate(
							  contact->penetrationPoints.second.triangleMeshLocalCoordinate,
							  triangleContact->secondVertices,
							  triangleContact->penetrationPoints.second.triangleMeshLocalCoordinate,
							  expected->penetrationPoints.second.rigidLocalPosition.getValue());
	}

	if (contactPresent)
	{
		return ::testing::AssertionSuccess();
	}
	else
	{
		std::stringstream calculatedText;
		size_t numCalculated = 0;
		for (const auto& contact : contactsList)
		{
			calculatedText << "Calculated Contact " << numCalculated << ":" << std::endl << *contact << std::endl;
			++numCalculated;
		}
		return ::testing::AssertionFailure() << "Expected contact not found in calculated contacts list:\nExpected:\n"
			<< *expected << "\nNumber of Calculated contacts: " << numCalculated << std::endl << calculatedText.str()
			<< std::endl;
	}
}

void contactsInfoEqualityTest(const std::list<std::shared_ptr<Contact>>& expectedContacts,
							  const std::list<std::shared_ptr<Contact>>& calculatedContacts,
							  bool expectedHasTriangleContactObject)
{
	SCOPED_TRACE("Comparing the contact info.");
	EXPECT_EQ(expectedContacts.size(), calculatedContacts.size());
	for (auto it = expectedContacts.begin(); it != expectedContacts.end(); ++it)
	{
		EXPECT_TRUE(isContactPresentInList(*it, calculatedContacts, expectedHasTriangleContactObject));
	}
}

void generateBoxPlaneContact(std::list<std::shared_ptr<Contact>>* expectedContacts,
							 const int expectedNumberOfContacts,
							 const int* expectedBoxIndicesInContacts,
							 const std::shared_ptr<BoxShape> box,
							 const Vector3d& boxTrans, const Quaterniond& boxQuat,
							 const std::shared_ptr<PlaneShape> plane,
							 const Vector3d& planeTrans, const Quaterniond& planeQuat)
{
	Vector3d vertex;
	Vector3d boxLocalVertex, planeLocalVertex;
	Vector3d planeNormalGlobal = planeQuat * plane->getNormal();
	Vector3d pointOnPlane = planeTrans + (planeNormalGlobal * plane->getD());
	double depth = 0.0;
	Vector3d collisionNormal = planeNormalGlobal;
	RigidTransform3d boxTransform = SurgSim::Math::makeRigidTransform(boxQuat, boxTrans);
	for (int i = 0; i < expectedNumberOfContacts; ++i)
	{
		boxLocalVertex = box->getVertex(expectedBoxIndicesInContacts[i]);
		vertex = boxTransform * boxLocalVertex;
		depth = -planeNormalGlobal.dot(vertex - pointOnPlane);
		planeLocalVertex = planeQuat.inverse() * (vertex + planeNormalGlobal * depth - planeTrans);
		std::pair<Location, Location> penetrationPoint;
		penetrationPoint.first.rigidLocalPosition.setValue(boxLocalVertex);
		penetrationPoint.second.rigidLocalPosition.setValue(planeLocalVertex);
		expectedContacts->push_back(std::make_shared<Contact>(
										COLLISION_DETECTION_TYPE_DISCRETE, depth, 1.0,
										Vector3d::Zero(), collisionNormal, penetrationPoint));
	}
}

void generateBoxDoubleSidedPlaneContact(std::list<std::shared_ptr<Contact>>* expectedContacts,
										const int expectedNumberOfContacts,
										const int* expectedBoxIndicesInContacts,
										const std::shared_ptr<BoxShape> box,
										const Vector3d& boxTrans, const Quaterniond& boxQuat,
										const std::shared_ptr<DoubleSidedPlaneShape> plane,
										const Vector3d& planeTrans, const Quaterniond& planeQuat,
										const bool collisionNormalIsPlaneNormal)
{
	Vector3d vertex;
	Vector3d boxLocalVertex, planeLocalVertex;
	Vector3d planeNormalGlobal = planeQuat * plane->getNormal();
	Vector3d pointOnPlane = planeTrans + (planeNormalGlobal * plane->getD());
	double depth = 0.0;
	Vector3d collisionNormal = planeNormalGlobal * (collisionNormalIsPlaneNormal ? 1.0 : -1.0);
	RigidTransform3d boxTransform = SurgSim::Math::makeRigidTransform(boxQuat, boxTrans);
	for (int i = 0; i < expectedNumberOfContacts; ++i)
	{
		boxLocalVertex = box->getVertex(expectedBoxIndicesInContacts[i]);
		vertex = boxTransform * boxLocalVertex;
		depth = planeNormalGlobal.dot(vertex - pointOnPlane);
		planeLocalVertex = planeQuat.inverse() * (vertex - planeNormalGlobal * depth - planeTrans);
		std::pair<Location, Location> penetrationPoint;
		penetrationPoint.first.rigidLocalPosition.setValue(boxLocalVertex);
		penetrationPoint.second.rigidLocalPosition.setValue(planeLocalVertex);
		expectedContacts->push_back(std::make_shared<Contact>(
										COLLISION_DETECTION_TYPE_DISCRETE, std::abs(depth),
										1.0, Vector3d::Zero(), collisionNormal, penetrationPoint));
	}
}

}; // namespace Collision
}; // namespace SurgSim
