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

void checkContactInfo(std::shared_ptr<Contact> contact, double expectedDepth,
					  Vector3d &expectedNormal, Vector3d &expectedPenetrationPointFirst,
					  Vector3d &expectedPenetrationPointSecond)
{
	EXPECT_NEAR(expectedDepth, contact->depth, SurgSim::Math::Geometry::DistanceEpsilon);
	EXPECT_TRUE(eigenEqual(expectedNormal, contact->normal));
	EXPECT_TRUE(contact->penetrationPoints.first.globalPosition.hasValue());
	EXPECT_TRUE(contact->penetrationPoints.second.globalPosition.hasValue());
	EXPECT_TRUE(eigenEqual(expectedPenetrationPointFirst,
							contact->penetrationPoints.first.globalPosition.getValue()));
	EXPECT_TRUE(eigenEqual(expectedPenetrationPointSecond,
							contact->penetrationPoints.second.globalPosition.getValue()));
}

::testing::AssertionResult isContactPresentInList(std::shared_ptr<Contact> expected,
												  const std::list<std::shared_ptr<Contact>>& contactsList)
{
	using SurgSim::Math::Geometry::ScalarEpsilon;

	bool contactPresent = false;
	for (auto it = contactsList.begin(); it != contactsList.end() && !contactPresent; ++it)
	{
		// Compare the normals.
		contactPresent = eigenEqual(expected->normal, it->get()->normal);
		// Compare the global position of first object.
		contactPresent &= eigenEqual(expected->penetrationPoints.first.globalPosition.getValue(),
									 it->get()->penetrationPoints.first.globalPosition.getValue());
		// Compare the global position of second object.
		contactPresent &= eigenEqual(expected->penetrationPoints.second.globalPosition.getValue(),
									 it->get()->penetrationPoints.second.globalPosition.getValue());
		// Compare the depth.
		contactPresent &= std::abs(expected->depth - it->get()->depth) <= ScalarEpsilon;
		// Check if the optional 'triangleLocalPosition' is present in expected contact.
		if (expected->penetrationPoints.first.triangleLocalPosition.hasValue())
		{
			EXPECT_TRUE(it->get()->penetrationPoints.first.triangleLocalPosition.hasValue());
			contactPresent &= expected->penetrationPoints.first.triangleLocalPosition.getValue().first ==
							  it->get()->penetrationPoints.first.triangleLocalPosition.getValue().first;
			auto triangleContact = std::static_pointer_cast<SurgSim::Collision::TriangleContact>(*it);
			if (triangleContact != nullptr)
			{
				Vector3d barycentricCoordinates =
					it->get()->penetrationPoints.first.triangleLocalPosition.getValue().second;
				eigenEqual(expected->penetrationPoints.first.globalPosition.getValue(),
					barycentricCoordinates[0] * triangleContact->firstVertices[0] +
					barycentricCoordinates[1] * triangleContact->firstVertices[1] +
					barycentricCoordinates[2] * triangleContact->firstVertices[2]);
			}
		}
		if (expected->penetrationPoints.second.triangleLocalPosition.hasValue())
		{
			EXPECT_TRUE(it->get()->penetrationPoints.second.triangleLocalPosition.hasValue());
			contactPresent &= expected->penetrationPoints.second.triangleLocalPosition.getValue().first ==
							  it->get()->penetrationPoints.second.triangleLocalPosition.getValue().first;
			auto triangleContact = std::static_pointer_cast<SurgSim::Collision::TriangleContact>(*it);
			if (triangleContact != nullptr)
			{
				Vector3d barycentricCoordinates =
					it->get()->penetrationPoints.second.triangleLocalPosition.getValue().second;
				eigenEqual(expected->penetrationPoints.second.globalPosition.getValue(),
					barycentricCoordinates[0] * triangleContact->secondVertices[0] +
					barycentricCoordinates[1] * triangleContact->secondVertices[1] +
					barycentricCoordinates[2] * triangleContact->secondVertices[2]);
			}
		}
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

void generateBoxPlaneContact(std::list<std::shared_ptr<Contact>>* expectedContacts,
							 const int expectedNumberOfContacts,
							 const int* expectedBoxIndicesInContacts,
							 const std::shared_ptr<BoxShape> box,
							 const Vector3d& boxTrans, const Quaterniond& boxQuat,
							 const std::shared_ptr<PlaneShape> plane,
							 const Vector3d& planeTrans, const Quaterniond& planeQuat)
{
	Vector3d vertex;
	Vector3d planeNormalGlobal = planeQuat * plane->getNormal();
	Vector3d pointOnPlane = planeTrans + (planeNormalGlobal * plane->getD());
	double depth = 0.0;
	Vector3d collisionNormal = planeNormalGlobal;
	RigidTransform3d boxTransform = SurgSim::Math::makeRigidTransform(boxQuat, boxTrans);
	for (int i = 0; i < expectedNumberOfContacts; ++i)
	{
		vertex = boxTransform * box->getVertex(expectedBoxIndicesInContacts[i]);
		std::pair<Location, Location> penetrationPoint;
		penetrationPoint.first.globalPosition.setValue(vertex);
		depth = planeNormalGlobal.dot(vertex - pointOnPlane);
		penetrationPoint.second.globalPosition.setValue(vertex - planeNormalGlobal * depth);
		expectedContacts->push_back(std::make_shared<Contact>(depth, Vector3d::Zero(),
															 collisionNormal, penetrationPoint));
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
	Vector3d planeNormalGlobal = planeQuat * plane->getNormal();
	Vector3d pointOnPlane = planeTrans + (planeNormalGlobal * plane->getD());
	double depth = 0.0;
	Vector3d collisionNormal = planeNormalGlobal * (collisionNormalIsPlaneNormal ? 1.0 : -1.0);
	RigidTransform3d boxTransform = SurgSim::Math::makeRigidTransform(boxQuat, boxTrans);
	for (int i = 0; i < expectedNumberOfContacts; ++i)
	{
		vertex = boxTransform * box->getVertex(expectedBoxIndicesInContacts[i]);
		std::pair<Location, Location> penetrationPoint;
		penetrationPoint.first.globalPosition.setValue(vertex);
		depth = planeNormalGlobal.dot(vertex - pointOnPlane);
		penetrationPoint.second.globalPosition.setValue(vertex - planeNormalGlobal * depth);
		expectedContacts->push_back(std::make_shared<Contact>(std::abs(depth), Vector3d::Zero(),
															 collisionNormal, penetrationPoint));
	}
}

}; // namespace Collision
}; // namespace SurgSim
