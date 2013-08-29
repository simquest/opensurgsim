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
#include <memory>

#include <SurgSim/Collision/UnitTests/RepresentationUtilities.h>
#include <SurgSim/Collision/UnitTests/MockCollisionRepresentation.h>

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>

#include <SurgSim/Physics/RigidShape.h>
#include <SurgSim/Collision/CollisionRepresentation.h>
#include <SurgSim/Collision/ContactCalculation.h>
#include <SurgSim/Collision/CollisionPair.h>

#include <SurgSim/Math/Geometry.h>
#include <SurgSim/Collision/RigidShapeCollisionRepresentation.h>

using SurgSim::Math::Vector3d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;

using SurgSim::Physics::RigidShape;

namespace SurgSim
{
namespace Collision
{

namespace
{
double epsilon = SurgSim::Math::Geometry::ScalarEpsilon;
}

static ::testing::AssertionResult eigenEqual(const Vector3d& left, const Vector3d& right, double epsilon)
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

static ::testing::AssertionResult isContactPresentInList(std::shared_ptr<Contact> expected,
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

static void contactsInfoEqualityTest(const std::list<std::shared_ptr<Contact>>& expectedContacts,
									 const std::list<std::shared_ptr<Contact>>& calculatedContacts)
{
    SCOPED_TRACE("Comparing the contact info.");

    EXPECT_EQ(expectedContacts.size(), calculatedContacts.size());

    for (auto it = expectedContacts.begin(); it != expectedContacts.end(); ++it)
    {
        EXPECT_TRUE(isContactPresentInList(*it, calculatedContacts));
    }
}

static Vector3d calculateBoxVertex(const int i, const double* size,
								   const Quaterniond& quat,
								   const Vector3d& trans)
{
    static const double multiplier[8][3] = {{-0.5, -0.5, -0.5}, {-0.5, -0.5, 0.5}, {-0.5, 0.5, 0.5}, {-0.5, 0.5, -0.5},
        {0.5, -0.5, -0.5}, {0.5, -0.5, 0.5}, {0.5, 0.5, 0.5}, {0.5, 0.5, -0.5}
    };

    return (quat * Vector3d(size[0] * multiplier[i][0], size[1] * multiplier[i][1], size[2] * multiplier[i][2])) +
           trans;
}

static void generateBoxPlaneContact(std::list<std::shared_ptr<Contact>>& expectedContacts,
									const int expectedNumberOfContacts, const int* expectedBoxIndicesInContacts,
									const double* size,
									const Vector3d& boxTrans, const Quaterniond& boxQuat,
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
        vertex = calculateBoxVertex(expectedBoxIndicesInContacts[i], size, boxQuat, boxTrans);
        std::pair<Location, Location> penetrationPoint;
        penetrationPoint.first.globalPosition.setValue(vertex);
        depth = planeNormalGlobal.dot(vertex - pointOnPlane);
        penetrationPoint.second.globalPosition.setValue(vertex - planeNormalGlobal * depth);
        expectedContacts.push_back(std::make_shared<Contact>(depth, Vector3d::Zero(),
                                                             collisionNormal, penetrationPoint));
    }
}

static void generateBoxDoubleSidedPlaneContact(std::list<std::shared_ptr<Contact>>& expectedContacts,
											   const int expectedNumberOfContacts,
											   const int* expectedBoxIndicesInContacts,
											   const double* size,
											   const Vector3d& boxTrans, const Quaterniond& boxQuat,
											   const Vector3d& planeNormal, const double planeD,
											   const Vector3d& planeTrans, const Quaterniond& planeQuat,
											   const bool collisionNormalIsPlaneNormal)
{
    Vector3d vertex;
    Vector3d planeNormalGlobal = planeQuat * planeNormal;
    Vector3d pointOnPlane = planeTrans + (planeNormalGlobal * planeD);
    double depth = 0.0;
    Vector3d collisionNormal = planeNormalGlobal * (collisionNormalIsPlaneNormal ? 1.0 : -1.0);
    for (int i = 0; i < expectedNumberOfContacts; ++i)
    {
        vertex = calculateBoxVertex(expectedBoxIndicesInContacts[i], size, boxQuat, boxTrans);
        std::pair<Location, Location> penetrationPoint;
        penetrationPoint.first.globalPosition.setValue(vertex);
        depth = planeNormalGlobal.dot(vertex - pointOnPlane);
        penetrationPoint.second.globalPosition.setValue(vertex - planeNormalGlobal * depth);
        expectedContacts.push_back(std::make_shared<Contact>(std::abs(depth), Vector3d::Zero(),
                                                             collisionNormal, penetrationPoint));
    }
}

}; // namespace Collision
}; // namespace SurgSim