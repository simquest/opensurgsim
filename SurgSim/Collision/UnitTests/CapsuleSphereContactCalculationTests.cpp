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
#include "SurgSim/Collision/CapsuleSphereContact.h"
#include "SurgSim/Math/CapsuleShape.h"
#include "SurgSim/Math/SphereShape.h"

using SurgSim::Math::CapsuleShape;
using SurgSim::Math::SphereShape;

namespace SurgSim
{
namespace Collision
{

void doCapsuleSphereTest(double capsuleHeight, double capsuleRadius,
						 const Vector3d& capsulePosition, const Quaterniond& capsuleQuat,
						 double sphereRadius, const Vector3d& spherePosition, const Quaterniond& sphereQuat,
						 bool hasContacts, double depth,
						 const Vector3d& sphereProjection = Vector3d::Zero(),
						 const Vector3d& expectedNorm = Vector3d::Zero())
{
	std::shared_ptr<CollisionPair> pair = std::make_shared<CollisionPair>(
		makeCapsuleRepresentation(capsuleHeight, capsuleRadius, capsuleQuat, capsulePosition),
		makeSphereRepresentation(sphereRadius, sphereQuat, spherePosition));

	CapsuleSphereDcdContact calc;
	calc.calculateContact(pair);
	EXPECT_EQ(hasContacts, pair->hasContacts());

	if (pair->hasContacts())
	{
		std::shared_ptr<Contact> contact(pair->getContacts().front());

		EXPECT_TRUE(eigenEqual(expectedNorm, contact->normal));
		EXPECT_NEAR(depth, contact->depth, SurgSim::Math::Geometry::DistanceEpsilon);
		EXPECT_TRUE(contact->penetrationPoints.first.rigidLocalPosition.hasValue());
		EXPECT_TRUE(contact->penetrationPoints.second.rigidLocalPosition.hasValue());

		Vector3d capsuleLocalNormal = capsuleQuat.inverse() * expectedNorm;
		Vector3d penetrationPoint0(sphereProjection - capsuleLocalNormal * capsuleRadius);
		Vector3d sphereLocalNormal = sphereQuat.inverse() * expectedNorm;
		Vector3d penetrationPoint1(sphereLocalNormal * sphereRadius);
		EXPECT_TRUE(eigenEqual(penetrationPoint0, contact->penetrationPoints.first.rigidLocalPosition.getValue()));
		EXPECT_TRUE(eigenEqual(penetrationPoint1, contact->penetrationPoints.second.rigidLocalPosition.getValue()));
	}
}

TEST(CapsuleSphereContactCalculationTests, UnitTests)
{
	{
		SCOPED_TRACE("No Intersection");
		doCapsuleSphereTest(0.2, 0.1, Vector3d::Zero(), Quaterniond::Identity(),
							0.1, Vector3d(1.0, 1.0, 1.0), Quaterniond::Identity(), false, 0.0);
	}

	{
		SCOPED_TRACE("Capsule along Y-axis, intersection with cylindrical part of the capsule");
		doCapsuleSphereTest(0.8, 0.5, Vector3d::Zero(), Quaterniond::Identity(),
							0.3, Vector3d(0.7, 0, 0), Quaterniond::Identity(), true, 0.1,
							Vector3d::Zero(), Vector3d(-1.0, 0.0, 0.0));
	}

	{
		SCOPED_TRACE("Capsule along X-axis, intersection with hemispherical part of the capsule");
		doCapsuleSphereTest(0.1, 0.2, Vector3d::Zero(),
							SurgSim::Math::makeRotationQuaternion(M_PI_2, Vector3d(0.0, 0.0, 1.0)),
							0.1, Vector3d(-0.2, 0.0, 0.0),
							Quaterniond::Identity(), true, 0.15,
							Vector3d(0.0, 0.05, 0.0), Vector3d(1.0, 0.0, 0.0));
	}

	{
		SCOPED_TRACE("Intersection, capsule roated along Z-axis clockwise 45 degrees");
		Vector3d sphereCenter(2.0, 0.0, 0.0);
		Vector3d sphereProjection(1.0, 1.0, 0.0);
		Vector3d expectedNormal = (sphereProjection - sphereCenter).normalized();

		doCapsuleSphereTest(2 * M_SQRT2, M_SQRT2, Vector3d::Zero(),
							SurgSim::Math::makeRotationQuaternion(-M_PI_4, Vector3d(0.0, 0.0, 1.0)),
							1.0, sphereCenter,
							Quaterniond::Identity(), true, 1,
							Vector3d(0.0, M_SQRT2, 0.0), expectedNormal);

	}
}

}; // namespace Collision
}; // namespace SurgSim
