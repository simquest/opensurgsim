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

#include <SurgSim/Collision/UnitTests/ContactCalculationTestsCommon.h>
#include <SurgSim/Collision/SphereSphereDcdContact.h>

namespace SurgSim
{
namespace Collision
{

void doSphereSphereTest(double r0, Vector3d p0, double r1, Vector3d p1, bool hasContacts, 
						double expectedDepth = 0.0, Vector3d expectedNormal = Vector3d::UnitX(),
						Vector3d expectedPenetrationPoint0 = Vector3d::Zero(),
						Vector3d expectedPenetrationPoint1 = Vector3d::Zero())
{
	SphereSphereDcdContact calc;
	std::shared_ptr<CollisionPair> pair =
		std::make_shared<CollisionPair>(makeSphereRepresentation(nullptr, r0,Quaterniond::Identity(),p0),
										makeSphereRepresentation(nullptr, r1,Quaterniond::Identity(),p1));

	calc.calculateContact(pair);
	EXPECT_EQ(hasContacts, pair->hasContacts());
	if (pair->hasContacts())
	{
		std::shared_ptr<Contact> contact = pair->getContacts().front();
		EXPECT_TRUE(eigenEqual(expectedNormal, contact->normal, epsilon));
		EXPECT_NEAR(expectedDepth, contact->depth, epsilon);
		EXPECT_TRUE(contact->penetrationPoints.first.globalPosition.hasValue());
		EXPECT_TRUE(contact->penetrationPoints.second.globalPosition.hasValue());

		EXPECT_TRUE(eigenEqual(expectedPenetrationPoint0,
							   contact->penetrationPoints.first.globalPosition.getValue(),
							   epsilon));
		EXPECT_TRUE(eigenEqual(expectedPenetrationPoint1,
							   contact->penetrationPoints.second.globalPosition.getValue(),
							   epsilon));
	}
}

TEST(SphereSphereContactCalculationTests, UnitTests)
{
	{
		SCOPED_TRACE("No Intersection");
		doSphereSphereTest(0.1, Vector3d(0.0,0.0,0.0), 0.1, Vector3d(1.0,1.0,1.0), false);
	}

	{
		SCOPED_TRACE("Sphere-Sphere intersection at origin");
		doSphereSphereTest(0.5, Vector3d(-0.5+epsilon/2.0,0.0,0.0), 0.5, Vector3d(0.5-epsilon/2.0,0.0,0.0),
						   true, epsilon, Vector3d(-1.0,0.0,0.0), Vector3d::Zero(), Vector3d::Zero());
	}

	{
		SCOPED_TRACE("Sphere-Sphere intersection");
		doSphereSphereTest(0.5, Vector3d(0.0,0.0,0.0), 0.5, Vector3d(0.5,0.0,0.0), true, 0.5,
						   Vector3d(-1.0,0.0,0.0), Vector3d(0.5,0.0,0.0), Vector3d(0.0,0.0,0.0));
	}
}

}; // namespace Collision
}; // namespace SurgSim