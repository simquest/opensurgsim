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
#include "SurgSim/Collision/SphereSphereContact.h"
#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/SphereShape.h"

using SurgSim::Math::Geometry::DistanceEpsilon;

namespace SurgSim
{
namespace Collision
{

void doSphereSphereTest(double r0, Vector3d p0, double r1, Vector3d p1, bool hasContacts,
						double expectedDepth = 0.0, Vector3d expectedNormal = Vector3d::UnitX(),
						Vector3d expectedPenetrationPoint0 = Vector3d::Zero(),
						Vector3d expectedPenetrationPoint1 = Vector3d::Zero())
{
	SphereSphereContact calc;

	auto sphere1 = std::make_shared<SurgSim::Math::SphereShape>(r0);
	auto sphereRep1 = std::make_shared<ShapeCollisionRepresentation>("TestSphereShapeCollisionRep1");
	sphereRep1->setShape(sphere1);
	sphereRep1->setLocalPose(SurgSim::Math::makeRigidTransform(Quaterniond::Identity(), p0));

	auto sphere2 = std::make_shared<SurgSim::Math::SphereShape>(r1);
	auto sphereRep2 = std::make_shared<ShapeCollisionRepresentation>("TestSphereShapeCollisionRep2");
	sphereRep2->setShape(sphere2);
	sphereRep2->setLocalPose(SurgSim::Math::makeRigidTransform(Quaterniond::Identity(), p1));

	std::shared_ptr<CollisionPair> pair = std::make_shared<CollisionPair>(sphereRep1, sphereRep2);

	calc.calculateContact(pair);
	EXPECT_EQ(hasContacts, pair->hasContacts());
	if (pair->hasContacts())
	{
		std::shared_ptr<Contact> contact = pair->getContacts().front();

		EXPECT_LT(-DistanceEpsilon, contact->depth);
		const double maxDepth = std::max(sphere1->getRadius(), sphere2->getRadius());
		EXPECT_GT(maxDepth + DistanceEpsilon, contact->depth);

		const Vector3d sphere2ToSphere1 = sphereRep1->getPose().translation() - sphereRep2->getPose().translation();
		if (!sphere2ToSphere1.isZero())
		{
			EXPECT_TRUE(eigenEqual(sphere2ToSphere1.normalized(), contact->normal));
		}

		EXPECT_TRUE(eigenEqual(expectedNormal, contact->normal));
		EXPECT_NEAR(expectedDepth, contact->depth, SurgSim::Math::Geometry::DistanceEpsilon);
		EXPECT_TRUE(contact->penetrationPoints.first.rigidLocalPosition.hasValue());
		EXPECT_TRUE(contact->penetrationPoints.second.rigidLocalPosition.hasValue());

		EXPECT_TRUE(eigenEqual(expectedPenetrationPoint0,
							   contact->penetrationPoints.first.rigidLocalPosition.getValue()));
		EXPECT_TRUE(eigenEqual(expectedPenetrationPoint1,
							   contact->penetrationPoints.second.rigidLocalPosition.getValue()));
	}
}

TEST(SphereSphereContactCalculationTests, UnitTests)
{
	using SurgSim::Math::Geometry::DistanceEpsilon;

	{
		SCOPED_TRACE("No Intersection");
		doSphereSphereTest(0.1, Vector3d(0.0,0.0,0.0), 0.1, Vector3d(1.0,1.0,1.0), false);
	}

	{
		SCOPED_TRACE("Sphere-Sphere intersection at origin");
		doSphereSphereTest(0.5,
						   Vector3d(-0.5+DistanceEpsilon/2.0,0.0,0.0), 0.5, Vector3d(0.5-DistanceEpsilon/2.0,0.0,0.0),
						   true, DistanceEpsilon, Vector3d(-1.0,0.0,0.0), Vector3d(0.5,0.0,0.0),
						   Vector3d(-0.5,0.0,0.0));
	}

	{
		SCOPED_TRACE("Sphere-Sphere intersection");
		doSphereSphereTest(0.5, Vector3d(0.0,0.0,0.0), 0.5, Vector3d(0.5,0.0,0.0), true, 0.5,
						   Vector3d(-1.0,0.0,0.0), Vector3d(0.5,0.0,0.0), Vector3d(-0.5,0.0,0.0));
	}
}

}; // namespace Collision
}; // namespace SurgSim
