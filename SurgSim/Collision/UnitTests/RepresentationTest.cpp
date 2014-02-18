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

#include "SurgSim/Collision/ContactCalculation.h"
#include "SurgSim/Collision/ShapeCollisionRepresentation.h"
#include "SurgSim/Collision/SpherePlaneDcdContact.h"
#include "SurgSim/Collision/UnitTests/ContactCalculationTestsCommon.h"

#include "SurgSim/Math/Shape.h"
#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Math/PlaneShape.h"

#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"

#include "SurgSim/Math/Geometry.h"

using SurgSim::Math::Vector3d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;

using SurgSim::Math::Shape;
using SurgSim::Math::PlaneShape;
using SurgSim::Math::SphereShape;

namespace SurgSim
{
namespace Collision
{

TEST(RepresentationTest, ZeroContactTest)
{
	std::shared_ptr<PlaneShape> plane = std::make_shared<PlaneShape>();
	std::shared_ptr<SphereShape> sphere = std::make_shared<SphereShape>(1.0);

	std::shared_ptr<Representation> planeRep = std::make_shared<ShapeCollisionRepresentation>(
		"Plane Shape", plane, SurgSim::Math::makeRigidTransform(Quaterniond::Identity(), Vector3d::Zero()));
	std::shared_ptr<Representation> sphereRep = std::make_shared<ShapeCollisionRepresentation>(
		"Sphere Shape",sphere, SurgSim::Math::makeRigidTransform(Quaterniond::Identity(), Vector3d(0.0, 2.0, 0.0)));

	std::shared_ptr<CollisionPair> pairSP = std::make_shared<CollisionPair>(sphereRep, planeRep);

	std::shared_ptr<SpherePlaneDcdContact> calContact = std::make_shared<SpherePlaneDcdContact>();

	calContact->calculateContact(pairSP);

	auto planeContacts = planeRep->getColliders();

	EXPECT_EQ(0u, planeContacts.size());
}


TEST(ContactCalculation, DidContactFrontTest)
{
	std::shared_ptr<PlaneShape> plane = std::make_shared<PlaneShape>();
	std::shared_ptr<SphereShape> sphere = std::make_shared<SphereShape>(1.0);

	Vector3d planeTrans(0.0, 0.5, 0.0);
	Vector3d sphereTrans(0.0, 1.0, 0.0);

	std::shared_ptr<Representation> planeRep = std::make_shared<ShapeCollisionRepresentation>(
		"Plane Shape", plane, SurgSim::Math::makeRigidTransform(Quaterniond::Identity(), planeTrans));
	std::shared_ptr<Representation> sphereRep = std::make_shared<ShapeCollisionRepresentation>(
		"Sphere Shape",sphere, SurgSim::Math::makeRigidTransform(Quaterniond::Identity(), sphereTrans));

	std::shared_ptr<CollisionPair> pairSP = std::make_shared<CollisionPair>(sphereRep, planeRep);

	std::shared_ptr<SpherePlaneDcdContact> calContact = std::make_shared<SpherePlaneDcdContact>();

	// SpherePlaneDcdContact
	calContact->calculateContact(pairSP);

	auto planeContacts = planeRep->getColliders();
	// Expect the plane collides with only one object.
	EXPECT_EQ(1u, planeContacts.size());
	// Expect the plane collides with the sphere object. 
	std::shared_ptr<Representation> collisionRep = planeContacts.front();
	std::shared_ptr<SurgSim::Math::Shape> actualShape = collisionRep->getShape();
	EXPECT_EQ(sphere, actualShape);
	// Expect contact points with sphere object
	auto contactPoints = collisionRep->getContacts(planeRep);

	// Expect depth of collision
	double expectedDepth = 0.5;
	EXPECT_NEAR(expectedDepth, (contactPoints.front())->depth, SurgSim::Math::Geometry::DistanceEpsilon);

	// Expect normal vector
	Vector3d expectedNorm (0.0, 1.0, 0.0);
	EXPECT_TRUE(eigenEqual(expectedNorm, (contactPoints.front())->normal));

	// Expect contact points
	Vector3d spherePenetration = sphereTrans - expectedNorm * sphere->getRadius();
	Vector3d planePenetration = sphereTrans - expectedNorm * (sphere->getRadius() - expectedDepth);

	EXPECT_TRUE(eigenEqual(spherePenetration,
			(contactPoints.front())->penetrationPoints.first.globalPosition.getValue()));
	EXPECT_TRUE(eigenEqual(planePenetration,
			(contactPoints.front())->penetrationPoints.second.globalPosition.getValue()));
}

}; // namespace Collision
}; // namespace SurgSim
