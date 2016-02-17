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
#include "SurgSim/Collision/SpherePlaneContact.h"

#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Math/PlaneShape.h"

#include "SurgSim/Collision/UnitTests/ContactCalculationTestsCommon.h"

#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"

using SurgSim::Math::Vector3d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;

using SurgSim::Math::PlaneShape;
using SurgSim::Math::SphereShape;

namespace SurgSim
{

namespace Collision
{

// This uses a concrete contact calculation as a placeholder, the current algorithm
// has the check for the pair in the superclass, therefore a test on one class should
// be sufficient
TEST(ContactCalculationTests, SwappedPairTest)
{
	std::shared_ptr<PlaneShape> plane = std::make_shared<PlaneShape>();
	std::shared_ptr<SphereShape> sphere = std::make_shared<SphereShape>(1.0);

	Vector3d trans(0.0, 0.0, 0.0);
	Quaterniond quat = Quaterniond::Identity();

	std::shared_ptr<ShapeCollisionRepresentation> planeRep =
		std::make_shared<ShapeCollisionRepresentation>("Plane Shape");
	planeRep->setShape(plane);
	planeRep->setLocalPose(SurgSim::Math::makeRigidTransform(quat, trans));

	std::shared_ptr<ShapeCollisionRepresentation> sphereRep =
		std::make_shared<ShapeCollisionRepresentation>("Sphere Shape");
	sphereRep->setShape(sphere);
	sphereRep->setLocalPose(SurgSim::Math::makeRigidTransform(quat, trans));

	std::shared_ptr<CollisionPair> pair1 = std::make_shared<CollisionPair>(sphereRep, planeRep);
	std::shared_ptr<CollisionPair> pair2 = std::make_shared<CollisionPair>(planeRep, sphereRep);

	std::shared_ptr<SpherePlaneContact> calc = std::make_shared<SpherePlaneContact>();

	EXPECT_NO_THROW(calc->calculateContact(pair1));
	EXPECT_NO_THROW(calc->calculateContact(pair2));

}

TEST(ContactCalculationTests, SwappedShapeTest)
{
	std::shared_ptr<PlaneShape> plane = std::make_shared<PlaneShape>();
	std::shared_ptr<SphereShape> sphere = std::make_shared<SphereShape>(1.0);
	std::shared_ptr<SpherePlaneContact> calc = std::make_shared<SpherePlaneContact>();

	auto transform = Math::RigidTransform3d::Identity();
	sphere->setPose(transform);
	plane->setPose(transform);

	ASSERT_NO_THROW(calc->calculateDcdContact(sphere, plane));
	ASSERT_NO_THROW(calc->calculateDcdContact(plane, sphere));

	auto planeRep = std::make_shared<ShapeCollisionRepresentation>("Plane Shape");
	planeRep->setShape(plane);
	planeRep->setLocalPose(transform);

	auto sphereRep = std::make_shared<ShapeCollisionRepresentation>("Sphere Shape");
	sphereRep->setShape(sphere);
	sphereRep->setLocalPose(transform);

	std::shared_ptr<CollisionPair> pair1 = std::make_shared<CollisionPair>(sphereRep, planeRep);

	calc->calculateContact(pair1);

	auto contacts1 = calc->calculateDcdContact(sphere, plane);
	auto contacts2 = calc->calculateDcdContact(plane, sphere);

	contactsInfoEqualityTest(pair1->getContacts(), contacts1);

	// Contacts2 should be flipped from contacts1
	for (auto& contact : contacts2)
	{
		contact->normal = -contact->normal;
		std::swap(contact->penetrationPoints.first, contact->penetrationPoints.second);
	}

	contactsInfoEqualityTest(contacts1 , contacts2);
}

}; // namespace Collision
}; // namespace SurgSim
