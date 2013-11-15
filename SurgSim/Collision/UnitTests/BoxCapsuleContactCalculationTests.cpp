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

#include <SurgSim/Collision/BoxCapsuleDcdContact.h>
#include <SurgSim/Collision/CollisionRepresentation.h>
#include <SurgSim/Collision/ShapeCollisionRepresentation.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>

using SurgSim::Math::BoxShape;
using SurgSim::Math::CapsuleShape;

namespace SurgSim
{
namespace Collision
{

void doBoxCapsuleTest(std::shared_ptr<BoxShape> box,
					  const SurgSim::Math::Quaterniond& boxQuat,
					  const SurgSim::Math::Vector3d& boxTrans,
					  std::shared_ptr<CapsuleShape> capsule,
					  const SurgSim::Math::Quaterniond& capsuleQuat,
					  const SurgSim::Math::Vector3d& capsuleTrans,
					  const int expectedInContact)
{
	std::shared_ptr<CollisionRepresentation> boxRep = std::make_shared<ShapeCollisionRepresentation>(
		"Collision Box 0", box, SurgSim::Math::makeRigidTransform(boxQuat, boxTrans));
	std::shared_ptr<CollisionRepresentation> capsuleRep = std::make_shared<ShapeCollisionRepresentation>(
		"Collision Capsule 0", capsule, SurgSim::Math::makeRigidTransform(capsuleQuat, capsuleTrans));

	// Perform collision detection.
	BoxCapsuleDcdContact calcContact;
	std::shared_ptr<CollisionPair> pair = std::make_shared<CollisionPair>(boxRep, capsuleRep);
	calcContact.calculateContact(pair);

	EXPECT_EQ(expectedInContact, pair->hasContacts());

	if (expectedInContact)
	{
		SurgSim::Math::Vector3d capsuleToBox;
		capsuleToBox = boxTrans - capsuleTrans;

		double depthMax = box->getSize().norm();
		depthMax += capsule->getLength() / 2.0 + capsule->getRadius();

		auto contacts = pair->getContacts();
		for (auto contact=contacts.cbegin(); contact!=contacts.cend(); ++contact)
		{
			if (! capsuleToBox.isZero())
			{
				// Check that each normal is pointing into the box
				EXPECT_LT(0.0, (*contact)->normal.dot(capsuleToBox));
			}

			// Check that the depth is sane
			EXPECT_LT(0.0, (*contact)->depth);
			EXPECT_GT(depthMax, (*contact)->depth);
		}
	}
}

TEST(BoxCapsuleContactCalculationTests, UnitTests)
{
	std::shared_ptr<BoxShape> box = std::make_shared<BoxShape>(1.0, 1.0, 1.0);
	std::shared_ptr<CapsuleShape> capsule = std::make_shared<CapsuleShape>(4.0, 1.0);
	SurgSim::Math::Quaterniond boxQuat;
	SurgSim::Math::Vector3d boxTrans;
	SurgSim::Math::Quaterniond capsuleQuat;
	SurgSim::Math::Vector3d capsuleTrans;

	{
		SCOPED_TRACE("No intersection, box in front of capsule");
		boxQuat = SurgSim::Math::makeRotationQuaternion(0.0, Vector3d(0.0, 0.0, 1.0));
		boxTrans = Vector3d(10.6, 0.0, 0.0);
		capsuleQuat = SurgSim::Math::makeRotationQuaternion(0.0, Vector3d(0.0, 0.0, 1.0));
		capsuleTrans = Vector3d::Zero();
		int expectedInContact = false;
		doBoxCapsuleTest(box, boxQuat, boxTrans, capsule, capsuleQuat, capsuleTrans, expectedInContact);
	}

	{
		SCOPED_TRACE("No intersection, capsule beyond corner of box");
		boxQuat = SurgSim::Math::makeRotationQuaternion(0.0, Vector3d(0.0, 0.0, 1.0));
		boxTrans = Vector3d::Zero();
		capsuleQuat = SurgSim::Math::makeRotationQuaternion(0.0, Vector3d(0.0, 0.0, 1.0));
		int expectedInContact = false;
		capsuleTrans = Vector3d(1.5, 0.0, 1.5);
		doBoxCapsuleTest(box, boxQuat, boxTrans, capsule, capsuleQuat, capsuleTrans, expectedInContact);
		capsuleTrans = Vector3d(1.5, 0.0, -1.5);
		doBoxCapsuleTest(box, boxQuat, boxTrans, capsule, capsuleQuat, capsuleTrans, expectedInContact);
		capsuleTrans = Vector3d(-1.5, 0.0, 1.5);
		doBoxCapsuleTest(box, boxQuat, boxTrans, capsule, capsuleQuat, capsuleTrans, expectedInContact);
		capsuleTrans = Vector3d(-1.5, 0.0, -1.5);
		doBoxCapsuleTest(box, boxQuat, boxTrans, capsule, capsuleQuat, capsuleTrans, expectedInContact);
	}

	{
		SCOPED_TRACE("No intersection, box below capsule");
		boxQuat = SurgSim::Math::makeRotationQuaternion(0.0, Vector3d(0.0, 0.0, 1.0));
		boxTrans = Vector3d(0.0, -3.6, 0.0);
		capsuleQuat = SurgSim::Math::makeRotationQuaternion(0.0, Vector3d(0.0, 0.0, 1.0));
		capsuleTrans = Vector3d::Zero();
		int expectedInContact = false;
		doBoxCapsuleTest(box, boxQuat, boxTrans, capsule, capsuleQuat, capsuleTrans, expectedInContact);
	}

	{
		SCOPED_TRACE("Intersection, box intersection with capsule side");
		boxQuat = SurgSim::Math::makeRotationQuaternion(0.0, Vector3d(0.0, 0.0, 1.0));
		boxTrans = Vector3d(1.0 , 0.0, 0.0);
		capsuleQuat = SurgSim::Math::makeRotationQuaternion(0.0, Vector3d(0.0, 0.0, 1.0));
		capsuleTrans = Vector3d::Zero();
		int expectedInContact = true;
		doBoxCapsuleTest(box, boxQuat, boxTrans, capsule, capsuleQuat, capsuleTrans, expectedInContact);
	}

	{
		SCOPED_TRACE("Intersection, box intersection with upside down capsule");
		boxQuat = SurgSim::Math::makeRotationQuaternion(0.0, Vector3d(0.0, 0.0, 1.0));
		boxTrans = Vector3d(1.0 , 0.0, 0.0);
		capsuleQuat = SurgSim::Math::makeRotationQuaternion(M_PI, Vector3d(0.0, 0.0, 1.0));
		capsuleTrans = Vector3d::Zero();
		int expectedInContact = true;
		doBoxCapsuleTest(box, boxQuat, boxTrans, capsule, capsuleQuat, capsuleTrans, expectedInContact);
	}

	{
		SCOPED_TRACE("Intersection, box intersection with z-axis capsule");
		boxQuat = SurgSim::Math::makeRotationQuaternion(0.0, Vector3d(0.0, 0.0, 1.0));
		boxTrans = Vector3d(1.0 , 0.0, 0.0);
		capsuleQuat = SurgSim::Math::makeRotationQuaternion(M_PI_2, Vector3d(1.0, 0.0, 0.0));
		capsuleTrans = Vector3d::Zero();
		int expectedInContact = true;
		doBoxCapsuleTest(box, boxQuat, boxTrans, capsule, capsuleQuat, capsuleTrans, expectedInContact);
	}

	{
		SCOPED_TRACE("Intersection, box intersection with x-axis capsule");
		boxQuat = SurgSim::Math::makeRotationQuaternion(M_PI, Vector3d(0.0, 0.0, 1.0));
		boxTrans = Vector3d(1.0 , 0.0, 0.0);
		capsuleQuat = SurgSim::Math::makeRotationQuaternion(M_PI_2, Vector3d(1.0, 0.0, 0.0));
		capsuleTrans = Vector3d::Zero();
		int expectedInContact = true;
		doBoxCapsuleTest(box, boxQuat, boxTrans, capsule, capsuleQuat, capsuleTrans, expectedInContact);
	}

	{
		SCOPED_TRACE("Intersection, box intersection with capsule cap");
		boxQuat = SurgSim::Math::makeRotationQuaternion(M_PI_2, Vector3d(0.0, 0.0, 1.0));
		boxTrans = Vector3d(0.1 , 0.0, 0.1);
		capsuleQuat = SurgSim::Math::makeRotationQuaternion(0.0, Vector3d(0.0, 0.0, 1.0));
		capsuleTrans = Vector3d(0.0 , 2.6, 0.0);
		int expectedInContact = true;
		doBoxCapsuleTest(box, boxQuat, boxTrans, capsule, capsuleQuat, capsuleTrans, expectedInContact);
	}

	{
		SCOPED_TRACE("No intersection, capsule near box corner, but not intersecting");
		boxQuat = SurgSim::Math::makeRotationQuaternion(0.0, Vector3d(0.0, 0.0, 1.0));
		boxTrans = Vector3d(0.0 , 0.0, 0.0);
		capsuleQuat = SurgSim::Math::makeRotationQuaternion(0.0, Vector3d(0.0, 0.0, 1.0));
		capsuleTrans = Vector3d(1.3 , 0.0, 1.3);
		int expectedInContact = false;
		doBoxCapsuleTest(box, boxQuat, boxTrans, capsule, capsuleQuat, capsuleTrans, expectedInContact);
	}

	{
		SCOPED_TRACE("Intersection, capsule intersecting with box corner");
		boxQuat = SurgSim::Math::makeRotationQuaternion(0.0, Vector3d(0.0, 0.0, 1.0));
		boxTrans = Vector3d(0.0 , 0.0, 0.0);
		capsuleQuat = SurgSim::Math::makeRotationQuaternion(0.0, Vector3d(0.0, 0.0, 1.0));
		capsuleTrans = Vector3d(1.2 , 0.0, 1.2);
		int expectedInContact = true;
		doBoxCapsuleTest(box, boxQuat, boxTrans, capsule, capsuleQuat, capsuleTrans, expectedInContact);
	}


	{
		SCOPED_TRACE("Intersection, box inside capsule");
		boxQuat = SurgSim::Math::makeRotationQuaternion(0.0, Vector3d(0.0, 0.0, 1.0));
		boxTrans = Vector3d::Zero();
		capsuleQuat = SurgSim::Math::makeRotationQuaternion(0.0, Vector3d(0.0, 0.0, 1.0));
		capsuleTrans = Vector3d::Zero();
		int expectedInContact = true;
		doBoxCapsuleTest(box, boxQuat, boxTrans, capsule, capsuleQuat, capsuleTrans, expectedInContact);
	}

	{
		SCOPED_TRACE("Intersection, capsule inside box");
		std::shared_ptr<BoxShape> bigBox = std::make_shared<BoxShape>(10.0, 10.0, 10.0);
		boxQuat = SurgSim::Math::makeRotationQuaternion(-M_PI_4, Vector3d(0.0, 1.0, 0.0));
		boxTrans = Vector3d::Zero();
		capsuleQuat = SurgSim::Math::makeRotationQuaternion(M_PI, Vector3d(1.0, 0.0, 0.0));
		capsuleTrans = Vector3d(0.0, 0.0, 0.0);
		int expectedInContact = true;
		doBoxCapsuleTest(bigBox, boxQuat, boxTrans, capsule, capsuleQuat, capsuleTrans, expectedInContact);
	}
}


}; // namespace Collision
}; // namespace SurgSim
