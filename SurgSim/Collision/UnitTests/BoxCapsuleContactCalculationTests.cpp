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

#include "SurgSim/Collision/BoxCapsuleContact.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Collision/ShapeCollisionRepresentation.h"
#include "SurgSim/Math/BoxShape.h"
#include "SurgSim/Math/CapsuleShape.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"

using SurgSim::Math::BoxShape;
using SurgSim::Math::CapsuleShape;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::makeRotationQuaternion;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Collision
{

void doBoxCapsuleTest(std::shared_ptr<BoxShape> box,
					  const Quaterniond& boxQuat,
					  const Vector3d& boxTrans,
					  std::shared_ptr<CapsuleShape> capsule,
					  const Quaterniond& capsuleQuat,
					  const Vector3d& capsuleTrans,
					  const bool expectedInContact)
{
	std::shared_ptr<ShapeCollisionRepresentation> boxRep =
		std::make_shared<ShapeCollisionRepresentation>("Collision Box 0");
	boxRep->setShape(box);
	boxRep->setLocalPose(makeRigidTransform(boxQuat, boxTrans));

	std::shared_ptr<ShapeCollisionRepresentation> capsuleRep =
		std::make_shared<ShapeCollisionRepresentation>("Collision Capsule 0");
	capsuleRep->setShape(capsule);
	capsuleRep->setLocalPose(makeRigidTransform(capsuleQuat, capsuleTrans));

	// Perform collision detection.
	BoxCapsuleDcdContact calcContact;
	std::shared_ptr<CollisionPair> pair = std::make_shared<CollisionPair>(boxRep, capsuleRep);
	calcContact.calculateContact(pair);

	EXPECT_EQ(expectedInContact, pair->hasContacts());

	if (expectedInContact)
	{
		Vector3d capsuleToBox = boxTrans - capsuleTrans;

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

			// Check that the locations are sane
			Vector3d boxPenetrationPoint =
				boxQuat * (*contact)->penetrationPoints.first.rigidLocalPosition.getValue() + boxTrans;
			Vector3d capsulePenetrationPoint =
				capsuleQuat * (*contact)->penetrationPoints.second.rigidLocalPosition.getValue() + capsuleTrans;
			EXPECT_GT(0.0, (*contact)->normal.dot(boxPenetrationPoint - boxTrans));
			EXPECT_LT(0.0, (*contact)->normal.dot(capsulePenetrationPoint - capsuleTrans));
		}
	}
}

TEST(BoxCapsuleContactCalculationTests, UnitTests)
{
	std::shared_ptr<BoxShape> box = std::make_shared<BoxShape>(1.0, 1.0, 1.0);
	std::shared_ptr<CapsuleShape> capsule = std::make_shared<CapsuleShape>(4.0, 1.0);
	Quaterniond boxQuat;
	Vector3d boxTrans;
	Quaterniond capsuleQuat;
	Vector3d capsuleTrans;

	{
		SCOPED_TRACE("No intersection, box in front of capsule");
		boxQuat = Quaterniond::Identity();
		boxTrans = Vector3d(10.6, 0.0, 0.0);
		capsuleQuat = Quaterniond::Identity();
		capsuleTrans = Vector3d::Zero();
		bool expectedInContact = false;
		doBoxCapsuleTest(box, boxQuat, boxTrans, capsule, capsuleQuat, capsuleTrans, expectedInContact);
	}

	{
		SCOPED_TRACE("No intersection, capsule beyond corner of box");
		boxQuat = Quaterniond::Identity();
		boxTrans = Vector3d::Zero();
		capsuleQuat = Quaterniond::Identity();
		bool expectedInContact = false;
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
		boxQuat = Quaterniond::Identity();
		boxTrans = Vector3d(0.0, -3.6, 0.0);
		capsuleQuat = Quaterniond::Identity();
		capsuleTrans = Vector3d::Zero();
		bool expectedInContact = false;
		doBoxCapsuleTest(box, boxQuat, boxTrans, capsule, capsuleQuat, capsuleTrans, expectedInContact);
	}

	{
		SCOPED_TRACE("Intersection, box intersection with capsule side");
		boxQuat = Quaterniond::Identity();
		boxTrans = Vector3d(1.0 , 0.0, 0.0);
		capsuleQuat = Quaterniond::Identity();
		capsuleTrans = Vector3d::Zero();
		bool expectedInContact = true;
		doBoxCapsuleTest(box, boxQuat, boxTrans, capsule, capsuleQuat, capsuleTrans, expectedInContact);
	}

	{
		SCOPED_TRACE("Intersection, box intersection with upside down capsule");
		boxQuat = Quaterniond::Identity();
		boxTrans = Vector3d(1.0 , 0.0, 0.0);
		capsuleQuat = makeRotationQuaternion(M_PI, Vector3d(0.0, 0.0, 1.0));
		capsuleTrans = Vector3d::Zero();
		bool expectedInContact = true;
		doBoxCapsuleTest(box, boxQuat, boxTrans, capsule, capsuleQuat, capsuleTrans, expectedInContact);
	}

	{
		SCOPED_TRACE("Intersection, box intersection with z-axis capsule");
		boxQuat = Quaterniond::Identity();
		boxTrans = Vector3d(1.0 , 0.0, 0.0);
		capsuleQuat = makeRotationQuaternion(M_PI_2, Vector3d(1.0, 0.0, 0.0));
		capsuleTrans = Vector3d::Zero();
		bool expectedInContact = true;
		doBoxCapsuleTest(box, boxQuat, boxTrans, capsule, capsuleQuat, capsuleTrans, expectedInContact);
	}

	{
		SCOPED_TRACE("Intersection, box intersection with x-axis capsule");
		boxQuat = makeRotationQuaternion(M_PI, Vector3d(0.0, 0.0, 1.0));
		boxTrans = Vector3d(1.0 , 0.0, 0.0);
		capsuleQuat = makeRotationQuaternion(M_PI_2, Vector3d(1.0, 0.0, 0.0));
		capsuleTrans = Vector3d::Zero();
		bool expectedInContact = true;
		doBoxCapsuleTest(box, boxQuat, boxTrans, capsule, capsuleQuat, capsuleTrans, expectedInContact);
	}

	{
		SCOPED_TRACE("Intersection, box intersection with capsule cap");
		boxQuat = makeRotationQuaternion(M_PI_2, Vector3d(0.0, 0.0, 1.0));
		boxTrans = Vector3d(0.1 , 0.0, 0.1);
		capsuleQuat = Quaterniond::Identity();
		capsuleTrans = Vector3d(0.0 , 2.6, 0.0);
		bool expectedInContact = true;
		doBoxCapsuleTest(box, boxQuat, boxTrans, capsule, capsuleQuat, capsuleTrans, expectedInContact);
	}

	{
		SCOPED_TRACE("No intersection, capsule near box corner, but not intersecting");
		boxQuat = Quaterniond::Identity();
		boxTrans = Vector3d(0.0 , 0.0, 0.0);
		capsuleQuat = Quaterniond::Identity();
		capsuleTrans = Vector3d(1.3 , 0.0, 1.3);
		bool expectedInContact = false;
		doBoxCapsuleTest(box, boxQuat, boxTrans, capsule, capsuleQuat, capsuleTrans, expectedInContact);
	}

	{
		SCOPED_TRACE("Intersection, capsule intersecting with box corner");
		boxQuat = Quaterniond::Identity();
		boxTrans = Vector3d(0.0 , 0.0, 0.0);
		capsuleQuat = Quaterniond::Identity();
		capsuleTrans = Vector3d(1.2 , 0.0, 1.2);
		bool expectedInContact = true;
		doBoxCapsuleTest(box, boxQuat, boxTrans, capsule, capsuleQuat, capsuleTrans, expectedInContact);
	}


	{
		SCOPED_TRACE("Intersection, box inside capsule");
		boxQuat = Quaterniond::Identity();
		boxTrans = Vector3d::Zero();
		capsuleQuat = Quaterniond::Identity();
		capsuleTrans = Vector3d::Zero();
		bool expectedInContact = true;
		doBoxCapsuleTest(box, boxQuat, boxTrans, capsule, capsuleQuat, capsuleTrans, expectedInContact);
	}

	{
		SCOPED_TRACE("Intersection, capsule inside box");
		std::shared_ptr<BoxShape> bigBox = std::make_shared<BoxShape>(10.0, 10.0, 10.0);
		boxQuat = makeRotationQuaternion(-M_PI_4, Vector3d(0.0, 1.0, 0.0));
		boxTrans = Vector3d::Zero();
		capsuleQuat = makeRotationQuaternion(M_PI, Vector3d(1.0, 0.0, 0.0));
		capsuleTrans = Vector3d(0.0, 0.0, 0.0);
		bool expectedInContact = true;
		doBoxCapsuleTest(bigBox, boxQuat, boxTrans, capsule, capsuleQuat, capsuleTrans, expectedInContact);
	}

	{
		SCOPED_TRACE("Intersection, capsule bottom at box center");
		boxQuat = Quaterniond::Identity();
		boxTrans = Vector3d::Zero();
		capsuleQuat = Quaterniond::Identity();
		capsuleTrans = Vector3d(0.0, -2.0, 0.0);
		bool expectedInContact = true;
		doBoxCapsuleTest(box, boxQuat, boxTrans, capsule, capsuleQuat, capsuleTrans, expectedInContact);
	}

	{
		SCOPED_TRACE("Intersection, capsule top at box center");
		boxQuat = Quaterniond::Identity();
		boxTrans = Vector3d::Zero();
		capsuleQuat = Quaterniond::Identity();
		capsuleTrans = Vector3d(0.0, 2.0, 0.0);
		bool expectedInContact = true;
		doBoxCapsuleTest(box, boxQuat, boxTrans, capsule, capsuleQuat, capsuleTrans, expectedInContact);
	}

	{
		SCOPED_TRACE("Intersection with box edge, box's point on edge");
		boxQuat = Quaterniond::Identity();
		boxTrans = Vector3d::Zero();
		capsuleQuat = makeRotationQuaternion(0.1, Vector3d::UnitZ().eval());
		capsuleTrans = Vector3d(1.52, 0.0, 0.0);
		bool expectedInContact = true;
		doBoxCapsuleTest(box, boxQuat, boxTrans, capsule, capsuleQuat, capsuleTrans, expectedInContact);
	}

	{
		SCOPED_TRACE("Intersection with box edge, box's point on edge, capsule point along vector towards box point");
		std::shared_ptr<BoxShape> box = std::make_shared<BoxShape>(0.0256, 0.0256, 0.0256);
		std::shared_ptr<CapsuleShape> capsule = std::make_shared<CapsuleShape>(0.01, 0.0063);
		boxQuat = Quaterniond::Identity();
		boxTrans = Vector3d(0.0254, 0.0524, 0.5128);
		capsuleQuat = Quaterniond::Identity();
		capsuleTrans = Vector3d(0.0081837091898594016, 0.074665473951012307, 0.50404931721342927);
		bool expectedInContact = true;
		doBoxCapsuleTest(box, boxQuat, boxTrans, capsule, capsuleQuat, capsuleTrans, expectedInContact);
	}

	{
		SCOPED_TRACE("Intersection with box corner");
		std::shared_ptr<BoxShape> box = std::make_shared<BoxShape>(0.0032, 0.0032, 0.0032);
		std::shared_ptr<CapsuleShape> capsule = std::make_shared<CapsuleShape>(0.01, 0.0063);
		boxQuat = Quaterniond::Identity();
		boxTrans = Vector3d(0.011, 0.038, 0.5496);
		capsuleQuat =
			Quaterniond(0.99814646292568798, 0.0035271245394549833, -0.023780789153701510, 0.055907709742473284);
		capsuleTrans = Vector3d(0.0059124370262071749, 0.031538130383304983, 0.54312746745813301);
		bool expectedInContact = true;
		doBoxCapsuleTest(box, boxQuat, boxTrans, capsule, capsuleQuat, capsuleTrans, expectedInContact);
	}

	{
		SCOPED_TRACE("Another intersection with box corner");
		std::shared_ptr<BoxShape> box =
			std::make_shared<BoxShape>(0.0008, 0.0008, 0.0008);
		std::shared_ptr<CapsuleShape> capsule = std::make_shared<CapsuleShape>(0.01, 0.0063);
		boxQuat = Quaterniond::Identity();
		boxTrans = Vector3d(0.005, 0.052, 0.5308);
		capsuleQuat =
			Quaterniond(0.71851427633922127, 0.00027205941221747750, 0.0021375922639339773, 0.69550887225089708);
		capsuleTrans = Vector3d(0.010224217835903153, 0.058515488684803690, 0.53177563225691493);
		bool expectedInContact = true;
		doBoxCapsuleTest(box, boxQuat, boxTrans, capsule, capsuleQuat, capsuleTrans, expectedInContact);
	}

	{
		SCOPED_TRACE("Intersection with box face, but closest point to box center is outside dilated box");
		std::shared_ptr<BoxShape> box =
			std::make_shared<BoxShape>(0.012800000000000004, 0.012799999999999999, 0.012800000000000034);
		std::shared_ptr<CapsuleShape> capsule = std::make_shared<CapsuleShape>(0.01, 0.0063);
		boxQuat = Quaterniond::Identity();
		boxTrans = Vector3d(0.019000000000000003, 0.045999999999999999, 0.51920000000000011);
		capsuleQuat =
			Quaterniond(0.71552146749248391, -0.00014598153123885886, 0.0013667288118696403, 0.69858939320544333);
		capsuleTrans = Vector3d(0.017905427782122299, 0.058803866737869748, 0.51747490113489192);
		bool expectedInContact = true;
		doBoxCapsuleTest(box, boxQuat, boxTrans, capsule, capsuleQuat, capsuleTrans, expectedInContact);
	}
}


}; // namespace Collision
}; // namespace SurgSim
