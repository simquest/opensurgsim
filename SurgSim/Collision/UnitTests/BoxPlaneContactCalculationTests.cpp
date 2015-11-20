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
#include "SurgSim/Collision/BoxPlaneContact.h"
#include "SurgSim/Math/Geometry.h"

using SurgSim::Math::BoxShape;
using SurgSim::Math::PlaneShape;
using SurgSim::Math::Geometry::DistanceEpsilon;

namespace SurgSim
{
namespace Collision
{

void doBoxPlaneTest(std::shared_ptr<BoxShape> box,
					const Quaterniond& boxQuat,
					const Vector3d& boxTrans,
					std::shared_ptr<PlaneShape> plane,
					const Quaterniond& planeQuat,
					const Vector3d& planeTrans,
					const int expectedNumberOfContacts,
					const int* expectedBoxIndicesInContacts)
{
	std::shared_ptr<ShapeCollisionRepresentation> boxRep =
		std::make_shared<ShapeCollisionRepresentation>("Collision Box 0");
	boxRep->setShape(box);
	boxRep->setLocalPose(SurgSim::Math::makeRigidTransform(boxQuat, boxTrans));

	std::shared_ptr<ShapeCollisionRepresentation> planeRep =
		std::make_shared<ShapeCollisionRepresentation>("Collision Plane 0");
	planeRep->setShape(plane);
	planeRep->setLocalPose(SurgSim::Math::makeRigidTransform(planeQuat, planeTrans));

	// First calculate the expected contact info.
	std::list<std::shared_ptr<Contact>> expectedContacts;
	if (expectedNumberOfContacts > 0)
	{
		generateBoxPlaneContact(&expectedContacts, expectedNumberOfContacts, expectedBoxIndicesInContacts,
								box, boxTrans, boxQuat, plane, planeTrans, planeQuat);
	}

	// Perform collision detection.
	BoxPlaneDcdContact calcContact;
	std::shared_ptr<CollisionPair> pair = std::make_shared<CollisionPair>(boxRep, planeRep);
	calcContact.calculateContact(pair);

	const Vector3d globalPlaneNormal = planeRep->getPose().linear() * plane->getNormal();
	const Vector3d planeToBox = boxTrans - planeTrans;
	Vector3d nearestPointOnPlane;
	const double distanceBoxPlane = SurgSim::Math::distancePointPlane(planeToBox, globalPlaneNormal,
		plane->getD(), &nearestPointOnPlane);

	const Vector3d boxRadii = box->getSize() / 2.0;
	const double minDepth = -distanceBoxPlane - boxRadii.norm();
	const double maxDepth = -distanceBoxPlane + boxRadii.norm();

	for (auto contact : pair->getContacts())
	{
		EXPECT_LT(-DistanceEpsilon, contact->depth);
		EXPECT_LT(minDepth - DistanceEpsilon, contact->depth);
		EXPECT_GT(maxDepth + DistanceEpsilon, contact->depth);
		EXPECT_TRUE(eigenEqual(globalPlaneNormal, contact->normal));
	}

	// Compare the contact info.
	contactsInfoEqualityTest(expectedContacts, pair->getContacts());
}

TEST(BoxPlaneContactCalculationTests, UnitTests)
{
	std::shared_ptr<BoxShape> box = std::make_shared<BoxShape>(1.0, 1.0, 1.0);
	std::shared_ptr<PlaneShape> plane = std::make_shared<PlaneShape>();
	SurgSim::Math::Quaterniond boxQuat;
	SurgSim::Math::Vector3d boxTrans;
	SurgSim::Math::Quaterniond planeQuat;
	SurgSim::Math::Vector3d planeTrans;
	SurgSim::Math::Quaterniond globalQuat;

	{
		SCOPED_TRACE("No intersection, box in front of rotated plane");
		boxQuat = SurgSim::Math::makeRotationQuaternion(0.5674, Vector3d(0.4332,0.927, 0.13557).normalized());
		boxTrans = Vector3d(3.4535,10.0,350.0);
		planeQuat = SurgSim::Math::makeRotationQuaternion(-M_PI_2, Vector3d(0.0,0.0,1.0));
		planeTrans = Vector3d::Zero();
		int expectedNumberOfContacts = 0;
		int expectedBoxIndicesInContacts[] = {0};
		doBoxPlaneTest(box, boxQuat, boxTrans, plane, planeQuat, planeTrans, expectedNumberOfContacts,
					   expectedBoxIndicesInContacts);
	}

	{
		SCOPED_TRACE("Intersection in front of plane, four contacts, rotated plane");
		boxQuat = SurgSim::Math::makeRotationQuaternion(1.233469, Vector3d(0.91834,0.39687,0.8271).normalized());
		boxTrans = Vector3d(0.5,10.0,350.0);
		planeQuat = boxQuat * SurgSim::Math::makeRotationQuaternion(-M_PI_2, Vector3d(0.0,0.0,1.0));
		planeTrans = boxTrans + boxQuat * Vector3d(-0.5,0.0,0.0);
		int expectedNumberOfContacts = 4;
		int expectedBoxIndicesInContacts[] = {0, 1, 2, 3};
		doBoxPlaneTest(box, boxQuat, boxTrans, plane, planeQuat, planeTrans, expectedNumberOfContacts,
					   expectedBoxIndicesInContacts);
	}

	{
		SCOPED_TRACE("Intersection in front of plane, two contacts, rotated plane");
		globalQuat = SurgSim::Math::makeRotationQuaternion(0.8753, Vector3d(0.235345,0.6754,0.4567).normalized());
		boxQuat = globalQuat * SurgSim::Math::makeRotationQuaternion(-M_PI_4, Vector3d(0.0,0.0,1.0));
		boxTrans = Vector3d(std::sqrt(0.5),230.0,540.0);
		planeQuat = globalQuat * SurgSim::Math::makeRotationQuaternion(-M_PI_2, Vector3d(0.0,0.0,1.0));
		planeTrans = boxTrans + globalQuat * Vector3d(-std::sqrt(0.5),0.0,0.0);
		int expectedNumberOfContacts = 2;
		int expectedBoxIndicesInContacts[] = {0, 1};
		doBoxPlaneTest(box, boxQuat, boxTrans, plane, planeQuat, planeTrans, expectedNumberOfContacts,
					   expectedBoxIndicesInContacts);
	}

	{
		SCOPED_TRACE("Intersection in front of plane, one contact, rotated plane");
		globalQuat = SurgSim::Math::makeRotationQuaternion(-0.3257, Vector3d(-0.4575,-0.8563,0.63457).normalized());
		double angle = -35.264389682754654315377000330019*(M_PI/180.0);
		boxQuat = globalQuat * (SurgSim::Math::makeRotationQuaternion(angle, Vector3d(0.0,1.0,0.0)) *
				 SurgSim::Math::makeRotationQuaternion(-M_PI_4, Vector3d(0.0,0.0,1.0)));
		boxTrans = Vector3d(std::sqrt(0.75),0.0,0.0);
		planeQuat = globalQuat * SurgSim::Math::makeRotationQuaternion(-M_PI_2, Vector3d(0.0,0.0,1.0));
		planeTrans = boxTrans + globalQuat * Vector3d(-std::sqrt(0.75),0.0,0.0);
		int expectedNumberOfContacts = 1;
		int expectedBoxIndicesInContacts[] = {1};
		doBoxPlaneTest(box, boxQuat, boxTrans, plane, planeQuat, planeTrans, expectedNumberOfContacts,
					   expectedBoxIndicesInContacts);
	}

	{
		SCOPED_TRACE("Intersection inside of plane, one contact, rotated plane");
		globalQuat = SurgSim::Math::makeRotationQuaternion(0.3465, Vector3d(54.4575,76.8563,43.63457).normalized());
		double angle = -35.264389682754654315377000330019*(M_PI/180.0);
		boxQuat = globalQuat * (SurgSim::Math::makeRotationQuaternion(angle, Vector3d(0.0,1.0,0.0)) *
				  SurgSim::Math::makeRotationQuaternion(-M_PI_4, Vector3d(0.0,0.0,1.0)));
		boxTrans = Vector3d(std::sqrt(0.73),0.0,0.0);
		planeQuat = globalQuat * SurgSim::Math::makeRotationQuaternion(-M_PI_2, Vector3d(0.0,0.0,1.0));
		planeTrans = boxTrans + globalQuat * Vector3d(-std::sqrt(0.75),0.0,0.0);
		int expectedNumberOfContacts = 1;
		int expectedBoxIndicesInContacts[] = {1};
		doBoxPlaneTest(box, boxQuat, boxTrans, plane, planeQuat, planeTrans, expectedNumberOfContacts,
					   expectedBoxIndicesInContacts);
	}

	{
		SCOPED_TRACE("Intersection in front of plane, two contacts, rotated plane");
		globalQuat = SurgSim::Math::makeRotationQuaternion(-0.8753, Vector3d(-1.235345,1.6754,1.4567).normalized());
		boxQuat = globalQuat * SurgSim::Math::makeRotationQuaternion(-M_PI_4, Vector3d(0.0,0.0,1.0));
		boxTrans = Vector3d(std::sqrt(0.45),230.0,540.0);
		planeQuat = globalQuat * SurgSim::Math::makeRotationQuaternion(-M_PI_2, Vector3d(0.0,0.0,1.0));
		planeTrans = boxTrans + globalQuat * Vector3d(-std::sqrt(0.5),0.0,0.0);
		int expectedNumberOfContacts = 2;
		int expectedBoxIndicesInContacts[] = {0, 1};
		doBoxPlaneTest(box, boxQuat, boxTrans, plane, planeQuat, planeTrans, expectedNumberOfContacts,
					   expectedBoxIndicesInContacts);
	}

	{
		SCOPED_TRACE("Intersection inside of plane, four contacts, rotated plane");
		boxQuat = SurgSim::Math::makeRotationQuaternion(.99763, Vector3d(0.19834,0.93687,0.2871).normalized());
		boxTrans = Vector3d(0.23,10.0,350.0);
		planeQuat = boxQuat * SurgSim::Math::makeRotationQuaternion(-M_PI_2, Vector3d(0.0,0.0,1.0));
		planeTrans = boxTrans + boxQuat * Vector3d(-0.5,0.0,0.0);
		int expectedNumberOfContacts = 4;
		int expectedBoxIndicesInContacts[] = {0, 1, 2, 3};
		doBoxPlaneTest(box, boxQuat, boxTrans, plane, planeQuat, planeTrans, expectedNumberOfContacts,
					   expectedBoxIndicesInContacts);
	}

	{
		SCOPED_TRACE("Intersection inside of plane - case 1, eight contacts, rotated plane");
		globalQuat = SurgSim::Math::makeRotationQuaternion(-0.8753, Vector3d(-1.235345,1.6754,1.4567).normalized());
		boxQuat = globalQuat * SurgSim::Math::makeRotationQuaternion(-M_PI_4, Vector3d(0.0,0.0,1.0));
		boxTrans = Vector3d(0.435,230.0,540.0);
		planeQuat = globalQuat * SurgSim::Math::makeRotationQuaternion(-M_PI_2, Vector3d(0.0,0.0,1.0));
		planeTrans = boxTrans + globalQuat * Vector3d(9.43523,0.0,0.0);
		int expectedNumberOfContacts = 8;
		int expectedBoxIndicesInContacts[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
		doBoxPlaneTest(box, boxQuat, boxTrans, plane, planeQuat, planeTrans, expectedNumberOfContacts,
					   expectedBoxIndicesInContacts);
	}

	{
		SCOPED_TRACE("Intersection inside of plane - case 2, eight contacts, rotated plane");
		globalQuat = SurgSim::Math::makeRotationQuaternion(1.4576, Vector3d(23.45,-98.24,42.46).normalized());
		double angle = -35.264389682754654315377000330019*(M_PI/180.0);
		boxQuat = globalQuat * SurgSim::Math::makeRotationQuaternion(angle, Vector3d(0.0,0.0,1.0)) *
				  SurgSim::Math::makeRotationQuaternion(-M_PI_4, Vector3d(0.0,1.0,0.0));
		boxTrans = Vector3d(0.34,0.0,0.0);
		planeQuat = globalQuat * SurgSim::Math::makeRotationQuaternion(-M_PI_2, Vector3d(0.0,0.0,1.0));
		planeTrans = boxTrans + globalQuat * Vector3d(5.345,0.0,0.0);
		int expectedNumberOfContacts = 8;
		int expectedBoxIndicesInContacts[] = {0, 1, 2, 3, 4, 5, 6, 7};
		doBoxPlaneTest(box, boxQuat, boxTrans, plane, planeQuat, planeTrans, expectedNumberOfContacts,
					   expectedBoxIndicesInContacts);
	}
}

}; // namespace Collision
}; // namespace SurgSim
