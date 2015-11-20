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
#include "SurgSim/Collision/BoxSphereContact.h"
#include "SurgSim/Math/BoxShape.h"
#include "SurgSim/Math/SphereShape.h"

using SurgSim::Math::BoxShape;
using SurgSim::Math::SphereShape;

namespace SurgSim
{
namespace Collision
{

void doBoxSphereTest(std::shared_ptr<BoxShape> box,
					 const Quaterniond& boxQuat,
					 const Vector3d& boxTrans,
					 std::shared_ptr<SphereShape> sphere,
					 const Quaterniond& sphereQuat,
					 const Vector3d& sphereTrans,
					 bool hasContacts = false,
					 double expectedDepth = 0.0,
					 Vector3d expectedNormal = Vector3d::UnitX(),
					 Vector3d expectedPenetrationPoint0 = Vector3d::Zero(),
					 Vector3d expectedPenetrationPoint1 = Vector3d::Zero())
{
	using SurgSim::Math::Geometry::DistanceEpsilon;
	using SurgSim::Math::Geometry::ScalarEpsilon;

	std::shared_ptr<ShapeCollisionRepresentation> boxRep =
		std::make_shared<ShapeCollisionRepresentation>("Collision Box 0");
	boxRep->setShape(box);
	boxRep->setLocalPose(SurgSim::Math::makeRigidTransform(boxQuat, boxTrans));

	std::shared_ptr<ShapeCollisionRepresentation> sphereRep =
		std::make_shared<ShapeCollisionRepresentation>("Collision Sphere 0");
	sphereRep->setShape(sphere);
	sphereRep->setLocalPose(SurgSim::Math::makeRigidTransform(sphereQuat, sphereTrans));

	// Perform collision detection.
	BoxSphereDcdContact calcContact;
	std::shared_ptr<CollisionPair> pair = std::make_shared<CollisionPair>(boxRep, sphereRep);
	calcContact.calculateContact(pair);

	// Compare contact info.
	EXPECT_EQ(hasContacts, pair->hasContacts());
	if (pair->hasContacts())
	{
		std::shared_ptr<Contact> contact = pair->getContacts().front();
		EXPECT_TRUE(eigenEqual(expectedNormal, contact->normal));
		EXPECT_NEAR(expectedDepth, contact->depth, DistanceEpsilon);
		EXPECT_TRUE(contact->penetrationPoints.first.rigidLocalPosition.hasValue());
		EXPECT_TRUE(contact->penetrationPoints.second.rigidLocalPosition.hasValue());

		EXPECT_TRUE(eigenEqual(expectedPenetrationPoint0,
							   contact->penetrationPoints.first.rigidLocalPosition.getValue()));
		EXPECT_TRUE(eigenEqual(expectedPenetrationPoint1,
							   contact->penetrationPoints.second.rigidLocalPosition.getValue()));
	}
}

TEST(BoxSphereContactCalculationTests, UnitTests)
{
	std::shared_ptr<BoxShape> box = std::make_shared<BoxShape>(1.0, 1.0, 1.0);
	std::shared_ptr<SphereShape> sphere = std::make_shared<SphereShape>(1.0);
	SurgSim::Math::Quaterniond boxQuat;
	SurgSim::Math::Vector3d boxTrans;
	SurgSim::Math::Quaterniond sphereQuat;
	SurgSim::Math::Vector3d sphereTrans;
	SurgSim::Math::Quaterniond globalQuat;
	SurgSim::Math::Vector3d globalTrans;

	{
		SCOPED_TRACE("No Intersection");
		boxQuat.setIdentity();
		boxTrans = Vector3d(100.0,0.0,0.0);
		sphereQuat.setIdentity();
		sphereTrans = Vector3d(0.0,0.0,0.0);
		doBoxSphereTest(box, boxQuat, boxTrans, sphere, sphereQuat, sphereTrans, false);
	}

	{
		SCOPED_TRACE("Intersection on top face");
		boxQuat.setIdentity();
		boxTrans = Vector3d(0.0,0.0,0.0);
		sphereQuat.setIdentity();
		sphereTrans = Vector3d(0.0,1.0,0.0);
		globalQuat = SurgSim::Math::makeRotationQuaternion(0.35465, Vector3d(0.3454, 0.78567, 0.234346).normalized());
		globalTrans = Vector3d(24.6,-32.67,87.53);
		boxQuat = globalQuat * boxQuat;
		boxTrans = globalQuat * boxTrans + globalTrans;
		sphereQuat = globalQuat * sphereQuat;
		sphereTrans = globalQuat * sphereTrans + globalTrans;
		doBoxSphereTest(box, boxQuat, boxTrans, sphere, sphereQuat, sphereTrans, true,
						0.5, // depth
						boxQuat * Vector3d(0.0,-1.0,0.0), // normal points into first representation of CollisionPair
						Vector3d(0.0,0.5,0.0), // box penetration point
						Vector3d(0.0,-1.0,0.0)); // sphere penetration point
	}

	{
		SCOPED_TRACE("Sphere center inside box, intersection on top face");
		boxQuat.setIdentity();
		boxTrans = Vector3d(0.0,0.0,0.0);
		sphereQuat.setIdentity();
		sphereTrans = Vector3d(0.0,0.05,0.0);
		globalQuat = SurgSim::Math::makeRotationQuaternion(0.35465, Vector3d(0.3454, 0.78567, 0.234346).normalized());
		globalTrans = Vector3d(24.6,-32.67,87.53);
		boxQuat = globalQuat * boxQuat;
		boxTrans = globalQuat * boxTrans + globalTrans;
		sphereQuat = globalQuat * sphereQuat;
		sphereTrans = globalQuat * sphereTrans + globalTrans;
		doBoxSphereTest(box, boxQuat, boxTrans, sphere, sphereQuat, sphereTrans, true,
			1.45, // depth
			boxQuat * Vector3d(0.0,-1.0,0.0), // normal points into first representation of CollisionPair
			Vector3d(0.0,0.5,0.0), // box penetration point
			Vector3d(0.0,-1.0,0.0)); // sphere penetration point
	}

	{
		SCOPED_TRACE("Intersection on bottom face");
		boxQuat.setIdentity();
		boxTrans = Vector3d(0.0,0.0,0.0);
		sphereQuat.setIdentity();
		sphereTrans = Vector3d(0.3345,-1.2,0.1234);
		globalQuat = SurgSim::Math::makeRotationQuaternion(-0.35465,
														   Vector3d(18.3454, -27.78567, 23.234346).normalized());
		globalTrans = Vector3d(234.6,326.67,987.53);
		boxQuat = globalQuat * boxQuat;
		boxTrans = globalQuat * boxTrans + globalTrans;
		sphereQuat = globalQuat * sphereQuat;
		sphereTrans = globalQuat * sphereTrans + globalTrans;
		doBoxSphereTest(box, boxQuat, boxTrans, sphere, sphereQuat, sphereTrans, true,
						0.3, // depth
						boxQuat * Vector3d(0.0,1.0,0.0), // normal points into first representation of CollisionPair
						Vector3d(0.3345,-0.5,0.1234), // box penetration point
						Vector3d(0.0,1.0,0.0)); // sphere penetration point
	}

	{
		SCOPED_TRACE("Sphere center inside box, intersection on bottom face");
		boxQuat.setIdentity();
		boxTrans = Vector3d(0.0,0.0,0.0);
		sphereQuat.setIdentity();
		sphereTrans = Vector3d(0.3345,-0.4,0.1234);
		globalQuat = SurgSim::Math::makeRotationQuaternion(-0.35465,
			Vector3d(18.3454, -27.78567, 23.234346).normalized());
		globalTrans = Vector3d(234.6,326.67,987.53);
		boxQuat = globalQuat * boxQuat;
		boxTrans = globalQuat * boxTrans + globalTrans;
		sphereQuat = globalQuat * sphereQuat;
		sphereTrans = globalQuat * sphereTrans + globalTrans;
		doBoxSphereTest(box, boxQuat, boxTrans, sphere, sphereQuat, sphereTrans, true,
			1.1, // depth
			boxQuat * Vector3d(0.0,1.0,0.0), // normal points into first representation of CollisionPair
			Vector3d(0.3345,-0.5,0.1234), // box penetration point
			Vector3d(0.0,1.0,0.0)); // sphere penetration point
	}

	{
		SCOPED_TRACE("Intersection on right face");
		boxQuat.setIdentity();
		boxTrans = Vector3d(23.545,3.4321,5.3421);
		sphereQuat.setIdentity();
		sphereTrans = boxTrans + Vector3d(1.2324,-0.2354,0.412);
		globalQuat = SurgSim::Math::makeRotationQuaternion(1.285, Vector3d(23.446, 13.786, 32.254).normalized());
		globalTrans = Vector3d(-249.6,532.67,977.53);
		boxQuat = globalQuat * boxQuat;
		boxTrans = globalQuat * boxTrans + globalTrans;
		sphereQuat = globalQuat * sphereQuat;
		sphereTrans = globalQuat * sphereTrans + globalTrans;
		doBoxSphereTest(box, boxQuat, boxTrans, sphere, sphereQuat, sphereTrans, true,
						0.2676, // depth
						boxQuat * Vector3d(-1.0,0.0,0.0), // normal points into first representation of CollisionPair
						Vector3d(0.5,-0.2354,0.412), // box penetration point
						Vector3d(-1.0,0.0,0.0)); // sphere penetration point
	}

	{
		SCOPED_TRACE("Sphere center inside box, intersection on right face");
		boxQuat.setIdentity();
		boxTrans = Vector3d(23.545,3.4321,5.3421);
		sphereQuat.setIdentity();
		sphereTrans = boxTrans + Vector3d(0.45,-0.2354,0.412);
		globalQuat = SurgSim::Math::makeRotationQuaternion(1.285, Vector3d(23.446, 13.786, 32.254).normalized());
		globalTrans = Vector3d(-249.6,532.67,977.53);
		boxQuat = globalQuat * boxQuat;
		boxTrans = globalQuat * boxTrans + globalTrans;
		sphereQuat = globalQuat * sphereQuat;
		sphereTrans = globalQuat * sphereTrans + globalTrans;
		doBoxSphereTest(box, boxQuat, boxTrans, sphere, sphereQuat, sphereTrans, true,
			1.05, // depth
			boxQuat * Vector3d(-1.0,0.0,0.0), // normal points into first representation of CollisionPair
			Vector3d(0.5,-0.2354,0.412), // box penetration point
			Vector3d(-1.0,0.0,0.0)); // sphere penetration point
	}

	{
		SCOPED_TRACE("Intersection on left face");
		boxQuat.setIdentity();
		boxTrans = Vector3d(876.324,6754.23,7343.76);
		sphereQuat.setIdentity();
		sphereTrans = boxTrans + Vector3d(-1.1223,0.2354,-0.412);
		globalQuat = SurgSim::Math::makeRotationQuaternion(0.276, Vector3d(0.945, 1.532, 0.896).normalized());
		globalTrans = Vector3d(-24.6,32.67,97.53);
		boxQuat = globalQuat * boxQuat;
		boxTrans = globalQuat * boxTrans + globalTrans;
		sphereQuat = globalQuat * sphereQuat;
		sphereTrans = globalQuat * sphereTrans + globalTrans;
		doBoxSphereTest(box, boxQuat, boxTrans, sphere, sphereQuat, sphereTrans, true,
						0.3777, // depth
						boxQuat * Vector3d(1.0,0.0,0.0), // normal points into first representation of CollisionPair
						Vector3d(-0.5,0.2354,-0.412), // box penetration point
						Vector3d(1.0,0.0,0.0)); // sphere penetration point
	}

	{
		SCOPED_TRACE("Sphere center inside box, intersection on left face");
		boxQuat.setIdentity();
		boxTrans = Vector3d(876.324,6754.23,7343.76);
		sphereQuat.setIdentity();
		sphereTrans = boxTrans + Vector3d(-0.3,0.2354,-0.012);
		globalQuat = SurgSim::Math::makeRotationQuaternion(0.276, Vector3d(0.945, 1.532, 0.896).normalized());
		globalTrans = Vector3d(-24.6,32.67,97.53);
		boxQuat = globalQuat * boxQuat;
		boxTrans = globalQuat * boxTrans + globalTrans;
		sphereQuat = globalQuat * sphereQuat;
		sphereTrans = globalQuat * sphereTrans + globalTrans;
		doBoxSphereTest(box, boxQuat, boxTrans, sphere, sphereQuat, sphereTrans, true,
			1.2, // depth
			boxQuat * Vector3d(1.0,0.0,0.0), // normal points into first representation of CollisionPair
			Vector3d(-0.5,0.2354,-0.012), // box penetration point
			Vector3d(1.0,0.0,0.0)); // sphere penetration point
	}

	{
		SCOPED_TRACE("Intersection on front face");
		boxQuat.setIdentity();
		boxTrans = Vector3d(0.3252,-0.64564,0.12345);
		sphereQuat.setIdentity();
		sphereTrans = boxTrans + Vector3d(0.1564,-0.2987,-0.8986);
		globalQuat = SurgSim::Math::makeRotationQuaternion(-1.32, Vector3d(235.67, 215.567, 146.345).normalized());
		globalTrans = Vector3d(224.6,132.67,27.53);
		boxQuat = globalQuat * boxQuat;
		boxTrans = globalQuat * boxTrans + globalTrans;
		sphereQuat = globalQuat * sphereQuat;
		sphereTrans = globalQuat * sphereTrans + globalTrans;
		doBoxSphereTest(box, boxQuat, boxTrans, sphere, sphereQuat, sphereTrans, true,
						0.6014, // depth
						boxQuat * Vector3d(0.0,0.0,1.0), // normal points into first representation of CollisionPair
						Vector3d(0.1564,-0.2987,-0.5), // box penetration point
						Vector3d(0.0,0.0,1.0)); // sphere penetration point
	}

	{
		SCOPED_TRACE("Sphere center inside box, intersection on front face");
		boxQuat.setIdentity();
		boxTrans = Vector3d(0.3252,-0.64564,0.12345);
		sphereQuat.setIdentity();
		sphereTrans = boxTrans + Vector3d(0.1564,-0.2987,-0.3986);
		globalQuat = SurgSim::Math::makeRotationQuaternion(-1.32, Vector3d(235.67, 215.567, 146.345).normalized());
		globalTrans = Vector3d(224.6,132.67,27.53);
		boxQuat = globalQuat * boxQuat;
		boxTrans = globalQuat * boxTrans + globalTrans;
		sphereQuat = globalQuat * sphereQuat;
		sphereTrans = globalQuat * sphereTrans + globalTrans;
		doBoxSphereTest(box, boxQuat, boxTrans, sphere, sphereQuat, sphereTrans, true,
			1.1014, // depth
			boxQuat * Vector3d(0.0,0.0,1.0), // normal points into first representation of CollisionPair
			Vector3d(0.1564,-0.2987,-0.5), // box penetration point
			Vector3d(0.0,0.0,1.0)); // sphere penetration point
	}

	{
		SCOPED_TRACE("Intersection on back face");
		boxQuat.setIdentity();
		boxTrans = Vector3d(24.345,-865.325,46.345);
		sphereQuat.setIdentity();
		sphereTrans = boxTrans + Vector3d(-0.2564,-0.4987,0.7986);
		globalQuat = SurgSim::Math::makeRotationQuaternion(1.2, Vector3d(25.67, -25.567, 16.345).normalized());
		globalTrans = Vector3d(24.6,3243.67,9762.53);
		boxQuat = globalQuat * boxQuat;
		boxTrans = globalQuat * boxTrans + globalTrans;
		sphereQuat = globalQuat * sphereQuat;
		sphereTrans = globalQuat * sphereTrans + globalTrans;
		doBoxSphereTest(box, boxQuat, boxTrans, sphere, sphereQuat, sphereTrans, true,
						0.7014, // depth
						boxQuat * Vector3d(0.0,0.0,-1.0), // normal points into first representation of CollisionPair
						Vector3d(-0.2564,-0.4987,0.5), // box penetration point
						Vector3d(0.0,0.0,-1.0)); // sphere penetration point
	}

	{
		SCOPED_TRACE("Sphere center inside box, intersection on back face");
		boxQuat.setIdentity();
		boxTrans = Vector3d(24.345,-865.325,46.345);
		sphereQuat.setIdentity();
		sphereTrans = boxTrans + Vector3d(-0.2564,-0.3987,0.48);
		globalQuat = SurgSim::Math::makeRotationQuaternion(1.2, Vector3d(25.67, -25.567, 16.345).normalized());
		globalTrans = Vector3d(24.6,3243.67,9762.53);
		boxQuat = globalQuat * boxQuat;
		boxTrans = globalQuat * boxTrans + globalTrans;
		sphereQuat = globalQuat * sphereQuat;
		sphereTrans = globalQuat * sphereTrans + globalTrans;
		doBoxSphereTest(box, boxQuat, boxTrans, sphere, sphereQuat, sphereTrans, true,
			1.02, // depth
			boxQuat * Vector3d(0.0,0.0,-1.0), // normal points into first representation of CollisionPair
			Vector3d(-0.2564,-0.3987,0.5), // box penetration point
			Vector3d(0.0,0.0,-1.0)); // sphere penetration point
	}
}

}; // namespace Collision
}; // namespace SurgSim
