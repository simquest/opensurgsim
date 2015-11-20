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
#include "SurgSim/Collision/SphereDoubleSidedPlaneContact.h"
#include "SurgSim/Math/PlaneShape.h"
#include "SurgSim/Math/SphereShape.h"

using SurgSim::Math::Shape;
using SurgSim::Math::SphereShape;
using SurgSim::Math::DoubleSidedPlaneShape;

namespace SurgSim
{
namespace Collision
{

void doSphereDoubleSidedPlaneTest(std::shared_ptr<SphereShape> sphere,
								  const Quaterniond& sphereQuat,
								  const Vector3d& sphereTrans,
								  std::shared_ptr<DoubleSidedPlaneShape> plane,
								  const Quaterniond& planeQuat,
								  const Vector3d& planeTrans,
								  bool expectedIntersect,
								  const double& expectedDepth = 0 ,
								  const Vector3d& expectedNorm = Vector3d::Zero())
{
	using SurgSim::Math::Geometry::ScalarEpsilon;
	using SurgSim::Math::Geometry::DistanceEpsilon;

	std::shared_ptr<ShapeCollisionRepresentation> planeRep =
		std::make_shared<ShapeCollisionRepresentation>("Collision Plane");
	planeRep->setShape(plane);
	planeRep->setLocalPose(SurgSim::Math::makeRigidTransform(planeQuat, planeTrans));

	std::shared_ptr<ShapeCollisionRepresentation> sphereRep =
		std::make_shared<ShapeCollisionRepresentation>("Collision Sphere");
	sphereRep->setShape(sphere);
	sphereRep->setLocalPose(SurgSim::Math::makeRigidTransform(sphereQuat, sphereTrans));

	SphereDoubleSidedPlaneDcdContact calcNormal;
	std::shared_ptr<CollisionPair> pair = std::make_shared<CollisionPair>(sphereRep, planeRep);

	// Again this replicates the way this is calculated in the contact calculation just with different
	// starting values
	Vector3d sphereLocalNormal = sphereQuat.inverse() * expectedNorm;
	Vector3d spherePenetration = -sphereLocalNormal * sphere->getRadius();
	Vector3d planePenetration = -sphereLocalNormal * (sphere->getRadius() - expectedDepth);
	planePenetration = (sphereQuat * planePenetration) + sphereTrans;
	planePenetration = planeQuat.inverse() * (planePenetration - planeTrans);

	calcNormal.calculateContact(pair);
	if (expectedIntersect)
	{
		ASSERT_TRUE(pair->hasContacts());
		std::shared_ptr<Contact> contact = pair->getContacts().front();
		EXPECT_NEAR(expectedDepth, contact->depth, 1e-10);
		EXPECT_TRUE(eigenEqual(expectedNorm, contact->normal));
		EXPECT_TRUE(contact->penetrationPoints.first.rigidLocalPosition.hasValue());
		EXPECT_TRUE(contact->penetrationPoints.second.rigidLocalPosition.hasValue());
		EXPECT_TRUE(eigenEqual(spherePenetration,
							   contact->penetrationPoints.first.rigidLocalPosition.getValue()));
		EXPECT_TRUE(eigenEqual(planePenetration,
							   contact->penetrationPoints.second.rigidLocalPosition.getValue()));
	}
	else
	{
		EXPECT_FALSE(pair->hasContacts());
	}
}

TEST(SphereDoubleSidedPlaneContactCalculationTests, UnitTests)
{
	std::shared_ptr<DoubleSidedPlaneShape> plane = std::make_shared<DoubleSidedPlaneShape>();
	std::shared_ptr<SphereShape> sphere = std::make_shared<SphereShape>(1.0);

	{
		SCOPED_TRACE("No Intersection, no transformation");
		doSphereDoubleSidedPlaneTest(sphere, Quaterniond::Identity(), Vector3d(0.0,2.0,0.0),
									 plane, Quaterniond::Identity(), Vector3d(0.0,0.5,0.0), false);
	}

	{
		SCOPED_TRACE("Intersection front, no transformation");
		doSphereDoubleSidedPlaneTest(sphere, Quaterniond::Identity(), Vector3d(0.0,1.0,0.0),
									 plane,Quaterniond::Identity(), Vector3d(0.0,0.5,0.0),
									 true, 0.5, Vector3d(0.0,1.0,0.0));
	}

	{
		SCOPED_TRACE("Intersection back, no transformation");
		doSphereDoubleSidedPlaneTest(sphere, Quaterniond::Identity(), Vector3d(0.0,0.0,0.0),
									 plane, Quaterniond::Identity(), Vector3d(0.0,0.5,0.0),
									 true, 0.5, Vector3d(0.0,-1.0,0.0));
	}

	{
		SCOPED_TRACE("Intersection front, sphere center on the plane, rotated plane");
		doSphereDoubleSidedPlaneTest(sphere, Quaterniond::Identity(), Vector3d(0.0,0,0.0),
									 plane, SurgSim::Math::makeRotationQuaternion(M_PI_2, Vector3d(1.0,0.0,0.0)),
									 Vector3d(0.0,0.0,0.0), true, 1.0, Vector3d(0.0,0.0,1.0));
	}

	{
		SCOPED_TRACE("Intersection front, rotated Plane");
		doSphereDoubleSidedPlaneTest(sphere, Quaterniond::Identity(), Vector3d(0.0,0.0,0.5),
									 plane, SurgSim::Math::makeRotationQuaternion(M_PI_2, Vector3d(1.0,0.0,0.0)),
									 Vector3d(0.0,0.0,0.0), true, 0.5, Vector3d(0.0,0.0,1.0));
	}

	{
		Vector3d planeTrans(365.321,-342.324,325.324);
		Quaterniond planeQuat = SurgSim::Math::makeRotationQuaternion(1.23456,
																	  Vector3d(0.234,-0.986,0.646).normalized());
		SCOPED_TRACE("Intersection front, rotated plane 2");
		doSphereDoubleSidedPlaneTest(sphere, Quaterniond::Identity(), planeQuat * (Vector3d(0.0,0.5,0.0)) + planeTrans,
									 plane, planeQuat, planeTrans, true, 0.5, planeQuat * Vector3d(0.0, 1.0, 0.0));
	}
}

TEST(SphereDoubleSidedPlaneContactCalculationTests, ShouldFail)
{
	std::shared_ptr<Shape> sphereShape = std::make_shared<SphereShape>(1.0);
	std::shared_ptr<Shape> doubleSidedPlaneShape = std::make_shared<DoubleSidedPlaneShape>();

	std::shared_ptr<ShapeCollisionRepresentation> reps0 =
		std::make_shared<ShapeCollisionRepresentation>("Collision Sphere 0");
	reps0->setShape(sphereShape);
	reps0->setLocalPose(SurgSim::Math::makeRigidTransform(Quaterniond::Identity(), Vector3d(1.0,0.0,0.0)));

	std::shared_ptr<ShapeCollisionRepresentation> repp0 =
		std::make_shared<ShapeCollisionRepresentation>("Collision Plane 0");
	repp0->setShape(doubleSidedPlaneShape);
	repp0->setLocalPose(SurgSim::Math::makeRigidTransform(Quaterniond::Identity(), Vector3d(0.5,0.0,0.0)));

	std::shared_ptr<ShapeCollisionRepresentation> reps1 =
		std::make_shared<ShapeCollisionRepresentation>("Collision Sphere 1");
	reps1->setShape(sphereShape);
	reps1->setLocalPose(SurgSim::Math::makeRigidTransform(Quaterniond::Identity(), Vector3d(1.0,0.0,0.0)));

	std::shared_ptr<ShapeCollisionRepresentation> repp1 =
		std::make_shared<ShapeCollisionRepresentation>("Collision Plane 1");
	repp1->setShape(doubleSidedPlaneShape);
	repp1->setLocalPose(SurgSim::Math::makeRigidTransform(Quaterniond::Identity(), Vector3d(0.5,0.0,0.0)));

	std::shared_ptr<CollisionPair> pairpp = std::make_shared<CollisionPair>(repp0, repp1);
	std::shared_ptr<CollisionPair> pairss = std::make_shared<CollisionPair>(reps0, reps1);

	SphereDoubleSidedPlaneDcdContact contact;

	EXPECT_ANY_THROW(contact.calculateContact(pairpp));
	EXPECT_ANY_THROW(contact.calculateContact(pairss));
}

}; // namespace Collision
}; // namespace SurgSim
