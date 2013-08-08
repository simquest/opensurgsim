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
#include <SurgSim/Physics/UnitTests/RepresentationUtilities.h>
#include <SurgSim/Physics/UnitTests/MockCollisionRepresentation.h>
#include <memory>

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>

#include <SurgSim/Physics/RigidRepresentationState.h>
#include <SurgSim/Physics/RigidShape.h>
#include <SurgSim/Physics/SphereShape.h>
#include <SurgSim/Physics/CollisionRepresentation.h>
#include <SurgSim/Physics/ContactCalculation.h>
#include <SurgSim/Physics/CollisionPair.h>

using SurgSim::Math::Vector3d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;

namespace
{
double epsilon = 1e-10;
}

::testing::AssertionResult eigenEqual(const Vector3d& left, const Vector3d& right, double epsilon)
{
	double dist = (left - right).norm();
	if (std::abs(dist) < epsilon)
	{
		return ::testing::AssertionSuccess();
	}
	else
	{
		return ::testing::AssertionFailure() << std::endl << "Vectors not close, expected: " << left.transpose() <<
			   std::endl << " result: " << right.transpose() << std::endl;
	}
}

namespace SurgSim
{
namespace Physics
{


namespace
{
std::shared_ptr<RigidShape> sphereShape = std::make_shared<SphereShape>(1.0);
std::shared_ptr<RigidShape> doubleSidedPlaneShape = std::make_shared<DoubleSidedPlaneShape>();

std::shared_ptr<CollisionRepresentation> rep0 = std::make_shared<MockCollisionRepresentation>
	("TestSphere 1", sphereShape, Quaterniond::Identity(), Vector3d(1.0,0.0,0.0));
std::shared_ptr<CollisionRepresentation> rep1 = std::make_shared<MockCollisionRepresentation>
	("TestSphere 2", sphereShape, Quaterniond::Identity(), Vector3d(0.5,0.0,0.0));

std::shared_ptr<CollisionPair> pair01 = std::make_shared<CollisionPair>(rep0, rep1);
}

TEST(ContactCalculationTests, DefaultCalculation)
{
	DefaultContactCalculation calcShouldLog(false);
	EXPECT_NO_THROW(calcShouldLog.calculateContact(pair01));
	EXPECT_FALSE(pair01->hasContacts());

	DefaultContactCalculation calcShouldThrow(true);
	EXPECT_ANY_THROW(calcShouldThrow.calculateContact(pair01));
	EXPECT_FALSE(pair01->hasContacts());
}


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


TEST(ContactCalculationTests, SphereSphereCalculation)
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
	std::shared_ptr<CollisionRepresentation> planeRep = std::make_shared<MockCollisionRepresentation>(
		"Collision Plane",
		plane,
		planeQuat,
		planeTrans);
	std::shared_ptr<CollisionRepresentation> sphereRep = std::make_shared<MockCollisionRepresentation>(
		"Collision Sphere",
		sphere,
		sphereQuat,
		sphereTrans);

	SphereDoubleSidedPlaneDcdContact calcNormal;
	std::shared_ptr<CollisionPair> pair = std::make_shared<CollisionPair>(sphereRep, planeRep);

	// Again this replicates the way this is calculated in the contact calculation just with different
	// starting values
	Vector3d spherePenetration = sphereTrans - expectedNorm * sphere->getRadius();
	Vector3d planePenetration = sphereTrans - expectedNorm * (sphere->getRadius() - expectedDepth);

	calcNormal.calculateContact(pair);
	if (expectedIntersect)
	{
		ASSERT_TRUE(pair->hasContacts());
		std::shared_ptr<Contact> contact = pair->getContacts().front();
		EXPECT_NEAR(expectedDepth, contact->depth, 1e-10);
		EXPECT_TRUE(eigenEqual(expectedNorm, contact->normal, epsilon));
		EXPECT_TRUE(contact->penetrationPoints.first.globalPosition.hasValue());
		EXPECT_TRUE(contact->penetrationPoints.second.globalPosition.hasValue());
		EXPECT_TRUE(eigenEqual(spherePenetration,
							   contact->penetrationPoints.first.globalPosition.getValue(),
							   epsilon));
		EXPECT_TRUE(eigenEqual(planePenetration,
							   contact->penetrationPoints.second.globalPosition.getValue(),
							   epsilon));
	}
	else
	{
		EXPECT_FALSE(pair->hasContacts());
	}
}

TEST(ContactCalculationTests, SphereDoubleSidedPlaneCalculation)
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
		Quaterniond planeQuat = SurgSim::Math::makeRotationQuaternion(1.23456, Vector3d(0.234,-0.986,0.646).normalized());
		SCOPED_TRACE("Intersection front, rotated plane 2");
		doSphereDoubleSidedPlaneTest(sphere, Quaterniond::Identity(), planeQuat * (Vector3d(0.0,0.5,0.0)) + planeTrans, plane,
									 planeQuat, planeTrans, true, 0.5, planeQuat * Vector3d(0.0, 1.0, 0.0));
	}
}

TEST(ContactCalculationTests, DoubleSidedPlaneSphereShouldFail)
{
	std::shared_ptr<CollisionRepresentation> reps0 = std::make_shared<MockCollisionRepresentation>(
		"Collision Sphere 0",
		sphereShape,
		Quaterniond::Identity(),
		Vector3d(1.0,0.0,0.0));

	std::shared_ptr<CollisionRepresentation> repp0 = std::make_shared<MockCollisionRepresentation>(
		"Collision Plane 0",
		doubleSidedPlaneShape,
		Quaterniond::Identity(),
		Vector3d(0.5,0.0,0.0));

	std::shared_ptr<CollisionRepresentation> reps1 = std::make_shared<MockCollisionRepresentation>(
		"Collision Sphere 1",
		sphereShape,
		Quaterniond::Identity(),
		Vector3d(1.0,0.0,0.0));

	std::shared_ptr<CollisionRepresentation> repp1 = std::make_shared<MockCollisionRepresentation>(
		"Collision Plane 1",
		doubleSidedPlaneShape,
		Quaterniond::Identity(),
		Vector3d(0.5,0.0,0.0));

	std::shared_ptr<CollisionPair> pairpp = std::make_shared<CollisionPair>(repp0, repp1);
	std::shared_ptr<CollisionPair> pairss = std::make_shared<CollisionPair>(reps0, reps1);

	SphereDoubleSidedPlaneDcdContact contact;

	EXPECT_ANY_THROW(contact.calculateContact(pairpp));
	EXPECT_ANY_THROW(contact.calculateContact(pairss));
}

void doSpherePlaneTest(std::shared_ptr<SphereShape> sphere,
					   const Quaterniond& sphereQuat,
					   const Vector3d& sphereTrans,
					   std::shared_ptr<PlaneShape> plane,
					   const Quaterniond& planeQuat,
					   const Vector3d& planeTrans,
					   bool expectedIntersect,
					   const double& expectedDepth = 0,
					   const Vector3d& expectedNorm = Vector3d::Zero())
{
	std::shared_ptr<CollisionRepresentation> planeRep =
		std::make_shared<MockCollisionRepresentation>("Plane Shape",plane,planeQuat,planeTrans);
	std::shared_ptr<CollisionRepresentation> sphereRep =
		std::make_shared<MockCollisionRepresentation>("Sphere Shape",sphere,sphereQuat,sphereTrans);

	SpherePlaneDcdContact calcNormal;
	std::shared_ptr<CollisionPair> pair = std::make_shared<CollisionPair>(sphereRep, planeRep);

	// Again this replicates the way this is calculated in the contact calculation just with different
	// starting values
	Vector3d spherePenetration = sphereTrans - expectedNorm * sphere->getRadius();
	Vector3d planePenetration = sphereTrans - expectedNorm * (sphere->getRadius() - expectedDepth);

	calcNormal.calculateContact(pair);
	if (expectedIntersect)
	{
		ASSERT_TRUE(pair->hasContacts());
		std::shared_ptr<Contact> contact = pair->getContacts().front();
		EXPECT_NEAR(expectedDepth, contact->depth, 1e-10);
		EXPECT_TRUE(eigenEqual(expectedNorm, contact->normal, epsilon));
		EXPECT_TRUE(contact->penetrationPoints.first.globalPosition.hasValue());
		EXPECT_TRUE(contact->penetrationPoints.second.globalPosition.hasValue());
		EXPECT_TRUE(eigenEqual(spherePenetration,
							   contact->penetrationPoints.first.globalPosition.getValue(),
							   epsilon));
		EXPECT_TRUE(eigenEqual(planePenetration,
							   contact->penetrationPoints.second.globalPosition.getValue(),
							   epsilon));
	}
	else
	{
		EXPECT_FALSE(pair->hasContacts());
	}
}

TEST(ContactCalculationTests, SpherePlaneCalculation)
{
	std::shared_ptr<PlaneShape> plane = std::make_shared<PlaneShape>();
	std::shared_ptr<SphereShape> sphere = std::make_shared<SphereShape>(1.0);

	{
		SCOPED_TRACE("No Intersection, no transformation");
		doSpherePlaneTest(sphere, Quaterniond::Identity(), Vector3d(0.0,2.0,0.0),
						  plane, Quaterniond::Identity(), Vector3d(0.0,0.5,0.0), false);
	}

	{
		SCOPED_TRACE("Intersection front, no transformation");
		doSpherePlaneTest(sphere, Quaterniond::Identity(), Vector3d(0.0,1.0,0.0),
						  plane,Quaterniond::Identity(), Vector3d(0.0,0.5,0.0),
						  true, 0.5, Vector3d(0.0,1.0,0.0));
	}

	{
		SCOPED_TRACE("Intersection back, no transformation");
		doSpherePlaneTest(sphere, Quaterniond::Identity(), Vector3d(0.0,0.0,0.0),
						  plane, Quaterniond::Identity(), Vector3d(0.0,0.5,0.0),
						  true, 1.5, Vector3d(0.0,1.0,0.0));
	}

	{
		SCOPED_TRACE("Intersection front, sphere center on the plane, rotated plane");
		doSpherePlaneTest(sphere, Quaterniond::Identity(), Vector3d(0.0,0,0.0),
						  plane, SurgSim::Math::makeRotationQuaternion(M_PI_2, Vector3d(1.0,0.0,0.0)),
						  Vector3d(0.0,0.0,0.0), true, 1.0, Vector3d(0.0,0.0,1.0));
	}

	{
		SCOPED_TRACE("Intersection front, rotated Plane");
		doSpherePlaneTest(sphere, Quaterniond::Identity(), Vector3d(0.0,0.0,0.5),
						  plane, SurgSim::Math::makeRotationQuaternion(M_PI_2, Vector3d(1.0,0.0,0.0)),
						  Vector3d(0.0,0.0,0.0), true, 0.5, Vector3d(0.0,0.0,1.0));
	}
}

void doCapsuleSphereTest(double capsuleHeight, double capsuleRadius,
						 const Vector3d& capsulePosition, const Quaterniond& capsuleQuat,
						 double sphereRadius, const Vector3d& spherePosition, const Quaterniond& sphereQuat,
						 bool hasContacts, double depth,
						 const Vector3d& sphereProjection = Vector3d::Zero(),
						 const Vector3d& expectedNorm = Vector3d::Zero())
{
	std::shared_ptr<CollisionPair> pair = std::make_shared<CollisionPair>(
		makeCapsuleRepresentation(nullptr, capsuleHeight, capsuleRadius, capsuleQuat, capsulePosition),
		makeSphereRepresentation(nullptr, sphereRadius, sphereQuat, spherePosition));

	CapsuleSphereDcdContact calc;
	calc.calculateContact(pair);
	EXPECT_EQ(hasContacts, pair->hasContacts());

	if (pair->hasContacts())
	{
		std::shared_ptr<Contact> contact(pair->getContacts().front());

		EXPECT_TRUE(eigenEqual(expectedNorm, contact->normal, epsilon));
		EXPECT_NEAR(depth, contact->depth, epsilon);
		EXPECT_TRUE(contact->penetrationPoints.first.globalPosition.hasValue());
		EXPECT_TRUE(contact->penetrationPoints.second.globalPosition.hasValue());

		Vector3d penetrationPoint0(sphereProjection - expectedNorm * capsuleRadius);
		Vector3d penetrationPoint1(spherePosition + expectedNorm * sphereRadius);
		EXPECT_TRUE(eigenEqual(penetrationPoint0, contact->penetrationPoints.first.globalPosition.getValue(), epsilon));
		EXPECT_TRUE(eigenEqual(penetrationPoint1, contact->penetrationPoints.second.globalPosition.getValue(),epsilon));
	}
}


TEST(ContactCalculationTests, CapsuleSphereCalculation)
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
							Vector3d(-0.05, 0.0, 0.0), Vector3d(1.0, 0.0, 0.0));
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
							sphereProjection, expectedNormal);

	}
}
}; // Physics
}; // SurgSim
