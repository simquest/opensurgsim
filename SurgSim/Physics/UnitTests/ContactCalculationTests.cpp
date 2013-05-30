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

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>

#include <SurgSim/Physics/Actors/RigidActorState.h>
#include <SurgSim/Physics/Actors/RigidShape.h>
#include <SurgSim/Physics/Actors/SphereShape.h>
#include <SurgSim/Physics/CollisionRepresentation.h>
#include <SurgSim/Physics/ContactCalculation.h>
#include <SurgSim/Physics/CollisionPair.h>




using SurgSim::Math::Vector3d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;

::testing::AssertionResult eigenEqual(const Vector3d& left, const Vector3d& right)
{
	if (left.isApprox(right))
	{
		return ::testing::AssertionSuccess();
	}
	else
	{
		return ::testing::AssertionFailure() << "Eigen values not near, expected: " << left << std::endl << " result: " << right;
	}
}

namespace SurgSim
{
namespace Physics
{

class RigidShapeCollisionRepresentation : public CollisionRepresentation
{
public:
	RigidShapeCollisionRepresentation(std::shared_ptr<RigidShape> shape, Quaterniond quat, Vector3d translation) :
		m_shape(shape), m_transform(SurgSim::Math::makeRigidTransform(quat, translation))
	{

	}

	virtual ~RigidShapeCollisionRepresentation() {}

	virtual int getShapeType() const
	{
		return m_shape->getType();
	}

	virtual const std::shared_ptr<SurgSim::Physics::RigidShape> getShape() const
	{
		return m_shape;
	}

	virtual const SurgSim::Math::RigidTransform3d& getLocalToWorldTransform() const
	{
		return m_transform;
	}



private:
	std::shared_ptr<RigidShape> m_shape;
	RigidTransform3d m_transform;
};


namespace {
	std::shared_ptr<RigidShape> shape0 = std::make_shared<SphereShape>(1.0);
	std::shared_ptr<RigidShape> shape1 = std::make_shared<SphereShape>(1.0);

	std::shared_ptr<CollisionRepresentation> rep0 = std::make_shared<RigidShapeCollisionRepresentation>
						(shape0, Quaterniond(), Vector3d(1.0,0.0,0.0));
	std::shared_ptr<CollisionRepresentation> rep1 = std::make_shared<RigidShapeCollisionRepresentation>
						(shape1, Quaterniond(), Vector3d(0.5,0.0,0.0));

	std::shared_ptr<CollisionPair> pair01 = std::make_shared<CollisionPair>(rep0, rep1);
}

std::shared_ptr<CollisionRepresentation> makeSpereRep(const double& radius, 
													  const Quaterniond& rotation = Quaterniond(), 
													  const Vector3d& position = Vector3d())
{
	std::shared_ptr<RigidShape> sphere = std::make_shared<SphereShape>(radius);
	std::shared_ptr<CollisionRepresentation> rep = std::make_shared<RigidShapeCollisionRepresentation>(sphere, rotation, position);
	return rep;
}

std::shared_ptr<CollisionRepresentation> makePlaneRep(const Vector3d& n, const double& d, 
													  const Quaterniond& rotation = Quaterniond(), 
													  const Vector3d& position = Vector3d())
{
	std::shared_ptr<RigidShape> plane = std::make_shared<PlaneShape>(n,d);
	std::shared_ptr<CollisionRepresentation> rep = std::make_shared<RigidShapeCollisionRepresentation>(plane, rotation, position);
	return rep;
}


TEST(CollisionPairTest, InitTest)
{
	// Default Constructor, needs to work for ReuseFactory
	EXPECT_NO_THROW({CollisionPair pair;});

	std::shared_ptr<CollisionRepresentation> rep0 = makeSpereRep(1.0, Quaterniond(), Vector3d(0.0,0.0,0.0));
	std::shared_ptr<CollisionRepresentation> rep1 = makeSpereRep(1.0, Quaterniond(), Vector3d(0.0,1.0,0.0));

	EXPECT_ANY_THROW({CollisionPair pair(rep0, rep0);});
	EXPECT_ANY_THROW({CollisionPair pair(nullptr, rep0);});
	EXPECT_ANY_THROW({CollisionPair pair(nullptr, nullptr);});
	EXPECT_ANY_THROW({CollisionPair pair(rep0, nullptr);});

	ASSERT_NO_THROW({CollisionPair pair(rep0, rep1);});
	CollisionPair pair(rep0,rep1);

	EXPECT_EQ(rep0, pair.getFirst());
	EXPECT_EQ(rep1, pair.getSecond());
	EXPECT_FALSE(pair.hasContacts());
	
}


TEST (ContactCalculationTests, DefaultCalculation)
{
	DefaultContactCalculation calcShouldLog(false);
	EXPECT_NO_THROW(calcShouldLog.calculateContact(pair01));
	EXPECT_FALSE(pair01->hasContacts());

	DefaultContactCalculation calcShouldThrow(true);
	EXPECT_ANY_THROW(calcShouldThrow.calculateContact(pair01));
	EXPECT_FALSE(pair01->hasContacts());
}


void doSphereSphereTest(double r0, Vector3d p0, double r1, Vector3d p1, bool hasContacts, double d)
{
	std::shared_ptr<ContactFactory> factory = std::make_shared<ContactFactory>();
	SphereSphereDcdContact calc(factory);
	std::shared_ptr<CollisionPair> pair = std::make_shared<CollisionPair>(makeSpereRep(r0,Quaterniond(),p0),
																		  makeSpereRep(r1,Quaterniond(),p1));

	calc.calculateContact(pair);
	EXPECT_EQ(hasContacts, pair->hasContacts());
	if(pair->hasContacts())
	{
		std::shared_ptr<Contact> contact = pair->getContacts().front();
		Vector3d dist = (p1 - p0).normalized();
		EXPECT_TRUE(eigenEqual(dist, contact->normal));
		EXPECT_NEAR(d, contact->depth, 1e-10);
	}
}


TEST (ContactCalculationTests, SphereSphereCalculation)
{
	{
		SCOPED_TRACE("No Intersection");
		doSphereSphereTest(0.1, Vector3d(0.0,0.0,0.0), 0.1, Vector3d(1.0,1.0,1.0), false, 0.0);
	}

	{
		SCOPED_TRACE("Intersection");
		doSphereSphereTest(0.5, Vector3d(0.0,0.0,0.0), 0.5, Vector3d(0.5,0,0), true, 0.5);
	}
}

void doSpherePlaneTest(std::shared_ptr<SphereShape> sphere, const Quaterniond& sphereQuat, const Vector3d& sphereTrans,
					   std::shared_ptr<PlaneShape> plane, const Quaterniond& planeQuat, const Vector3d& planeTrans,
					   bool expectedIntersect, const double& expectedDepth = 0 , const Vector3d& expectedNorm = Vector3d())
{
		std::shared_ptr<CollisionRepresentation> planeRep = std::make_shared<RigidShapeCollisionRepresentation>(plane,planeQuat,planeTrans);
		std::shared_ptr<CollisionRepresentation> sphereRep = std::make_shared<RigidShapeCollisionRepresentation>(sphere,sphereQuat,sphereTrans);
		std::shared_ptr<ContactFactory> factory = std::make_shared<ContactFactory>();
		std::shared_ptr<CollisionPair> pair = std::make_shared<CollisionPair>(sphereRep, planeRep);

		SpherePlaneDcdContact calc(factory);
		calc.calculateContact(pair);
		if (expectedIntersect)
		{
			ASSERT_TRUE(pair->hasContacts());
			std::shared_ptr<Contact> contact = pair->getContacts().front();
			EXPECT_NEAR(expectedDepth, contact->depth, 1e-10);
			EXPECT_TRUE(eigenEqual(expectedNorm, contact->normal));
		}
		else
		{
			EXPECT_FALSE(pair->hasContacts());
		}

}

TEST(ContactCalculationTests, SperePlaneCalculation)
{
	std::shared_ptr<PlaneShape> plane = std::make_shared<PlaneShape>(Vector3d(0.0,1.0,0.0),-0.5);
	std::shared_ptr<SphereShape> sphere = std::make_shared<SphereShape>(1.0);

	{
		SCOPED_TRACE("No Intersection, no transformation");
		doSpherePlaneTest(sphere,Quaterniond::Identity(), Vector3d(0.0,2.0,0.0),
						  plane,Quaterniond::Identity(), Vector3d(0.0,0.0,0.0),
						  false);

	}	

	{
 		SCOPED_TRACE("Intersection front, no transformation");
		doSpherePlaneTest(sphere,Quaterniond::Identity(), Vector3d(0.0,1.0,0.0),
						  plane,Quaterniond::Identity(), Vector3d(0.0,0.0,0.0),
						 true, 0.5, Vector3d(0.0,1.0,0.0));
	}

	{
		SCOPED_TRACE("Intersection back, no transformation");
		doSpherePlaneTest(sphere,Quaterniond::Identity(), Vector3d(0.0,0,0.0),
			plane,Quaterniond::Identity(), Vector3d(0.0,0.0,0.0),
			true, 0.5, Vector3d(0.0,-1.0,0.0));
	}

	
}



}; // Physics
}; // SurgSim
