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

#include "../CollisionRepresentation.h"
#include "../Actors/RigidActorState.h"
#include "../Actors/RigidShape.h"
#include "../ContactCalculation.h"
#include "../Actors/SphereShape.h"
#include "../CollisionPair.h"




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
	RigidShapeCollisionRepresentation(std::shared_ptr<RigidShape> shape, Vector3d translation) :
		m_shape(shape), m_transform(SurgSim::Math::makeRigidTransform(Quaterniond(), translation))
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

	std::shared_ptr<CollisionRepresentation> rep0 = std::make_shared<RigidShapeCollisionRepresentation>(shape0, Vector3d(1.0,0.0,0.0));
	std::shared_ptr<CollisionRepresentation> rep1 = std::make_shared<RigidShapeCollisionRepresentation>(shape1, Vector3d(0.5,0.0,0.0));

	std::shared_ptr<CollisionPair> pair01 = std::make_shared<CollisionPair>(rep0, rep1);
}

std::shared_ptr<CollisionRepresentation> makeSpereRep(const double& radius, const Vector3d& position)
{
	std::shared_ptr<RigidShape> sphere = std::make_shared<SphereShape>(radius);
	std::shared_ptr<CollisionRepresentation> rep = std::make_shared<RigidShapeCollisionRepresentation>
		(sphere, position);
	return rep;
}


TEST(CollisionPairTest, InitTest)
{
	// Default Constructor, needs to work for ReuseFactory
	EXPECT_NO_THROW({CollisionPair pair;});

	std::shared_ptr<CollisionRepresentation> rep0 = makeSpereRep(1.0, Vector3d(0.0,0.0,0.0));
	std::shared_ptr<CollisionRepresentation> rep1 = makeSpereRep(1.0, Vector3d(0.0,1.0,0.0));

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
	std::shared_ptr<CollisionPair> pair = std::make_shared<CollisionPair>(makeSpereRep(r0,p0),
										makeSpereRep(r1,p1));

	calc.calculateContact(pair);
	EXPECT_EQ(hasContacts, pair->hasContacts());
	if(pair->hasContacts())
	{
		std::shared_ptr<Contact> contact = pair->getContacts().front();
		Vector3d dist = (p1 - p0).normalized();
		EXPECT_TRUE(eigenEqual(dist, contact->normal));
		// Verify the contact point ... 
		
		Vector3d expected = p1 + contact->normal * (r0 + r1);
		Vector3d moved = p0 + contact->normal * contact->depth;
		EXPECT_TRUE(eigenEqual(expected, moved));
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
		doSphereSphereTest(0.5, Vector3d(0.0,0.0,0.0), 0.5, Vector3d(0.5,0,0), true, 0.25);
	}
}



}; // Physics
}; // SurgSim
