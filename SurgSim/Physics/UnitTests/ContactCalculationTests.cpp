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

TEST (ContactCalculationTests, DefaultCalculation)
{
	DefaultContactCalculation calcShouldLog(false);
	EXPECT_NO_THROW(calcShouldLog.calculateContact(pair01));
	EXPECT_FALSE(pair01->hasContacts());

	DefaultContactCalculation calcShouldThrow(true);
	EXPECT_ANY_THROW(calcShouldThrow.calculateContact(pair01));
	EXPECT_FALSE(pair01->hasContacts());
}

}; // Physics
}; // SurgSim
