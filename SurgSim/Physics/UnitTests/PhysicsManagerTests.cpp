// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
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

/// \file
/// Tests for the PhysicsManager class. Note that PhysicsManagerTest, the test fixture
/// is declared as a friend class in PhysicsManager to make it easier to test the
/// add and removal of components, for this to work correctly PhysicsManagerTest is required
/// to be in the SurgSim::Physics namespace.


#include <gmock/gmock.h>

#include <string>
#include <memory>

#include "SurgSim/Collision/ShapeCollisionRepresentation.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Particles/SphRepresentation.h"
#include "SurgSim/Physics/ConstraintComponent.h"
#include "SurgSim/Physics/DeformableCollisionRepresentation.h"
#include "SurgSim/Physics/PhysicsManager.h"
#include "SurgSim/Physics/Representation.h"
#include "SurgSim/Physics/FixedRepresentation.h"
#include "SurgSim/Physics/UnitTests/MockObjects.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::Framework::Runtime;
using SurgSim::Framework::Component;
using SurgSim::Physics::FixedRepresentation;
using SurgSim::Physics::PhysicsManager;
using SurgSim::Math::Vector3d;


namespace SurgSim
{

namespace Collision
{
class MockContactFilter : public Collision::ContactFilter
{
public:
	MockContactFilter(const std::string& name) : ContactFilter(name) {}
	MOCK_METHOD0(doWakeUp, bool());
	MOCK_METHOD0(doInitialize, bool());
	MOCK_METHOD2(doFilterContacts, void(const std::shared_ptr<Physics::PhysicsManagerState>& state,
										const std::shared_ptr<CollisionPair>& pairs));

};
}

namespace Physics
{

class PhysicsManagerTest : public ::testing::Test
{
public:
	virtual void SetUp()
	{
		physicsManager = std::make_shared<PhysicsManager>();
	}

	virtual void TearDown()
	{
	}


	bool testDoAddComponent(const std::shared_ptr<Component>& component)
	{
		return physicsManager->executeAdditions(component);
	}

	bool testDoRemoveComponent(const std::shared_ptr<Component>& component)
	{
		return physicsManager->executeRemovals(component);
	}

	std::shared_ptr<PhysicsManager> physicsManager;
};

TEST_F(PhysicsManagerTest, InitTest)
{
	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();
	runtime->addManager(physicsManager);
	EXPECT_NO_THROW(runtime->start());
	EXPECT_NO_THROW(runtime->stop());
}

TEST_F(PhysicsManagerTest, AddRemoveRepresentation)
{
	std::shared_ptr<FixedRepresentation> representation1 = std::make_shared<FixedRepresentation>("Rep1");
	std::shared_ptr<FixedRepresentation> representation2 = std::make_shared<FixedRepresentation>("Rep2");

	EXPECT_TRUE(testDoAddComponent(representation1));
	EXPECT_TRUE(testDoAddComponent(representation2));
	EXPECT_FALSE(testDoAddComponent(representation1));

	EXPECT_TRUE(testDoRemoveComponent(representation1));
	EXPECT_FALSE(testDoRemoveComponent(representation1));
	EXPECT_TRUE(testDoRemoveComponent(representation2));
}

TEST_F(PhysicsManagerTest, AddRemoveCollisionRepresentation)
{
	auto representation1 = std::make_shared<Collision::ShapeCollisionRepresentation>("Rep1");
	auto representation2 = std::make_shared<Collision::ShapeCollisionRepresentation>("Rep2");

	EXPECT_TRUE(testDoAddComponent(representation1));
	EXPECT_TRUE(testDoAddComponent(representation2));
	EXPECT_FALSE(testDoAddComponent(representation1));

	EXPECT_TRUE(testDoRemoveComponent(representation1));
	EXPECT_FALSE(testDoRemoveComponent(representation1));
	EXPECT_TRUE(testDoRemoveComponent(representation2));
}

TEST_F(PhysicsManagerTest, AddRemoveContactFilter)
{
	auto filter1 = std::make_shared<Collision::MockContactFilter>("filter1");
	auto filter2 = std::make_shared<Collision::MockContactFilter>("filter2");

	EXPECT_TRUE(testDoAddComponent(filter1));
	EXPECT_TRUE(testDoAddComponent(filter2));
	EXPECT_FALSE(testDoAddComponent(filter1));

	EXPECT_TRUE(testDoRemoveComponent(filter1));
	EXPECT_FALSE(testDoRemoveComponent(filter1));
	EXPECT_TRUE(testDoRemoveComponent(filter2));

}

TEST_F(PhysicsManagerTest, AddRemoveConstraintComponent)
{
	auto constraintComponent1 = std::make_shared<ConstraintComponent>("component1");
	auto constraintComponent2 = std::make_shared<ConstraintComponent>("component2");

	constraintComponent1->setConstraint(
		makeMockConstraint(std::make_shared<MockRepresentation>(), std::make_shared<MockRepresentation>()));
	constraintComponent2->setConstraint(
		makeMockConstraint(std::make_shared<MockRepresentation>(), std::make_shared<MockRepresentation>()));

	EXPECT_TRUE(testDoAddComponent(constraintComponent1));
	EXPECT_TRUE(testDoAddComponent(constraintComponent2));
	EXPECT_FALSE(testDoAddComponent(constraintComponent1));

	EXPECT_TRUE(testDoRemoveComponent(constraintComponent1));
	EXPECT_FALSE(testDoRemoveComponent(constraintComponent1));
	EXPECT_TRUE(testDoRemoveComponent(constraintComponent2));
}

TEST_F(PhysicsManagerTest, AddRemoveParticleRepresentation)
{
	auto representation1 = std::make_shared<Particles::SphRepresentation>("Rep1");
	auto representation2 = std::make_shared<Particles::SphRepresentation>("Rep2");

	EXPECT_TRUE(testDoAddComponent(representation1));
	EXPECT_TRUE(testDoAddComponent(representation2));
	EXPECT_FALSE(testDoAddComponent(representation1));

	EXPECT_TRUE(testDoRemoveComponent(representation1));
	EXPECT_FALSE(testDoRemoveComponent(representation1));
	EXPECT_TRUE(testDoRemoveComponent(representation2));
}

TEST_F(PhysicsManagerTest, SetComputations)
{
	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();
	runtime->addManager(physicsManager);
	EXPECT_NO_THROW(physicsManager->setComputations(createDcdPipeline()));
	EXPECT_NO_THROW(runtime->start());
	EXPECT_ANY_THROW(physicsManager->setComputations(createDcdPipeline()));
}

TEST_F(PhysicsManagerTest, RunCcd)
{
	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();
	runtime->addManager(physicsManager);
	EXPECT_NO_THROW(physicsManager->setComputations(createCcdPipeline()));
	EXPECT_NO_THROW(runtime->start());
	EXPECT_NO_THROW(runtime->stop());
}

TEST_F(PhysicsManagerTest, RunDcd)
{
	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();
	runtime->addManager(physicsManager);
	EXPECT_NO_THROW(physicsManager->setComputations(createDcdPipeline()));
	EXPECT_NO_THROW(runtime->start());
	EXPECT_NO_THROW(runtime->stop());
}


}; // namespace Physics
}; // namespace SurgSim

