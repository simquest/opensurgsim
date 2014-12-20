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

/// \file
/// Tests for the PhysicsManager class. Note that PhysicsManagerTest, the test fixture
/// is declared as a friend class in PhysicsManager to make it easier to test the
/// add and removal of components, for this to work correctly PhysicsManagerTest is required
/// to be in the SurgSim::Physics namespace.

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/ConstraintComponent.h"
#include "SurgSim/Physics/DeformableCollisionRepresentation.h"
#include "SurgSim/Physics/FixedRepresentation.h"
#include "SurgSim/Physics/PhysicsManager.h"
#include "SurgSim/Physics/Representation.h"
#include "SurgSim/Physics/SolveMlcp.h"
#include "SurgSim/Physics/UnitTests/MockObjects.h"

using SurgSim::Framework::Component;
using SurgSim::Framework::Runtime;
using SurgSim::Math::Vector3d;
using SurgSim::Physics::FixedRepresentation;
using SurgSim::Physics::PhysicsManager;

namespace SurgSim
{
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

	const std::vector<std::shared_ptr<SurgSim::Collision::CollisionPair>>* getExcludedCollisionPairs(
		const SurgSim::Physics::PhysicsManager& physicsManager)
	{
		return &physicsManager.m_excludedCollisionPairs;
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

TEST_F(PhysicsManagerTest, AddRemoveExcludedCollisionPair)
{
	auto physicsManager = std::make_shared<PhysicsManager>();

	auto rep1 = std::make_shared<SurgSim::Physics::DeformableCollisionRepresentation>("rep1");
	auto rep2 = std::make_shared<SurgSim::Physics::DeformableCollisionRepresentation>("rep2");
	auto rep3 = std::make_shared<SurgSim::Physics::DeformableCollisionRepresentation>("rep3");

	auto collisionPairs = getExcludedCollisionPairs(*physicsManager);
	EXPECT_EQ(0u, collisionPairs->size());

	{
		SCOPED_TRACE("Add once.");
		EXPECT_NO_THROW(physicsManager->addExcludedCollisionPair(rep1, rep2));
		EXPECT_EQ(1u, collisionPairs->size());
	}

	{
		SCOPED_TRACE("Double add.");
		EXPECT_NO_THROW(physicsManager->addExcludedCollisionPair(rep1, rep2));
		EXPECT_EQ(1u, collisionPairs->size());
	}

	{
		SCOPED_TRACE("Removal.");
		EXPECT_NO_THROW(physicsManager->removeExcludedCollisionPair(rep1, rep2));
		EXPECT_EQ(0u, collisionPairs->size());
	}

	{
		SCOPED_TRACE("Double removal.");
		EXPECT_NO_THROW(physicsManager->removeExcludedCollisionPair(rep1, rep2));
		EXPECT_EQ(0u, collisionPairs->size());
	}

	{
		SCOPED_TRACE("Adding multiple.");
		EXPECT_NO_THROW(physicsManager->addExcludedCollisionPair(rep1, rep2));
		EXPECT_NO_THROW(physicsManager->addExcludedCollisionPair(rep1, rep3));
		EXPECT_NO_THROW(physicsManager->addExcludedCollisionPair(rep2, rep3));
		EXPECT_EQ(3u, collisionPairs->size());
	}

	{
		SCOPED_TRACE("Removing multiple.");
		EXPECT_NO_THROW(physicsManager->removeExcludedCollisionPair(rep1, rep2));
		EXPECT_NO_THROW(physicsManager->removeExcludedCollisionPair(rep1, rep3));
		EXPECT_NO_THROW(physicsManager->removeExcludedCollisionPair(rep2, rep3));
		EXPECT_EQ(0u, collisionPairs->size());
	}

	{
		SCOPED_TRACE("Add and remove with swapped representations.");
		EXPECT_NO_THROW(physicsManager->addExcludedCollisionPair(rep1, rep2));
		EXPECT_EQ(1u, collisionPairs->size());

		EXPECT_NO_THROW(physicsManager->removeExcludedCollisionPair(rep2, rep1));
		EXPECT_EQ(0u, collisionPairs->size());
	}
}

TEST_F(PhysicsManagerTest, GetSetSolverParameters)
{
	auto physicsManager = std::make_shared<PhysicsManager>();
	SolveMlcp solver;

	EXPECT_EQ(solver.getMaxIterations(), physicsManager->getMaxIterations());
	const int newMaxIterations = physicsManager->getMaxIterations() + 10;
	physicsManager->setMaxIterations(newMaxIterations);
	EXPECT_EQ(newMaxIterations, physicsManager->getMaxIterations());

	EXPECT_EQ(solver.getPrecision(), physicsManager->getPrecision());
	const double newPrecision = physicsManager->getPrecision() * 10.0;
	physicsManager->setPrecision(newPrecision);
	EXPECT_EQ(newPrecision, physicsManager->getPrecision());

	EXPECT_EQ(solver.getContactTolerance(), physicsManager->getContactTolerance());
	const double newContactTolerance = physicsManager->getContactTolerance() * 10.0;
	physicsManager->setContactTolerance(newContactTolerance);
	EXPECT_EQ(newContactTolerance, physicsManager->getContactTolerance());
}

}; // namespace Physics
}; // namespace SurgSim

