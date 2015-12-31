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

/// \file UpdateCollisionRepresentationsTest.cpp
/// Tests for the UpdateCollisionRepresentations Class

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Math/Shape.h"
#include "SurgSim/Physics/UnitTests/MockObjects.h"
#include "SurgSim/Physics/PhysicsManagerState.h"
#include "SurgSim/Physics/UpdateCollisionRepresentations.h"

using SurgSim::Physics::MockCollisionRepresentation;
using SurgSim::Physics::PhysicsManagerState;
using SurgSim::Physics::UpdateCollisionRepresentations;

TEST(UpdateCollisionRepresentationsTest, Construction)
{
	EXPECT_NO_THROW(UpdateCollisionRepresentations computation(true););
	EXPECT_NO_THROW(UpdateCollisionRepresentations computation(false););
}

TEST(UpdateCollisionRepresentationsTest, Update)
{
	std::shared_ptr<PhysicsManagerState> state = std::make_shared<PhysicsManagerState>();

	// Create collision representations.
	auto collision1 = std::make_shared<MockCollisionRepresentation>("Collision1");
	auto collision2 = std::make_shared<MockCollisionRepresentation>("Collision2");
	auto collision3 = std::make_shared<MockCollisionRepresentation>("Collision3");

	// Setup the state.
	std::vector<std::shared_ptr<SurgSim::Collision::Representation>> collisions;
	collisions.push_back(collision1);
	collisions.push_back(collision2);
	collisions.push_back(collision3);
	state->setCollisionRepresentations(collisions);

	// Setup collision pairs.
	std::vector<std::shared_ptr<SurgSim::Collision::CollisionPair>> pairs;
	auto pair = std::make_shared<SurgSim::Collision::CollisionPair>(collision1, collision2);
	pairs.push_back(pair);
	state->setCollisionPairs(pairs);

	// Test the m_numberOfTimesUpdateCalled before calling update().
	EXPECT_EQ(0, collision1->getNumberOfTimesUpdateCalled());
	EXPECT_EQ(0, collision2->getNumberOfTimesUpdateCalled());
	EXPECT_EQ(0, collision3->getNumberOfTimesUpdateCalled());

	// Set the local active flags.
	collision1->setLocalActive(true);
	collision2->setLocalActive(false);
	collision3->setLocalActive(true);

	// Test computation.update()
	SurgSim::Physics::UpdateCollisionRepresentations computation(false);
	std::shared_ptr<PhysicsManagerState> newState = computation.update(1.0, state);

	// Test the m_numberOfTimesUpdateCalled after calling update().
	EXPECT_EQ(1, collision1->getNumberOfTimesUpdateCalled());
	EXPECT_EQ(0, collision2->getNumberOfTimesUpdateCalled());
	EXPECT_EQ(0, collision3->getNumberOfTimesUpdateCalled());
}