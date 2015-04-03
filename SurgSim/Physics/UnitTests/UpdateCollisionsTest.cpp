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

/// \file UpdateCollisionsTest.cpp
/// Tests for the UpdateCollisions Class

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Math/Shape.h"
#include "SurgSim/Physics/UnitTests/MockObjects.h"
#include "SurgSim/Physics/PhysicsManagerState.h"
#include "SurgSim/Physics/UpdateCollisions.h"

using SurgSim::Physics::MockCollisionRepresentation;
using SurgSim::Physics::PhysicsManagerState;
using SurgSim::Physics::UpdateCollisions;

TEST(UpdateCollisionsTest, Construction)
{
	EXPECT_NO_THROW(UpdateCollisions computation(true););
	EXPECT_NO_THROW(UpdateCollisions computation(false););
}

TEST(UpdateCollisionsTest, Update)
{
	std::shared_ptr<PhysicsManagerState> state = std::make_shared<PhysicsManagerState>();

	// Create two collision representations.
	auto collision1 = std::make_shared<MockCollisionRepresentation>("Collision1");
	auto collision2 = std::make_shared<MockCollisionRepresentation>("Collision2");

	// Setup the state.
	std::vector<std::shared_ptr<SurgSim::Collision::Representation>> collisions;
	collisions.push_back(collision1);
	collisions.push_back(collision2);
	state->setCollisionRepresentations(collisions);

	// Test the m_numberOfTimesUpdateCalled before calling update().
	EXPECT_EQ(0, collision1->getNumberOfTimesUpdateCalled());
	EXPECT_EQ(0, collision2->getNumberOfTimesUpdateCalled());

	// Set the local active flags.
	collision1->setLocalActive(true);
	collision2->setLocalActive(false);

	// Test compuation.update()
	SurgSim::Physics::UpdateCollisions computation(false);
	std::shared_ptr<PhysicsManagerState> newState = computation.update(1.0, state);

	// Test the m_numberOfTimesUpdateCalled before calling update().
	EXPECT_EQ(1, collision1->getNumberOfTimesUpdateCalled());
	EXPECT_EQ(0, collision2->getNumberOfTimesUpdateCalled());
}