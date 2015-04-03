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
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Physics/Constraint.h"
#include "SurgSim/Physics/ContactConstraintData.h"
#include "SurgSim/Physics/PhysicsManagerState.h"
#include "SurgSim/Physics/RigidCollisionRepresentation.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Physics/RigidContact.h"
#include "SurgSim/Physics/UnitTests/MockObjects.h"

namespace SurgSim
{
namespace Physics
{

TEST(ComputationTests, InitTest)
{
	EXPECT_NO_THROW({MockComputation c;});
	MockComputation c;
	EXPECT_FALSE(c.isCopyingState());

	MockComputation d(true);
	EXPECT_TRUE(d.isCopyingState());
}

TEST(ComputationTests, CopyStateTest)
{
	std::shared_ptr<PhysicsManagerState> state0 = std::make_shared<PhysicsManagerState>();

	MockComputation c;
	auto state1 = c.update(1.0, state0);

	EXPECT_EQ(state0.get(), state1.get());

	c.setDoCopyState(true);
	EXPECT_TRUE(c.isCopyingState());

	auto state2 = c.update(1.0, state0);
	EXPECT_NE(state0.get(), state2.get());
}

TEST(ComputationTests, PreparePhysicsState)
{
	auto physicsState = std::make_shared<PhysicsManagerState>();
	EXPECT_EQ(0, physicsState->getActiveConstraints().size());

	// Setup the state.
	std::vector<std::shared_ptr<Representation>> expectedRepresentations;
	std::vector<std::shared_ptr<SurgSim::Collision::Representation>> expectedCollisionRepresentations;

	// Add a representation.
	auto rigid1 = std::make_shared<RigidRepresentation>("rigid1");
	expectedRepresentations.push_back(rigid1);

	// Add a second representation.  This one has a collision representation.
	auto rigid2 = std::make_shared<RigidRepresentation>("rigid2");
	auto collisionRepresentation = std::make_shared<SurgSim::Physics::RigidCollisionRepresentation>("rigid2 collision");
	rigid2->setCollisionRepresentation(collisionRepresentation);
	expectedRepresentations.push_back(rigid2);
	expectedCollisionRepresentations.push_back(collisionRepresentation);
	physicsState->setRepresentations(expectedRepresentations);

	// Add a constraint.
	std::vector<std::shared_ptr<Constraint>> expectedConstraints;
	// Constraint type.
	auto constraintType = SurgSim::Math::MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT;

	{
		// Create first representation.
		auto rigid1 = std::make_shared<RigidRepresentation>("rigid1");
		// Create second representation.
		auto rigid2 = std::make_shared<RigidRepresentation>("rigid2");

		// Create the constraint specific data.
		std::shared_ptr<ContactConstraintData> data = std::make_shared<ContactConstraintData>();

		// Create the constraint.
		expectedConstraints.push_back(std::make_shared<Constraint>(constraintType, data,
			rigid1, SurgSim::DataStructures::Location(SurgSim::Math::Vector3d::Zero()),
			rigid2, SurgSim::DataStructures::Location(SurgSim::Math::Vector3d::Zero())));
		physicsState->setConstraintGroup(SurgSim::Physics::CONSTRAINT_GROUP_TYPE_CONTACT, expectedConstraints);
	}

	// Add a collision representation.
	auto collisionRepresentation2 = std::make_shared<SurgSim::Physics::RigidCollisionRepresentation>("collision2");
	collisionRepresentation2->setLocalActive(false);
	expectedCollisionRepresentations.push_back(collisionRepresentation2);
	physicsState->setCollisionRepresentations(expectedCollisionRepresentations);

	// Call update on Computation.
	MockComputation c;
	physicsState = c.update(0.0, physicsState);

	// Check the active representations list.
	std::vector<std::shared_ptr<Representation>> actualRepresentations;
	actualRepresentations = physicsState->getActiveRepresentations();
	ASSERT_EQ(2, actualRepresentations.size());
	EXPECT_EQ(rigid1, actualRepresentations.front());
	EXPECT_EQ(rigid2, actualRepresentations.back());

	// Check the active constraints list.
	std::vector<std::shared_ptr<Constraint>> actualConstraints;
	actualConstraints = physicsState->getActiveConstraints();
	ASSERT_EQ(1, actualConstraints.size());
	EXPECT_EQ(expectedConstraints.front(), actualConstraints.front());

	// Check the active collisions list.
	std::vector<std::shared_ptr<SurgSim::Collision::Representation>> actualCollisionRepresentations;
	actualCollisionRepresentations = physicsState->getActiveCollisionRepresentations();
	ASSERT_EQ(1, actualCollisionRepresentations.size());
	EXPECT_EQ(collisionRepresentation, actualCollisionRepresentations.front());
}

}; // namespace Physics
}; // namespace SurgSim
