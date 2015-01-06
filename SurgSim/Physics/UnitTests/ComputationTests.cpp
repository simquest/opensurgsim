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
#include "SurgSim/Physics/Computation.h"
#include "SurgSim/Physics/Constraint.h"
#include "SurgSim/Physics/ContactConstraintData.h"
#include "SurgSim/Physics/PhysicsManagerState.h"
#include "SurgSim/Physics/RigidCollisionRepresentation.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Physics/RigidRepresentationContact.h"

namespace SurgSim
{
namespace Physics
{

class MockComputation : public Computation
{
public:
	explicit MockComputation(bool doCopyState = false) : Computation(doCopyState)
	{

	}

protected:
	std::shared_ptr<PhysicsManagerState> doUpdate(
		const double& dt,
		const std::shared_ptr<PhysicsManagerState>& state) override
	{
		return state;
	}
};

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

	// Add a representation.
	auto rigid1 = std::make_shared<RigidRepresentation>("rigid1");
	expectedRepresentations.push_back(rigid1);

	// Add a second representation.  This one has a collision representation.
	auto rigid2 = std::make_shared<RigidRepresentation>("rigid2");
	auto collisionRepresentation = std::make_shared<SurgSim::Physics::RigidCollisionRepresentation>("rigid2 collision");
	rigid2->setCollisionRepresentation(collisionRepresentation);
	expectedRepresentations.push_back(rigid2);
	physicsState->setRepresentations(expectedRepresentations);

	// Add a constraint.
	std::vector<std::shared_ptr<Constraint>> expectedConstraints;

	{
		// Create first side of a constraint.
		auto rigid1 = std::make_shared<RigidRepresentation>("rigid1");
		auto rigid1LocalizationTyped = std::make_shared<RigidRepresentationLocalization>();
		rigid1LocalizationTyped->setRepresentation(rigid1);
		std::shared_ptr<Localization> rigid1Localization = rigid1LocalizationTyped;
		auto rigid1Contact = std::make_shared<RigidRepresentationContact>();

		// Create second side of a constraint.
		auto rigid2 = std::make_shared<RigidRepresentation>("rigid2");
		auto rigid2LocalizationTyped = std::make_shared<RigidRepresentationLocalization>();
		rigid2LocalizationTyped->setRepresentation(rigid2);
		std::shared_ptr<Localization> rigid2Localization = rigid2LocalizationTyped;
		auto rigid2Contact = std::make_shared<RigidRepresentationContact>();

		// Create the constraint specific data.
		std::shared_ptr<ContactConstraintData> data = std::make_shared<ContactConstraintData>();

		// Create the constraint.
		auto constraint1 = std::make_shared<Constraint>(data, rigid1Contact, rigid1Localization,
			rigid2Contact, rigid2Localization);

		// Check the active constraints.
		expectedConstraints.push_back(constraint1);
		physicsState->setConstraintGroup(SurgSim::Physics::CONSTRAINT_GROUP_TYPE_CONTACT, expectedConstraints);
	}

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
}

}; // namespace Physics
}; // namespace SurgSim
