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
/// Tests for the PhysicsManagerState class.

#include <gtest/gtest.h>

#include <string>
#include <memory>
#include <unordered_map>
#include <vector>

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Physics/Constraint.h"
#include "SurgSim/Physics/ContactConstraintData.h"
#include "SurgSim/Physics/MlcpMapping.h"
#include "SurgSim/Physics/PhysicsManagerState.h"
#include "SurgSim/Physics/RigidCollisionRepresentation.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Physics/RigidRepresentationContact.h"

using SurgSim::Physics::Constraint;
using SurgSim::Physics::ContactConstraintData;
using SurgSim::Physics::Localization;
using SurgSim::Physics::MlcpMapping;
using SurgSim::Physics::PhysicsManagerState;
using SurgSim::Physics::Representation;
using SurgSim::Physics::RigidCollisionRepresentation;
using SurgSim::Physics::RigidRepresentation;
using SurgSim::Physics::RigidRepresentationContact;
using SurgSim::Physics::RigidRepresentationLocalization;

TEST(PhysicsManagerStateTest, SetGetRigidRepresentations)
{
	auto physicsState = std::make_shared<PhysicsManagerState>();
	std::vector<std::shared_ptr<Representation>> expectedRepresentations;
	std::vector<std::shared_ptr<Representation>> actualRepresentations;
	std::unordered_map<std::shared_ptr<SurgSim::Collision::Representation>,
		std::shared_ptr<Representation>> actualCollisionsToPhysicsMap;

	// Add a representation.
	auto rigid1 = std::make_shared<RigidRepresentation>("rigid1");
	expectedRepresentations.push_back(rigid1);
	physicsState->setRepresentations(expectedRepresentations);
	actualRepresentations = physicsState->getRepresentations();
	ASSERT_EQ(1, actualRepresentations.size());
	EXPECT_EQ(rigid1, actualRepresentations.back());
	actualCollisionsToPhysicsMap = physicsState->getCollisionToPhysicsMap();
	EXPECT_EQ(0, actualCollisionsToPhysicsMap.size());

	// Add a second representation.  This one has a collision representation.
	auto rigid2 = std::make_shared<RigidRepresentation>("rigid2");
	auto collisionRepresentation = std::make_shared<SurgSim::Physics::RigidCollisionRepresentation>("rigid2 collision");
	rigid2->setCollisionRepresentation(collisionRepresentation);
	expectedRepresentations.push_back(rigid2);
	physicsState->setRepresentations(expectedRepresentations);
	actualRepresentations = physicsState->getRepresentations();
	ASSERT_EQ(2, actualRepresentations.size());
	EXPECT_EQ(rigid2, actualRepresentations.back());

	// check the collisionsToPhysicsMap
	actualCollisionsToPhysicsMap = physicsState->getCollisionToPhysicsMap();
	ASSERT_EQ(1, actualCollisionsToPhysicsMap.size());
	EXPECT_EQ(rigid2, actualCollisionsToPhysicsMap[collisionRepresentation]);
}

TEST(PhysicsManagerStateTest, SetGetRepresentationsMapping)
{
	auto physicsState = std::make_shared<PhysicsManagerState>();
	MlcpMapping<Representation> expectedRepresentationsIndexMapping;
	MlcpMapping<Representation> actualRepresentationsIndexMapping;

	// Add a representation.
	auto rigid1 = std::make_shared<RigidRepresentation>("rigid1");
	expectedRepresentationsIndexMapping.setValue(rigid1.get(), 13);
	physicsState->setRepresentationsMapping(expectedRepresentationsIndexMapping);
	actualRepresentationsIndexMapping = physicsState->getRepresentationsMapping();
	std::shared_ptr<Representation> rigid1AsRepresentation = rigid1;
	EXPECT_EQ(13, actualRepresentationsIndexMapping.getValue(rigid1AsRepresentation.get()));

	// Add a second representation.  This one has a collision representation.
	auto rigid2 = std::make_shared<RigidRepresentation>("rigid2");
	auto collisionRepresentation = std::make_shared<SurgSim::Physics::RigidCollisionRepresentation>("rigid2 collision");
	rigid2->setCollisionRepresentation(collisionRepresentation);
	expectedRepresentationsIndexMapping.setValue(rigid2.get(), 17);
	physicsState->setRepresentationsMapping(expectedRepresentationsIndexMapping);
	actualRepresentationsIndexMapping = physicsState->getRepresentationsMapping();
	std::shared_ptr<Representation> rigid2AsRepresentation = rigid2;
	EXPECT_EQ(17, actualRepresentationsIndexMapping.getValue(rigid2AsRepresentation.get()));
}

TEST(PhysicsManagerStateTest, SetGetActiveRepresentations)
{
	auto physicsState = std::make_shared<PhysicsManagerState>();
	std::vector<std::shared_ptr<Representation>> expectedRepresentations;
	std::vector<std::shared_ptr<Representation>> actualRepresentations;

	// Add a representation.
	auto rigid1 = std::make_shared<RigidRepresentation>("rigid1");
	expectedRepresentations.push_back(rigid1);
	physicsState->setActiveRepresentations(expectedRepresentations);

	// Filter the active representations and test.
	actualRepresentations = physicsState->getActiveRepresentations();
	ASSERT_EQ(1, actualRepresentations.size());
	EXPECT_EQ(rigid1, actualRepresentations.back());

	// Add a second representation.  This one has a collision representation.
	auto rigid2 = std::make_shared<RigidRepresentation>("rigid2");
	auto collisionRepresentation = std::make_shared<SurgSim::Physics::RigidCollisionRepresentation>("rigid2 collision");
	rigid2->setCollisionRepresentation(collisionRepresentation);
	expectedRepresentations.push_back(rigid2);
	physicsState->setActiveRepresentations(expectedRepresentations);

	// Filter the active representations and test.
	actualRepresentations = physicsState->getActiveRepresentations();
	ASSERT_EQ(2, actualRepresentations.size());
	EXPECT_EQ(rigid2, actualRepresentations.back());
}

TEST(PhysicsManagerStateTest, SetGetCollisionRepresentations)
{
	auto physicsState = std::make_shared<PhysicsManagerState>();
	std::vector<std::shared_ptr<SurgSim::Collision::Representation>> expectedRepresentations;
	std::vector<std::shared_ptr<SurgSim::Collision::Representation>> actualRepresentations;

	// Add a collision representation.
	std::shared_ptr<SurgSim::Collision::Representation> collision1 =
		std::make_shared<RigidCollisionRepresentation>("collision1");

	expectedRepresentations.push_back(collision1);
	physicsState->setCollisionRepresentations(expectedRepresentations);
	actualRepresentations = physicsState->getCollisionRepresentations();
	ASSERT_EQ(1, actualRepresentations.size());
	EXPECT_EQ(collision1, actualRepresentations.back());

	// Add a second collision representation.
	std::shared_ptr<SurgSim::Collision::Representation> collision2 =
		std::make_shared<RigidCollisionRepresentation>("collision2");
	expectedRepresentations.push_back(collision2);
	physicsState->setCollisionRepresentations(expectedRepresentations);
	actualRepresentations = physicsState->getCollisionRepresentations();
	ASSERT_EQ(2, actualRepresentations.size());
	EXPECT_EQ(collision2, actualRepresentations.back());
}

TEST(PhysicsManagerStateTest, SetGetCollisionPairs)
{
	auto physicsState = std::make_shared<PhysicsManagerState>();
	std::vector<std::shared_ptr<SurgSim::Collision::CollisionPair>> expectedPairs;
	std::vector<std::shared_ptr<SurgSim::Collision::CollisionPair>> actualPairs;

	// Add a collision pair.
	auto pair1 = std::make_shared<SurgSim::Collision::CollisionPair>();
	expectedPairs.push_back(pair1);
	physicsState->setCollisionPairs(expectedPairs);
	actualPairs = physicsState->getCollisionPairs();
	ASSERT_EQ(1, actualPairs.size());
	EXPECT_EQ(pair1, actualPairs.back());

	// Add a second collision representation.
	auto pair2 = std::make_shared<SurgSim::Collision::CollisionPair>();
	expectedPairs.push_back(pair2);
	physicsState->setCollisionPairs(expectedPairs);
	actualPairs = physicsState->getCollisionPairs();
	ASSERT_EQ(2, actualPairs.size());
	EXPECT_EQ(pair2, actualPairs.back());
}

TEST(PhysicsManagerStateTest, SetGetConstraintGroup)
{
	auto physicsState = std::make_shared<PhysicsManagerState>();
	std::vector<std::shared_ptr<Constraint>> expectedConstraints;
	std::vector<std::shared_ptr<Constraint>> actualConstraints;

	// We need a populated constraint to check the constraintsIndexMapping.
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

	// Check the constraintGroup.
	expectedConstraints.push_back(constraint1);
	physicsState->setConstraintGroup(SurgSim::Physics::CONSTRAINT_GROUP_TYPE_CONTACT, expectedConstraints);
	actualConstraints = physicsState->getConstraintGroup(SurgSim::Physics::CONSTRAINT_GROUP_TYPE_CONTACT);
	ASSERT_EQ(1, actualConstraints.size());
	EXPECT_EQ(constraint1, actualConstraints.back());

	// Create a second constraint.
	auto constraint2 = std::make_shared<Constraint>(data, rigid1Contact, rigid1Localization,
		rigid2Contact, rigid2Localization);

	// Check the constraintGroup.
	expectedConstraints.push_back(constraint2);
	physicsState->setConstraintGroup(SurgSim::Physics::CONSTRAINT_GROUP_TYPE_CONTACT, expectedConstraints);
	actualConstraints = physicsState->getConstraintGroup(SurgSim::Physics::CONSTRAINT_GROUP_TYPE_CONTACT);
	ASSERT_EQ(2, actualConstraints.size());
	EXPECT_EQ(constraint2, actualConstraints.back());
}

TEST(PhysicsManagerStateTest, SetGetConstraintsMapping)
{
	auto physicsState = std::make_shared<PhysicsManagerState>();
	MlcpMapping<Constraint> expectedConstraintsIndexMapping;
	MlcpMapping<Constraint> actualConstraintsIndexMapping;

	// We need a populated constraint to check the constraintsIndexMapping.
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

	// Check the constraintGroup.
	expectedConstraintsIndexMapping.setValue(constraint1.get(), 5);
	physicsState->setConstraintsMapping(expectedConstraintsIndexMapping);
	actualConstraintsIndexMapping = physicsState->getConstraintsMapping();
	std::shared_ptr<Constraint> constraint1AsConstraint = constraint1;
	ASSERT_EQ(5, actualConstraintsIndexMapping.getValue(constraint1AsConstraint.get()));
}

TEST(PhysicsManagerStateTest, SetGetActiveConstraints)
{
	auto physicsState = std::make_shared<PhysicsManagerState>();
	std::vector<std::shared_ptr<Constraint>> expectedConstraints;
	std::vector<std::shared_ptr<Constraint>> actualConstraints;

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
	physicsState->setActiveConstraints(expectedConstraints);
	actualConstraints = physicsState->getActiveConstraints();
	ASSERT_EQ(1, actualConstraints.size());
	EXPECT_EQ(constraint1, actualConstraints.back());

	// Create a second constraint.
	auto constraint2 = std::make_shared<Constraint>(data, rigid1Contact, rigid1Localization,
		rigid2Contact, rigid2Localization);

	// Check the active constraints.
	expectedConstraints.push_back(constraint2);
	physicsState->setActiveConstraints(expectedConstraints);
	actualConstraints = physicsState->getActiveConstraints();
	ASSERT_EQ(2, actualConstraints.size());
	EXPECT_EQ(constraint1, actualConstraints.front());
	EXPECT_EQ(constraint2, actualConstraints.back());
}

TEST(PhysicsManagerStateTest, GetMlcpProblem)
{
	auto physicsState = std::make_shared<PhysicsManagerState>();

	// Check non-const getter.
	EXPECT_NO_THROW(physicsState->getMlcpProblem().A.resize(21, 8));
	EXPECT_EQ(21, physicsState->getMlcpProblem().A.rows());
	EXPECT_EQ(8, physicsState->getMlcpProblem().A.cols());

	// Check const getter.
	std::shared_ptr<const PhysicsManagerState> constPhysicsState = std::make_shared<PhysicsManagerState>();
	EXPECT_NO_THROW(constPhysicsState->getMlcpProblem());
}

TEST(PhysicsManagerStateTest, GetMlcpSolution)
{
	auto physicsState = std::make_shared<PhysicsManagerState>();

	// Check non-const getter.
	EXPECT_NO_THROW(physicsState->getMlcpSolution().x.resize(23));
	EXPECT_EQ(23, physicsState->getMlcpSolution().x.size());

	// Check const getter.
	std::shared_ptr<const PhysicsManagerState> constPhysicsState = std::make_shared<PhysicsManagerState>();
	EXPECT_NO_THROW(constPhysicsState->getMlcpSolution());
}
