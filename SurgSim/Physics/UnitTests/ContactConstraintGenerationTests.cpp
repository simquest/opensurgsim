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

#include <utility>

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/CcdDcdCollision.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/DoubleSidedPlaneShape.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Constraint.h"
#include "SurgSim/Physics/ContactConstraintGeneration.h"
#include "SurgSim/Physics/PhysicsManagerState.h"
#include "SurgSim/Physics/RigidCollisionRepresentation.h"
#include "SurgSim/Physics/RigidRepresentation.h"

using SurgSim::Collision::CollisionPair;
using SurgSim::Collision::ContactCalculation;
using SurgSim::Math::DoubleSidedPlaneShape;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::SphereShape;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Physics
{

struct ContactConstraintGenerationTests: public ::testing::Test
{
	virtual void SetUp()
	{
		rigid0 = std::make_shared<RigidRepresentation>("Physics Representation 0");
		rigid0->setShape(std::make_shared<SphereShape>(2.0));
		collision0 = std::make_shared<RigidCollisionRepresentation>("Collision Representation 0");
		rigid0->setCollisionRepresentation(collision0);
		representations.push_back(rigid0);

		rigid1 = std::make_shared<RigidRepresentation>("Physics Representation 1");
		rigid1->setShape(std::make_shared<DoubleSidedPlaneShape>());
		collision1 = std::make_shared<RigidCollisionRepresentation>("Collision Representation 1");
		rigid1->setCollisionRepresentation(collision1);
		representations.push_back(rigid1);

		state = std::make_shared<PhysicsManagerState>();
		state->setRepresentations(representations);

		auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
		ASSERT_TRUE(collision0->initialize(runtime));
		ASSERT_TRUE(collision0->wakeUp());
		ASSERT_TRUE(collision1->initialize(runtime));
		ASSERT_TRUE(collision1->wakeUp());

		collision0->update(0.0);
		collision1->update(0.0);
	}

	virtual void TearDown()
	{
	}

	std::shared_ptr<SurgSim::Collision::Representation> collision0;
	std::shared_ptr<RigidRepresentation> rigid0;

	std::shared_ptr<SurgSim::Collision::Representation> collision1;
	std::shared_ptr<RigidRepresentation> rigid1;


	std::shared_ptr<PhysicsManagerState> state;
	std::vector<std::shared_ptr<SurgSim::Physics::Representation>> representations;

	std::vector<std::shared_ptr<CollisionPair>> pairs;
};

TEST_F(ContactConstraintGenerationTests, BasicTest)
{
	std::shared_ptr<CollisionPair> pair = std::make_shared<CollisionPair>(collision0, collision1);
	// Test case setup, create a pair with a contact and set up the physics state with it
	SurgSim::Collision::SphereDoubleSidedPlaneContact contactCalculation;

	contactCalculation.calculateContact(pair);
	ASSERT_TRUE(pair->hasContacts());

	pairs.push_back(pair);

	state->setCollisionPairs(pairs);

	ContactConstraintGeneration generator;
	generator.update(0.1, state);

	ASSERT_EQ(1u, state->getConstraintGroup(CONSTRAINT_GROUP_TYPE_CONTACT).size());

	auto constraints = state->getConstraintGroup(CONSTRAINT_GROUP_TYPE_CONTACT);
	auto constraint = constraints[0];

	auto localizations = constraint->getLocalizations();
	auto implementations = constraint->getImplementations();
	auto data = constraint->getData();

	EXPECT_NE(nullptr, localizations.first);
	EXPECT_NE(nullptr, localizations.second);
	EXPECT_NE(nullptr, implementations.first);
	EXPECT_NE(nullptr, implementations.second);
	EXPECT_NE(nullptr, data);

}

TEST_F(ContactConstraintGenerationTests, CountTest)
{
	std::shared_ptr<CollisionPair> pair;
	SurgSim::Collision::SphereDoubleSidedPlaneContact contactCalculation;

	pair = std::make_shared<CollisionPair>(collision0, collision1);
	contactCalculation.calculateContact(pair);
	contactCalculation.calculateContact(pair);

	pairs.push_back(pair);

	pair = std::make_shared<CollisionPair>(collision0, collision1);
	contactCalculation.calculateContact(pair);
	pairs.push_back(pair);

	pair = std::make_shared<CollisionPair>(collision0, collision1);
	pairs.push_back(pair);


	state->setCollisionPairs(pairs);
	ContactConstraintGeneration generator;
	generator.update(0.1, state);

	// 3 Contacts should generate 3 constraints
	ASSERT_EQ(3u, state->getConstraintGroup(CONSTRAINT_GROUP_TYPE_CONTACT).size());

}

TEST_F(ContactConstraintGenerationTests, InactivePhysics)
{
	std::shared_ptr<CollisionPair> pair = std::make_shared<CollisionPair>(collision0, collision1);
	SurgSim::Collision::SphereDoubleSidedPlaneContact contactCalculation;
	contactCalculation.calculateContact(pair);
	pairs.push_back(pair);
	state->setCollisionPairs(pairs);
	ContactConstraintGeneration generator;

	rigid0->setLocalActive(false);
	rigid1->setLocalActive(true);
	generator.update(0.1, state);
	ASSERT_EQ(0u, state->getConstraintGroup(CONSTRAINT_GROUP_TYPE_CONTACT).size());

	rigid0->setLocalActive(true);
	rigid1->setLocalActive(false);
	generator.update(0.1, state);
	ASSERT_EQ(0u, state->getConstraintGroup(CONSTRAINT_GROUP_TYPE_CONTACT).size());

	rigid0->setLocalActive(false);
	rigid1->setLocalActive(false);
	generator.update(0.1, state);
	ASSERT_EQ(0u, state->getConstraintGroup(CONSTRAINT_GROUP_TYPE_CONTACT).size());

	rigid0->setLocalActive(true);
	rigid1->setLocalActive(true);
	generator.update(0.1, state);
	ASSERT_EQ(1u, state->getConstraintGroup(CONSTRAINT_GROUP_TYPE_CONTACT).size());
}

TEST_F(ContactConstraintGenerationTests, LocalPoses)
{
	//Move the collision representation away from the physics representation for the sphere
	collision0->setLocalPose(makeRigidTransform(Quaterniond::Identity(), Vector3d(5.0, 0.0, 0.0)));

	std::shared_ptr<CollisionPair> pair = std::make_shared<CollisionPair>(collision0, collision1);
	SurgSim::Collision::SphereDoubleSidedPlaneContact contactCalculation;

	contactCalculation.calculateContact(pair);
	ASSERT_EQ(1u, pair->getContacts().size());

	pairs.push_back(pair);
	state->setCollisionPairs(pairs);
	ContactConstraintGeneration generator;
	generator.update(0.1, state);

	ASSERT_EQ(1u, state->getConstraintGroup(CONSTRAINT_GROUP_TYPE_CONTACT).size());
	auto constraint = state->getConstraintGroup(CONSTRAINT_GROUP_TYPE_CONTACT)[0];

	auto localization = constraint->getLocalizations().first;
	auto location = pair->getContacts().front()->penetrationPoints.first;

	Vector3d localizationGlobalPosition = localization->calculatePosition();
	Vector3d locationGlobalPosition = collision0->getPose() * location.rigidLocalPosition.getValue();
	EXPECT_TRUE(localizationGlobalPosition.isApprox(locationGlobalPosition)) <<
			"The contact location is not in the same position as the localization produced by constraint generation";
}


}; // namespace Physics
}; // namespace SurgSim
