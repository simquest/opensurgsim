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

/// \file DcdCollisionTests.cpp
/// Tests for the DcdCollision Class

#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include "SurgSim/Blocks/SphereElement.h"
#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Physics/DcdCollision.h"
#include "SurgSim/Physics/PhysicsManagerState.h"

using SurgSim::Math::Vector3d;


class DcdCollisionTest : public ::testing::Test
{
public:
	virtual void SetUp()
	{
		sphere1 = std::make_shared<SurgSim::Blocks::SphereElement>("Sphere1");
		sphere2 = std::make_shared<SurgSim::Blocks::SphereElement>("Sphere2");

		runtime = std::make_shared<SurgSim::Framework::Runtime>();
		auto scene = runtime->getScene();
		scene->addSceneElement(sphere1);
		scene->addSceneElement(sphere2);

		std::vector<std::shared_ptr<SurgSim::Physics::Representation>> physicsRepresentations;
		physicsRepresentations.push_back(sphere1->getComponents<SurgSim::Physics::Representation>()[0]);
		physicsRepresentations.push_back(sphere2->getComponents<SurgSim::Physics::Representation>()[0]);

		std::vector<std::shared_ptr<SurgSim::Collision::Representation>> collisionRepresentations;
		collisionRepresentations.push_back(sphere1->getComponents<SurgSim::Collision::Representation>()[0]);
		collisionRepresentations.push_back(sphere2->getComponents<SurgSim::Collision::Representation>()[0]);

		state = std::make_shared<SurgSim::Physics::PhysicsManagerState>();
		state->setRepresentations(physicsRepresentations);
		state->setCollisionRepresentations(collisionRepresentations);


	}

	virtual void TearDown()
	{
	}

	std::shared_ptr<SurgSim::Framework::SceneElement> sphere1;
	std::shared_ptr<SurgSim::Framework::SceneElement> sphere2;

	std::shared_ptr<SurgSim::Physics::PhysicsManagerState> state;
	SurgSim::Physics::DcdCollision computation;

	std::shared_ptr<SurgSim::Framework::Runtime> runtime;
};

namespace SurgSim
{
namespace Physics
{

TEST_F(DcdCollisionTest, RigidRigidCollisionTest)
{
	sphere2->setPose(Math::makeRigidTransform(Math::Quaterniond::Identity(), Vector3d(0.0, 0.0, 0.5)));

	std::shared_ptr<PhysicsManagerState> newState = computation.update(1.0, state);
	ASSERT_EQ(1u, newState->getCollisionPairs().size());
	EXPECT_TRUE(newState->getCollisionPairs().at(0)->hasContacts());
}

TEST_F(DcdCollisionTest, InactiveTest1)
{
	sphere1->setActive(false);
	sphere2->setPose(Math::makeRigidTransform(Math::Quaterniond::Identity(), Vector3d(0.0, 0.0, 0.5)));

	std::shared_ptr<PhysicsManagerState> newState = computation.update(1.0, state);
	ASSERT_EQ(0u, newState->getCollisionPairs().size());
}

TEST_F(DcdCollisionTest, InactiveTest2)
{
	sphere2->setActive(false);
	sphere2->setPose(Math::makeRigidTransform(Math::Quaterniond::Identity(), Vector3d(0.0, 0.0, 0.5)));

	std::shared_ptr<PhysicsManagerState> newState = computation.update(1.0, state);
	ASSERT_EQ(0u, newState->getCollisionPairs().size());
}

TEST_F(DcdCollisionTest, InactiveTest3)
{
	sphere1->setActive(false);
	sphere2->setActive(false);
	sphere2->setPose(Math::makeRigidTransform(Math::Quaterniond::Identity(), Vector3d(0.0, 0.0, 0.5)));

	std::shared_ptr<PhysicsManagerState> newState = computation.update(1.0, state);
	ASSERT_EQ(0u, newState->getCollisionPairs().size());
}

TEST_F(DcdCollisionTest, ExlcudeCollisionsTest1)
{
	sphere1->getComponents<Collision::Representation>()[0]->ignore("Sphere2/Sphere Collision Representation");
	sphere2->setPose(Math::makeRigidTransform(Math::Quaterniond::Identity(), Vector3d(0.0, 0.0, 0.5)));

	std::shared_ptr<PhysicsManagerState> newState = computation.update(1.0, state);
	ASSERT_EQ(0u, newState->getCollisionPairs().size());
}

TEST_F(DcdCollisionTest, ExlcudeCollisionsTest2)
{
	sphere2->getComponents<Collision::Representation>()[0]->ignore("Sphere1/Sphere Collision Representation");
	sphere2->setPose(Math::makeRigidTransform(Math::Quaterniond::Identity(), Vector3d(0.0, 0.0, 0.5)));

	std::shared_ptr<PhysicsManagerState> newState = computation.update(1.0, state);
	ASSERT_EQ(0u, newState->getCollisionPairs().size());
}

TEST_F(DcdCollisionTest, ExlcudeCollisionsTest3)
{
	sphere1->getComponents<Collision::Representation>()[0]->ignore("Sphere2/Sphere Collision Representation");
	sphere2->getComponents<Collision::Representation>()[0]->ignore("Sphere1/Sphere Collision Representation");
	sphere2->setPose(Math::makeRigidTransform(Math::Quaterniond::Identity(), Vector3d(0.0, 0.0, 0.5)));

	std::shared_ptr<PhysicsManagerState> newState = computation.update(1.0, state);
	ASSERT_EQ(0u, newState->getCollisionPairs().size());
}


};
};
