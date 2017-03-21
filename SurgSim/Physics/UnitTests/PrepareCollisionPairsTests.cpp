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

/// \file PrepareCollisionPairsTests.cpp
/// Tests for the PrepareCollisionPairs Class

#include <gtest/gtest.h>
#include <memory>
#include <vector>


#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Physics/PhysicsManagerState.h"
#include "SurgSim/Physics/PrepareCollisionPairs.h"
#include "SurgSim/Physics/UnitTests/MockObjects.h"

using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Physics
{
class PrepareCollisionPairsTest : public ::testing::Test
{
public:
	virtual void SetUp()
	{
		sphere1 = std::make_shared<SphereCollisionElement>("Sphere1");
		sphere2 = std::make_shared<SphereCollisionElement>("Sphere2");

		runtime = std::make_shared<SurgSim::Framework::Runtime>();
		auto scene = runtime->getScene();
		scene->addSceneElement(sphere1);
		scene->addSceneElement(sphere2);

		physicsRepresentations.push_back(sphere1->getComponents<Representation>()[0]);
		physicsRepresentations.push_back(sphere2->getComponents<Representation>()[0]);

		sphere1Collision = sphere1->getComponents<Collision::Representation>()[0];
		sphere2Collision = sphere2->getComponents<Collision::Representation>()[0];
		sphere1Collision->update(0.0);
		sphere2Collision->update(0.0);
		collisionRepresentations.push_back(sphere1Collision);
		collisionRepresentations.push_back(sphere2Collision);

		computation = std::make_shared<PrepareCollisionPairs>(false);
	}

	void prepareState()
	{
		auto stateTmp = std::make_shared<PhysicsManagerState>();
		stateTmp->setRepresentations(physicsRepresentations);
		stateTmp->setCollisionRepresentations(collisionRepresentations);
		state = computation->update(0.0, stateTmp); // This step prepares the collision pairs
	}

	virtual void TearDown()
	{
	}

	std::vector<std::shared_ptr<Representation>> physicsRepresentations;
	std::vector<std::shared_ptr<Collision::Representation>> collisionRepresentations;

	std::shared_ptr<Framework::SceneElement> sphere1;
	std::shared_ptr<Collision::Representation> sphere1Collision;

	std::shared_ptr<Framework::SceneElement> sphere2;
	std::shared_ptr<Collision::Representation> sphere2Collision;

	std::shared_ptr<PhysicsManagerState> state;
	std::shared_ptr<PrepareCollisionPairs> computation;

	std::shared_ptr<Framework::Runtime> runtime;
};



TEST_F(PrepareCollisionPairsTest, RigidRigidCollisionTest)
{
	sphere2->setPose(Math::makeRigidTransform(Math::Quaterniond::Identity(), Vector3d(0.0, 0.0, 0.5)));

	prepareState();

	std::shared_ptr<PhysicsManagerState> newState = computation->update(1.0, state);
	ASSERT_EQ(1u, newState->getCollisionPairs().size());
}

TEST_F(PrepareCollisionPairsTest, InactiveTest1)
{
	sphere1->setActive(false);
	sphere2->setPose(Math::makeRigidTransform(Math::Quaterniond::Identity(), Vector3d(0.0, 0.0, 0.5)));

	prepareState();

	std::shared_ptr<PhysicsManagerState> newState = computation->update(1.0, state);
	ASSERT_EQ(0u, newState->getCollisionPairs().size());
}

TEST_F(PrepareCollisionPairsTest, InactiveTest2)
{
	sphere2->setActive(false);
	sphere2->setPose(Math::makeRigidTransform(Math::Quaterniond::Identity(), Vector3d(0.0, 0.0, 0.5)));

	prepareState();

	std::shared_ptr<PhysicsManagerState> newState = computation->update(1.0, state);
	ASSERT_EQ(0u, newState->getCollisionPairs().size());
}

TEST_F(PrepareCollisionPairsTest, InactiveTest3)
{
	sphere1->setActive(false);
	sphere2->setActive(false);
	sphere2->setPose(Math::makeRigidTransform(Math::Quaterniond::Identity(), Vector3d(0.0, 0.0, 0.5)));

	prepareState();

	std::shared_ptr<PhysicsManagerState> newState = computation->update(1.0, state);
	ASSERT_EQ(0u, newState->getCollisionPairs().size());
}

TEST_F(PrepareCollisionPairsTest, ExlcudeCollisionsTest1)
{
	sphere1Collision->ignore("Sphere2/Collision");
	sphere2->setPose(Math::makeRigidTransform(Math::Quaterniond::Identity(), Vector3d(0.0, 0.0, 0.5)));

	prepareState();

	std::shared_ptr<PhysicsManagerState> newState = computation->update(1.0, state);
	EXPECT_EQ(0u, newState->getCollisionPairs().size());
}

TEST_F(PrepareCollisionPairsTest, ExlcudeCollisionsTest2)
{
	sphere2Collision->ignore("Sphere1/Collision");
	sphere2->setPose(Math::makeRigidTransform(Math::Quaterniond::Identity(), Vector3d(0.0, 0.0, 0.5)));

	prepareState();

	std::shared_ptr<PhysicsManagerState> newState = computation->update(1.0, state);
	EXPECT_EQ(0u, newState->getCollisionPairs().size());
}

TEST_F(PrepareCollisionPairsTest, ExlcudeCollisionsTest3)
{
	sphere1Collision->ignore("Sphere2/Collision");
	sphere2Collision->ignore("Sphere1/Collision");
	sphere2->setPose(Math::makeRigidTransform(Math::Quaterniond::Identity(), Vector3d(0.0, 0.0, 0.5)));

	prepareState();

	std::shared_ptr<PhysicsManagerState> newState = computation->update(1.0, state);
	EXPECT_EQ(0u, newState->getCollisionPairs().size());
}

TEST_F(PrepareCollisionPairsTest, IgnoreContinuousTypeCollisions)
{
	sphere1Collision->setCollisionDetectionType(Collision::COLLISION_DETECTION_TYPE_CONTINUOUS);
	sphere2Collision->setCollisionDetectionType(Collision::COLLISION_DETECTION_TYPE_CONTINUOUS);
	sphere2->setPose(Math::makeRigidTransform(Math::Quaterniond::Identity(), Vector3d(0.0, 0.0, 0.5)));

	prepareState();

	std::shared_ptr<PhysicsManagerState> newState = computation->update(1.0, state);
	ASSERT_EQ(1u, newState->getCollisionPairs().size());
}

TEST_F(PrepareCollisionPairsTest, IgnoreNoneTypeCollisions)
{
	sphere2Collision->setCollisionDetectionType(Collision::COLLISION_DETECTION_TYPE_NONE);
	sphere2->setPose(Math::makeRigidTransform(Math::Quaterniond::Identity(), Vector3d(0.0, 0.0, 0.5)));

	prepareState();

	std::shared_ptr<PhysicsManagerState> newState = computation->update(1.0, state);
	ASSERT_EQ(0u, newState->getCollisionPairs().size());
}

TEST_F(PrepareCollisionPairsTest, SelfCollisionPair)
{
	sphere2->setPose(Math::makeRigidTransform(Math::Quaterniond::Identity(), Vector3d(0.0, 0.0, 0.5)));
	sphere2Collision->setSelfCollisionDetectionType(Collision::COLLISION_DETECTION_TYPE_DISCRETE);

	prepareState();
	std::shared_ptr<PhysicsManagerState> newState = computation->update(1.0, state);
	ASSERT_EQ(2u, newState->getCollisionPairs().size());
}

// Check for a bug fix in PrepareCollisionPairs where Scenes with 1 representation did not check for self collision
TEST_F(PrepareCollisionPairsTest, SingleSelfCollisionPair)
{
	physicsRepresentations.pop_back();
	collisionRepresentations.pop_back();
	sphere1->setPose(Math::makeRigidTransform(Math::Quaterniond::Identity(), Vector3d(0.0, 0.0, 0.5)));
	sphere1Collision->setSelfCollisionDetectionType(Collision::COLLISION_DETECTION_TYPE_DISCRETE);

	prepareState();
	std::shared_ptr<PhysicsManagerState> newState = computation->update(1.0, state);
	ASSERT_EQ(1u, newState->getCollisionPairs().size());
}

};
};
