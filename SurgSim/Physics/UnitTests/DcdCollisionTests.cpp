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
#include "SurgSim/Physics/PrepareCollisionPairs.h"

using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Physics
{

TEST(DcdCollisionTest, ConstructorTest)
{
	ASSERT_NO_THROW(std::make_shared<SurgSim::Physics::DcdCollision>(false));
	ASSERT_NO_THROW(std::make_shared<SurgSim::Physics::DcdCollision>(true));
}

TEST(DcdCollisionTest, RigidRigidCollisionTest)
{
	auto sphere1 = std::make_shared<SurgSim::Blocks::SphereElement>("Sphere1");
	auto sphere2 = std::make_shared<SurgSim::Blocks::SphereElement>("Sphere2");
	sphere2->setPose(Math::makeRigidTransform(Math::Quaterniond::Identity(), Vector3d(0.0, 0.0, 0.5)));

	auto runtime = std::make_shared<SurgSim::Framework::Runtime>();
	auto scene = runtime->getScene();
	scene->addSceneElement(sphere1);
	scene->addSceneElement(sphere2);

	std::vector<std::shared_ptr<SurgSim::Physics::Representation>> physicsRepresentations;
	physicsRepresentations.push_back(sphere1->getComponents<SurgSim::Physics::Representation>()[0]);
	physicsRepresentations.push_back(sphere2->getComponents<SurgSim::Physics::Representation>()[0]);

	auto sphere1Collision = sphere1->getComponents<SurgSim::Collision::Representation>()[0];
	auto sphere2Collision = sphere2->getComponents<SurgSim::Collision::Representation>()[0];
	sphere1Collision->update(0.0);
	sphere2Collision->update(0.0);
	std::vector<std::shared_ptr<SurgSim::Collision::Representation>> collisionRepresentations;
	collisionRepresentations.push_back(sphere1Collision);
	collisionRepresentations.push_back(sphere2Collision);

	auto prepareState = std::make_shared<SurgSim::Physics::PrepareCollisionPairs>(false);
	auto stateTmp = std::make_shared<SurgSim::Physics::PhysicsManagerState>();
	stateTmp->setRepresentations(physicsRepresentations);
	stateTmp->setCollisionRepresentations(collisionRepresentations);
	auto state = prepareState->update(0.0, stateTmp); // This step prepares the collision pairs
	ASSERT_EQ(1u, state->getCollisionPairs().size());

	auto dcdCollision = std::make_shared<SurgSim::Physics::DcdCollision>(false);
	std::shared_ptr<PhysicsManagerState> newState = dcdCollision->update(1.0, state);
	EXPECT_TRUE(newState->getCollisionPairs().at(0)->hasContacts());
}

};
};
