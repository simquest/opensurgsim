// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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
#include "SurgSim/Collision/ShapeCollisionRepresentation.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Physics/DcdCollision.h"
#include "SurgSim/Physics/PhysicsManager.h"
#include "SurgSim/Physics/PhysicsManagerState.h"
#include "SurgSim/Physics/PrepareCollisionPairs.h"

using SurgSim::Math::Vector3d;


namespace SurgSim
{
namespace Physics
{

TEST(DcdCollisionTest, ConstructorTest)
{
	ASSERT_NO_THROW(std::make_shared<DcdCollision>(false));
	ASSERT_NO_THROW(std::make_shared<DcdCollision>(true));
}

TEST(DcdCollisionTest, RigidRigidCollisionTest)
{
	auto sphere1 = std::make_shared<Blocks::SphereElement>("Sphere1");
	auto sphere2 = std::make_shared<Blocks::SphereElement>("Sphere2");
	sphere2->setPose(Math::makeRigidTransform(Math::Quaterniond::Identity(), Vector3d(0.0, 0.0, 0.5)));

	auto runtime = std::make_shared<Framework::Runtime>();
	auto scene = runtime->getScene();
	scene->addSceneElement(sphere1);
	scene->addSceneElement(sphere2);

	std::vector<std::shared_ptr<Physics::Representation>> physicsRepresentations;
	physicsRepresentations.push_back(sphere1->getComponents<Physics::Representation>()[0]);
	physicsRepresentations.push_back(sphere2->getComponents<Physics::Representation>()[0]);

	auto sphere1Collision = sphere1->getComponents<Collision::Representation>()[0];
	auto sphere2Collision = sphere2->getComponents<Collision::Representation>()[0];
	sphere1Collision->update(0.0);
	sphere2Collision->update(0.0);
	std::vector<std::shared_ptr<Collision::Representation>> collisionRepresentations;
	collisionRepresentations.push_back(sphere1Collision);
	collisionRepresentations.push_back(sphere2Collision);

	auto prepareState = std::make_shared<PrepareCollisionPairs>(false);
	auto stateTmp = std::make_shared<PhysicsManagerState>();
	stateTmp->setRepresentations(physicsRepresentations);
	stateTmp->setCollisionRepresentations(collisionRepresentations);
	auto state = prepareState->update(0.0, stateTmp); // This step prepares the collision pairs
	ASSERT_EQ(1u, state->getCollisionPairs().size());

	auto dcdCollision = std::make_shared<DcdCollision>(false);
	std::shared_ptr<PhysicsManagerState> newState = dcdCollision->update(1.0, state);
	EXPECT_TRUE(newState->getCollisionPairs().at(0)->hasContacts());
}

TEST(DcdCollisionTest, Deactivate)
{
	auto runtime = std::make_shared<Framework::Runtime>();
	auto physicsManager = std::make_shared<PhysicsManager>();
	runtime->addManager(physicsManager);

	auto sphere1 = std::make_shared<Collision::ShapeCollisionRepresentation>("Sphere1");
	sphere1->setShape(std::make_shared<Math::SphereShape>(1.0));

	auto sphere2 = std::make_shared<Collision::ShapeCollisionRepresentation>("Sphere2");
	sphere2->setShape(std::make_shared<Math::SphereShape>(1.0));
	sphere2->setLocalPose(Math::makeRigidTransform(Math::Quaterniond::Identity(), Vector3d(0.0, 0.0, 0.5)));

	auto scene = runtime->getScene();
	auto element = std::make_shared<Framework::BasicSceneElement>("Element");
	element->addComponent(sphere2);
	element->addComponent(sphere1);
	scene->addSceneElement(element);

	runtime->start(true);
	boost::this_thread::sleep(boost::posix_time::milliseconds(50));
	runtime->step();
	boost::this_thread::sleep(boost::posix_time::milliseconds(50));
	EXPECT_TRUE(sphere1->collidedWith(sphere2));
	EXPECT_TRUE(sphere2->collidedWith(sphere1));

	sphere1->setLocalActive(false);
	runtime->step();
	boost::this_thread::sleep(boost::posix_time::milliseconds(50));
	EXPECT_FALSE(sphere1->collidedWith(sphere2));
	EXPECT_FALSE(sphere2->collidedWith(sphere1));

	runtime->stop();
}

};
};
