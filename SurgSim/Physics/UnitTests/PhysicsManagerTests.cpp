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

#include <string>
#include <memory>

#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Physics/PhysicsManager.h>
#include <SurgSim/Physics/Actors/Actor.h>
#include <SurgSim/Physics/Actors/FixedActor.h>
#include <SurgSim/Math/Vector.h>

using SurgSim::Framework::Runtime;
using SurgSim::Physics::FixedActor;
using SurgSim::Physics::PhysicsManager;
using SurgSim::Math::Vector3d;

struct PhysicsManagerTest : public ::testing::Test
{
	virtual void SetUp()
	{
		runtime = std::make_shared<Runtime>();
		physicsManager = std::make_shared<PhysicsManager>();

		runtime->addManager(physicsManager);
		runtime->start();
	}

	virtual void TearDown()
	{
		runtime->stop();
	}


	std::shared_ptr<Runtime> runtime;
	std::shared_ptr<PhysicsManager> physicsManager;
};

TEST_F(PhysicsManagerTest, InitTest)
{
	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();
	std::shared_ptr<PhysicsManager> physicsManager = std::make_shared<PhysicsManager>();

	runtime->addManager(physicsManager);
	EXPECT_NO_THROW(runtime->start());
	EXPECT_NO_THROW(runtime->stop());
}

TEST_F(PhysicsManagerTest, AddRemoveComponent)
{
	std::shared_ptr<FixedActor> actor1= std::make_shared<FixedActor>("Actor1");
	std::shared_ptr<FixedActor> actor2 = std::make_shared<FixedActor>("Actor2");

	EXPECT_TRUE(physicsManager->addComponent(actor1));
	EXPECT_TRUE(physicsManager->addComponent(actor2));
	EXPECT_FALSE(physicsManager->addComponent(actor1));

	EXPECT_TRUE(physicsManager->removeComponent(actor1));
	EXPECT_FALSE(physicsManager->removeComponent(actor1));
	EXPECT_TRUE(physicsManager->removeComponent(actor2));
}
