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

#include "boost/thread/thread.hpp"

#include "SurgSim/Framework/BehaviorManager.h"
#include "SurgSim/Framework/UnitTests/MockObjects.h"

using SurgSim::Framework::BehaviorManager;
using SurgSim::Framework::Runtime;
using SurgSim::Framework::Scene;
using SurgSim::Framework::SceneElement;

TEST(BehaviorManagerTest, BehaviorInitTest)
{
	std::shared_ptr<Runtime> runtime(new Runtime());
	std::shared_ptr<BehaviorManager> behaviorManager(new BehaviorManager());

	runtime->addManager(behaviorManager);
	auto scene = runtime->getScene();
	std::shared_ptr<SceneElement> element(new MockSceneElement());
	std::shared_ptr<MockBehavior> behavior(new MockBehavior("MockBehavior"));
	std::shared_ptr<MockComponent> component(new MockComponent("Test Component"));

	element->addComponent(behavior);
	element->addComponent(component);
	scene->addSceneElement(element);
	EXPECT_TRUE(element->isInitialized());
	EXPECT_TRUE(behavior->isInitialized());
	EXPECT_TRUE(component->isInitialized());

	{
		// Set the SceneElement to inactive will also set all components it owns to inactive
		element->setActive(false);
		EXPECT_FALSE(behavior->isActive());
		EXPECT_FALSE(component->isActive());

		// BehaviorManager will not update inactive behaviors.
		runtime->start();
		EXPECT_TRUE(behaviorManager->isInitialized());
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
		runtime->stop();
		EXPECT_EQ(0, behavior->updateCount);
	}

	{
		// Set the SceneElement to active will also set all components it owns to active
		element->setActive(true);
		EXPECT_TRUE(behavior->isActive());
		EXPECT_TRUE(component->isActive());

		// BehaviorManager will update active behaviors.
		runtime->start();
		EXPECT_TRUE(behaviorManager->isInitialized());
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
		runtime->stop();
		EXPECT_GT(behavior->updateCount, 0);
	}

	{
		behavior->updateCount = 0;
		// Set only the behavior to inactive, its 'updateCount' will not be updated.
		behavior->setActive(false);
		EXPECT_FALSE(behavior->isActive());
		EXPECT_TRUE(component->isActive());

		// BehaviorManager will not update inactive behaviors.
		runtime->start();
		EXPECT_TRUE(behaviorManager->isInitialized());
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
		runtime->stop();
		EXPECT_EQ(0, behavior->updateCount);
	}

	{
		// Set 'behavior' to active, its 'updateCount' will be updated.
		behavior->setActive(true);
		EXPECT_TRUE(behavior->isActive());

		// BehaviorManager will update active behaviors.
		runtime->start();
		EXPECT_TRUE(behaviorManager->isInitialized());
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
		runtime->stop();

		EXPECT_TRUE(behavior->isAwake());
		EXPECT_FALSE(component->isAwake());
		EXPECT_GT(behavior->updateCount, 0);
	}
}
