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

#include <SurgSim/Framework/BehaviorManager.h>

#include "MockObjects.h"
#include "boost/thread/thread.hpp"

using namespace SurgSim::Framework;

TEST(BehaviorManagerTest, AddRemoveTest)
{
	std::shared_ptr<BehaviorManager> manager(new BehaviorManager());
	std::shared_ptr<MockBehavior> behavior(new MockBehavior("Test Behavior1"));
	std::shared_ptr<MockBehavior> behavior2(new MockBehavior("Test Behavior2"));
	std::shared_ptr<MockComponent> component(new MockComponent("Test Component"));


	EXPECT_EQ(0,behavior->updateCount);
	EXPECT_TRUE(manager->addComponent(behavior));
	EXPECT_TRUE(manager->addComponent(behavior2));

	// Intermediate refactoring change, this will be deprecated after moving to threadsafe add
	EXPECT_FALSE(manager->addComponent(behavior));

	// This should return true because the manager is not concerned
	// with base components
	EXPECT_FALSE(manager->addComponent(component));

	EXPECT_TRUE(manager->removeComponent(behavior));
	EXPECT_FALSE(manager->removeComponent(behavior));
	EXPECT_TRUE(manager->removeComponent(behavior2));
	EXPECT_FALSE(manager->removeComponent(behavior2));
}

TEST(BehaviorManagerTest, BehaviorInitTest)
{
	std::shared_ptr<Runtime> runtime(new Runtime());
	std::shared_ptr<BehaviorManager> behaviorManager(new BehaviorManager());

	runtime->addManager(behaviorManager);
	std::shared_ptr<Scene> scene(new Scene());
	std::shared_ptr<SceneElement> element(new MockSceneElement());
	std::shared_ptr<MockBehavior> behavior(new MockBehavior("MockBehavior"));

	element->addComponent(behavior);
	scene->addSceneElement(element);
	runtime->setScene(scene);

	runtime->start();
	EXPECT_TRUE(behaviorManager->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	runtime->stop();

	EXPECT_TRUE(behavior->isInitialized);
	EXPECT_TRUE(behavior->isAwoken);
	EXPECT_GT(behavior->updateCount, 0);

}
