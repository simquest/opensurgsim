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

	runtime->start();
	EXPECT_TRUE(behaviorManager->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(250));
	EXPECT_TRUE(behavior->isAwake());
	runtime->stop();

	EXPECT_FALSE(behavior->isAwake());
	EXPECT_FALSE(component->isAwake());
	EXPECT_GT(behavior->updateCount, 0);

}

TEST(BehaviorManagerTest, UpdateTest)
{
	auto runtime = std::make_shared<Runtime>();
	auto scene = runtime->getScene();
	auto behaviorManager = std::make_shared<BehaviorManager>();
	auto element = std::make_shared<MockSceneElement>();
	auto behavior = std::make_shared<MockBehavior>("MockBehavior");

	runtime->addManager(behaviorManager);
	element->addComponent(behavior);
	scene->addSceneElement(element);
	EXPECT_TRUE(element->isInitialized());
	EXPECT_TRUE(behavior->isInitialized());

	behavior->setLocalActive(false);
	behavior->updateCount = 0;

	// BehaviorManager will not update inactive behaviors.
	runtime->start();
	EXPECT_TRUE(behaviorManager->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	EXPECT_EQ(0, behavior->updateCount);

	// Turn on the behavior, it will be updated.
	behavior->setLocalActive(true);
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	EXPECT_GT(behavior->updateCount, 0);

	// Turn off the behavior, it will not be updated any more.
	behavior->setLocalActive(false);
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	auto count = behavior->updateCount;
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	EXPECT_EQ(behavior->updateCount, count);

	runtime->stop();
}
