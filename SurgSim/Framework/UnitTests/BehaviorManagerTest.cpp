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

#include <SurgSim/Framework/UnitTests/MockObjects.h>
#include "boost/thread/thread.hpp"

using SurgSim::Framework::BehaviorManager;
using SurgSim::Framework::Runtime;
using SurgSim::Framework::Scene;
using SurgSim::Framework::SceneElement;

TEST(BehaviorManagerTest, BehaviorInitTest)
{
	std::shared_ptr<Runtime> runtime(new Runtime());
	std::shared_ptr<BehaviorManager> behaviorManager(new BehaviorManager());

	runtime->addManager(behaviorManager);
	std::shared_ptr<Scene> scene(new Scene());
	std::shared_ptr<SceneElement> element(new MockSceneElement());
	std::shared_ptr<MockBehavior> behavior(new MockBehavior("MockBehavior"));
	std::shared_ptr<MockComponent> component(new MockComponent("Test Component"));


	element->addComponent(behavior);
	element->addComponent(component);
	scene->addSceneElement(element);
	runtime->setScene(scene);

	runtime->start();
	EXPECT_TRUE(behaviorManager->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	runtime->stop();

	EXPECT_TRUE(behavior->isInitialized);
	EXPECT_TRUE(behavior->isAwake());
	EXPECT_FALSE(component->isInitialized());
	EXPECT_FALSE(component->isAwake());
	EXPECT_GT(behavior->updateCount, 0);

}
