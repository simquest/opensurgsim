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
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/SceneElement.h"
#include "MockObjects.h"  //NOLINT

using SurgSim::Framework::Runtime;
using SurgSim::Framework::Scene;
using SurgSim::Framework::Logger;

TEST(RuntimeTest, Constructor)
{
	EXPECT_NO_THROW({std::shared_ptr<Runtime> runtime(new Runtime());});
	std::shared_ptr<Runtime> runtime(new Runtime());
	EXPECT_NE(nullptr, runtime->getScene());

	EXPECT_NO_THROW(std::make_shared<Runtime>());
	EXPECT_NO_THROW(std::make_shared<Runtime>("config.txt"));
	EXPECT_THROW(std::make_shared<Runtime>("Non-exist-file"), SurgSim::Framework::AssertionFailure);
}

TEST(RuntimeTest, AddManager)
{
	std::shared_ptr<Runtime> runtime(new Runtime());
	std::shared_ptr<MockManager> manager(new MockManager());

	runtime->addManager(manager);

	EXPECT_TRUE(runtime->start());

	EXPECT_TRUE(manager->isInitialized());

	EXPECT_TRUE(runtime->isRunning());

	EXPECT_TRUE(runtime->stop());

	EXPECT_FALSE(runtime->isRunning());

	EXPECT_FALSE(manager->isRunning());
}

TEST(RuntimeTest, InitFailureDeathTest)
{
	std::shared_ptr<Runtime> runtime(new Runtime());
	std::shared_ptr<MockManager> managerSucceeds(new MockManager());
	std::shared_ptr<MockManager> managerFails(new MockManager(false,true));

	runtime->addManager(managerSucceeds);
	runtime->addManager(managerFails);

	ASSERT_DEATH(runtime->start(), "");
}

TEST(RuntimeTest, StartupFailureDeathTest)
{
	std::shared_ptr<Runtime> runtime(new Runtime());
	std::shared_ptr<MockManager> managerSucceeds(new MockManager());
	std::shared_ptr<MockManager> managerFails(new MockManager(true,false));

	runtime->addManager(managerSucceeds);
	runtime->addManager(managerFails);

	ASSERT_DEATH(runtime->start(), "");
}

TEST(RuntimeTest, SceneInitialization)
{
	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();
	std::shared_ptr<MockManager> manager = std::make_shared<MockManager>();
	runtime->addManager(manager);
	std::shared_ptr<Scene> scene = runtime->getScene();

	std::vector<std::shared_ptr<MockSceneElement>> elements;
	std::vector<std::shared_ptr<MockComponent>> components;

	elements.push_back(std::make_shared<MockSceneElement>("one"));
	elements.push_back(std::make_shared<MockSceneElement>("two"));
	components.push_back(std::make_shared<MockComponent>("one"));
	components.push_back(std::make_shared<MockComponent>("two"));
	components.push_back(std::make_shared<MockComponent>("three"));
	components.push_back(std::make_shared<MockComponent>("four"));

	scene->addSceneElement(elements[0]);
	scene->addSceneElement(elements[1]);
	elements[0]->addComponent(components[0]);
	elements[0]->addComponent(components[1]);
	elements[1]->addComponent(components[2]);
	elements[1]->addComponent(components[3]);

	EXPECT_FALSE(manager->didInitialize);
	EXPECT_FALSE(manager->didStartUp);
	EXPECT_FALSE(manager->didBeforeStop);

	runtime->start();

	EXPECT_TRUE(manager->didInitialize);
	EXPECT_TRUE(manager->didStartUp);
	EXPECT_FALSE(manager->didBeforeStop);

	for (int i = 0;  i < 2;  ++i)
	{
		EXPECT_NE(nullptr, elements[i]->getRuntime());
		EXPECT_TRUE(elements[i]->didInit);
	}

	for (int i=0; i<4; i++)
	{
		EXPECT_TRUE(components[i]->didInit);
	}

	runtime->stop();
	EXPECT_TRUE(manager->didBeforeStop);
}

TEST(RuntimeTest, PausedStep)
{
	std::shared_ptr<Runtime> runtime(new Runtime());
	std::shared_ptr<MockManager> manager1(new MockManager());
	std::shared_ptr<MockManager> manager2(new MockManager());

	runtime->addManager(manager1);
	runtime->addManager(manager2);

	runtime->start(true);

	EXPECT_TRUE(runtime->isPaused());

	boost::this_thread::sleep(boost::posix_time::milliseconds(150));

	EXPECT_TRUE(manager1->isSynchronous());
	EXPECT_TRUE(manager2->isSynchronous());

	int count = manager1->count;

	runtime->step();
	boost::this_thread::sleep(boost::posix_time::milliseconds(150));
	EXPECT_EQ(count+1,manager1->count);

	runtime->step();
	boost::this_thread::sleep(boost::posix_time::milliseconds(150));
	EXPECT_EQ(count+2,manager1->count);

	runtime->stop();
}


TEST(RuntimeTest, PauseResume)
{
	std::shared_ptr<Runtime> runtime(new Runtime());
	std::shared_ptr<MockManager> manager1(new MockManager());
	std::shared_ptr<MockManager> manager2(new MockManager());

	runtime->addManager(manager1);
	runtime->addManager(manager2);

	runtime->start(false);


	boost::this_thread::sleep(boost::posix_time::milliseconds(150));

	EXPECT_FALSE(manager1->isSynchronous());
	EXPECT_FALSE(manager2->isSynchronous());

	runtime->pause();

	boost::this_thread::sleep(boost::posix_time::milliseconds(150));
	EXPECT_TRUE(manager1->isSynchronous());
	EXPECT_TRUE(manager2->isSynchronous());

	runtime->step();
	boost::this_thread::sleep(boost::posix_time::milliseconds(150));
	runtime->step();
	boost::this_thread::sleep(boost::posix_time::milliseconds(150));
	runtime->resume();
	boost::this_thread::sleep(boost::posix_time::milliseconds(150));

	EXPECT_FALSE(manager1->isSynchronous());
	EXPECT_FALSE(manager2->isSynchronous());

	runtime->stop();
}

TEST(RuntimeTest, AddComponentAddDuringRuntime)
{
	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();
	std::shared_ptr<MockManager> manager = std::make_shared<MockManager>();
	runtime->addManager(manager);
	std::shared_ptr<Scene> scene = runtime->getScene();

	std::vector<std::shared_ptr<MockComponent>> components;

	auto element = std::make_shared<MockSceneElement>("one");
	components.push_back(std::make_shared<MockComponent>("one"));
	components.push_back(std::make_shared<MockComponent>("two"));

	scene->addSceneElement(element);
	element->addComponent(components[0]);

	runtime->start(true);

	EXPECT_TRUE(manager->didInitialize);
	EXPECT_TRUE(manager->didStartUp);
	EXPECT_FALSE(manager->didBeforeStop);

	// Make sure we are out of initialization completely
	runtime->step();

	EXPECT_FALSE(components[1]->isInitialized());
	EXPECT_FALSE(components[1]->isAwake());

	EXPECT_TRUE(element->addComponent(components[1]));

	EXPECT_TRUE(components[1]->isInitialized());
	EXPECT_FALSE(components[1]->isAwake());

	runtime->step();
	runtime->step(); // Right now step is still non-blocking, make sure the thread has finished processing...

	EXPECT_TRUE(components[1]->isInitialized());
	EXPECT_TRUE(components[1]->isAwake());

	boost::this_thread::sleep(boost::posix_time::milliseconds(150));

	runtime->stop();
}

TEST(RuntimeTest, LoadAndAddScene)
{
	auto runtime = std::make_shared<Runtime>("config.txt");
	
	EXPECT_TRUE(runtime->loadScene("SceneTestData/scene.yaml"));
	
	auto scene0 = runtime->getScene();
	
	EXPECT_NE(nullptr, scene0);
	EXPECT_EQ(2L, scene0->getSceneElements().size());
	
	EXPECT_TRUE(runtime->loadScene("SceneTestData/scene.yaml"));

	auto scene1 = runtime->getScene();
	
	EXPECT_NE(nullptr, scene1);
	EXPECT_NE(scene0, scene1);

	EXPECT_TRUE(runtime->addScene("SceneTestData/scene.yaml"));

	auto scene2 = runtime->getScene();

	EXPECT_NE(nullptr, scene2);
	EXPECT_EQ(scene1, scene2);
	
	EXPECT_EQ(4L, scene2->getSceneElements().size());
}
