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
#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Framework/Scene.h>
#include <SurgSim/Framework/SceneElement.h>
#include "MockObjects.h"  //NOLINT

using SurgSim::Framework::Runtime;
using SurgSim::Framework::Scene;
using SurgSim::Framework::Logger;

TEST(RuntimeTest, Constructor)
{
	EXPECT_NO_THROW({std::shared_ptr<Runtime> runtime(new Runtime());});

	Runtime runtime;
	ASSERT_ANY_THROW(runtime.getSharedPtr());
}

TEST(RuntimeTest, SetScene)
{
	std::shared_ptr<Runtime> runtime(new Runtime());
	std::shared_ptr<Scene> scene(new Scene());
	EXPECT_NO_THROW(runtime->setScene(scene));
}

TEST(RuntimeTest, AddManager)
{
	std::shared_ptr<Runtime> runtime(new Runtime());
	std::shared_ptr<MockManager> manager(new MockManager());

	runtime->addManager(manager);

	EXPECT_TRUE(runtime->start());

	EXPECT_TRUE(manager->isInitialized());


	EXPECT_TRUE(runtime->stop());

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

TEST(RuntimeTest, LoggerManagement)
{
	std::shared_ptr<Runtime> runtime(new Runtime());
	std::shared_ptr<Logger> newLogger = runtime->getLogger("TestLogger");

	ASSERT_NE(nullptr, newLogger);
	EXPECT_EQ("TestLogger", newLogger->getName());

	std::shared_ptr<Logger> oldLogger = runtime->getLogger("TestLogger");
	EXPECT_EQ(oldLogger, newLogger);
}

TEST(RuntimeTest, SceneInitialisation)
{
	std::shared_ptr<Runtime> runtime(new Runtime());
	std::shared_ptr<Scene> scene(new Scene());

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

	runtime->setScene(scene);
	runtime->start();

	for (int i = 0;  i < 2;  ++i)
	{
		EXPECT_NE(nullptr, elements[i]->getRuntime());
		EXPECT_TRUE(elements[i]->didInit);
		EXPECT_TRUE(elements[i]->didWakeUp);
	}

	for (int i=0; i<4; i++)
	{
		EXPECT_TRUE(components[i]->didInit);
		EXPECT_TRUE(components[i]->didWakeUp);
	}
}



