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
#include "MockObjects.h"  //NOLINT
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/FrameworkConvert.h"

#include "SurgSim/Testing/Utilities.h"

namespace SurgSim
{
namespace Framework
{

TEST(SceneTest, ConstructorTest)
{
	ASSERT_NO_THROW({Scene scene(std::make_shared<Runtime>());});
}

TEST(SceneTest, ElementManagement)
{
	auto runtime(std::make_shared<Runtime>());
	std::shared_ptr<Scene> scene = runtime->getScene();
	std::shared_ptr<MockSceneElement> element1(new MockSceneElement("one"));
	std::shared_ptr<MockSceneElement> element2(new MockSceneElement("two"));

	EXPECT_EQ(0u, scene->getSceneElements().size());

	scene->addSceneElement(element1);
	EXPECT_EQ(1u, scene->getSceneElements().size());
	scene->addSceneElement(element2);
	EXPECT_EQ(2u, scene->getSceneElements().size());

	EXPECT_ANY_THROW(scene->addSceneElement(element1));
	EXPECT_EQ(2u, scene->getSceneElements().size());

	EXPECT_EQ(element1, scene->getSceneElement("one"));
	EXPECT_EQ(nullptr, scene->getSceneElement("three"));
}

TEST(SceneTest, AddAndTestScene)
{
	auto runtime(std::make_shared<Runtime>());
	std::shared_ptr<Scene> scene = runtime->getScene();
	std::shared_ptr<MockSceneElement> element = std::make_shared<MockSceneElement>("element");
	std::shared_ptr<MockComponent> component = std::make_shared<MockComponent>("component");

	EXPECT_TRUE(element->addComponent(component));
	scene->addSceneElement(element);

	EXPECT_EQ(scene, component->getScene());
	EXPECT_EQ(element, component->getSceneElement());
	EXPECT_EQ(scene, element->getScene());
}

TEST(SceneTest, CheckForExpiredRuntime)
{
	auto runtime = std::make_shared<Runtime>();
	auto scene = std::make_shared<Scene>(runtime);

	auto element0 = std::make_shared<MockSceneElement>("element0");
	auto element1 = std::make_shared<MockSceneElement>("element1");

	auto component0 = std::make_shared<MockComponent>("component0");
	auto component1 = std::make_shared<MockComponent>("component1");

	// This is the normal behavior
	EXPECT_NO_THROW(scene->addSceneElement(element0));
	EXPECT_NO_THROW(element0->addComponent(component0));

	// invalidate runtime pointer, this will cause the weak_ptr inside of scene and sceneelement
	// to expire
	runtime.reset();

	// We should not be able to do this with an expired runtime pointer
	EXPECT_THROW(scene->addSceneElement(element1), SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(element0->addComponent(component1), SurgSim::Framework::AssertionFailure);
}

TEST(SceneTest, YamlTest)
{
	auto runtime = std::make_shared<Runtime>();
	auto scene = std::make_shared<Scene>(runtime);

	auto element0 = std::make_shared<BasicSceneElement>("element0");
	auto element1 = std::make_shared<BasicSceneElement>("element1");

	auto component0 = std::make_shared<MockComponent>("component0");
	auto component1 = std::make_shared<MockComponent>("component1");

	element0->addComponent(component0);
	element1->addComponent(component1);
	scene->addSceneElement(element0);
	scene->addSceneElement(element1);

	YAML::Node node;
	ASSERT_NO_THROW(node = scene) << "Failed to serialize Scene.";

	EXPECT_TRUE(node.IsMap());
	EXPECT_EQ(1u, node.size());

	std::shared_ptr<Scene> newScene = std::make_shared<Scene>(runtime);

	ASSERT_NO_THROW(newScene->decode(node));
	EXPECT_EQ(2u, newScene->getSceneElements().size());

}

TEST(SceneTest, LoadSceneTest)
{
	auto runtime = std::make_shared<Runtime>("config.txt");

	ASSERT_NO_THROW(runtime->loadScene("SceneTestData/scene.yaml"));

	auto scene = runtime->getScene();
	auto element = scene->getSceneElement("element0");
	auto component = element->getComponent("component0");
	component->setLocalActive(false);

	// If we get stale components this would assert as the components would be initialized alread
	ASSERT_NO_THROW(runtime->loadScene("SceneTestData/scene.yaml"));

	scene = runtime->getScene();
	element = scene->getSceneElement("element0");
	component = element->getComponent("component0");

	// Another check for fresh components, if they are from the cache, this would be false
	EXPECT_TRUE(component->isLocalActive());
}

TEST(SceneTest, SceneElementGroups)
{

	using SurgSim::Testing::contains;

	auto runtime = std::make_shared<Runtime>("config.txt");
	auto scene = runtime->getScene();

	auto element1 = std::make_shared<BasicSceneElement>("element1");
	auto element2 = std::make_shared<BasicSceneElement>("element2");

	element1->addToGroup("One");

	auto& groups = scene->getGroups();

	EXPECT_TRUE(groups.getGroups().empty());

	scene->addSceneElement(element1);

	EXPECT_EQ(1L, groups.getGroups().size());
	EXPECT_TRUE(contains(groups.getGroups(), "One"));
	EXPECT_TRUE(contains(groups.getMembers("One"), element1));

	scene->addSceneElement(element2);
	EXPECT_EQ(1L, groups.getGroups().size());

	element2->addToGroup("Two");
	EXPECT_EQ(2L, groups.getGroups().size());
	EXPECT_TRUE(contains(groups.getGroups(), "Two"));
	EXPECT_TRUE(contains(groups.getMembers("Two"), element2));

	element1->removeFromGroup("One");
	EXPECT_EQ(1L, groups.getGroups().size());
	EXPECT_TRUE(contains(groups.getGroups(), "Two"));
	EXPECT_TRUE(contains(groups.getMembers("Two"), element2));

	element1->addToGroup("One");


	std::vector<std::string> groupNames;
	groupNames.push_back("Three");
	groupNames.push_back("Four");

	element1->setGroups(groupNames);
	EXPECT_EQ(4L, groups.getGroups().size());

}

}
}
