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
#include <SurgSim/Framework/SceneElement.h>
#include "MockObjects.h"

using SurgSim::Framework::Component;
using SurgSim::Framework::SceneElement;

TEST(SceneElementTest, Constructor)
{
	ASSERT_NO_THROW({MockSceneElement element;});
}

TEST(SceneElementTest, UpdateFunctions)
{
	MockSceneElement element;

	element.update(1.0);
	EXPECT_TRUE(element.didUpdate);

	element.lateUpdate(1.0);
	EXPECT_TRUE(element.didLateUpdate);

	element.fixedRateUpdate(1.0);
	EXPECT_TRUE(element.didFixedUpdate);
}

TEST(SceneElementTest, AddAndAccessComponents)
{
	MockSceneElement element;
	std::shared_ptr<MockComponent> component1(new MockComponent("TestComponent1"));
	std::shared_ptr<MockComponent> component2(new MockComponent("TestComponent2"));

	EXPECT_TRUE(element.addComponent(component1));
	EXPECT_TRUE(element.addComponent(component2));

	// Should not be able to add two with the same name
	EXPECT_FALSE(element.addComponent(component1));

	// Should not be able to add nullptr component
	EXPECT_ANY_THROW(element.addComponent(nullptr));

	std::shared_ptr<Component> fetched(element.getComponent("TestComponent1"));
	ASSERT_NE(nullptr, fetched);
	EXPECT_EQ("TestComponent1", fetched->getName());

	fetched = element.getComponent("Random");
	EXPECT_EQ(nullptr, fetched);
}

TEST(SceneElementTest, RemoveComponents)
{
	MockSceneElement element;
	std::shared_ptr<MockComponent> component1(new MockComponent("TestComponent1"));
	std::shared_ptr<MockComponent> component2(new MockComponent("TestComponent2"));

	EXPECT_TRUE(element.addComponent(component1));
	EXPECT_TRUE(element.addComponent(component2));


	EXPECT_TRUE(element.removeComponent("TestComponent2"));
	EXPECT_EQ(nullptr, element.getComponent("TestComponent2"));

	// Add back should work
	EXPECT_TRUE(element.addComponent(component2));

	EXPECT_TRUE(element.removeComponent(component1));
	EXPECT_EQ(nullptr, element.getComponent("TestComponent1"));
}

TEST(SceneElementTest, GetComponentsTest)
{
	MockSceneElement element;
	std::shared_ptr<MockComponent> component1(new MockComponent("TestComponent1"));
	std::shared_ptr<MockComponent> component2(new MockComponent("TestComponent2"));

	element.addComponent(component1);
	EXPECT_EQ(1U, element.getComponents().size());

	element.addComponent(component2);
	EXPECT_EQ(2U, element.getComponents().size());

	std::vector<std::shared_ptr<Component>> components = element.getComponents();

	EXPECT_NE(components.end(), std::find(components.cbegin(), components.cend(), component1));
	EXPECT_NE(components.end(), std::find(components.cbegin(), components.cend(), component2));

	element.removeComponent(component1);
	components = element.getComponents();
	EXPECT_EQ(1U, components.size());
}

TEST(SceneElementTest, InitComponentTest)
{
	MockSceneElement element;
	std::shared_ptr<MockComponent> component1(new MockComponent("TestComponent1"));
	std::shared_ptr<MockComponent> component2(new MockComponent("TestComponent2"));

	element.addComponent(component1);
	element.addComponent(component2);

	element.initialize();

	EXPECT_TRUE(element.didInit);
	EXPECT_FALSE(element.didWakeUp);
	EXPECT_TRUE(component1->didInit);
	EXPECT_FALSE(component1->didWakeUp);
	EXPECT_TRUE(component2->didInit);
	EXPECT_FALSE(component2->didWakeUp);

	element.wakeUp();

	EXPECT_TRUE(element.didWakeUp);
	EXPECT_TRUE(component1->didWakeUp);
	EXPECT_TRUE(component2->didWakeUp);
}