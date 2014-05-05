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
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/Vector.h"

#include "MockObjects.h"  //NOLINT

using SurgSim::Framework::Component;
using SurgSim::Framework::SceneElement;

TEST(SceneElementTest, Constructor)
{
	ASSERT_NO_THROW({MockSceneElement element;});
}

TEST(SceneElementTest, Pose)
{
	using SurgSim::Math::makeRigidTransform;
	using SurgSim::Math::Quaterniond;
	using SurgSim::Math::RigidTransform3d;
	using SurgSim::Math::Vector3d;

	MockSceneElement element;
	EXPECT_TRUE(element.getPose().isApprox(RigidTransform3d::Identity()));

	RigidTransform3d pose(makeRigidTransform(Quaterniond(0.0, 1.0, 0.0, 0.0), Vector3d(1.0, 2.0, 3.0)));
	element.setPose(pose);
	EXPECT_TRUE(element.getPose().isApprox(pose));
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

TEST(SceneElementTest, AddAndTestComponents)
{
	std::shared_ptr<MockSceneElement> element = std::make_shared<MockSceneElement>();
	std::shared_ptr<MockComponent> component = std::make_shared<MockComponent>("TestComponent");

	EXPECT_TRUE(element->addComponent(component));

	EXPECT_EQ(component->getSceneElement(), element);

	// Should be able to get the same scene between element and component after addComponent
	EXPECT_EQ(component->getScene(), element->getScene() );
}

TEST(SceneElementTest, AddAndAccessComponents)
{
	std::shared_ptr<MockSceneElement> element(new MockSceneElement());

	std::shared_ptr<MockComponent> component1(new MockComponent("TestComponent1"));
	std::shared_ptr<MockComponent> component2(new MockComponent("TestComponent2"));


	EXPECT_TRUE(element->addComponent(component1));
	EXPECT_TRUE(element->addComponent(component2));

	// Should not be able to add two with the same name
	EXPECT_FALSE(element->addComponent(component1));

	// Should not be able to add nullptr component
	EXPECT_ANY_THROW(element->addComponent(nullptr));

	std::shared_ptr<Component> fetched(element->getComponent("TestComponent1"));
	ASSERT_NE(nullptr, fetched);
	EXPECT_EQ("TestComponent1", fetched->getName());

	fetched = element->getComponent("Random");
	EXPECT_EQ(nullptr, fetched);
}

TEST(SceneElementTest, RemoveComponents)
{
	std::shared_ptr<MockSceneElement> element(new MockSceneElement());

	std::shared_ptr<MockComponent> component1(new MockComponent("TestComponent1"));
	std::shared_ptr<MockComponent> component2(new MockComponent("TestComponent2"));

	EXPECT_TRUE(element->addComponent(component1));
	EXPECT_TRUE(element->addComponent(component2));


	EXPECT_TRUE(element->removeComponent("TestComponent2"));
	EXPECT_EQ(nullptr, element->getComponent("TestComponent2"));

	// Add back should work
	EXPECT_TRUE(element->addComponent(component2));

	EXPECT_TRUE(element->removeComponent(component1));
	EXPECT_EQ(nullptr, element->getComponent("TestComponent1"));
}

TEST(SceneElementTest, GetComponentsTest)
{
	std::shared_ptr<MockSceneElement> element(new MockSceneElement());

	std::shared_ptr<MockComponent> component1(new MockComponent("TestComponent1"));
	std::shared_ptr<MockComponent> component2(new MockComponent("TestComponent2"));

	element->addComponent(component1);
	EXPECT_EQ(1u, element->getComponents().size());

	element->addComponent(component2);
	EXPECT_EQ(2u, element->getComponents().size());

	std::vector<std::shared_ptr<Component>> components = element->getComponents();

	EXPECT_NE(components.end(), std::find(components.cbegin(), components.cend(), component1));
	EXPECT_NE(components.end(), std::find(components.cbegin(), components.cend(), component2));

	element->removeComponent(component1);
	components = element->getComponents();
	EXPECT_EQ(1u, components.size());
}

TEST(SceneElementTest, GetTypedComponentsTests)
{
	std::shared_ptr<SceneElement> element(new MockSceneElement());
	std::shared_ptr<MockBehavior> behavior(new MockBehavior("MockBehavior"));
	std::shared_ptr<MockComponent> component1(new MockComponent("Test Component1"));
	std::shared_ptr<MockComponent> component2(new MockComponent("Test Component2"));

	element->addComponent(behavior);
	element->addComponent(component1);
	element->addComponent(component2);

	EXPECT_EQ(1u, element->getComponents<MockBehavior>().size());
	EXPECT_EQ(2u, element->getComponents<MockComponent>().size());

	element->removeComponent(component1);
	EXPECT_EQ(1u, element->getComponents<MockComponent>().size());

	element->removeComponent(component2);
	EXPECT_EQ(0u, element->getComponents<MockComponent>().size());

}

TEST(SceneElementTest, InitComponentTest)
{
	std::shared_ptr<MockSceneElement> element(new MockSceneElement());
	std::shared_ptr<MockComponent> component1(new MockComponent("TestComponent1"));
	std::shared_ptr<MockComponent> component2(new MockComponent("TestComponent2"));

	element->addComponent(component1);
	element->addComponent(component2);

	element->initialize();

	EXPECT_TRUE(element->didInit);
}

TEST(SceneElementTest, DoubleInitTest)
{
	std::shared_ptr<MockSceneElement> element(new MockSceneElement());

	EXPECT_FALSE(element->didInit);

	element->initialize();
	EXPECT_TRUE(element->didInit);

	ASSERT_ANY_THROW(element->initialize());
}

