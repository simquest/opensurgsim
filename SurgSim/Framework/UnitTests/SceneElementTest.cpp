// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/PoseComponent.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Testing/Utilities.h"

#include "MockObjects.h"  //NOLINT

using SurgSim::Framework::Component;
using SurgSim::Framework::PoseComponent;
using SurgSim::Framework::SceneElement;

TEST(SceneElementTest, Constructor)
{
	ASSERT_NO_THROW(MockSceneElement element);
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
	EXPECT_TRUE(element.getPoseComponent()->getPose().isApprox(pose));
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

TEST(SceneElementTest, AddAndTestComponentsAssembly)
{
	std::shared_ptr<SurgSim::Framework::Runtime> runtime = std::make_shared<SurgSim::Framework::Runtime>();
	std::shared_ptr<MockManager> manager = std::make_shared<MockManager>();
	std::shared_ptr<MockSceneElement> element = std::make_shared<MockSceneElement>();
	std::shared_ptr<MockComponent> component = std::make_shared<MockComponent>("TestComponent");
	runtime->addManager(manager);

	EXPECT_TRUE(element->addComponent(component));

	// SceneElement in Component should be set immediately
	EXPECT_EQ(nullptr, component->getScene());
	EXPECT_NE(nullptr, component->getSceneElement());

	// Scene and SceneElement should be set after add
	runtime->getScene()->addSceneElement(element);
	EXPECT_EQ(element->getScene(), component->getScene());
	EXPECT_EQ(element, component->getSceneElement());

	// Verify the component made it to the manager
	runtime->start(true);
	boost::this_thread::sleep(boost::posix_time::milliseconds(150));
	ASSERT_EQ(1, manager->getComponents().size());
	EXPECT_EQ(component, manager->getComponents()[0]);
	runtime->stop();
}

class ConstructorSceneElement : public MockSceneElement
{
public:
	ConstructorSceneElement()
	{
		component = std::make_shared<MockComponent>("TestComponent");
		addComponent(component);
	}

	std::shared_ptr<MockComponent> component;
};

TEST(SceneElementTest, AddAndTestComponentsConstructor)
{
	std::shared_ptr<SurgSim::Framework::Runtime> runtime = std::make_shared<SurgSim::Framework::Runtime>();
	std::shared_ptr<MockManager> manager = std::make_shared<MockManager>();
	std::shared_ptr<ConstructorSceneElement> element = std::make_shared<ConstructorSceneElement>();
	auto component = element->component;
	runtime->addManager(manager);


	// Scene and SceneElement in Component will not be set until initialization (add to scene).
	EXPECT_EQ(nullptr, component->getScene());
	EXPECT_EQ(nullptr, component->getSceneElement());

	// Scene and SceneElement should be set after add
	runtime->getScene()->addSceneElement(element);
	EXPECT_EQ(element->getScene(), component->getScene());
	EXPECT_EQ(element, component->getSceneElement());

	// Verify the component made it to the manager
	runtime->start(true);
	boost::this_thread::sleep(boost::posix_time::milliseconds(150));
	ASSERT_EQ(1, manager->getComponents().size());
	EXPECT_EQ(component, manager->getComponents()[0]);
	runtime->stop();
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

TEST(SceneElementTest, RemoveComponent)
{
	std::shared_ptr<SurgSim::Framework::Runtime> runtime = std::make_shared<SurgSim::Framework::Runtime>();
	std::shared_ptr<MockManager> manager = std::make_shared<MockManager>();
	std::shared_ptr<MockSceneElement> element(new MockSceneElement());

	std::shared_ptr<MockComponent> component1(new MockComponent("TestComponent1"));
	std::shared_ptr<MockComponent> component2(new MockComponent("TestComponent2"));

	EXPECT_TRUE(element->addComponent(component1));
	EXPECT_TRUE(element->addComponent(component2));

	runtime->addManager(manager);
	runtime->getScene()->addSceneElement(element);
	runtime->start(true);
	boost::this_thread::sleep(boost::posix_time::milliseconds(150));
	EXPECT_EQ(2, manager->getComponents().size());

	EXPECT_TRUE(element->removeComponent("TestComponent2"));
	EXPECT_EQ(nullptr, element->getComponent("TestComponent2"));
	runtime->step();
	boost::this_thread::sleep(boost::posix_time::milliseconds(150));
	EXPECT_EQ(1, manager->getComponents().size());

	EXPECT_TRUE(element->removeComponent(component1));
	EXPECT_EQ(nullptr, element->getComponent("TestComponent1"));
	runtime->step();
	boost::this_thread::sleep(boost::posix_time::milliseconds(150));
	EXPECT_EQ(0, manager->getComponents().size());

	runtime->stop();
}

TEST(SceneElementTest, RemoveComponents)
{
	std::shared_ptr<SurgSim::Framework::Runtime> runtime = std::make_shared<SurgSim::Framework::Runtime>();
	std::shared_ptr<MockManager> manager = std::make_shared<MockManager>();
	std::shared_ptr<MockSceneElement> element(new MockSceneElement());

	std::shared_ptr<MockComponent> component1(new MockComponent("TestComponent1"));
	std::shared_ptr<MockComponent> component2(new MockComponent("TestComponent2"));

	EXPECT_TRUE(element->addComponent(component1));
	EXPECT_TRUE(element->addComponent(component2));

	runtime->addManager(manager);
	runtime->getScene()->addSceneElement(element);
	runtime->start(true);
	boost::this_thread::sleep(boost::posix_time::milliseconds(150));
	EXPECT_EQ(2, manager->getComponents().size());

	element->removeComponents();
	runtime->step();
	boost::this_thread::sleep(boost::posix_time::milliseconds(150));
	EXPECT_EQ(0, manager->getComponents().size());

	runtime->stop();
}

TEST(SceneElementTest, GetComponentsTest)
{
	std::shared_ptr<MockSceneElement> element(new MockSceneElement());

	std::shared_ptr<MockComponent> component1(new MockComponent("TestComponent1"));
	std::shared_ptr<MockComponent> component2(new MockComponent("TestComponent2"));

	element->addComponent(component1);
	EXPECT_EQ(2u, element->getComponents().size());

	element->addComponent(component2);
	EXPECT_EQ(3u, element->getComponents().size());

	std::vector<std::shared_ptr<Component>> components = element->getComponents();

	EXPECT_NE(components.end(), std::find(components.cbegin(), components.cend(), component1));
	EXPECT_NE(components.end(), std::find(components.cbegin(), components.cend(), component2));

	element->removeComponent(component1);
	components = element->getComponents();
	EXPECT_EQ(2u, components.size());
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

TEST(SceneElement, InGroup)
{
	std::shared_ptr<MockSceneElement> element(new MockSceneElement());

	EXPECT_FALSE(element->inGroup("xxx"));
	element->addToGroup("One");
	EXPECT_TRUE(element->inGroup("One"));
	EXPECT_FALSE(element->inGroup("xxx"));
}

TEST(SceneElementTest, NoSceneGroupsTest)
{
	std::shared_ptr<MockSceneElement> element(new MockSceneElement());
	EXPECT_TRUE(element->getGroups().empty());

	element->addToGroup("One");
	EXPECT_TRUE(element->inGroup("One"));

	element->addToGroup("Two");
	EXPECT_TRUE(element->inGroup("One"));
	EXPECT_TRUE(element->inGroup("Two"));

	element->removeFromGroup("One");
	EXPECT_TRUE(element->inGroup("Two"));
	EXPECT_FALSE(element->inGroup("One"));

	std::vector<std::string> newGroups;
	newGroups.push_back("Three");
	newGroups.push_back("Four");

	element->setGroups(newGroups);
	EXPECT_FALSE(element->inGroup("One"));
	EXPECT_FALSE(element->inGroup("Two"));
	EXPECT_TRUE(element->inGroup("Three"));
	EXPECT_TRUE(element->inGroup("Four"));

	std::vector<std::string> empty;
	element->setGroups(empty);
	EXPECT_FALSE(element->inGroup("One"));
	EXPECT_FALSE(element->inGroup("Two"));
	EXPECT_FALSE(element->inGroup("Three"));
	EXPECT_FALSE(element->inGroup("Four"));

}

TEST(SceneElementTest, Include)
{
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");

	std::string file;

	ASSERT_TRUE(runtime->getApplicationData()->tryFindFile("SceneElementTest/includer.yaml", &file));

	YAML::Node node = YAML::LoadFile(file);
	ASSERT_TRUE(node.IsSequence());

	auto elements = node.as<std::vector<std::shared_ptr<SurgSim::Framework::SceneElement>>>();

	EXPECT_EQ(4u, elements.size());
}


TEST(SceneElementTest, IncludeCircle)
{
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");

	std::string file;

	ASSERT_TRUE(runtime->getApplicationData()->tryFindFile("SceneElementTest/circle-1.yaml", &file));

	YAML::Node node = YAML::LoadFile(file);
	ASSERT_TRUE(node.IsSequence());

	ASSERT_THROW(auto elements = node.as<std::vector<std::shared_ptr<SurgSim::Framework::SceneElement>>>(),
				 SurgSim::Framework::AssertionFailure);
}

TEST(SceneElementTest, SingleAsArray)
{
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");

	std::string file;

	ASSERT_TRUE(runtime->getApplicationData()->tryFindFile("SceneElementTest/single.yaml", &file));

	YAML::Node node = YAML::LoadFile(file);
	ASSERT_FALSE(node.IsSequence());

	auto elements = node.as<std::vector<std::shared_ptr<SurgSim::Framework::SceneElement>>>();

	EXPECT_EQ(1u, elements.size());
}
