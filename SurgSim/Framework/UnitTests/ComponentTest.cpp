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


///\file ComponentManagerTests.cpp test the basic functionality of the component manager
///		 mostly through a mock manager that exposes the private interface and implements
///		 the simplest version of the abstract interface.

#include <boost/uuid/uuid_io.hpp>
#include <gtest/gtest.h>

#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/UnitTests/MockObjects.h"
#include "SurgSim/Testing/SerializationMockComponent.h"

using SurgSim::Framework::Component;
using SurgSim::Framework::Runtime;
using SurgSim::Framework::Scene;

/// Simple component class, no additional features
class TestComponent1 : public Component
{
public:
	explicit TestComponent1(const std::string& name) : Component(name)
	{

	}

	virtual bool doInitialize()
	{
		return true;
	}

	virtual bool doWakeUp()
	{
		return true;
	}

	std::string getClassName() const override
	{
		return "TestComponent1";
	}
};

/// TestComponent with properties, automatic registration in the factory
class TestComponent2 : public Component
{
public:
	explicit TestComponent2(const std::string& name) :
		Component(name),
		valueOne(999),
		valueTwo(888)
	{
		SURGSIM_ADD_SERIALIZABLE_PROPERTY(TestComponent2, int, ValueOne, getValueOne, setValueOne);
		SURGSIM_ADD_SERIALIZABLE_PROPERTY(TestComponent2, int, ValueTwo, getValueTwo, setValueTwo);
	}

	virtual bool doInitialize()
	{
		return true;
	}

	virtual bool doWakeUp()
	{
		return true;
	}

	int getValueOne() const
	{
		return valueOne;
	}
	void setValueOne(int val)
	{
		valueOne = val;
	}
	int getValueTwo() const
	{
		return valueTwo;
	}
	void setValueTwo(int val)
	{
		valueTwo = val;
	}

	std::string getClassName() const override
	{
		return "TestComponent2";
	}


private:
	int valueOne;
	int valueTwo;
};



/// Testcomponent with references to other components
class TestComponent3 : public Component
{
public:
	explicit TestComponent3(const std::string& name) :
		Component(name)
	{
		SURGSIM_ADD_SERIALIZABLE_PROPERTY(
			TestComponent3,
			std::shared_ptr<Component>,
			ComponentOne,
			getComponentOne,
			setComponentOne);

		SURGSIM_ADD_SERIALIZABLE_PROPERTY(
			TestComponent3,
			std::shared_ptr<TestComponent2>,
			ComponentTwo,
			getComponentTwo,
			setComponentTwo);
	}

	virtual bool doInitialize()
	{
		return true;
	}

	virtual bool doWakeUp()
	{
		return true;
	}

	std::shared_ptr<Component> getComponentOne() const
	{
		return m_componentOne;
	}
	void setComponentOne(std::shared_ptr<Component> val)
	{
		m_componentOne = val;
	}
	std::shared_ptr<TestComponent2> getComponentTwo() const
	{
		return m_componentTwo;
	}
	void setComponentTwo(std::shared_ptr<TestComponent2> val)
	{
		m_componentTwo = val;
	}

	std::string getClassName() const override
	{
		return "TestComponent3";
	}

private:

	std::shared_ptr<Component> m_componentOne;
	std::shared_ptr<TestComponent2> m_componentTwo;
};

namespace
{
SURGSIM_REGISTER(SurgSim::Framework::Component, TestComponent2, TestComponent2);
SURGSIM_REGISTER(SurgSim::Framework::Component, TestComponent3, TestComponent3);
}

TEST(ComponentTests, Constructor)
{
	ASSERT_NO_THROW({MockComponent component("Component");});
}

TEST(ComponentTests, SetAndGetSceneElementTest)
{
	std::shared_ptr<Component> mock1 = std::make_shared<MockComponent>("Component");

	std::shared_ptr<MockSceneElement> element1(new MockSceneElement("one"));

	mock1->setSceneElement(element1);

	EXPECT_EQ(element1, mock1->getSceneElement());

}

TEST(ComponentTests, SetAndGetSceneTest)
{
	std::shared_ptr<Component> mock1 = std::make_shared<MockComponent>("Component");

	std::shared_ptr<Scene> scene = std::make_shared<Scene>(std::make_shared<Runtime>());

	mock1->setScene(scene);

	EXPECT_EQ(scene, mock1->getScene());

}

TEST(ComponentTests, PointerEncode)
{
	auto component = std::make_shared<TestComponent1>("TestComponent");
	YAML::Node node = YAML::convert<std::shared_ptr<Component>>::encode(component);

	EXPECT_FALSE(node.IsNull());
	EXPECT_TRUE(node.IsMap());
	EXPECT_EQ(1u, node.size());
	std::string className = node.begin()->first.as<std::string>();
	YAML::Node data = node[className];

	EXPECT_EQ(2u, data.size());
	EXPECT_EQ("TestComponent1", className);
	EXPECT_EQ("TestComponent", data["Name"].as<std::string>());
	EXPECT_EQ(to_string(component->getUuid()), data["Id"].as<std::string>());
}

TEST(ComponentTests, NullPointerSerialization)
{
	// Encode a nullptr
	std::shared_ptr<Component> emptyComponent;
	EXPECT_NO_THROW(YAML::convert<std::shared_ptr<Component>>::encode(emptyComponent));
	YAML::Node node = YAML::convert<std::shared_ptr<Component>>::encode(emptyComponent);
	EXPECT_FALSE(node.IsMap());

	// Decode an empty node onto an empty receiver
	std::shared_ptr<Component> emptyComponentReceiver;
	EXPECT_NO_THROW(YAML::convert<std::shared_ptr<Component>>::decode(node, emptyComponentReceiver););
	EXPECT_EQ(nullptr, emptyComponentReceiver);

	// Decode an empty node onto a non-empty receiver.
	std::shared_ptr<Component> component = std::make_shared<MockComponent>("TestMockComponent");
	EXPECT_NO_THROW(YAML::convert<std::shared_ptr<Component>>::decode(node, component););
	EXPECT_NE(nullptr, component);
	EXPECT_EQ("TestMockComponent", component->getName());

	// A derived class of component having empty components.
	auto container = std::make_shared<TestComponent3>("TestComponent3");
	YAML::Node containerNode = YAML::convert<Component>::encode(*container);
	std::shared_ptr<TestComponent3> newContainer = std::dynamic_pointer_cast<TestComponent3>(
				containerNode.as<std::shared_ptr<Component>>());
	EXPECT_EQ(nullptr, newContainer->getComponentOne());
	EXPECT_EQ(nullptr, newContainer->getComponentTwo());
	EXPECT_EQ(container->getName(), newContainer->getName());
}

TEST(ComponentTests, ConvertFactoryTest)
{
	Component::getFactory().registerClass<TestComponent1>("TestComponent1");

	YAML::Node node;
	node["TestComponent1"]["Name"] = "ComponentName";
	node["TestComponent1"]["Id"] = "ConvertFactoryTest_TestComponent1";

	auto component = node.as<std::shared_ptr<Component>>();

	auto testComponent = std::dynamic_pointer_cast<TestComponent1>(component);

	EXPECT_NE(nullptr, testComponent);
	EXPECT_EQ("ComponentName", testComponent->getName());
	EXPECT_EQ("TestComponent1", testComponent->getClassName());

	YAML::Node invalidNode;
	node["Invalid"]["Name"] = "Other";
	node["Invalid"]["Id"] = "ConvertFactoryTest_TestComponent2";

	// Should not be able to convert this class
	EXPECT_ANY_THROW({auto result = invalidNode.as<std::shared_ptr<Component>>();});
}

TEST(ComponentTests, MacroRegistrationTest)
{
	YAML::Node node;
	node["TestComponent2"]["Name"] = "ComponentName";
	node["TestComponent2"]["Id"] = "AutomaticRegistrationTest_ComponentName";


	auto component = node.as<std::shared_ptr<Component>>();

	auto testComponent = std::dynamic_pointer_cast<TestComponent2>(component);

	EXPECT_NE(nullptr, testComponent);
	EXPECT_EQ("ComponentName", testComponent->getName());
	EXPECT_EQ("TestComponent2", testComponent->getClassName());
	EXPECT_TRUE(testComponent->isActive());
}

TEST(ComponentTests, DecodeSharedReferences)
{
	YAML::Node node;
	node["TestComponent2"]["Name"] = "OneComponentName";
	node["TestComponent2"]["Id"] = "DecodeSharedReferences_OneComponentName";

	auto component1 = node.as<std::shared_ptr<Component>>();
	EXPECT_NE(nullptr, component1);
	EXPECT_TRUE(component1->isActive());

	auto component1copy = node.as<std::shared_ptr<Component>>();
	EXPECT_NE(nullptr, component1copy);
	EXPECT_EQ(component1, component1copy);
	EXPECT_TRUE(component1copy->isActive());

	node["TestComponent2"]["Id"] = "DecodeSharedReferences_TwoComponentName";

	auto component2 = node.as<std::shared_ptr<Component>>();
	EXPECT_NE(nullptr, component2);
	EXPECT_NE(component2, component1);
	EXPECT_NE(component2, component1copy);
	EXPECT_TRUE(component2->isActive());

	node["TestComponent2"]["IsLocalActive"] = false;
	auto component3 = node.as<std::shared_ptr<Component>>();
	EXPECT_FALSE(component3->isLocalActive());
}

TEST(ComponentTests, EncodeComponent)
{
	auto component = std::make_shared<TestComponent2>("TestComponent");
	component->setValueOne(1);
	component->setValueTwo(2);

	YAML::Node node = YAML::convert<Component>::encode(*component);
	std::string className = node.begin()->first.as<std::string>();
	YAML::Node data = node[className];

	EXPECT_EQ("TestComponent", (data["Name"].IsDefined() ? data["Name"].as<std::string>() : "undefined !"));
	EXPECT_EQ("TestComponent2", className);
	EXPECT_EQ(1, (data["ValueOne"].IsDefined() ? data["ValueOne"].as<int>() : 0xbad));
	EXPECT_EQ(2, (data["ValueTwo"].IsDefined() ? data["ValueTwo"].as<int>() : 0xbad));
	EXPECT_TRUE(data["IsLocalActive"].IsDefined());
	EXPECT_TRUE(data["IsLocalActive"].as<bool>());
}

TEST(ComponentTests, DecodeComponent)
{
	YAML::Node node;
	node["TestComponent2"]["Name"] = "TestComponentName";
	node["TestComponent2"]["Id"] = "DecodeComponent_TestComponentName";
	node["TestComponent2"]["ValueOne"] = 100;
	node["TestComponent2"]["ValueTwo"] = 101;
	node["TestComponent2"]["IsLocalActive"] = false;


	auto component = node.as<std::shared_ptr<Component>>();

	auto testComponent = std::dynamic_pointer_cast<TestComponent2>(component);

	EXPECT_NE(nullptr, testComponent);
	EXPECT_EQ("TestComponentName", testComponent->getName());
	EXPECT_EQ(100, testComponent->getValueOne());
	EXPECT_EQ(101, testComponent->getValueTwo());
	EXPECT_FALSE(testComponent->isLocalActive());
}

TEST(ComponentTests, ComponentReferences)
{
	auto containerComponent = std::make_shared<TestComponent3>("Root");
	auto componentOne = std::make_shared<TestComponent2>("Component1");
	auto componentTwo = std::make_shared<TestComponent2>("Component2");

	componentOne->setValueOne(100);
	componentOne->setValueTwo(101);

	componentTwo->setValueOne(200);
	componentTwo->setValueTwo(201);

	containerComponent->setComponentOne(componentOne);
	containerComponent->setComponentTwo(componentTwo);

	// Push the components onto the node, note that one component is serialized before the container
	// and the other after the container, this is intentional to test referencing
	YAML::Node node;

	// because yaml internally does not know about our conversions we have to make them explicit
	node.push_back(YAML::convert<Component>::encode(*componentOne));
	node.push_back(YAML::convert<Component>::encode(*containerComponent));
	node.push_back(YAML::convert<Component>::encode(*componentTwo));

	// Convert from node to shared component
	auto resultOne =
		std::dynamic_pointer_cast<TestComponent2>(node[0].as<std::shared_ptr<Component>>());
	auto resultContainer =
		std::dynamic_pointer_cast<TestComponent3>(node[1].as<std::shared_ptr<Component>>());
	auto resultTwo =
		std::dynamic_pointer_cast<TestComponent2>(node[2].as<std::shared_ptr<Component>>());

	// All of the components should have been de-serialized
	ASSERT_NE(nullptr, resultContainer);
	ASSERT_NE(nullptr, resultOne);
	ASSERT_NE(nullptr, resultTwo);

	// The references should have been resolved correctly
	ASSERT_EQ(resultContainer->getComponentOne(), resultOne);
	ASSERT_EQ(resultContainer->getComponentTwo(), resultTwo);


	// All components should have the correct values ...
	EXPECT_EQ(componentOne->getValueOne(),
			  boost::any_cast<int>(resultContainer->getComponentOne()->getValue("ValueOne")));

	EXPECT_EQ(componentOne->getValueTwo(),
			  boost::any_cast<int>(resultContainer->getComponentOne()->getValue("ValueTwo")));

	EXPECT_EQ(componentTwo->getValueOne(),
			  boost::any_cast<int>(resultContainer->getComponentTwo()->getValue("ValueOne")));

	EXPECT_EQ(componentTwo->getValueTwo(),
			  boost::any_cast<int>(resultContainer->getComponentTwo()->getValue("ValueTwo")));
}

TEST(ComponentTests, MockComponent)
{
	auto component = SurgSim::Framework::Component::getFactory().create("MockComponent", "testcomponent");

	ASSERT_NE(nullptr, component);

	/// SerializationMockComponent does not have an explicit definition anywhere in the code
	/// there is no SerializationMockComponent, but this should still succeed, this test protects
	/// against linker optimization
	auto nonDefinedComponent = SurgSim::Framework::Component::getFactory().create(
								   "SerializationMockComponent",
								   "othercomponent");

	ASSERT_NE(nullptr, nonDefinedComponent) << "It looks like SerializationMockComponent was lost during linkage.";

	YAML::Node node = YAML::convert<Component>::encode(*nonDefinedComponent);

	auto roundtripComponent = node.as<std::shared_ptr<Component>>();

	ASSERT_NE(nullptr, roundtripComponent);

	EXPECT_EQ("SerializationMockComponent", roundtripComponent->getClassName());
	EXPECT_EQ("othercomponent", roundtripComponent->getName());
}

#include "SurgSim/Framework/PoseComponent.h"
#include "SurgSim/Math/RigidTransform.h"

TEST(ComponentTests, PoseComponentTest)
{
	std::shared_ptr<SurgSim::Framework::Component> component;
	ASSERT_NO_THROW(component = SurgSim::Framework::Component::getFactory().create(
									"SurgSim::Framework::PoseComponent",
									"pose"));

	EXPECT_EQ("SurgSim::Framework::PoseComponent", component->getClassName());

	SurgSim::Math::RigidTransform3d pose(SurgSim::Math::RigidTransform3d::Identity());

	component->setValue("Pose", pose);
	YAML::Node node(YAML::convert<SurgSim::Framework::Component>::encode(*component));

	auto decoded = std::dynamic_pointer_cast<SurgSim::Framework::PoseComponent>(
					   node.as<std::shared_ptr<SurgSim::Framework::Component>>());

	EXPECT_NE(nullptr, decoded);
	EXPECT_TRUE(pose.isApprox(decoded->getValue<SurgSim::Math::RigidTransform3d>("Pose")));
}


TEST(ComponentTests, SetActiveTest)
{
	std::shared_ptr<Component> component = std::make_shared<MockComponent>("Component");
	EXPECT_TRUE(component->isActive());
	EXPECT_TRUE(component->isLocalActive());
	EXPECT_NO_THROW(component->setLocalActive(false));
	EXPECT_FALSE(component->isActive());
	EXPECT_FALSE(component->isLocalActive());

	// RW property test
	component->setValue("IsLocalActive", true);
	EXPECT_TRUE(component->isActive());
	EXPECT_TRUE(component->isLocalActive());
	EXPECT_TRUE(component->getValue<bool>("IsActive"));
	component->setValue("IsLocalActive", false);
	EXPECT_FALSE(component->isActive());
	EXPECT_FALSE(component->isLocalActive());
	EXPECT_FALSE(component->getValue<bool>("IsActive"));

	auto sceneElement = std::make_shared<SurgSim::Framework::BasicSceneElement>("SceneElement");
	sceneElement->addComponent(component);
	EXPECT_TRUE(sceneElement->isActive());

	// An inactive component in an active SceneElement is 'inactive'.
	EXPECT_FALSE(component->isActive());

	// An active component in an active SceneElement is 'active'.
	component->setLocalActive(true);
	EXPECT_TRUE(component->isActive());

	sceneElement->setActive(false);
	// An active component in an inactive SceneElement is 'inactive'.
	EXPECT_FALSE(component->isActive());

	// An inactive component in an inactive SceneElement is 'inactive'.
	component->setLocalActive(false);
	EXPECT_FALSE(component->isActive());

	// During serialization, it's Component::m_isActive being serialized, not Component::isActive().
	component->setValue("IsLocalActive", true);
	YAML::Node node = sceneElement->encode(true);
	YAML::Node data = node["SurgSim::Framework::BasicSceneElement"];

	// Decode the component only.
	std::shared_ptr<SurgSim::Framework::Component> decodedComponent;
	for (auto nodeIt = data["Components"].begin(); nodeIt != data["Components"].end(); ++nodeIt)
	{
		if ("MockComponent" == nodeIt->begin()->first.as<std::string>())
		{
			decodedComponent = nodeIt->as<std::shared_ptr<MockComponent>>();
			break;
		}
	}
	ASSERT_NE(nullptr, decodedComponent);
	EXPECT_EQ(nullptr, decodedComponent->getSceneElement());
	EXPECT_TRUE(decodedComponent->isActive());
	EXPECT_TRUE(decodedComponent->isLocalActive());

	// Decode the component with a SceneElement. The SceneElement's activity (active/inactive) will
	// affect the return value of Component::isActive().
	decodedComponent = nullptr;
	auto decodedSceneElement = std::make_shared<SurgSim::Framework::BasicSceneElement>("Decoded");
	decodedSceneElement->decode(node);
	EXPECT_FALSE(decodedSceneElement->isActive());
	decodedComponent = decodedSceneElement->getComponent("Component");
	EXPECT_NE(nullptr, decodedComponent->getSceneElement());
	EXPECT_FALSE(decodedComponent->isActive());
	EXPECT_TRUE(decodedComponent->isLocalActive());
}

TEST(ComponentTests, CheckAndConvertTest)
{
	using SurgSim::Framework::checkAndConvert;

	auto original = std::make_shared<MockComponent>("test");
	auto other = std::make_shared<MockBehavior>("other");
	std::shared_ptr<Component> source = original;
	std::shared_ptr<MockComponent> result;

	EXPECT_NO_THROW(result = checkAndConvert<MockComponent>(source, "MockComponent"));
	EXPECT_EQ(result, original);


	EXPECT_ANY_THROW(result = checkAndConvert<MockComponent>(other, "MockBehavior"));
}
