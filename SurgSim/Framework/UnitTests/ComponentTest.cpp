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

#include <gtest/gtest.h>

#include <boost/uuid/uuid_io.hpp>

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/FrameworkConvert.h"

#include "SurgSim/Framework/UnitTests/MockObjects.h"
#include "SurgSim/Framework/UnitTests/SerializationMockComponent.h"


using SurgSim::Framework::Component;
using SurgSim::Framework::Scene;
using SurgSim::Framework::Runtime;

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
SURGSIM_REGISTER(SurgSim::Framework::Component, TestComponent2);
SURGSIM_REGISTER(SurgSim::Framework::Component, TestComponent3);
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
	std::shared_ptr<Component> emptyComponent;
	EXPECT_ANY_THROW(YAML::convert<std::shared_ptr<Component>>::encode(emptyComponent));

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
}

TEST(ComponentTests, DecodeSharedReferences)
{
	YAML::Node node;
	node["TestComponent2"]["Name"] = "OneComponentName";
	node["TestComponent2"]["Id"] = "DecodeSharedReferences_OneComponentName";

	auto component1 = node.as<std::shared_ptr<Component>>();
	EXPECT_NE(nullptr, component1);

	auto component1copy = node.as<std::shared_ptr<Component>>();
	EXPECT_NE(nullptr, component1copy);
	EXPECT_EQ(component1, component1copy);

	node["TestComponent2"]["Id"] = "DecodeSharedReferences_TwoComponentName";

	auto component2 = node.as<std::shared_ptr<Component>>();
	EXPECT_NE(nullptr, component2);
	EXPECT_NE(component2, component1);
	EXPECT_NE(component2, component1copy);
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
}

TEST(ComponentTests, DecodeComponent)
{
	YAML::Node node;
	node["TestComponent2"]["Name"] = "TestComponentName";
	node["TestComponent2"]["Id"] = "DecodeComponent_TestComponentName";
	node["TestComponent2"]["ValueOne"] = 100;
	node["TestComponent2"]["ValueTwo"] = 101;

	auto component = node.as<std::shared_ptr<Component>>();

	auto testComponent = std::dynamic_pointer_cast<TestComponent2>(component);

	EXPECT_NE(nullptr, testComponent);
	EXPECT_EQ("TestComponentName", testComponent->getName());
	EXPECT_EQ(100, testComponent->getValueOne());
	EXPECT_EQ(101, testComponent->getValueTwo());
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
	/// there is not SerializationMockComponent, but this should still suceed, this test protects
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