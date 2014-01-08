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
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/Component.h"

#include "SurgSim/Framework/UnitTests/MockObjects.h"
#include <mutex>

using SurgSim::Framework::Component;
using SurgSim::Framework::Scene;
using SurgSim::Framework::Runtime;

class TestComponent1 : public SurgSim::Framework::Component
{
public:
	TestComponent1(const std::string& name) : Component(name)
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

	std::string getClassName()
	{
		return "TestComponent1";
	}
};

class TestComponent2 : public SurgSim::Framework::Component
{
public:
	TestComponent2(const std::string& name) :
		Component(name),
		valueOne(999),
		valueTwo(999)
	{
		SURGSIM_ADD_SERIALIZABLE_PROPERTY(TestComponent2, int, valueOne, getValueOne, setValueOne);
		SURGSIM_ADD_SERIALIZABLE_PROPERTY(TestComponent2, int, valueTwo, getValueTwo, setValueTwo);
	}

	virtual bool doInitialize()
	{
		return true;
	}

	virtual bool doWakeUp()
	{
		return true;
	}

	int getValueOne() const { return valueOne; }
	void setValueOne(int val) { valueOne = val; }
	int getValueTwo() const { return valueTwo; }
	void setValueTwo(int val) { valueTwo = val; }

	std::string getClassName() const override
	{
		return "TestComponent2";
	}

private:
	class MetaData
	{
	public:
		MetaData()
		{
			YAML::convert<std::shared_ptr<SurgSim::Framework::Component>>::registerClass<TestComponent2>("TestComponent2");
		}


	};

	static MetaData Meta;

	int valueOne;
	int valueTwo;
};

TestComponent2::MetaData TestComponent2::Meta;

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

	std::shared_ptr<Scene> scene = std::make_shared<Scene>();

	mock1->setScene(scene);

	EXPECT_EQ(scene, mock1->getScene());

}

TEST(ComponentTests, ConvertFactoryTest)
{
	YAML::convert<std::shared_ptr<SurgSim::Framework::Component>>::registerClass<TestComponent1>("TestComponent1");

	YAML::Node node;
	node["name"] = "ComponentName";
	node["className"] = "TestComponent1";
	node["id"] = "ComponentId";

	auto component = node.as<std::shared_ptr<SurgSim::Framework::Component>>();

	auto testComponent = std::dynamic_pointer_cast<TestComponent1>(component);

	EXPECT_NE(nullptr, testComponent);
	EXPECT_EQ("ComponentName", testComponent->getName());
	EXPECT_EQ("TestComponent1", testComponent->getClassName());
}

TEST(ComponentTests, AutomaticRegistrationTest)
{
	YAML::Node node;
	node["name"] = "ComponentName";
	node["className"] = "TestComponent2";
	node["id"] = "FakeId";


	auto component = node.as<std::shared_ptr<SurgSim::Framework::Component>>();

	auto testComponent = std::dynamic_pointer_cast<TestComponent2>(component);

	EXPECT_NE(nullptr, testComponent);
	EXPECT_EQ("ComponentName", testComponent->getName());
	EXPECT_EQ("TestComponent2", testComponent->getClassName());
}

TEST(ComponentTests, DecodeSharedReferences)
{
	YAML::Node node;
	node["name"] = "OneComponentName";
	node["className"] = "TestComponent2";
	node["id"] = "OneComponentName";

	auto component1 = node.as<std::shared_ptr<SurgSim::Framework::Component>>();
	EXPECT_NE(nullptr, component1);

	auto component1copy = node.as<std::shared_ptr<SurgSim::Framework::Component>>();
	EXPECT_NE(nullptr, component1copy);
	EXPECT_EQ(component1, component1copy);

	node["id"] = "TwoComponentName";

	auto component2 = node.as<std::shared_ptr<SurgSim::Framework::Component>>();
	EXPECT_NE(nullptr, component2);
	EXPECT_NE(component2, component1);
	EXPECT_NE(component2, component1copy);
}

TEST(ComponentTests, EncodeComponent)
{
	auto component = std::make_shared<TestComponent2>("TestComponent");
	component->setValueOne(1);
	component->setValueTwo(2);

	YAML::Node node = YAML::convert<std::shared_ptr<Component>>::encode(component);

	EXPECT_EQ("TestComponent", (node["name"].IsDefined() ? node["name"].as<std::string>() : "undefined !"));
	EXPECT_EQ("TestComponent2", (node["className"].IsDefined() ? node["className"].as<std::string>() : "undefined !"));
	EXPECT_EQ(1, (node["valueOne"].IsDefined() ? node["valueOne"].as<int>() : 0xbad));
	EXPECT_EQ(2, (node["valueTwo"].IsDefined() ? node["valueTwo"].as<int>() : 0xbad));
}

TEST(ComponentTests, DecodeComponent)
{
	YAML::Node node;
	node["name"] = "TestComponentName";
	node["className"] = "TestComponent2";
	node["id"] = "TestId";
	node["valueOne"] = 100;
	node["valueTwo"] = 101;

	auto component = node.as<std::shared_ptr<SurgSim::Framework::Component>>();

	auto testComponent = std::dynamic_pointer_cast<TestComponent2>(component);

	EXPECT_NE(nullptr, testComponent);
	EXPECT_EQ("TestComponentName", testComponent->getName());
	EXPECT_EQ(100, testComponent->getValueOne());
	EXPECT_EQ(101, testComponent->getValueTwo());
}
