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

/// \file
/// Tests for the InputManaget class. Note that InputManagerTest, the test fixture
/// is declared as a friend class in InputManager to make it easier to test the
/// add and removal of components, for this to work correctly PhysicsManagerTest is required
/// to be in the SurgSim::Physics namespace.

#include <memory>
#include <string>
#include <gtest/gtest.h>

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Input/CommonDevice.h"
#include "SurgSim/Input/OutputProducerInterface.h"
#include "SurgSim/Input/InputManager.h"
#include "SurgSim/Input/InputComponent.h"
#include "SurgSim/Input/OutputComponent.h"
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Matrix.h"

#include "SurgSim/Input/UnitTests/TestDevice.h"

using SurgSim::Framework::Runtime;
using SurgSim::Framework::Component;
using SurgSim::Input::DeviceInterface;
using SurgSim::Input::CommonDevice;
using SurgSim::Input::OutputProducerInterface;
using SurgSim::Input::InputManager;
using SurgSim::Input::InputComponent;
using SurgSim::Input::OutputComponent;
using SurgSim::DataStructures::DataGroup;
using SurgSim::DataStructures::DataGroupBuilder;


class MockComponent : public SurgSim::Framework::Component
{
public:
	explicit MockComponent(const std::string& name = "MockComponent") : Component(name) {}
	virtual ~MockComponent() {}

protected:
	virtual bool doInitialize()
	{
		return true;
	}
	virtual bool doWakeUp()
	{
		return true;
	}
};
namespace SurgSim
{
namespace Input
{

class InputManagerTest : public ::testing::Test
{
public:
	virtual void SetUp()
	{
		testDevice1 = std::make_shared<TestDevice>("TestDevice1");
		testDevice2 = std::make_shared<TestDevice>("TestDevice2");

		runtime = std::make_shared<Runtime>();
		inputManager = std::make_shared<InputManager>();

		runtime->addManager(inputManager);
		runtime->start();

		inputManager->addDevice(testDevice1);
		inputManager->addDevice(testDevice2);
	}

	virtual void TearDown()
	{
		runtime->stop();
	}

	bool testDoAddComponent(const std::shared_ptr<Component>& component)
	{
		return inputManager->executeAdditions(component);
	}

	bool testDoRemoveComponent(const std::shared_ptr<Component>& component)
	{
		return inputManager->executeRemovals(component);
	}

	std::shared_ptr<TestDevice> testDevice1;
	std::shared_ptr<TestDevice> testDevice2;

	std::shared_ptr<Runtime> runtime;
	std::shared_ptr<InputManager> inputManager;
};

TEST_F(InputManagerTest, DeviceAddRemove)
{

	std::shared_ptr<DeviceInterface> testDevice3 = std::make_shared<TestDevice>("TestDevice3");
	std::shared_ptr<DeviceInterface> testDevice4 = std::make_shared<TestDevice>("TestDevice4");

	EXPECT_TRUE(inputManager->addDevice(testDevice3));
	EXPECT_TRUE(inputManager->addDevice(testDevice4));
	EXPECT_FALSE(inputManager->addDevice(testDevice3));
	EXPECT_TRUE(inputManager->removeDevice(testDevice4));
	EXPECT_FALSE(inputManager->removeDevice(testDevice4));
	EXPECT_TRUE(inputManager->addDevice(testDevice4));
}

TEST_F(InputManagerTest, InputAddRemove)
{
	std::shared_ptr<InputComponent> listener1 = std::make_shared<InputComponent>("Component1");
	std::shared_ptr<InputComponent> listener2 = std::make_shared<InputComponent>("Component2");
	std::shared_ptr<InputComponent> listener3 = std::make_shared<InputComponent>("Component3");
	std::shared_ptr<InputComponent> notvalid = std::make_shared<InputComponent>("Component4");

	listener1->setDeviceName("TestDevice1");
	listener2->setDeviceName("TestDevice1");
	listener3->setDeviceName("TestDevice2");
	notvalid->setDeviceName("NonExistantDevice");

	// Add various listeners to the input manager
	EXPECT_TRUE(testDoAddComponent(listener1));
	EXPECT_TRUE(testDoAddComponent(listener2));
	EXPECT_TRUE(testDoAddComponent(listener3));
	EXPECT_FALSE(testDoAddComponent(notvalid));

	// Excercise adds and removes

	// Duplicate false on duplicate will become deprecated
	EXPECT_FALSE(testDoAddComponent(listener1));
	EXPECT_TRUE(testDoRemoveComponent(listener1));
	EXPECT_FALSE(testDoRemoveComponent(listener1));

	// Should not be able to add random components
	std::shared_ptr<MockComponent> component = std::make_shared<MockComponent>();
	EXPECT_FALSE(testDoAddComponent(component));
}

TEST_F(InputManagerTest, InputfromDevice)
{
	std::string data;
	SurgSim::DataStructures::DataGroup dataGroup;

	std::shared_ptr<InputComponent> listener1 = std::make_shared<InputComponent>("Component1");
	listener1->setDeviceName("TestDevice1");

	testDoAddComponent(listener1);

	EXPECT_NO_THROW(listener1->getData(&dataGroup));

	testDevice1->pushInput("avalue");
	EXPECT_NO_THROW(listener1->getData(&dataGroup));
	EXPECT_TRUE(dataGroup.strings().get("helloWorld", &data));
	EXPECT_EQ("avalue", data);

	testDevice1->pushInput("bvalue");
	EXPECT_NO_THROW(listener1->getData(&dataGroup));
	EXPECT_TRUE(dataGroup.strings().get("helloWorld", &data));
	EXPECT_EQ("bvalue", data);
}

TEST_F(InputManagerTest, OutputAddRemove)
{
	std::shared_ptr<OutputComponent> output1 = std::make_shared<OutputComponent>("Component1");
	std::shared_ptr<OutputComponent> output2 = std::make_shared<OutputComponent>("Component2");
	std::shared_ptr<OutputComponent> output3 = std::make_shared<OutputComponent>("Component3");
	std::shared_ptr<OutputComponent> invalid = std::make_shared<OutputComponent>("Component4");
	output1->setDeviceName("TestDevice1");
	output2->setDeviceName("TestDevice1");
	output3->setDeviceName("TestDevice2");
	invalid->setDeviceName("InvalidDevice");
	EXPECT_TRUE(testDoAddComponent(output1));
	EXPECT_FALSE(testDoAddComponent(output2)); // same device already attached to an OutputComponent
	EXPECT_FALSE(testDoAddComponent(output2));
	EXPECT_TRUE(testDoAddComponent(output3));
	EXPECT_FALSE(testDoAddComponent(invalid));
	EXPECT_TRUE(testDoRemoveComponent(output1));
	EXPECT_FALSE(testDoRemoveComponent(output1));
}

TEST_F(InputManagerTest, OutputPush)
{
	std::shared_ptr<OutputComponent> output = std::make_shared<OutputComponent>("Component1");
	output->setDeviceName("TestDevice1");
	EXPECT_TRUE(testDoAddComponent(output));
	DataGroupBuilder builder;
	builder.addString("data");
	DataGroup data = builder.createData();
	data.strings().set("data", "outputdata");
	output->setData(data);
	EXPECT_TRUE(testDevice1->pullOutput());
	EXPECT_EQ("outputdata", testDevice1->lastPulledData);
}

TEST_F(InputManagerTest, TypeTest)
{
	EXPECT_EQ(SurgSim::Framework::MANAGER_TYPE_INPUT, inputManager->getType());
}

}; // namespace Input
}; // namespace SurgSim

