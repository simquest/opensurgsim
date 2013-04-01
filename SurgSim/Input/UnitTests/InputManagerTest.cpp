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

/** @file
 * Tests for the CommonDevice class.
 */

#include <memory>
#include <string>
#include <gtest/gtest.h>

#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Input/CommonDevice.h>
#include <SurgSim/Input/InputConsumerInterface.h>
#include <SurgSim/Input/OutputProducerInterface.h>
#include <SurgSim/Input/InputManager.h>
#include <SurgSim/Input/InputComponent.h>
#include <SurgSim/Input/OutputComponent.h>
#include <SurgSim/DataStructures/DataGroup.h>
#include <SurgSim/DataStructures/DataGroupBuilder.h>
#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Math/Matrix.h>

#include "TestDevice.h"

using SurgSim::Framework::Runtime;
using SurgSim::Input::DeviceInterface;
using SurgSim::Input::CommonDevice;
using SurgSim::Input::InputConsumerInterface;
using SurgSim::Input::OutputProducerInterface;
using SurgSim::Input::InputManager;
using SurgSim::Input::InputComponent;
using SurgSim::Input::OutputComponent;
using SurgSim::DataStructures::DataGroup;
using SurgSim::DataStructures::DataGroupBuilder;


class InputManagerTest : public ::testing::Test
{
protected:
	virtual void SetUp() 
	{
		testDevice1 = std::make_shared<TestDevice>("TestDevice1");
		testDevice2 = std::make_shared<TestDevice>("TestDevice2");
	 
		runtime = std::make_shared<Runtime>();
		inputManager = std::make_shared<InputManager>();
	 
		runtime->addWorkerThread(inputManager);
		runtime->start();

		inputManager->addDevice(testDevice1);
		inputManager->addDevice(testDevice2);
	}



	virtual void TearDown()
	{
		runtime->stop();
	}



	std::shared_ptr<TestDevice> testDevice1;
	std::shared_ptr<TestDevice> testDevice2;

	std::shared_ptr<Runtime> runtime;
	std::shared_ptr<InputManager> inputManager;
};

std::shared_ptr<OutputComponent> createOutputComponent(std::string name, std::string deviceName)
{
	DataGroupBuilder builder;
	builder.addString("data");
	DataGroup data = builder.createData();
	data.strings().put("data", "data");

	return std::make_shared<OutputComponent>(name,deviceName,data);
}


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
	std::shared_ptr<InputComponent> listener1 = std::make_shared<InputComponent>("Component1","TestDevice1");
	std::shared_ptr<InputComponent> listener2 = std::make_shared<InputComponent>("Component2","TestDevice1");
	std::shared_ptr<InputComponent> listener3 = std::make_shared<InputComponent>("Component3","TestDevice2");
	std::shared_ptr<InputComponent> notvalid = std::make_shared<InputComponent>("Component3","NonExistantDevice");

	// Add various listeners to the input manager
	EXPECT_TRUE(inputManager->addComponent(listener1));
	EXPECT_TRUE(inputManager->addComponent(listener2));
	EXPECT_TRUE(inputManager->addComponent(listener3));
	EXPECT_FALSE(inputManager->addComponent(notvalid));

	// Excercise adds and removes
	EXPECT_FALSE(inputManager->addComponent(listener1));
	EXPECT_TRUE(inputManager->removeComponent(listener1));
	EXPECT_FALSE(inputManager->removeComponent(listener1));
}

TEST_F(InputManagerTest, InputfromDevice)
{
	std::shared_ptr<InputComponent> listener1 = std::make_shared<InputComponent>("Component1","TestDevice1");
	inputManager->addComponent(listener1);

	testDevice1->pushInput("avalue");

	std::string data;
	listener1->getInputData().strings().get("helloWorld",data);
	EXPECT_EQ("avalue",data);

	testDevice1->pushInput("bvalue");
	listener1->getInputData().strings().get("helloWorld",data);
	EXPECT_EQ("bvalue",data);
}

TEST_F(InputManagerTest, OutputAddRemove)
{
	std::shared_ptr<OutputComponent> output1 = createOutputComponent("Component1", "TestDevice1");
	std::shared_ptr<OutputComponent> output2 = createOutputComponent("Component2", "TestDevice1");
	std::shared_ptr<OutputComponent> output3 = createOutputComponent("Component2", "TestDevice2");
	std::shared_ptr<OutputComponent> invalid = createOutputComponent("Component3", "InvalidDevice");

	EXPECT_TRUE(inputManager->addComponent(output1));
	EXPECT_TRUE(inputManager->addComponent(output2));
	EXPECT_FALSE(inputManager->addComponent(output2));
	EXPECT_TRUE(inputManager->addComponent(output3));
	EXPECT_FALSE(inputManager->addComponent(invalid));
	EXPECT_TRUE(inputManager->removeComponent(output1));
	EXPECT_FALSE(inputManager->removeComponent(output1));
}

TEST_F(InputManagerTest, OutputPush)
{
	std::shared_ptr<OutputComponent> output = createOutputComponent("Component1", "TestDevice1");
	EXPECT_TRUE(inputManager->addComponent(output));
	output->getOutputData().strings().put("data","outputdata");
	EXPECT_TRUE(testDevice1->pullOutput());
	EXPECT_EQ("outputdata",testDevice1->lastPulledData);
}
