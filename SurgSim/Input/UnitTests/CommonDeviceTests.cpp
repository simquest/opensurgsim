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
#include <SurgSim/Input/CommonDevice.h>
#include <SurgSim/Input/InputConsumerInterface.h>
#include <SurgSim/Input/OutputProducerInterface.h>
#include <SurgSim/DataStructures/DataGroup.h>
#include <SurgSim/DataStructures/DataGroupBuilder.h>
#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Math/Matrix.h>

using SurgSim::Input::CommonDevice;
using SurgSim::Input::InputConsumerInterface;
using SurgSim::Input::OutputProducerInterface;
using SurgSim::DataStructures::DataGroup;
using SurgSim::DataStructures::DataGroupBuilder;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Matrix44d;


class TestDevice : public CommonDevice
{
public:
	TestDevice(const std::string& uniqueName) :
		CommonDevice(uniqueName, buildInputData())
	{
	}

	virtual bool initialize();

	virtual bool finalize();

	virtual void pushInput();

	virtual bool pullOutput();

	const DataGroup& getOutputData() const;

	/// Builds the data layout for the application input (i.e. device output).
	static DataGroup buildInputData();
};

// required by the DeviceInterface API
bool TestDevice::initialize()
{
	return true;
}

// required by the DeviceInterface API
bool TestDevice::finalize()
{
	return true;
}

// expose the pushInput method to the world
void TestDevice::pushInput()
{
	CommonDevice::pushInput();
}

// expose the pullOutput method to the world
bool TestDevice::pullOutput()
{
	return CommonDevice::pullOutput();
}

// expose the getOutputData method to the world
const DataGroup& TestDevice::getOutputData() const
{
	return CommonDevice::getOutputData();
}

DataGroup TestDevice::buildInputData()
{
	DataGroupBuilder builder;
	builder.addString("helloWorld");
	DataGroup data = builder.createData();
	data.strings().put("helloWorld", "data");
	return data;
}


struct TestInputConsumer : public InputConsumerInterface
{
public:
	TestInputConsumer() :
		m_numTimesReceivedInput(0)
	{
	}

	virtual void handleInput(const std::string& device, const DataGroup& inputData);

	int m_numTimesReceivedInput;
	DataGroup m_lastReceivedInput;
};

void TestInputConsumer::handleInput(const std::string& device, const DataGroup& inputData)
{
	++m_numTimesReceivedInput;
	m_lastReceivedInput = inputData;
}


struct TestOutputProducer : public OutputProducerInterface
{
public:
	TestOutputProducer() :
		m_numTimesRequestedOutput(0),
		m_refuseToProduce(false)
	{
		DataGroupBuilder builder;
		builder.addInteger("value");
		m_nextSentOutput = builder.createData();
		m_nextSentOutput.integers().put("value", 123);
	}

	virtual bool requestOutput(const std::string& device, DataGroup* outputData);

	int m_numTimesRequestedOutput;
	bool m_refuseToProduce;
	DataGroup m_nextSentOutput;
};

bool TestOutputProducer::requestOutput(const std::string& device, DataGroup* outputData)
{
	++m_numTimesRequestedOutput;

	if (m_refuseToProduce)
	{
		return false;
	}
	else
	{
		*outputData = m_nextSentOutput;
		return true;
	}
}


// OK, let's start testing...

TEST(CommonDeviceTests, CanConstruct)
{
	EXPECT_NO_THROW({TestDevice device("MyTestDevice");});
}

TEST(CommonDeviceTests, Name)
{
	TestDevice device("MyTestDevice");
	EXPECT_EQ("MyTestDevice", device.getName());
}

TEST(CommonDeviceTests, AddInputConsumer)
{
	TestDevice device("MyTestDevice");
	std::shared_ptr<TestInputConsumer> consumer = std::make_shared<TestInputConsumer>();
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);

	EXPECT_TRUE(device.addInputConsumer(consumer));
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);

	EXPECT_FALSE(device.addInputConsumer(consumer));
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);

	std::shared_ptr<TestInputConsumer> consumer2 = std::make_shared<TestInputConsumer>();
	EXPECT_EQ(0, consumer2->m_numTimesReceivedInput);

	EXPECT_TRUE(device.addInputConsumer(consumer2));
	EXPECT_EQ(0, consumer2->m_numTimesReceivedInput);
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);
}

TEST(CommonDeviceTests, SetOutputProducer)
{
	TestDevice device("MyTestDevice");
	std::shared_ptr<TestOutputProducer> producer = std::make_shared<TestOutputProducer>();
	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);

	EXPECT_TRUE(device.setOutputProducer(producer));
	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);

	EXPECT_FALSE(device.setOutputProducer(producer));
	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);

	std::shared_ptr<TestOutputProducer> producer2 = std::make_shared<TestOutputProducer>();
	EXPECT_EQ(0, producer2->m_numTimesRequestedOutput);

	EXPECT_TRUE(device.setOutputProducer(producer2));
	EXPECT_EQ(0, producer2->m_numTimesRequestedOutput);
	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);
}

TEST(CommonDeviceTests, PushInput)
{
	TestDevice device("MyTestDevice");
	std::shared_ptr<TestInputConsumer> consumer1 = std::make_shared<TestInputConsumer>();
	std::shared_ptr<TestInputConsumer> consumer2 = std::make_shared<TestInputConsumer>();
	std::shared_ptr<TestOutputProducer> producer = std::make_shared<TestOutputProducer>();
	EXPECT_TRUE(device.addInputConsumer(consumer1));
	EXPECT_TRUE(device.addInputConsumer(consumer2));
	EXPECT_TRUE(device.setOutputProducer(producer));
	EXPECT_EQ(0, consumer1->m_numTimesReceivedInput);
	EXPECT_EQ(0, consumer2->m_numTimesReceivedInput);
	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);

	device.pushInput();
	EXPECT_EQ(1, consumer1->m_numTimesReceivedInput);
	EXPECT_EQ(1, consumer2->m_numTimesReceivedInput);
	EXPECT_TRUE(consumer1->m_lastReceivedInput.strings().hasCurrentData("helloWorld"));
	EXPECT_TRUE(consumer2->m_lastReceivedInput.strings().hasCurrentData("helloWorld"));
	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);

	// invalidate the state
	consumer1->m_lastReceivedInput.resetAll();
	consumer2->m_lastReceivedInput.resetAll();
	EXPECT_EQ(1, consumer1->m_numTimesReceivedInput);
	EXPECT_EQ(1, consumer2->m_numTimesReceivedInput);
	EXPECT_FALSE(consumer1->m_lastReceivedInput.strings().hasCurrentData("helloWorld"));
	EXPECT_FALSE(consumer2->m_lastReceivedInput.strings().hasCurrentData("helloWorld"));

	device.pushInput();
	EXPECT_EQ(2, consumer1->m_numTimesReceivedInput);
	EXPECT_EQ(2, consumer2->m_numTimesReceivedInput);
	EXPECT_TRUE(consumer1->m_lastReceivedInput.strings().hasCurrentData("helloWorld"));
	EXPECT_TRUE(consumer2->m_lastReceivedInput.strings().hasCurrentData("helloWorld"));
	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);
}

TEST(CommonDeviceTests, PullOutput)
{
	TestDevice device("MyTestDevice");
	std::shared_ptr<TestOutputProducer> producer1 = std::make_shared<TestOutputProducer>();
	std::shared_ptr<TestOutputProducer> producer2 = std::make_shared<TestOutputProducer>();
	std::shared_ptr<TestInputConsumer> consumer = std::make_shared<TestInputConsumer>();
	EXPECT_TRUE(device.setOutputProducer(producer1));
	EXPECT_TRUE(device.setOutputProducer(producer2));
	EXPECT_TRUE(device.addInputConsumer(consumer));
	EXPECT_EQ(0, producer1->m_numTimesRequestedOutput);
	EXPECT_EQ(0, producer2->m_numTimesRequestedOutput);
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);

	EXPECT_TRUE(device.pullOutput());
	EXPECT_EQ(0, producer1->m_numTimesRequestedOutput);
	EXPECT_EQ(1, producer2->m_numTimesRequestedOutput);
	EXPECT_TRUE(device.getOutputData().integers().hasCurrentData("value"));
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);

	EXPECT_TRUE(device.pullOutput());
	EXPECT_EQ(0, producer1->m_numTimesRequestedOutput);
	EXPECT_EQ(2, producer2->m_numTimesRequestedOutput);
	EXPECT_TRUE(device.getOutputData().integers().hasCurrentData("value"));
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);

	// Test what happens when the producer returns false.
	producer2->m_refuseToProduce = true;
	EXPECT_FALSE(device.pullOutput());
	EXPECT_EQ(0, producer1->m_numTimesRequestedOutput);
	EXPECT_EQ(3, producer2->m_numTimesRequestedOutput);
	EXPECT_FALSE(device.getOutputData().integers().hasCurrentData("value"));
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);
}

TEST(CommonDeviceTests, RemoveInputConsumer)
{
	TestDevice device("MyTestDevice");
	EXPECT_FALSE(device.removeInputConsumer(std::shared_ptr<TestInputConsumer>()));
	EXPECT_FALSE(device.removeInputConsumer(std::shared_ptr<TestInputConsumer>(nullptr)));

	std::shared_ptr<TestInputConsumer> consumer1 = std::make_shared<TestInputConsumer>();
	std::shared_ptr<TestInputConsumer> consumer2 = std::make_shared<TestInputConsumer>();
	EXPECT_FALSE(device.removeInputConsumer(consumer1));
	EXPECT_TRUE(device.addInputConsumer(consumer1));
	EXPECT_TRUE(device.addInputConsumer(consumer2));
	EXPECT_EQ(0, consumer1->m_numTimesReceivedInput);
	EXPECT_EQ(0, consumer2->m_numTimesReceivedInput);

	EXPECT_FALSE(device.removeInputConsumer(std::shared_ptr<TestInputConsumer>()));
	EXPECT_FALSE(device.removeInputConsumer(std::shared_ptr<TestInputConsumer>(nullptr)));

	device.pushInput();
	EXPECT_EQ(1, consumer1->m_numTimesReceivedInput);
	EXPECT_EQ(1, consumer2->m_numTimesReceivedInput);
	EXPECT_TRUE(consumer1->m_lastReceivedInput.strings().hasCurrentData("helloWorld"));
	EXPECT_TRUE(consumer2->m_lastReceivedInput.strings().hasCurrentData("helloWorld"));

	// invalidate the state
	consumer1->m_lastReceivedInput.resetAll();
	consumer2->m_lastReceivedInput.resetAll();
	EXPECT_EQ(1, consumer1->m_numTimesReceivedInput);
	EXPECT_EQ(1, consumer2->m_numTimesReceivedInput);
	EXPECT_FALSE(consumer1->m_lastReceivedInput.strings().hasCurrentData("helloWorld"));
	EXPECT_FALSE(consumer2->m_lastReceivedInput.strings().hasCurrentData("helloWorld"));

	EXPECT_TRUE(device.removeInputConsumer(consumer1));
	device.pushInput();
	EXPECT_EQ(1, consumer1->m_numTimesReceivedInput);
	EXPECT_EQ(2, consumer2->m_numTimesReceivedInput);
	EXPECT_FALSE(consumer1->m_lastReceivedInput.strings().hasCurrentData("helloWorld"));
	EXPECT_TRUE(consumer2->m_lastReceivedInput.strings().hasCurrentData("helloWorld"));

	EXPECT_FALSE(device.removeInputConsumer(consumer1));
	device.pushInput();
	EXPECT_EQ(1, consumer1->m_numTimesReceivedInput);
	EXPECT_EQ(3, consumer2->m_numTimesReceivedInput);
	EXPECT_FALSE(consumer1->m_lastReceivedInput.strings().hasCurrentData("helloWorld"));
	EXPECT_TRUE(consumer2->m_lastReceivedInput.strings().hasCurrentData("helloWorld"));
}

TEST(CommonDeviceTests, RemoveOutputProducer)
{
	TestDevice device("MyTestDevice");
	EXPECT_FALSE(device.removeOutputProducer(std::shared_ptr<TestOutputProducer>()));
	EXPECT_FALSE(device.removeOutputProducer(std::shared_ptr<TestOutputProducer>(nullptr)));

	std::shared_ptr<TestOutputProducer> producer1 = std::make_shared<TestOutputProducer>();
	std::shared_ptr<TestOutputProducer> producer2 = std::make_shared<TestOutputProducer>();
	EXPECT_FALSE(device.removeOutputProducer(producer1));
	EXPECT_TRUE(device.setOutputProducer(producer1));
	EXPECT_TRUE(device.setOutputProducer(producer2));
	EXPECT_EQ(0, producer1->m_numTimesRequestedOutput);
	EXPECT_EQ(0, producer2->m_numTimesRequestedOutput);

	EXPECT_FALSE(device.removeOutputProducer(std::shared_ptr<TestOutputProducer>()));
	EXPECT_FALSE(device.removeOutputProducer(std::shared_ptr<TestOutputProducer>(nullptr)));

	EXPECT_TRUE(device.pullOutput());
	EXPECT_EQ(0, producer1->m_numTimesRequestedOutput);
	EXPECT_EQ(1, producer2->m_numTimesRequestedOutput);
	EXPECT_TRUE(device.getOutputData().integers().hasCurrentData("value"));

	EXPECT_FALSE(device.removeOutputProducer(producer1));
	EXPECT_TRUE(device.pullOutput());
	EXPECT_EQ(0, producer1->m_numTimesRequestedOutput);
	EXPECT_EQ(2, producer2->m_numTimesRequestedOutput);
	EXPECT_TRUE(device.getOutputData().integers().hasCurrentData("value"));

	EXPECT_TRUE(device.removeOutputProducer(producer2));
	EXPECT_FALSE(device.pullOutput());
	EXPECT_EQ(0, producer1->m_numTimesRequestedOutput);
	EXPECT_EQ(2, producer2->m_numTimesRequestedOutput);
	EXPECT_FALSE(device.getOutputData().integers().hasCurrentData("value"));

	EXPECT_FALSE(device.removeOutputProducer(producer2));
	EXPECT_FALSE(device.pullOutput());
	EXPECT_EQ(0, producer1->m_numTimesRequestedOutput);
	EXPECT_EQ(2, producer2->m_numTimesRequestedOutput);
	EXPECT_FALSE(device.getOutputData().integers().hasCurrentData("value"));
}
