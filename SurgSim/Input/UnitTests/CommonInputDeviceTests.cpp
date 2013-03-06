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
 * Tests for the CommonInputDevice class.
 */

#include <memory>
#include <string>
#include <gtest/gtest.h>
#include <SurgSim/Input/CommonInputDevice.h>
#include <SurgSim/Input/InputDeviceListenerInterface.h>
#include <SurgSim/DataStructures/DataGroup.h>
#include <SurgSim/DataStructures/DataGroupBuilder.h>
#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Math/Matrix.h>

using SurgSim::Input::CommonInputDevice;
using SurgSim::Input::InputDeviceListenerInterface;
using SurgSim::DataStructures::DataGroup;
using SurgSim::DataStructures::DataGroupBuilder;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Matrix44d;


class TestDevice : public CommonInputDevice
{
public:
	TestDevice(const std::string& uniqueName) :
		CommonInputDevice(uniqueName, buildInputData())
	{
	}

	virtual bool initialize();

	virtual bool finalize();

	virtual void pushInput();

	virtual bool pullOutput();

	const DataGroup& getOutputData() const
	{
		return CommonInputDevice::getOutputData();
	}

	/// Builds the data layout for the application input (i.e. device output).
	static DataGroup buildInputData();
};

// required by the InputDeviceInterface API
bool TestDevice::initialize()
{
	return true;
}

// required by the InputDeviceInterface API
bool TestDevice::finalize()
{
	return true;
}

// expose the pushInput method to the world
void TestDevice::pushInput()
{
	CommonInputDevice::pushInput();
}

// expose the pullOutput method to the world
bool TestDevice::pullOutput()
{
	return CommonInputDevice::pullOutput();
}

DataGroup TestDevice::buildInputData()
{
	DataGroupBuilder builder;
	builder.addString("helloWorld");
	DataGroup data = builder.createData();
	data.strings().put("helloWorld", "data");
	return data;
}


struct TestListener : public InputDeviceListenerInterface
{
public:
	TestListener() :
		m_numTimesReceivedInput(0),
		m_numTimesRequestedOutput(0)
	{
	}

	virtual void handleInput(const std::string& device, const DataGroup& inputData);
	virtual bool requestOutput(const std::string& device, DataGroup* outputData);

	int m_numTimesReceivedInput;
	int m_numTimesRequestedOutput;
	DataGroup m_lastReceivedInput;
};

void TestListener::handleInput(const std::string& device, const DataGroup& inputData)
{
	++m_numTimesReceivedInput;
	m_lastReceivedInput = inputData;
}

bool TestListener::requestOutput(const std::string& device, DataGroup* outputData)
{
	++m_numTimesRequestedOutput;
	return false;
}


struct TestOutputListener : public TestListener
{
public:
	TestOutputListener()
	{
		DataGroupBuilder builder;
		builder.addInteger("value");
		m_nextSentOutput = builder.createData();
		m_nextSentOutput.integers().put("value", 123);
	}

	virtual bool requestOutput(const std::string& device, DataGroup* outputData);

	DataGroup m_nextSentOutput;
};

bool TestOutputListener::requestOutput(const std::string& device, DataGroup* outputData)
{
	++m_numTimesRequestedOutput;
	*outputData = m_nextSentOutput;
	return true;
}


// OK, let's start testing...

TEST(CommonInputDeviceTests, CanConstruct)
{
	EXPECT_NO_THROW({TestDevice device("MyTestDevice");});
}

TEST(CommonInputDeviceTests, Name)
{
	TestDevice device("MyTestDevice");
	EXPECT_EQ("MyTestDevice", device.getName());
}

TEST(CommonInputDeviceTests, AddInputListener)
{
	TestDevice device("MyTestDevice");
	std::shared_ptr<TestListener> listener = std::make_shared<TestListener>();
	EXPECT_EQ(0, listener->m_numTimesReceivedInput);

	EXPECT_TRUE(device.addInputListener(listener));
	EXPECT_EQ(0, listener->m_numTimesReceivedInput);

	EXPECT_FALSE(device.addInputListener(listener));
	EXPECT_FALSE(device.addListener(listener));
	EXPECT_EQ(0, listener->m_numTimesReceivedInput);
}

TEST(CommonInputDeviceTests, AddListener)
{
	TestDevice device("MyTestDevice");
	std::shared_ptr<TestListener> listener = std::make_shared<TestListener>();
	EXPECT_EQ(0, listener->m_numTimesReceivedInput);

	EXPECT_TRUE(device.addListener(listener));
	EXPECT_EQ(0, listener->m_numTimesReceivedInput);

	EXPECT_FALSE(device.addListener(listener));
	EXPECT_FALSE(device.addInputListener(listener));
	EXPECT_EQ(0, listener->m_numTimesReceivedInput);
}

TEST(CommonInputDeviceTests, PushInput)
{
	for (int pass = 0;  pass < 2;  ++pass)
	{
		TestDevice device("MyTestDevice");
		std::shared_ptr<TestListener> listener = std::make_shared<TestListener>();
		EXPECT_EQ(0, listener->m_numTimesReceivedInput);

		if (pass == 0)
		{
			EXPECT_TRUE(device.addInputListener(listener));
		}
		else
		{
			EXPECT_TRUE(device.addListener(listener));
		}

		for (int i = 1;  i <= 3;  ++i)
		{
			EXPECT_EQ(i-1, listener->m_numTimesReceivedInput);

			device.pushInput();
			EXPECT_EQ(i, listener->m_numTimesReceivedInput);
			EXPECT_TRUE(listener->m_lastReceivedInput.strings().hasCurrentData("helloWorld"));

			listener->m_lastReceivedInput.resetAll();  // invalidate the state for next loop
			EXPECT_EQ(i, listener->m_numTimesReceivedInput);
			EXPECT_FALSE(listener->m_lastReceivedInput.strings().hasCurrentData("helloWorld"));
		}
	}
}

TEST(CommonInputDeviceTests, PullOutput)
{
	TestDevice device("MyTestDevice");
	std::shared_ptr<TestOutputListener> listener = std::make_shared<TestOutputListener>();
	EXPECT_EQ(0, listener->m_numTimesRequestedOutput);

	EXPECT_TRUE(device.addListener(listener));
	EXPECT_EQ(0, listener->m_numTimesRequestedOutput);

	EXPECT_TRUE(device.pullOutput());
	EXPECT_EQ(1, listener->m_numTimesRequestedOutput);
	EXPECT_TRUE(device.getOutputData().integers().hasCurrentData("value"));

	EXPECT_TRUE(device.pullOutput());
	EXPECT_EQ(2, listener->m_numTimesRequestedOutput);

	EXPECT_TRUE(device.removeListener(listener));
	EXPECT_FALSE(device.pullOutput());
	EXPECT_EQ(2, listener->m_numTimesRequestedOutput);
}

TEST(CommonInputDeviceTests, DontPullOutput)
{
	TestDevice device("MyTestDevice");
	std::shared_ptr<TestOutputListener> listener = std::make_shared<TestOutputListener>();
	EXPECT_EQ(0, listener->m_numTimesRequestedOutput);

	EXPECT_TRUE(device.addInputListener(listener));
	EXPECT_EQ(0, listener->m_numTimesRequestedOutput);

	EXPECT_FALSE(device.pullOutput());
	EXPECT_EQ(0, listener->m_numTimesRequestedOutput);

	EXPECT_FALSE(device.pullOutput());
	EXPECT_EQ(0, listener->m_numTimesRequestedOutput);
}

TEST(CommonInputDeviceTests, RemoveListener)
{
	TestDevice device("MyTestDevice");
	std::shared_ptr<TestListener> listener = std::make_shared<TestListener>();
	EXPECT_EQ(0, listener->m_numTimesReceivedInput);

	EXPECT_TRUE(device.addInputListener(listener));
	EXPECT_EQ(0, listener->m_numTimesReceivedInput);

	device.pushInput();
	EXPECT_EQ(1, listener->m_numTimesReceivedInput);
	EXPECT_TRUE(listener->m_lastReceivedInput.strings().hasCurrentData("helloWorld"));

	listener->m_lastReceivedInput.resetAll();  // invalidate the state for next loop
	EXPECT_EQ(1, listener->m_numTimesReceivedInput);
	EXPECT_FALSE(listener->m_lastReceivedInput.strings().hasCurrentData("helloWorld"));

	EXPECT_TRUE(device.removeListener(listener));
	EXPECT_EQ(1, listener->m_numTimesReceivedInput);

	device.pushInput();
	EXPECT_EQ(1, listener->m_numTimesReceivedInput);
	EXPECT_FALSE(listener->m_lastReceivedInput.strings().hasCurrentData("helloWorld"));

	EXPECT_FALSE(device.removeListener(listener));
	EXPECT_EQ(1, listener->m_numTimesReceivedInput);
}
