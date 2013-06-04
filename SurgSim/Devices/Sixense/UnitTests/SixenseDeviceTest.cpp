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

/// \file
/// Tests for the SixenseDevice class.

#include <memory>
#include <string>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <gtest/gtest.h>
#include "SurgSim/Devices/Sixense/SixenseDevice.h"
//#include "SurgSim/Devices/Sixense/SixenseManager.h"  // only needed if calling SixenseManager::setDefaultLogLevel()
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Input/InputConsumerInterface.h"
#include "SurgSim/Input/OutputProducerInterface.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Matrix.h"

using SurgSim::Device::SixenseDevice;
using SurgSim::Device::SixenseManager;
using SurgSim::DataStructures::DataGroup;
using SurgSim::Input::InputConsumerInterface;
using SurgSim::Input::OutputProducerInterface;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Matrix44d;


struct TestListener : public InputConsumerInterface, public OutputProducerInterface
{
public:
	TestListener() :
		m_numTimesInitializedInput(0),
		m_numTimesReceivedInput(0),
		m_numTimesRequestedOutput(0)
	{
	}

	virtual void initializeInput(const std::string& device, const DataGroup& inputData);
	virtual void handleInput(const std::string& device, const DataGroup& inputData);
	virtual bool requestOutput(const std::string& device, DataGroup* outputData);

	int m_numTimesInitializedInput;
	int m_numTimesReceivedInput;
	int m_numTimesRequestedOutput;
	DataGroup m_lastReceivedInput;
};

void TestListener::initializeInput(const std::string& device, const DataGroup& inputData)
{
	++m_numTimesInitializedInput;
}

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


TEST(SixenseDeviceTest, CreateDevice)
{
	//SixenseManager::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<SixenseDevice> device = SixenseDevice::create("TestSixense");
	ASSERT_TRUE(device != nullptr) << "Initialization failed.  Is a Sixense/Hydra device plugged in?";
}

TEST(SixenseDeviceTest, Name)
{
	//SixenseManager::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<SixenseDevice> device = SixenseDevice::create("TestSixense");
	ASSERT_TRUE(device != nullptr) << "Initialization failed.  Is a Sixense/Hydra device plugged in?";
	EXPECT_EQ("TestSixense", device->getName());
}

TEST(SixenseDeviceTest, CreateDeviceSeveralTimes)
{
	//SixenseManager::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	for (int i = 0;  i < 6;  ++i)
	{
		std::shared_ptr<SixenseDevice> device = SixenseDevice::create("TestSixense");
		ASSERT_TRUE(device != nullptr) << "Initialization failed.  Is a Sixense/Hydra device plugged in?";
		// the device will be destroyed here
	}
}

TEST(SixenseDeviceTest, CreateSeveralDevices)
{
	//SixenseManager::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<SixenseDevice> device1 = SixenseDevice::create("Sixense1");
	ASSERT_TRUE(device1 != nullptr) << "Initialization failed.  Is a Sixense/Hydra device plugged in?";

	std::shared_ptr<SixenseDevice> device2 = SixenseDevice::create("Sixense2");
	ASSERT_TRUE(device2 != nullptr) << "Initialization failed for second controller." <<
		"  Is only one controller plugged in?";

	// We can't check what happens with the managers, since those are no longer a part of the device's API...

	std::shared_ptr<SixenseDevice> device3 = SixenseDevice::create("Sixense3");
	if (! device3)
	{
		std::cerr << "[Warning: third Sixense/Hydra controller not actually created; is it plugged in?]" << std::endl;
	}
}

// Create a string representation from an int.
// C++11 adds std::to_string() to do this for various types, but VS2010 only half-supports that.
template <typename T>
inline std::string makeString(T value)
{
	std::ostringstream out;
	out << value;
	return out.str();
}

TEST(SixenseDeviceTest, CreateAllDevices)
{
	//SixenseManager::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::vector<std::shared_ptr<SixenseDevice>> devices;

	for (int i = 1;  ;  ++i)
	{
		std::string name = "Sixense" + makeString(i);
		std::shared_ptr<SixenseDevice> device = SixenseDevice::create(name);
		if (device == nullptr)
		{
			break;
		}
		devices.emplace_back(std::move(device));
	}

	std::cout << devices.size() << " devices initialized." << std::endl;
	ASSERT_GT(devices.size(), 0U) << "Initialization failed.  Is a Sixense/Hydra device plugged in?";
	EXPECT_GT(devices.size(), 1U) << "Controller #2 initialization failed.  Is only one controller plugged in?";
}

TEST(SixenseDeviceTest, InputConsumer)
{
	//SixenseManager::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<SixenseDevice> device = SixenseDevice::create("TestSixense");
	ASSERT_TRUE(device != nullptr) << "Initialization failed.  Is a Sixense/Hydra device plugged in?";

	std::shared_ptr<TestListener> consumer = std::make_shared<TestListener>();
	EXPECT_EQ(0, consumer->m_numTimesInitializedInput);
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);

	EXPECT_FALSE(device->removeInputConsumer(consumer));
	EXPECT_EQ(0, consumer->m_numTimesInitializedInput);
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);

	EXPECT_TRUE(device->addInputConsumer(consumer));

	// Adding the same input consumer again should fail.
	EXPECT_FALSE(device->addInputConsumer(consumer));

	// Sleep for a second, to see how many times the consumer is invoked.
	// (A Sixense device updates internally at 60Hz, but our code currently runs at 120Hz to reduce latency.)
	boost::this_thread::sleep_until(boost::chrono::steady_clock::now() + boost::chrono::milliseconds(1000));

	EXPECT_TRUE(device->removeInputConsumer(consumer));

	// Removing the same input consumer again should fail.
	EXPECT_FALSE(device->removeInputConsumer(consumer));

	// Check the number of invocations.
	EXPECT_EQ(1, consumer->m_numTimesInitializedInput);
	EXPECT_GE(consumer->m_numTimesReceivedInput, 100);
	EXPECT_LE(consumer->m_numTimesReceivedInput, 140);

	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasData("pose"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.scalars().hasData("trigger"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.scalars().hasData("joystickX"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.scalars().hasData("joystickY"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.booleans().hasData("buttonTrigger"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.booleans().hasData("buttonBumper"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.booleans().hasData("button1"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.booleans().hasData("button2"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.booleans().hasData("button3"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.booleans().hasData("button4"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.booleans().hasData("buttonStart"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.booleans().hasData("buttonJoystick"));
}

TEST(SixenseDeviceTest, OutputProducer)
{
	//SixenseManager::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<SixenseDevice> device = SixenseDevice::create("TestSixense");
	ASSERT_TRUE(device != nullptr) << "Initialization failed.  Is a Sixense/Hydra device plugged in?";

	std::shared_ptr<TestListener> producer = std::make_shared<TestListener>();
	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);

	EXPECT_FALSE(device->removeOutputProducer(producer));
	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);

	EXPECT_TRUE(device->setOutputProducer(producer));

	// Sleep for a second, to see how many times the producer is invoked.
	// (A Sixense device is does not request any output.)
	boost::this_thread::sleep_until(boost::chrono::steady_clock::now() + boost::chrono::milliseconds(1000));

	EXPECT_TRUE(device->removeOutputProducer(producer));

	// Removing the same input producer again should fail.
	EXPECT_FALSE(device->removeOutputProducer(producer));

	// Check the number of invocations.
	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);
}
