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
/// Tests for the LabJackDevice class.

#include <memory>
#include <string>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <gtest/gtest.h>
#include "SurgSim/Devices/LabJack/LabJackDevice.h"
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Input/InputConsumerInterface.h"
#include "SurgSim/Input/OutputProducerInterface.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Matrix.h"

using SurgSim::Device::LabJackDevice;
using SurgSim::Device::LabJackScaffold;
using SurgSim::DataStructures::DataGroup;
using SurgSim::Input::InputConsumerInterface;
using SurgSim::Input::OutputProducerInterface;

struct RawTestListener : public InputConsumerInterface, public OutputProducerInterface
{
public:
	RawTestListener() :
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

void RawTestListener::initializeInput(const std::string& device, const DataGroup& inputData)
{
	++m_numTimesInitializedInput;
}

void RawTestListener::handleInput(const std::string& device, const DataGroup& inputData)
{
	++m_numTimesReceivedInput;
	m_lastReceivedInput = inputData;
}

bool RawTestListener::requestOutput(const std::string& device, DataGroup* outputData)
{
	++m_numTimesRequestedOutput;
	return false;
}

namespace
{
void testCreateDeviceSeveralTimes(bool doSleep)
{
	for (int i = 0;  i < 6;  ++i)
	{
		std::shared_ptr<LabJackDevice> device = std::make_shared<LabJackDevice>("TestLabJack");
		ASSERT_TRUE(device != nullptr) << "Device creation failed.";
		ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a LabJack device plugged in?";
		if (doSleep)
		{
			boost::this_thread::sleep_until(boost::chrono::steady_clock::now() + boost::chrono::milliseconds(100));
		}
		// the device will be destroyed here
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
};

TEST(LabJackDeviceTest, CreateUninitializedDevice)
{
	//LabJackScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<LabJackDevice> device = std::make_shared<LabJackDevice>("TestLabJack");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
}

TEST(LabJackDeviceTest, CreateAndInitializeDevice)
{
	//LabJackScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<LabJackDevice> device = std::make_shared<LabJackDevice>("TestLabJack");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	EXPECT_FALSE(device->isInitialized());
	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a LabJack device plugged in?";
	EXPECT_TRUE(device->isInitialized());
}

TEST(LabJackDeviceTest, Name)
{
	//LabJackScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<LabJackDevice> device = std::make_shared<LabJackDevice>("TestLabJack");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	EXPECT_EQ("TestLabJack", device->getName());
	EXPECT_TRUE(device->initialize()) << "Initialization failed.  Is a LabJack device plugged in?";
	EXPECT_EQ("TestLabJack", device->getName());
}

TEST(LabJackDeviceTest, CreateDeviceSeveralTimes)
{
	//LabJackScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	testCreateDeviceSeveralTimes(true);
}

TEST(LabJackDeviceTest, CreateSeveralDevices)
{
	//LabJackScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<LabJackDevice> device1 = std::make_shared<LabJackDevice>("LabJack1");
	ASSERT_TRUE(device1 != nullptr) << "Device creation failed.";
	ASSERT_TRUE(device1->initialize()) << "Initialization failed.  Is a LabJack device plugged in?";

	// We can't check what happens with the scaffolds, since those are no longer a part of the device's API...

	std::shared_ptr<LabJackDevice> device2 = std::make_shared<LabJackDevice>("LabJack2");
	ASSERT_TRUE(device2 != nullptr) << "Device creation failed.";
	if (!device2->initialize())
	{
		std::cerr << "[Warning: second LabJack controller did not come up; is it plugged in?]" << std::endl;
	}
}

TEST(LabJackDeviceTest, CreateDevicesWithSameName)
{
	//LabJackScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<LabJackDevice> device1 = std::make_shared<LabJackDevice>("LabJack");
	ASSERT_TRUE(device1 != nullptr) << "Device creation failed.";
	ASSERT_TRUE(device1->initialize()) << "Initialization failed.  Is a LabJack device plugged in?";

	std::shared_ptr<LabJackDevice> device2 = std::make_shared<LabJackDevice>("LabJack");
	ASSERT_TRUE(device2 != nullptr) << "Device creation failed.";
	ASSERT_FALSE(device2->initialize()) << "Initialization succeeded despite duplicate name.";
}

TEST(LabJackDeviceTest, CreateAllDevices)
{
	//LabJackScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::vector<std::shared_ptr<LabJackDevice>> devices;

	for (int i = 1;  ;  ++i)
	{
		std::string name = "LabJack" + makeString(i);
		std::shared_ptr<LabJackDevice> device = std::make_shared<LabJackDevice>(name);
		ASSERT_TRUE(device != nullptr) << "Device creation failed.";
		if (!device->initialize())
		{
			break;
		}
		devices.emplace_back(std::move(device));
	}

	std::cout << devices.size() << " devices initialized." << std::endl;
	ASSERT_GT(devices.size(), 0U) << "Initialization failed.  Is a LabJack device plugged in?";
}

TEST(LabJackDeviceTest, InputConsumer)
{
	//LabJackScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<LabJackDevice> device = std::make_shared<LabJackDevice>("TestLabJack");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a LabJack device plugged in?";

	std::shared_ptr<RawTestListener> consumer = std::make_shared<RawTestListener>();
	EXPECT_EQ(0, consumer->m_numTimesInitializedInput);
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);

	EXPECT_FALSE(device->removeInputConsumer(consumer));
	EXPECT_EQ(0, consumer->m_numTimesInitializedInput);
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);

	EXPECT_TRUE(device->addInputConsumer(consumer));

	// Adding the same input consumer again should fail.
	EXPECT_FALSE(device->addInputConsumer(consumer));

	// Sleep to see how many times the consumer is invoked.
	boost::this_thread::sleep_until(boost::chrono::steady_clock::now() + boost::chrono::milliseconds(200));

	EXPECT_TRUE(device->removeInputConsumer(consumer));

	// Removing the same input consumer again should fail.
	EXPECT_FALSE(device->removeInputConsumer(consumer));

	// Check the number of invocations.
	EXPECT_EQ(1, consumer->m_numTimesInitializedInput);
	// The update rate is not compared against a larger lower bound because the LabJack's response time is highly
	// dependent on the model, the connection (e.g., whether a high-speed USB2 hub is between the device and the
	// USB host), the number of USB frames required, and the calculations necessary for inputs/outputs.
	// See section 3.1 - Command/Response in the LabJack User's Guide.
	EXPECT_GE(consumer->m_numTimesReceivedInput, 1);
	EXPECT_LE(consumer->m_numTimesReceivedInput, 1.1 * device->getMaximumUpdateRate());

	EXPECT_TRUE(consumer->m_lastReceivedInput.customData().hasData(SurgSim::DataStructures::Names::DIGITAL_INPUTS));
	EXPECT_TRUE(consumer->m_lastReceivedInput.customData().hasData(SurgSim::DataStructures::Names::TIMER_INPUTS));
}

TEST(LabJackDeviceTest, OutputProducer)
{
	//LabJackScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<LabJackDevice> device = std::make_shared<LabJackDevice>("TestLabJack");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a LabJack device plugged in?";

	std::shared_ptr<RawTestListener> producer = std::make_shared<RawTestListener>();
	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);

	EXPECT_FALSE(device->removeOutputProducer(producer));
	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);

	EXPECT_TRUE(device->setOutputProducer(producer));

	// Sleep to see how many times the producer is invoked.
	boost::this_thread::sleep_until(boost::chrono::steady_clock::now() + boost::chrono::milliseconds(200));

	EXPECT_TRUE(device->removeOutputProducer(producer));

	// Removing the same input producer again should fail.
	EXPECT_FALSE(device->removeOutputProducer(producer));

	// Check the number of invocations.
	// The update rate is not compared against a larger lower bound because the LabJack's response time is highly
	// dependent on the model, the connection (e.g., whether a high-speed USB2 hub is between the device and the
	// USB host), the number of USB frames required, and the calculations necessary for inputs/outputs.
	// See section 3.1 - Command/Response in the LabJack User's Guide.
	EXPECT_GE(producer->m_numTimesRequestedOutput, 1);
	EXPECT_LE(producer->m_numTimesRequestedOutput, 1.1 * device->getMaximumUpdateRate());
}

TEST(LabJackDeviceTest, GettersAndSetters)
{
	std::shared_ptr<LabJackDevice> device = std::make_shared<LabJackDevice>("TestLabJack");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";

	const SurgSim::Device::LabJackType type = SurgSim::Device::LABJACKTYPE_U6;
	EXPECT_NO_THROW(device->setType(type));
	EXPECT_EQ(type, device->getType());

	const SurgSim::Device::LabJackConnection connection = SurgSim::Device::LABJACKCONNECTION_USB;
	EXPECT_NO_THROW(device->setConnection(connection));
	EXPECT_EQ(connection, device->getConnection());

	const std::string address = "14";
	EXPECT_NO_THROW(device->setAddress(address));
	EXPECT_EQ(address, device->getAddress());

	std::unordered_set<int> digitalInputChannels;
	digitalInputChannels.insert(2);
	digitalInputChannels.insert(11);
	EXPECT_NO_THROW(device->setDigitalInputChannels(digitalInputChannels));
	EXPECT_EQ(digitalInputChannels, device->getDigitalInputChannels());

	std::unordered_set<int> digitalOutputChannels;
	digitalOutputChannels.insert(3);
	digitalOutputChannels.insert(13);
	digitalOutputChannels.insert(17);
	EXPECT_NO_THROW(device->setDigitalOutputChannels(digitalOutputChannels));
	EXPECT_EQ(digitalOutputChannels, device->getDigitalOutputChannels());

	const SurgSim::Device::LabJackTimerBase timerBase = SurgSim::Device::LABJACKTIMERBASE_DEFAULT;
	EXPECT_NO_THROW(device->setTimerBase(timerBase));
	EXPECT_EQ(timerBase, device->getTimerBase());

	const int timerDivisor = 7;
	EXPECT_NO_THROW(device->setTimerClockDivisor(timerDivisor));
	EXPECT_EQ(timerDivisor, device->getTimerClockDivisor());

	const int pinOffset = 3;
	EXPECT_NO_THROW(device->setTimerCounterPinOffset(pinOffset));
	EXPECT_EQ(pinOffset, device->getTimerCounterPinOffset());

	std::unordered_map<int,SurgSim::Device::LabJackTimerMode> timers;
	timers[0] = SurgSim::Device::LABJACKTIMERMODE_QUAD;
	timers[3] = SurgSim::Device::LABJACKTIMERMODE_FREQOUT;
	EXPECT_NO_THROW(device->setTimers(timers));
	EXPECT_EQ(timers, device->getTimers());

	const double rate = 300.0;
	EXPECT_NO_THROW(device->setMaximumUpdateRate(rate));
	EXPECT_NEAR(rate, device->getMaximumUpdateRate(), 1e-9);
}



TEST(LabJackDeviceTest, NoSettingAfterInitialization)
{
	std::shared_ptr<LabJackDevice> device = std::make_shared<LabJackDevice>("TestLabJack");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a LabJack device plugged in?";

	const SurgSim::Device::LabJackType type = SurgSim::Device::LABJACKTYPE_U6;
	EXPECT_THROW(device->setType(type), SurgSim::Framework::AssertionFailure);

	const SurgSim::Device::LabJackConnection connection = SurgSim::Device::LABJACKCONNECTION_USB;
	EXPECT_THROW(device->setConnection(connection), SurgSim::Framework::AssertionFailure);

	const std::string address = "14";
	EXPECT_THROW(device->setAddress(address), SurgSim::Framework::AssertionFailure);

	std::unordered_set<int> digitalInputChannels;
	digitalInputChannels.insert(2);
	digitalInputChannels.insert(11);
	EXPECT_THROW(device->setDigitalInputChannels(digitalInputChannels), SurgSim::Framework::AssertionFailure);

	std::unordered_set<int> digitalOutputChannels;
	digitalOutputChannels.insert(3);
	digitalOutputChannels.insert(13);
	digitalOutputChannels.insert(17);
	EXPECT_THROW(device->setDigitalOutputChannels(digitalOutputChannels), SurgSim::Framework::AssertionFailure);

	const SurgSim::Device::LabJackTimerBase timerBase = SurgSim::Device::LABJACKTIMERBASE_DEFAULT;
	EXPECT_THROW(device->setTimerBase(timerBase), SurgSim::Framework::AssertionFailure);

	const int timerDivisor = 7;
	EXPECT_THROW(device->setTimerClockDivisor(timerDivisor), SurgSim::Framework::AssertionFailure);

	const int pinOffset = 3;
	EXPECT_THROW(device->setTimerCounterPinOffset(pinOffset), SurgSim::Framework::AssertionFailure);

	std::unordered_map<int,SurgSim::Device::LabJackTimerMode> timers;
	timers[0] = SurgSim::Device::LABJACKTIMERMODE_QUAD;
	timers[3] = SurgSim::Device::LABJACKTIMERMODE_FREQOUT;
	EXPECT_THROW(device->setTimers(timers), SurgSim::Framework::AssertionFailure);

	const double rate = 300.0;
	EXPECT_THROW(device->setMaximumUpdateRate(rate), SurgSim::Framework::AssertionFailure);
}