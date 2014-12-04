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
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Testing/MockInputOutput.h"

using SurgSim::Device::LabJackDevice;
using SurgSim::Device::LabJackScaffold;
using SurgSim::DataStructures::DataGroup;
using SurgSim::Input::InputConsumerInterface;
using SurgSim::Input::OutputProducerInterface;
using SurgSim::Testing::MockInputOutput;

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

TEST(LabJackDeviceTest, InputConsumer)
{
	std::shared_ptr<LabJackDevice> device = std::make_shared<LabJackDevice>("TestLabJack");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a LabJack device plugged in?";

	std::shared_ptr<MockInputOutput> consumer = std::make_shared<MockInputOutput>();
	EXPECT_EQ(0, consumer->m_numTimesInitializedInput);
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);

	EXPECT_FALSE(device->removeInputConsumer(consumer));
	EXPECT_EQ(0, consumer->m_numTimesInitializedInput);
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);

	EXPECT_TRUE(device->addInputConsumer(consumer));
	EXPECT_TRUE(device->setOutputProducer(consumer));

	// Adding the same input consumer again should fail.
	EXPECT_FALSE(device->addInputConsumer(consumer));

	// Sleep to see how many times the consumer is invoked.
	boost::this_thread::sleep_until(boost::chrono::steady_clock::now() + boost::chrono::milliseconds(1000));

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

	EXPECT_TRUE(consumer->m_lastReceivedInput.booleans().hasEntry(SurgSim::DataStructures::Names::DIGITAL_INPUT_PREFIX +
		std::to_string(0))); // The LabJackDevice provides entries for digital input lines 0 - 23.

	EXPECT_TRUE(consumer->m_lastReceivedInput.scalars().hasEntry(SurgSim::DataStructures::Names::TIMER_INPUT_PREFIX +
		std::to_string(0))); // The LabJackDevice provides entries for timer input lines 0 - 6.
}

TEST(LabJackDeviceTest, OutputProducer)
{
	//LabJackScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<LabJackDevice> device = std::make_shared<LabJackDevice>("TestLabJack");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a LabJack device plugged in?";

	std::shared_ptr<MockInputOutput> producer = std::make_shared<MockInputOutput>();
	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);

	EXPECT_FALSE(device->removeOutputProducer(producer));
	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);

	EXPECT_TRUE(device->setOutputProducer(producer));

	// Sleep to see how many times the producer is invoked.
	boost::this_thread::sleep_until(boost::chrono::steady_clock::now() + boost::chrono::milliseconds(1000));

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

	const SurgSim::Device::LabJack::Model model = SurgSim::Device::LabJack::MODEL_U6;
	EXPECT_NO_THROW(device->setModel(model));
	EXPECT_EQ(model, device->getModel());

	const SurgSim::Device::LabJack::Connection connection = SurgSim::Device::LabJack::CONNECTION_USB;
	EXPECT_NO_THROW(device->setConnection(connection));
	EXPECT_EQ(connection, device->getConnection());

	const std::string address = "14";
	EXPECT_NO_THROW(device->setAddress(address));
	EXPECT_EQ(address, device->getAddress());

	bool reset = true;
	device->setResetOnDestruct(reset);
	EXPECT_EQ(reset, device->getResetOnDestruct());
	reset = false;
	device->setResetOnDestruct(reset);
	EXPECT_EQ(reset, device->getResetOnDestruct());

	std::unordered_set<int> digitalInputChannels;
	digitalInputChannels.insert(2);
	digitalInputChannels.insert(11);
	EXPECT_NO_THROW(device->enableDigitalInput(SurgSim::Device::LabJack::FIO2));
	EXPECT_NO_THROW(device->enableDigitalInput(SurgSim::Device::LabJack::EIO3));
	EXPECT_EQ(digitalInputChannels, device->getDigitalInputs());

	digitalInputChannels.insert(14);
	EXPECT_NO_THROW(device->setDigitalInputs(digitalInputChannels));
	EXPECT_EQ(digitalInputChannels, device->getDigitalInputs());

	std::unordered_set<int> digitalOutputChannels;
	digitalOutputChannels.insert(3);
	digitalOutputChannels.insert(17);
	EXPECT_NO_THROW(device->enableDigitalOutput(SurgSim::Device::LabJack::FIO3));
	EXPECT_NO_THROW(device->enableDigitalOutput(SurgSim::Device::LabJack::CIO1));
	EXPECT_EQ(digitalOutputChannels, device->getDigitalOutputs());

	digitalOutputChannels.insert(5);
	EXPECT_NO_THROW(device->setDigitalOutputs(digitalOutputChannels));
	EXPECT_EQ(digitalOutputChannels, device->getDigitalOutputs());

	const SurgSim::Device::LabJack::TimerBase timerBase = SurgSim::Device::LabJack::TIMERBASE_DEFAULT;
	EXPECT_NO_THROW(device->setTimerBase(timerBase));
	EXPECT_EQ(timerBase, device->getTimerBase());

	const int timerDivisor = 7;
	EXPECT_NO_THROW(device->setTimerClockDivisor(timerDivisor));
	EXPECT_EQ(timerDivisor, device->getTimerClockDivisor());

	const int pinOffset = 3;
	EXPECT_NO_THROW(device->setTimerCounterPinOffset(pinOffset));
	EXPECT_EQ(pinOffset, device->getTimerCounterPinOffset());

	std::unordered_map<int, SurgSim::Device::LabJack::TimerSettings> timers;
	SurgSim::Device::LabJack::TimerSettings quadrature =
		{SurgSim::Device::LabJack::TIMERMODE_QUADRATURE, SurgSim::DataStructures::OptionalValue<int>()};
	timers[0] = quadrature;
	SurgSim::Device::LabJack::TimerSettings frequencyOutput =
		{SurgSim::Device::LabJack::TIMERMODE_FREQUENCY_OUTPUT, SurgSim::DataStructures::OptionalValue<int>(234)};
	timers[3] = frequencyOutput;
	EXPECT_NO_THROW(device->enableTimer(SurgSim::Device::LabJack::TIMER0,
		SurgSim::Device::LabJack::TIMERMODE_QUADRATURE));
	EXPECT_NO_THROW(device->enableTimer(SurgSim::Device::LabJack::TIMER3,
		SurgSim::Device::LabJack::TIMERMODE_FREQUENCY_OUTPUT, 234));
	EXPECT_EQ(timers, device->getTimers());

	SurgSim::Device::LabJack::TimerSettings dutyCycle =
		{SurgSim::Device::LabJack::TIMERMODE_DUTY_CYCLE, SurgSim::DataStructures::OptionalValue<int>()};
	timers[4] = dutyCycle;
	EXPECT_NO_THROW(device->setTimers(timers));
	EXPECT_EQ(timers, device->getTimers());

	const double rate = 300.0;
	EXPECT_NO_THROW(device->setMaximumUpdateRate(rate));
	EXPECT_NEAR(rate, device->getMaximumUpdateRate(), 1e-9);

	std::unordered_map<int, SurgSim::Device::LabJack::AnalogInputSettings> analogInputs;
	const SurgSim::Device::LabJack::AnalogInputSettings differentialRangeAndChannel =
		{SurgSim::Device::LabJack::Range::RANGE_10,
		SurgSim::DataStructures::OptionalValue<int>(1)};
	analogInputs[2] = differentialRangeAndChannel;
	const SurgSim::Device::LabJack::AnalogInputSettings singleEndedRange =
		{SurgSim::Device::LabJack::Range::RANGE_0_POINT_1,
		SurgSim::DataStructures::OptionalValue<int>()};
	analogInputs[5] = singleEndedRange;
	EXPECT_NO_THROW(device->enableAnalogInput(SurgSim::Device::LabJack::AIN2,
		SurgSim::Device::LabJack::Range::RANGE_10, 1));
	EXPECT_NO_THROW(device->enableAnalogInput(SurgSim::Device::LabJack::AIN5,
		SurgSim::Device::LabJack::Range::RANGE_0_POINT_1));
	EXPECT_EQ(analogInputs, device->getAnalogInputs());

	const SurgSim::Device::LabJack::AnalogInputSettings anotherRange =
	{SurgSim::Device::LabJack::Range::RANGE_0_POINT_01,
	SurgSim::DataStructures::OptionalValue<int>()};
	analogInputs[6] = anotherRange;
	EXPECT_NO_THROW(device->setAnalogInputs(analogInputs));
	EXPECT_EQ(analogInputs, device->getAnalogInputs());

	const int resolution = 3;
	EXPECT_NO_THROW(device->setAnalogInputResolution(resolution));
	EXPECT_EQ(resolution, device->getAnalogInputResolution());

	const int settling = 2;
	EXPECT_NO_THROW(device->setAnalogInputSettling(settling));
	EXPECT_EQ(settling, device->getAnalogInputSettling());

	std::unordered_set<int> analogOutputChannels;
	analogOutputChannels.insert(1);
	EXPECT_NO_THROW(device->enableAnalogOutput(SurgSim::Device::LabJack::DAC1));
	EXPECT_EQ(analogOutputChannels, device->getAnalogOutputs());

	analogOutputChannels.insert(0);
	EXPECT_NO_THROW(device->setAnalogOutputs(analogOutputChannels));
	EXPECT_EQ(analogOutputChannels, device->getAnalogOutputs());
}



TEST(LabJackDeviceTest, NoSettingAfterInitialization)
{
	std::shared_ptr<LabJackDevice> device = std::make_shared<LabJackDevice>("TestLabJack");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a LabJack device plugged in?";

	EXPECT_THROW(device->setModel(SurgSim::Device::LabJack::MODEL_U6), SurgSim::Framework::AssertionFailure);

	EXPECT_THROW(device->setConnection(SurgSim::Device::LabJack::CONNECTION_USB), SurgSim::Framework::AssertionFailure);

	EXPECT_THROW(device->setAddress("14"), SurgSim::Framework::AssertionFailure);

	EXPECT_THROW(device->enableDigitalInput(SurgSim::Device::LabJack::FIO2), SurgSim::Framework::AssertionFailure);

	EXPECT_THROW(device->setDigitalInputs(std::unordered_set<int>()), SurgSim::Framework::AssertionFailure);

	EXPECT_THROW(device->enableDigitalOutput(SurgSim::Device::LabJack::FIO3), SurgSim::Framework::AssertionFailure);

	EXPECT_THROW(device->setDigitalOutputs(std::unordered_set<int>()), SurgSim::Framework::AssertionFailure);

	EXPECT_THROW(device->setTimerBase(SurgSim::Device::LabJack::TIMERBASE_DEFAULT),
		SurgSim::Framework::AssertionFailure);

	EXPECT_THROW(device->setTimerClockDivisor(7), SurgSim::Framework::AssertionFailure);

	EXPECT_THROW(device->setTimerCounterPinOffset(3), SurgSim::Framework::AssertionFailure);

	EXPECT_THROW(device->enableTimer(SurgSim::Device::LabJack::TIMER0, SurgSim::Device::LabJack::TIMERMODE_QUADRATURE),
		SurgSim::Framework::AssertionFailure);

	EXPECT_THROW(device->setTimers(std::unordered_map<int,
		SurgSim::Device::LabJack::TimerSettings>()),
		SurgSim::Framework::AssertionFailure);

	EXPECT_THROW(device->setMaximumUpdateRate(300.0), SurgSim::Framework::AssertionFailure);

	EXPECT_THROW(device->enableAnalogInput(SurgSim::Device::LabJack::AIN2,
		SurgSim::Device::LabJack::Range::RANGE_10, 1),
		SurgSim::Framework::AssertionFailure);

	EXPECT_THROW(device->setAnalogInputs(std::unordered_map<int,
		SurgSim::Device::LabJack::AnalogInputSettings>()),
		SurgSim::Framework::AssertionFailure);

	EXPECT_THROW(device->setAnalogInputResolution(3), SurgSim::Framework::AssertionFailure);

	EXPECT_THROW(device->setAnalogInputSettling(2), SurgSim::Framework::AssertionFailure);

	EXPECT_THROW(device->enableAnalogOutput(SurgSim::Device::LabJack::DAC0), SurgSim::Framework::AssertionFailure);

	EXPECT_THROW(device->setAnalogOutputs(std::unordered_set<int>()), SurgSim::Framework::AssertionFailure);
}