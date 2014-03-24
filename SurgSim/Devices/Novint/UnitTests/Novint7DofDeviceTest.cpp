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
/// Tests for the Novint7DofDevice class.

#include <memory>
#include <string>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <gtest/gtest.h>
#include "SurgSim/Devices/Novint/Novint7DofDevice.h"
//#include "SurgSim/Devices/Novint/NovintScaffold.h"  // only needed if calling setDefaultLogLevel()
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Input/InputConsumerInterface.h"
#include "SurgSim/Input/OutputProducerInterface.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Framework/Clock.h"

using SurgSim::Device::Novint7DofDevice;
using SurgSim::Device::NovintScaffold;
using SurgSim::DataStructures::DataGroup;
using SurgSim::Input::InputConsumerInterface;
using SurgSim::Input::OutputProducerInterface;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Matrix44d;
using SurgSim::Framework::Clock;


// Define common device names used in the Novint device tests.
extern const char* const NOVINT_TEST_DEVICE_NAME;
extern const char* const NOVINT_TEST_DEVICE_NAME_2;


struct Test7DofListener : public InputConsumerInterface, public OutputProducerInterface
{
public:
	Test7DofListener() :
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

void Test7DofListener::initializeInput(const std::string& device, const DataGroup& inputData)
{
	++m_numTimesInitializedInput;
}

void Test7DofListener::handleInput(const std::string& device, const DataGroup& inputData)
{
	++m_numTimesReceivedInput;
	m_lastReceivedInput = inputData;
}

bool Test7DofListener::requestOutput(const std::string& device, DataGroup* outputData)
{
	++m_numTimesRequestedOutput;
	return false;
}


TEST(Novint7DofDeviceTest, CreateUninitializedDevice)
{
	//NovintScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<Novint7DofDevice> device =
		std::make_shared<Novint7DofDevice>("TestFalcon", NOVINT_TEST_DEVICE_NAME);
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
}

TEST(Novint7DofDeviceTest, CreateAndInitializeDevice)
{
	//NovintScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<Novint7DofDevice> device =
		std::make_shared<Novint7DofDevice>("TestFalcon", NOVINT_TEST_DEVICE_NAME);
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	EXPECT_FALSE(device->isInitialized());
	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a Novint device plugged in?";
	EXPECT_TRUE(device->isInitialized());
}

// Note: this should work if the "Default Falcon" device can be initialized... but we have no reason to think it can,
// so I'm going to disable the test.
TEST(Novint7DofDeviceTest, DISABLED_CreateAndInitializeDefaultDevice)
{
	//NovintScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<Novint7DofDevice> device = std::make_shared<Novint7DofDevice>("TestFalcon", "");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	EXPECT_FALSE(device->isInitialized());
	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a Novint device plugged in?";
	EXPECT_TRUE(device->isInitialized());
}

TEST(Novint7DofDeviceTest, Name)
{
	//NovintScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<Novint7DofDevice> device =
		std::make_shared<Novint7DofDevice>("TestFalcon", NOVINT_TEST_DEVICE_NAME);
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	EXPECT_EQ("TestFalcon", device->getName());
	EXPECT_TRUE(device->initialize()) << "Initialization failed.  Is a Novint device plugged in?";
	EXPECT_EQ("TestFalcon", device->getName());
}

static void testCreateDeviceSeveralTimes(bool doSleep)
{
	for (int i = 0;  i < 6;  ++i)
	{
		std::shared_ptr<Novint7DofDevice> device =
			std::make_shared<Novint7DofDevice>("TestFalcon", NOVINT_TEST_DEVICE_NAME);
		ASSERT_TRUE(device != nullptr) << "Device creation failed.";
		ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a Novint device plugged in?";
		if (doSleep)
		{
			boost::this_thread::sleep_until(Clock::now() + boost::chrono::milliseconds(100));
		}
		// the device will be destroyed here
	}
}

TEST(Novint7DofDeviceTest, CreateDeviceSeveralTimes)
{
	//NovintScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	testCreateDeviceSeveralTimes(true);
}

TEST(Novint7DofDeviceTest, CreateSeveralDevices)
{
	//NovintScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<Novint7DofDevice> device1 =
		std::make_shared<Novint7DofDevice>("Novint1", NOVINT_TEST_DEVICE_NAME);
	ASSERT_TRUE(device1 != nullptr) << "Device creation failed.";
	ASSERT_TRUE(device1->initialize()) << "Initialization failed.  Is a Novint device plugged in?";

	// We can't check what happens with the scaffolds, since those are no longer a part of the device's API...

	std::shared_ptr<Novint7DofDevice> device2 =
		std::make_shared<Novint7DofDevice>("Novint2", NOVINT_TEST_DEVICE_NAME_2);
	ASSERT_TRUE(device2 != nullptr) << "Device creation failed.";
	if (! device2->initialize())
	{
		std::cerr << "[Warning: second Novint did not come up; is it plugged in?]" << std::endl;
	}
}

TEST(Novint7DofDeviceTest, CreateDevicesWithSameName)
{
	//NovintScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<Novint7DofDevice> device1 =
		std::make_shared<Novint7DofDevice>("Novint", NOVINT_TEST_DEVICE_NAME);
	ASSERT_TRUE(device1 != nullptr) << "Device creation failed.";
	ASSERT_TRUE(device1->initialize()) << "Initialization failed.  Is a Novint device plugged in?";

	std::shared_ptr<Novint7DofDevice> device2 =
		std::make_shared<Novint7DofDevice>("Novint", NOVINT_TEST_DEVICE_NAME_2);
	ASSERT_TRUE(device2 != nullptr) << "Device creation failed.";
	ASSERT_FALSE(device2->initialize()) << "Initialization succeeded despite duplicate name.";
}

TEST(Novint7DofDeviceTest, CreateDevicesWithSameInitializationName)
{
	//NovintScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<Novint7DofDevice> device1 =
		std::make_shared<Novint7DofDevice>("Novint1", NOVINT_TEST_DEVICE_NAME);
	ASSERT_TRUE(device1 != nullptr) << "Device creation failed.";
	ASSERT_TRUE(device1->initialize()) << "Initialization failed.  Is a Novint device plugged in?";

	std::shared_ptr<Novint7DofDevice> device2 =
		std::make_shared<Novint7DofDevice>("Novint2", NOVINT_TEST_DEVICE_NAME);
	ASSERT_TRUE(device2 != nullptr) << "Device creation failed.";
	ASSERT_FALSE(device2->initialize()) << "Initialization succeeded despite duplicate initialization name.";
}

TEST(Novint7DofDeviceTest, InputConsumer)
{
	//NovintScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<Novint7DofDevice> device =
		std::make_shared<Novint7DofDevice>("TestFalcon", NOVINT_TEST_DEVICE_NAME);
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	EXPECT_TRUE(device->initialize()) << "Initialization failed.  Is a Novint device plugged in?";

	std::shared_ptr<Test7DofListener> consumer = std::make_shared<Test7DofListener>();
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);

	EXPECT_FALSE(device->removeInputConsumer(consumer));
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);

	EXPECT_TRUE(device->addInputConsumer(consumer));

	// Adding the same input consumer again should fail.
	EXPECT_FALSE(device->addInputConsumer(consumer));

	// Sleep for a second, to see how many times the consumer is invoked.
	// (A Novint device is supposed to run at 1KHz.)
	boost::this_thread::sleep_until(Clock::now() + boost::chrono::milliseconds(10000));

	EXPECT_TRUE(device->removeInputConsumer(consumer));

	// Removing the same input consumer again should fail.
	EXPECT_FALSE(device->removeInputConsumer(consumer));

	// Check the number of invocations.
	EXPECT_GE(consumer->m_numTimesReceivedInput, 10*700);
	EXPECT_LE(consumer->m_numTimesReceivedInput, 10*1300);

	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasData(SurgSim::DataStructures::Names::POSE));
	EXPECT_TRUE(consumer->m_lastReceivedInput.booleans().hasData(SurgSim::DataStructures::Names::BUTTON_1));
}

TEST(Novint7DofDeviceTest, OutputProducer)
{
	//NovintScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<Novint7DofDevice> device =
		std::make_shared<Novint7DofDevice>("TestFalcon", NOVINT_TEST_DEVICE_NAME);
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	EXPECT_TRUE(device->initialize()) << "Initialization failed.  Is a Novint device plugged in?";

	std::shared_ptr<Test7DofListener> producer = std::make_shared<Test7DofListener>();
	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);

	EXPECT_FALSE(device->removeOutputProducer(producer));
	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);

	EXPECT_TRUE(device->setOutputProducer(producer));

	// Sleep for a second, to see how many times the producer is invoked.
	// (A Novint Falcon device is supposed to run at 1KHz.)
	boost::this_thread::sleep_until(Clock::now() + boost::chrono::milliseconds(10000));

	EXPECT_TRUE(device->removeOutputProducer(producer));

	// Removing the same input producer again should fail.
	EXPECT_FALSE(device->removeOutputProducer(producer));

	// Check the number of invocations.
	EXPECT_GE(producer->m_numTimesRequestedOutput, 10*700);
	EXPECT_LE(producer->m_numTimesRequestedOutput, 10*1300);
}
