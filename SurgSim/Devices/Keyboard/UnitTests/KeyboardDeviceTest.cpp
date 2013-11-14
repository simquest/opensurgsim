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
/// Tests for the KeyboardDevice class.

#include <memory>
#include <string>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <gtest/gtest.h>
#include <SurgSim/Devices/Keyboard/KeyboardDevice.h>
#include <SurgSim/DataStructures/DataGroup.h>
#include <SurgSim/Input/InputConsumerInterface.h>
#include <SurgSim/Input/OutputProducerInterface.h>
#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Math/Matrix.h>

using SurgSim::Device::KeyboardDevice;
using SurgSim::Device::KeyboardScaffold;
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


TEST(KeyboardDeviceTest, CreateUninitializedDevice)
{
	std::shared_ptr<KeyboardDevice> device = std::make_shared<KeyboardDevice>("TestKeyboard");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
}

TEST(KeyboardDeviceTest, CreateAndInitializeDevice)
{
	//KeyboardScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<KeyboardDevice> device = std::make_shared<KeyboardDevice>("TestKeyboard");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	EXPECT_FALSE(device->isInitialized());
	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a Keyboard device plugged in?";
	EXPECT_TRUE(device->isInitialized());
}

TEST(KeyboardDeviceTest, CreateAndInitializeDefaultDevice)
{
	//KeyboardScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<KeyboardDevice> device = std::make_shared<KeyboardDevice>("TestKeyboard");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	EXPECT_FALSE(device->isInitialized());
	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a Keyboard device plugged in?";
	EXPECT_TRUE(device->isInitialized());
}

TEST(KeyboardDeviceTest, Name)
{
	//KeyboardScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<KeyboardDevice> device = std::make_shared<KeyboardDevice>("TestKeyboard");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	EXPECT_EQ("TestKeyboard", device->getName());
	EXPECT_TRUE(device->initialize()) << "Initialization failed.  Is a Keyboard device plugged in?";
	EXPECT_EQ("TestKeyboard", device->getName());
}

static void testCreateDeviceSeveralTimes(bool doSleep)
{
	for (int i = 0;  i < 6;  ++i)
	{
		std::shared_ptr<KeyboardDevice> device = std::make_shared<KeyboardDevice>("TestKeyboard");
		ASSERT_TRUE(device != nullptr) << "Device creation failed.";
		ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a Keyboard device plugged in?";
		if (doSleep)
		{
			boost::this_thread::sleep_until(boost::chrono::steady_clock::now() + boost::chrono::milliseconds(100));
		}
		// the device will be destroyed here
	}
}

TEST(KeyboardDeviceTest, CreateDeviceSeveralTimes)
{
	//KeyboardScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	testCreateDeviceSeveralTimes(true);
}

TEST(KeyboardDeviceTest, CreateSeveralDevices)
{
	//KeyboardScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<KeyboardDevice> device1 = std::make_shared<KeyboardDevice>("Keyboard1");
	ASSERT_TRUE(device1 != nullptr) << "Device creation failed.";
	ASSERT_TRUE(device1->initialize()) << "Initialization failed.  Is a Keyboard device plugged in?";

	// We can't check what happens with the scaffolds, since those are no longer a part of the device's API...

	std::shared_ptr<KeyboardDevice> device2 = std::make_shared<KeyboardDevice>("Keyboard2");
	ASSERT_TRUE(device2 != nullptr) << "Device creation failed.";
	if (! device2->initialize())
	{
		std::cerr << "[Warning: second Keyboard did not come up; is it plugged in?]" << std::endl;
	}
}

TEST(KeyboardDeviceTest, CreateDevicesWithSameName)
{
	//KeyboardScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<KeyboardDevice> device1 = std::make_shared<KeyboardDevice>("Keyboard");
	ASSERT_TRUE(device1 != nullptr) << "Device creation failed.";
	ASSERT_TRUE(device1->initialize()) << "Initialization failed.  Is a Keyboard device plugged in?";

	std::shared_ptr<KeyboardDevice> device2 = std::make_shared<KeyboardDevice>("Keyboard");
	ASSERT_TRUE(device2 != nullptr) << "Device creation failed.";
	ASSERT_FALSE(device2->initialize()) << "Initialization succeeded despite duplicate name.";
}

TEST(KeyboardDeviceTest, InputConsumer)
{
	//KeyboardScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<KeyboardDevice> device = std::make_shared<KeyboardDevice>("TestKeyboard");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	EXPECT_TRUE(device->initialize()) << "Initialization failed.  Is a Keyboard device plugged in?";

	std::shared_ptr<TestListener> consumer = std::make_shared<TestListener>();
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);

	EXPECT_FALSE(device->removeInputConsumer(consumer));
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);

	EXPECT_TRUE(device->addInputConsumer(consumer));

	// Adding the same input consumer again should fail.
	EXPECT_FALSE(device->addInputConsumer(consumer));

	// Sleep for a second, to see how many times the consumer is invoked.
	// (A Keyboard device is supposed to run at 1KHz.)
	boost::this_thread::sleep_until(boost::chrono::steady_clock::now() + boost::chrono::milliseconds(1000));

	EXPECT_TRUE(device->removeInputConsumer(consumer));

	// Removing the same input consumer again should fail.
	EXPECT_FALSE(device->removeInputConsumer(consumer));

	//// Check the number of invocations.
	//EXPECT_GE(consumer->m_numTimesReceivedInput, 700);
	//EXPECT_LE(consumer->m_numTimesReceivedInput, 1300);

	EXPECT_TRUE(consumer->m_lastReceivedInput.integers().hasEntry("key"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.integers().hasEntry("key_modifier"));
}

//TEST(KeyboardDeviceTest, OutputProducer)
//{
//	//KeyboardScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
//	std::shared_ptr<KeyboardDevice> device = std::make_shared<KeyboardDevice>("TestKeyboard", "Default Keyboard");
//	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
//	EXPECT_TRUE(device->initialize()) << "Initialization failed.  Is a Keyboard device plugged in?";
//
//	std::shared_ptr<TestListener> producer = std::make_shared<TestListener>();
//	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);
//
//	EXPECT_FALSE(device->removeOutputProducer(producer));
//	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);
//
//	EXPECT_TRUE(device->setOutputProducer(producer));
//
//	// Sleep for a second, to see how many times the producer is invoked.
//	// (A Keyboard device is supposed to run at 1KHz.)
//	boost::this_thread::sleep_until(boost::chrono::steady_clock::now() + boost::chrono::milliseconds(1000));
//
//	EXPECT_TRUE(device->removeOutputProducer(producer));
//
//	// Removing the same input producer again should fail.
//	EXPECT_FALSE(device->removeOutputProducer(producer));
//
//	// Check the number of invocations.
//	EXPECT_GE(producer->m_numTimesRequestedOutput, 700);
//	EXPECT_LE(producer->m_numTimesRequestedOutput, 1300);
//}
