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
 * Tests for the PhantomDevice and PhantomManager classes.
 */

#include <memory>
#include <string>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <gtest/gtest.h>
#include <SurgSim/Devices/Phantom/PhantomDevice.h>
#include <SurgSim/Devices/Phantom/PhantomManager.h>
#include <SurgSim/DataStructures/DataGroup.h>
#include <SurgSim/Input/InputConsumerInterface.h>
#include <SurgSim/Input/OutputProducerInterface.h>
#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Math/Matrix.h>

using SurgSim::Device::PhantomDevice;
using SurgSim::Device::PhantomManager;
using SurgSim::DataStructures::DataGroup;
using SurgSim::Input::InputConsumerInterface;
using SurgSim::Input::OutputProducerInterface;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Matrix44d;


struct TestListener : public InputConsumerInterface, public OutputProducerInterface
{
public:
	TestListener() :
		m_numTimesReceivedInput(0),
		m_numTimesRequestedOutput(0)
	{
	}

	virtual void initializeInput(const std::string& device, const DataGroup& initialInput)
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


TEST(PhantomDeviceTest, CanConstructManager)
{
	ASSERT_NO_THROW({PhantomManager manager;});
}

TEST(PhantomDeviceTest, CreateAndReleaseDevice)
{
	PhantomManager manager;
	std::shared_ptr<PhantomDevice> device = manager.createDevice("TestPhantom", "Default PHANToM");
	ASSERT_TRUE(device) << "Initialization failed.  Is a Phantom device plugged in?";
	manager.releaseDevice(device);
}

TEST(PhantomDeviceTest, DestroyManagerWithRunningDevice)
{
	PhantomManager manager;
	std::shared_ptr<PhantomDevice> device = manager.createDevice("TestPhantom", "Default PHANToM");
	ASSERT_TRUE(device) << "Initialization failed.  Is a Phantom device plugged in?";

	// NB: device NOT released here!
}

TEST(PhantomDeviceTest, Name)
{
	PhantomManager manager;
	std::shared_ptr<PhantomDevice> device = manager.createDevice("TestPhantom", "Default PHANToM");
	ASSERT_TRUE(device) << "Initialization failed.  Is a Phantom device plugged in?";
	EXPECT_EQ("TestPhantom", device->getName());
}

TEST(PhantomDeviceTest, CreateDeviceTwice)
{
	PhantomManager manager;
	{
		std::shared_ptr<PhantomDevice> device = manager.createDevice("TestPhantom", "Default PHANToM");
		ASSERT_TRUE(device) << "Initialization failed.  Is a Phantom device plugged in?";
		manager.releaseDevice(device);
	}
	{
		std::shared_ptr<PhantomDevice> device = manager.createDevice("TestPhantom", "Default PHANToM");
		ASSERT_TRUE(device) << "Initialization failed.  Is a Phantom device plugged in?";
		manager.releaseDevice(device);
	}
}

TEST(PhantomDeviceTest, CreateTwoDevices)
{
	PhantomManager manager;

	std::shared_ptr<PhantomDevice> device1 = manager.createDevice("FirstPhantom", "Default PHANToM");
	ASSERT_TRUE(device1) << "Initialization failed.  Is a Phantom device plugged in?";

	std::shared_ptr<PhantomDevice> device2 = manager.createDevice("SecondPhantom", "Second PHANToM");
	if (! device2)
	{
		std::cerr << "[Warning: second Phantom device not actually created; is it plugged in?]" << std::endl;
	}
}

TEST(PhantomDeviceTest, InputConsumer)
{
	PhantomManager manager;
	std::shared_ptr<PhantomDevice> device = manager.createDevice("TestPhantom", "Default PHANToM");
	ASSERT_TRUE(device) << "Initialization failed.  Is a Phantom device plugged in?";

	std::shared_ptr<TestListener> consumer = std::make_shared<TestListener>();
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);

	EXPECT_FALSE(device->removeInputConsumer(consumer));
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);

	EXPECT_TRUE(device->addInputConsumer(consumer));

	// Adding the same input consumer again should fail.
	EXPECT_FALSE(device->addInputConsumer(consumer));

	// Sleep for a second, to see how many times the consumer is invoked.
	// (A Phantom device is supposed to run at 1KHz.)
	boost::this_thread::sleep_until(boost::chrono::steady_clock::now() + boost::chrono::milliseconds(1000));

	EXPECT_TRUE(device->removeInputConsumer(consumer));

	// Removing the same input consumer again should fail.
	EXPECT_FALSE(device->removeInputConsumer(consumer));

	// Check the number of invocations.
	EXPECT_GE(consumer->m_numTimesReceivedInput, 700);
	EXPECT_LE(consumer->m_numTimesReceivedInput, 1300);

	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasCurrentData("pose"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.booleans().hasCurrentData("button0"));
}

TEST(PhantomDeviceTest, OutputProducer)
{
	PhantomManager manager;
	std::shared_ptr<PhantomDevice> device = manager.createDevice("TestPhantom", "Default PHANToM");
	ASSERT_TRUE(device) << "Initialization failed.  Is a Phantom device plugged in?";

	std::shared_ptr<TestListener> producer = std::make_shared<TestListener>();
	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);

	EXPECT_FALSE(device->removeOutputProducer(producer));
	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);

	EXPECT_TRUE(device->setOutputProducer(producer));

	// Sleep for a second, to see how many times the producer is invoked.
	// (A Phantom device is supposed to run at 1KHz.)
	boost::this_thread::sleep_until(boost::chrono::steady_clock::now() + boost::chrono::milliseconds(1000));

	EXPECT_TRUE(device->removeOutputProducer(producer));

	// Removing the same input producer again should fail.
	EXPECT_FALSE(device->removeOutputProducer(producer));

	// Check the number of invocations.
	EXPECT_GE(producer->m_numTimesRequestedOutput, 700);
	EXPECT_LE(producer->m_numTimesRequestedOutput, 1300);
}
