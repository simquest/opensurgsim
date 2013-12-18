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
/// Tests for the TrackIRDevice class.

#include <memory>
#include <string>

#include <boost/thread.hpp>
#include <boost/chrono.hpp>

#include <gtest/gtest.h>

#include "SurgSim/Devices/TrackIR/TrackIRDevice.h"
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Input/InputConsumerInterface.h"
#include "SurgSim/Input/OutputProducerInterface.h"

using SurgSim::Device::TrackIRDevice;
using SurgSim::Device::TrackIRScaffold;
using SurgSim::DataStructures::DataGroup;
using SurgSim::Input::InputConsumerInterface;
using SurgSim::Input::OutputProducerInterface;


struct TestListener : public InputConsumerInterface, public OutputProducerInterface
{
public:
	TestListener() :
		m_numTimesInitializedInput(0),
		m_numTimesReceivedInput(0),
		m_numTimesRequestedOutput(0)
	{
	}

	virtual void initializeInput(const std::string& device, const DataGroup& inputData)
	{
		++m_numTimesInitializedInput;
	}
	virtual void handleInput(const std::string& device, const DataGroup& inputData)
	{
		++m_numTimesReceivedInput;
		m_lastReceivedInput = inputData;
	}
	virtual bool requestOutput(const std::string& device, DataGroup* outputData)
	{
		++m_numTimesRequestedOutput;
		return false;
	}

	int m_numTimesInitializedInput;
	int m_numTimesReceivedInput;
	int m_numTimesRequestedOutput;
	DataGroup m_lastReceivedInput;
};


TEST(TrackIRDeviceTest, CreateUninitializedDevice)
{
	std::shared_ptr<TrackIRDevice> device = std::make_shared<TrackIRDevice>("TrackIR");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
}

TEST(TrackIRDeviceTest, CreateAndInitializeDevice)
{
	std::shared_ptr<TrackIRDevice> device = std::make_shared<TrackIRDevice>("TrackIR");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	EXPECT_FALSE(device->isInitialized());
	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a TrackIR device plugged in?";
	EXPECT_TRUE(device->isInitialized());
}

TEST(TrackIRDeviceTest, CreateAndInitializeDefaultDevice)
{
	std::shared_ptr<TrackIRDevice> device = std::make_shared<TrackIRDevice>("TrackIR");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	EXPECT_FALSE(device->isInitialized());
	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a TrackIR device plugged in?";
	EXPECT_TRUE(device->isInitialized());
}

TEST(TrackIRDeviceTest, Name)
{
	std::shared_ptr<TrackIRDevice> device = std::make_shared<TrackIRDevice>("TrackIR");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	EXPECT_EQ("TrackIR", device->getName());
	EXPECT_TRUE(device->initialize()) << "Initialization failed.  Is a TrackIR device plugged in?";
	EXPECT_EQ("TrackIR", device->getName());
}

TEST(TrackIRDeviceTest, CreateDeviceTwice)
{
	std::shared_ptr<TrackIRDevice> device = std::make_shared<TrackIRDevice>("TrackIR");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a TrackIR device plugged in?";
	{
		boost::this_thread::sleep_until(boost::chrono::steady_clock::now() + boost::chrono::milliseconds(100));
	}
}

TEST(TrackIRDeviceTest, CreateDevicesWithSameName)
{
	std::shared_ptr<TrackIRDevice> device1 = std::make_shared<TrackIRDevice>("TrackIR");
	ASSERT_TRUE(device1 != nullptr) << "Device creation failed.";
	ASSERT_TRUE(device1->initialize()) << "Initialization failed.  Is a TrackIR device plugged in?";

	std::shared_ptr<TrackIRDevice> device2 = std::make_shared<TrackIRDevice>("TrackIR");
	ASSERT_TRUE(device2 != nullptr) << "Device creation failed.";
	ASSERT_FALSE(device2->initialize()) << "Initialization succeeded despite duplicate name.";
}

TEST(TrackIRDeviceTest, InputConsumer)
{
	std::shared_ptr<TrackIRDevice> device = std::make_shared<TrackIRDevice>("TrackIR");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	EXPECT_TRUE(device->initialize()) << "Initialization failed.  Is a TrackIR device plugged in?";

	std::shared_ptr<TestListener> consumer = std::make_shared<TestListener>();
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);

	EXPECT_FALSE(device->removeInputConsumer(consumer));
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);

	EXPECT_TRUE(device->addInputConsumer(consumer));

	// Adding the same input consumer again should fail.
	EXPECT_FALSE(device->addInputConsumer(consumer));

	// Sleep for one second, to see how many times the consumer is invoked.
	// (A TrackIR device is supposed to run at 120FPS/240Hz.)
	// (The thread to poll data out of TrackIR is running at 60Hz.)
	boost::this_thread::sleep_until(boost::chrono::steady_clock::now() + boost::chrono::milliseconds(1000));

	EXPECT_TRUE(device->removeInputConsumer(consumer));

	// Removing the same input consumer again should fail.
	EXPECT_FALSE(device->removeInputConsumer(consumer));

	// Check the number of invocations.
	EXPECT_GE(consumer->m_numTimesReceivedInput, 50);
	EXPECT_LE(consumer->m_numTimesReceivedInput, 100);

	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasData("pose"));
}

TEST(TrackIRDeviceTest, OutputProducer)
{
	std::shared_ptr<TrackIRDevice> device = std::make_shared<TrackIRDevice>("TrackIR");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	EXPECT_TRUE(device->initialize()) << "Initialization failed.  Is a TrackIR device plugged in?";

	std::shared_ptr<TestListener> producer = std::make_shared<TestListener>();
	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);

	EXPECT_FALSE(device->removeOutputProducer(producer));
	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);

	EXPECT_TRUE(device->setOutputProducer(producer));

	// Sleep for one second, to see how many times the producer is invoked.
	// (A TrackIR device is supposed to run at 120FPS = 240Hz.)
	// (The thread to poll data out of TrackIR is running at 60Hz.)
	boost::this_thread::sleep_until(boost::chrono::steady_clock::now() + boost::chrono::milliseconds(1000));

	EXPECT_TRUE(device->removeOutputProducer(producer));

	// Removing the same input producer again should fail.
	EXPECT_FALSE(device->removeOutputProducer(producer));

	// Check the number of invocations.
	EXPECT_GE(producer->m_numTimesRequestedOutput, 50);
	EXPECT_LE(producer->m_numTimesRequestedOutput, 100);
}
