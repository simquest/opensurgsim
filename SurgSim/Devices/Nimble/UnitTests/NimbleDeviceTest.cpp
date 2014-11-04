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
/// Tests for the NimbleDevice class.

#include <memory>
#include <string>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <gtest/gtest.h>
#include "SurgSim/Devices/Nimble/NimbleDevice.h"
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Testing/MockInputOutput.h"

using SurgSim::Device::NimbleDevice;
using SurgSim::Device::NimbleScaffold;
using SurgSim::DataStructures::DataGroup;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Matrix44d;
using SurgSim::Testing::MockInputOutput;

TEST(NimbleDeviceTest, CreateUninitializedDevice)
{
	std::shared_ptr<NimbleDevice> device = std::make_shared<NimbleDevice>("TestNimble");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
}

TEST(NimbleDeviceTest, CreateAndInitializeDevice)
{
	std::shared_ptr<NimbleDevice> device = std::make_shared<NimbleDevice>("TestNimble");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	EXPECT_FALSE(device->isInitialized());
	ASSERT_TRUE(device->initialize()) << "Initialization failed.";
	EXPECT_TRUE(device->isInitialized());
	boost::this_thread::sleep_until(boost::chrono::steady_clock::now() + boost::chrono::milliseconds(100));
}

TEST(NimbleDeviceTest, Name)
{
	std::shared_ptr<NimbleDevice> device = std::make_shared<NimbleDevice>("TestNimble");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	EXPECT_EQ("TestNimble", device->getName());
	EXPECT_TRUE(device->initialize()) << "Initialization failed.";
	EXPECT_EQ("TestNimble", device->getName());
}

TEST(NimbleDeviceTest, CreateDeviceSeveralTimes)
{
	for (int i = 0;  i < 6;  ++i)
	{
		std::shared_ptr<NimbleDevice> device = std::make_shared<NimbleDevice>("TestNimble");
		ASSERT_TRUE(device != nullptr) << "Device creation failed.";
		ASSERT_TRUE(device->initialize()) << "Initialization failed.";
		// Rapidly initializing and destructing causes the thread destructor to be called before it has been
		// stopped (by calling stop()), this sleep prevents the problem.
		boost::this_thread::sleep_until(boost::chrono::steady_clock::now() + boost::chrono::milliseconds(100));
	}
}

TEST(NimbleDeviceTest, CreateSeveralDevices)
{
	std::shared_ptr<NimbleDevice> device1 = std::make_shared<NimbleDevice>("Nimble1");
	ASSERT_TRUE(device1 != nullptr) << "Device creation failed.";
	ASSERT_TRUE(device1->initialize()) << "Initialization failed.";

	std::shared_ptr<NimbleDevice> device2 = std::make_shared<NimbleDevice>("Nimble2");
	ASSERT_TRUE(device2 != nullptr) << "Device creation failed.";
	ASSERT_TRUE(device2->initialize()) << "Initialization failed.";
}

TEST(NimbleDeviceTest, InputConsumer)
{
	std::shared_ptr<NimbleDevice> device = std::make_shared<NimbleDevice>("TestNimbleLeft");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	ASSERT_TRUE(device->initialize()) << "Initialization failed.";

	std::shared_ptr<MockInputOutput> consumer = std::make_shared<MockInputOutput>();
	EXPECT_EQ(0, consumer->m_numTimesInitializedInput);
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);

	EXPECT_FALSE(device->removeInputConsumer(consumer));
	EXPECT_EQ(0, consumer->m_numTimesInitializedInput);
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);

	EXPECT_TRUE(device->addInputConsumer(consumer));

	// Adding the same input consumer again should fail.
	EXPECT_FALSE(device->addInputConsumer(consumer));

	// Sleep for a second, to see how many times the consumer is invoked.
	// (The Nimble server updates internally at 30Hz, but our code currently runs at 1kHz.)
	boost::this_thread::sleep_until(boost::chrono::steady_clock::now() + boost::chrono::milliseconds(1000));

	EXPECT_TRUE(device->removeInputConsumer(consumer));

	// Removing the same input consumer again should fail.
	EXPECT_FALSE(device->removeInputConsumer(consumer));

	// Check the number of invocations.
	// The input is only pushed if a valid pose was detected. So, the minimum m_numTimesReceivedInput is 1.
	EXPECT_EQ(1, consumer->m_numTimesInitializedInput);
	EXPECT_GE(consumer->m_numTimesReceivedInput, 1);
	EXPECT_LE(consumer->m_numTimesReceivedInput, 30);

	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasData(SurgSim::DataStructures::Names::POSE));
	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasData("ThumbProximal"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasData("ThumbIntermediate"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasData("ThumbDistal"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasData("IndexFingerProximal"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasData("IndexFingerIntermediate"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasData("IndexFingerDistal"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasData("MiddleFingerProximal"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasData("MiddleFingerIntermediate"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasData("MiddleFingerDistal"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasData("RingFingerProximal"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasData("RingFingerIntermediate"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasData("RingFingerDistal"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasData("SmallFingerProximal"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasData("SmallFingerIntermediate"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasData("SmallFingerDistal"));
}

TEST(NimbleDeviceTest, OutputProducer)
{
	//NimbleScaffold::setDefaultLogLevel(SurgSim::Framework::LOG_LEVEL_DEBUG);
	std::shared_ptr<NimbleDevice> device = std::make_shared<NimbleDevice>("TestNimble");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	ASSERT_TRUE(device->initialize()) << "Initialization failed.";

	std::shared_ptr<MockInputOutput> producer = std::make_shared<MockInputOutput>();
	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);

	EXPECT_FALSE(device->removeOutputProducer(producer));
	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);

	EXPECT_TRUE(device->setOutputProducer(producer));

	// Sleep for a second, to see how many times the producer is invoked.
	// (A Nimble device is does not request any output.)
	boost::this_thread::sleep_until(boost::chrono::steady_clock::now() + boost::chrono::milliseconds(1000));

	EXPECT_TRUE(device->removeOutputProducer(producer));

	// Removing the same input producer again should fail.
	EXPECT_FALSE(device->removeOutputProducer(producer));

	// Check the number of invocations.
	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);
}
