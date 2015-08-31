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
/// Tests for the LeapDevice class.

#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <gtest/gtest.h>
#include <memory>
#include <string>

#include "SurgSim/Devices/Leap/LeapDevice.h"
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Testing/MockInputOutput.h"

using SurgSim::Devices::LeapDevice;
using SurgSim::DataStructures::DataGroup;
using SurgSim::Testing::MockInputOutput;


TEST(LeapDeviceTest, CreateUninitializedDevice)
{
	std::shared_ptr<LeapDevice> device;
	ASSERT_NO_THROW({ device = std::make_shared<LeapDevice>("TestLeap"); });
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
}

TEST(LeapDeviceTest, CreateAndInitializeDevice)
{
	std::shared_ptr<LeapDevice> device = std::make_shared<LeapDevice>("TestLeap");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	EXPECT_FALSE(device->isInitialized());
	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a Leap device plugged in?";
	EXPECT_TRUE(device->isInitialized());
}

TEST(LeapDeviceTest, Name)
{
	std::shared_ptr<LeapDevice> device = std::make_shared<LeapDevice>("TestLeap");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	EXPECT_EQ("TestLeap", device->getName());
	EXPECT_TRUE(device->initialize()) << "Initialization failed.  Is a Leap device plugged in?";
	EXPECT_EQ("TestLeap", device->getName());
}

TEST(LeapDeviceTest, HandType)
{
	std::shared_ptr<LeapDevice> device = std::make_shared<LeapDevice>("TestLeap");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";

	EXPECT_EQ(SurgSim::Devices::HANDTYPE_RIGHT, device->getHandType());

	device->setHandType(SurgSim::Devices::HANDTYPE_LEFT);
	EXPECT_EQ(SurgSim::Devices::HANDTYPE_LEFT, device->getHandType());

	device->setHandType(SurgSim::Devices::HANDTYPE_RIGHT);
	EXPECT_EQ(SurgSim::Devices::HANDTYPE_RIGHT, device->getHandType());
}

TEST(LeapDeviceTest, TrackingMode)
{
	{
		std::shared_ptr<LeapDevice> device = std::make_shared<LeapDevice>("TestLeap");
		EXPECT_THROW(device->getTrackingMode(), SurgSim::Framework::AssertionFailure)
			<< "TrackingMode not previously set, nor device initialized, should not be able to determine tracking mode";
	}
	{
		std::shared_ptr<LeapDevice> device = std::make_shared<LeapDevice>("TestLeap");
		ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a Leap device plugged in?";
		EXPECT_EQ(SurgSim::Devices::LEAP_TRACKING_MODE_DESKTOP, device->getTrackingMode())
			<< "Default tracking mode should be LEAP_TRACKING_MODE_DESKTOP.";
	}
	{
		std::shared_ptr<LeapDevice> device = std::make_shared<LeapDevice>("TestLeap");
		device->setTrackingMode(SurgSim::Devices::LEAP_TRACKING_MODE_HMD);
		EXPECT_EQ(SurgSim::Devices::LEAP_TRACKING_MODE_HMD, device->getTrackingMode());
		ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a Leap device plugged in?";
		boost::this_thread::sleep_until(boost::chrono::steady_clock::now() + boost::chrono::milliseconds(100));
		EXPECT_EQ(SurgSim::Devices::LEAP_TRACKING_MODE_HMD, device->getTrackingMode())
			<< "HMD Tracking Mode not set. This could be do to user settings in the LeapControlPanel." << std::endl
			<< "Disable 'Auto-orient Tracking' in Settings>>Tracking.";
	}
}

TEST(LeapDeviceTest, ProvidingImages)
{
	std::shared_ptr<LeapDevice> device = std::make_shared<LeapDevice>("TestLeap");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";

	EXPECT_FALSE(device->isProvidingImages());

	device->setProvideImages(true);
	EXPECT_TRUE(device->isProvidingImages());

	EXPECT_FALSE(device->isInitialized());
	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a Leap device plugged in?";
	EXPECT_TRUE(device->isInitialized());

	EXPECT_THROW(device->setProvideImages(true), SurgSim::Framework::AssertionFailure);
}

TEST(LeapDeviceTest, CreateDevicesWithSameName)
{
	std::shared_ptr<LeapDevice> device1 = std::make_shared<LeapDevice>("TestLeap");
	ASSERT_TRUE(device1 != nullptr) << "Device creation failed.";
	ASSERT_TRUE(device1->initialize()) << "Initialization failed.  Is a Leap device plugged in?";

	std::shared_ptr<LeapDevice> device2 = std::make_shared<LeapDevice>("TestLeap");
	ASSERT_TRUE(device2 != nullptr) << "Device creation failed.";
	ASSERT_FALSE(device2->initialize()) << "Initialization succeeded despite duplicate name.";
}

TEST(LeapDeviceTest, InputConsumer)
{
	std::shared_ptr<LeapDevice> device = std::make_shared<LeapDevice>("TestLeap");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a Leap device plugged in?";

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
	boost::this_thread::sleep_until(boost::chrono::steady_clock::now() + boost::chrono::milliseconds(1000));

	EXPECT_TRUE(device->removeInputConsumer(consumer));
	// Removing the same input consumer again should fail.
	EXPECT_FALSE(device->removeInputConsumer(consumer));

	// Check the number of invocations.
	EXPECT_EQ(1, consumer->m_numTimesInitializedInput);
	EXPECT_GE(consumer->m_numTimesReceivedInput, 5);
	EXPECT_LE(consumer->m_numTimesReceivedInput, 120);

	EXPECT_TRUE(consumer->m_lastReceivedInput.images().hasEntry("left"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.images().hasEntry("right"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasEntry("pose"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasEntry("ThumbProximal"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasEntry("ThumbIntermediate"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasEntry("ThumbDistal"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasEntry("IndexFingerProximal"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasEntry("IndexFingerIntermediate"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasEntry("IndexFingerDistal"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasEntry("MiddleFingerProximal"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasEntry("MiddleFingerIntermediate"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasEntry("MiddleFingerDistal"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasEntry("RingFingerProximal"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasEntry("RingFingerIntermediate"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasEntry("RingFingerDistal"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasEntry("SmallFingerProximal"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasEntry("SmallFingerIntermediate"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasEntry("SmallFingerDistal"));
}

TEST(LeapDeviceTest, OutputProducer)
{
	std::shared_ptr<LeapDevice> device = std::make_shared<LeapDevice>("TestLeap");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a Leap device plugged in?";

	std::shared_ptr<MockInputOutput> producer = std::make_shared<MockInputOutput>();
	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);

	EXPECT_FALSE(device->removeOutputProducer(producer));
	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);

	EXPECT_TRUE(device->setOutputProducer(producer));

	// Sleep for a second, to see how many times the producer is invoked.
	boost::this_thread::sleep_until(boost::chrono::steady_clock::now() + boost::chrono::milliseconds(1000));

	EXPECT_TRUE(device->removeOutputProducer(producer));

	// Removing the same input producer again should fail.
	EXPECT_FALSE(device->removeOutputProducer(producer));

	// Check the number of invocations.
	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);
}
