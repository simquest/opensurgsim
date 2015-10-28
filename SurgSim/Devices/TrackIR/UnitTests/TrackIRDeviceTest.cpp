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
#include "SurgSim/Testing/MockInputOutput.h"

using SurgSim::Devices::TrackIRDevice;
using SurgSim::DataStructures::DataGroup;
using SurgSim::Input::InputConsumerInterface;
using SurgSim::Testing::MockInputOutput;

TEST(TrackIRDeviceTest, CreateAndInitializeDevice)
{
	std::shared_ptr<TrackIRDevice> device = std::make_shared<TrackIRDevice>("TrackIR");
	ASSERT_TRUE(nullptr != device) << "Device creation failed.";

	EXPECT_FALSE(device->isInitialized());
	EXPECT_EQ("TrackIR", device->getName());

	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a TrackIR device plugged in?";

	EXPECT_TRUE(device->isInitialized());
	EXPECT_EQ("TrackIR", device->getName());
}

TEST(TrackIRDeviceTest, CreateDevicesWithSameName)
{
	std::shared_ptr<TrackIRDevice> device1 = std::make_shared<TrackIRDevice>("TrackIR");
	ASSERT_TRUE(nullptr != device1) << "Device creation failed.";
	ASSERT_TRUE(device1->initialize()) << "Initialization failed.  Is a TrackIR device plugged in?";

	std::shared_ptr<TrackIRDevice> device2 = std::make_shared<TrackIRDevice>("TrackIR");
	ASSERT_TRUE(nullptr != device2) << "Device creation failed.";
	ASSERT_ANY_THROW(device2->initialize()) << "Initialization succeeded despite duplicate name.";
}

TEST(TrackIRDeviceTest, RegisterMoreThanOneDevice)
{
	std::shared_ptr<TrackIRDevice> device1 = std::make_shared<TrackIRDevice>("TrackIR");
	ASSERT_TRUE(nullptr != device1) << "Device creation failed.";
	ASSERT_TRUE(device1->initialize()) << "Initialization failed.  Is a TrackIR device plugged in?";

	std::shared_ptr<TrackIRDevice> device2 = std::make_shared<TrackIRDevice>("TrackIR2");
	ASSERT_TRUE(nullptr != device2) << "Device creation failed.";
	ASSERT_ANY_THROW(device2->initialize()) << "Two TrackIR cameras are registered.";
}

TEST(TrackIRDeviceTest, InputConsumer)
{
	std::shared_ptr<TrackIRDevice> device = std::make_shared<TrackIRDevice>("TrackIR");
	ASSERT_TRUE(nullptr != device) << "Device creation failed.";
	EXPECT_TRUE(device->initialize()) << "Initialization failed.  Is a TrackIR device plugged in?";

	std::shared_ptr<MockInputOutput> consumer = std::make_shared<MockInputOutput>();
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);
	EXPECT_TRUE(device->addInputConsumer(consumer));

	// Sleep for one second, to see how many times the consumer is invoked.
	// (TrackIR device sample rate is 120FPS.)
	// (The thread to poll data out of TrackIR is running at default 100Hz.)
	boost::this_thread::sleep_until(boost::chrono::steady_clock::now() + boost::chrono::milliseconds(1000));

	EXPECT_TRUE(device->removeInputConsumer(consumer));

	// Check the number of invocations.
	EXPECT_GE(consumer->m_numTimesReceivedInput, 50);
	EXPECT_LE(consumer->m_numTimesReceivedInput, 120);

	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().isValid());
}
