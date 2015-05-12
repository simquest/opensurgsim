// This file is a part of the OpenSurgSim project.
// Copyright 2015, SimQuest Solutions Inc.
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

#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <gtest/gtest.h>
#include <memory>

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Devices/Oculus/OculusDevice.h"
#include "SurgSim/Testing/MockInputOutput.h"

using SurgSim::Device::OculusDevice;
using SurgSim::DataStructures::DataGroup;
using SurgSim::Input::InputConsumerInterface;
using SurgSim::Testing::MockInputOutput;

TEST(OculusDeviceTest, CreateAndInitializeDevice)
{
	auto device = std::make_shared<OculusDevice>("Oculus");
	ASSERT_TRUE(nullptr != device) << "Device creation failed.";

	EXPECT_FALSE(device->isInitialized());
	EXPECT_EQ("Oculus", device->getName());

	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is an Oculus device plugged in?";

	EXPECT_TRUE(device->isInitialized());
	EXPECT_EQ("Oculus", device->getName());
}

TEST(OculusDeviceTest, Factory)
{
	std::shared_ptr<SurgSim::Input::DeviceInterface> device;
	ASSERT_NO_THROW(device = SurgSim::Input::DeviceInterface::getFactory().create(
								 "SurgSim::Device::OculusDevice", "Device"));
	EXPECT_NE(nullptr, device);
}

TEST(OculusDeviceTest, FinalizeDevice)
{
	auto device = std::make_shared<OculusDevice>("Oculus");
	ASSERT_TRUE(nullptr != device) << "Device creation failed.";

	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is an Oculus device plugged in?";
	EXPECT_TRUE(device->isInitialized());
	EXPECT_EQ("Oculus", device->getName());

	ASSERT_TRUE(device->finalize()) << "Finalization failed.";
	EXPECT_FALSE(device->isInitialized());
	EXPECT_EQ("Oculus", device->getName());
}

TEST(OculusDeviceTest, RegisterMoreThanOneDevice)
{
	auto device1 = std::make_shared<OculusDevice>("Oculus");
	ASSERT_TRUE(nullptr != device1) << "Device creation failed.";
	ASSERT_TRUE(device1->initialize()) << "Initialization failed.  Is an Oculus device plugged in?";

	auto device2 = std::make_shared<OculusDevice>("Oculus2");
	ASSERT_TRUE(nullptr != device2) << "Device creation failed.";
	EXPECT_THROW(device2->initialize(), SurgSim::Framework::AssertionFailure);
}

TEST(OculusDeviceTest, RegisterAndUnregisterDevice)
{
	auto device = std::make_shared<OculusDevice>("Oculus");
	ASSERT_NE(nullptr, device) << "Device creation failed.";
	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is an Oculus device plugged in?";
	ASSERT_TRUE(device->finalize()) << "Finalization failed.";
}

TEST(OculusDeviceTest, InputConsumer)
{
	auto device = std::make_shared<OculusDevice>("Oculus");
	ASSERT_TRUE(nullptr != device) << "Device creation failed.";
	EXPECT_TRUE(device->initialize()) << "Initialization failed.  Is an Oculus device plugged in?";

	auto consumer = std::make_shared<MockInputOutput>();
	EXPECT_EQ(0, consumer->m_numTimesInitializedInput);
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);

	EXPECT_FALSE(device->removeInputConsumer(consumer));
	EXPECT_EQ(0, consumer->m_numTimesInitializedInput);
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);

	EXPECT_TRUE(device->addInputConsumer(consumer));

	// Adding the same input consumer again should fail.
	EXPECT_FALSE(device->addInputConsumer(consumer));

	// Sleep for a second, to see how many times the consumer is invoked.
	boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));

	EXPECT_TRUE(device->removeInputConsumer(consumer));
	// Removing the same input consumer again should fail.
	EXPECT_FALSE(device->removeInputConsumer(consumer));

	// Check the number of invocations.
	EXPECT_EQ(1, consumer->m_numTimesInitializedInput);
	EXPECT_GE(consumer->m_numTimesReceivedInput, 800);
	EXPECT_LE(consumer->m_numTimesReceivedInput, 1200);

	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasData("pose"));
}
