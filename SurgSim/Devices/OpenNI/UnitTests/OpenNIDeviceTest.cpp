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
/// Tests for the OpenNIDevice class.

#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <gtest/gtest.h>
#include <memory>
#include <string>

#include "SurgSim/Devices/OpenNI/OpenNIDevice.h"
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Testing/MockInputOutput.h"

using SurgSim::Device::OpenNIDevice;
using SurgSim::Device::OpenNIScaffold;
using SurgSim::DataStructures::DataGroup;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Matrix44d;
using SurgSim::Testing::MockInputOutput;

TEST(OpenNIDeviceTest, CreateUninitializedDevice)
{
	std::shared_ptr<OpenNIDevice> device = std::make_shared<OpenNIDevice>("TestOpenNI");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
}

TEST(OpenNIDeviceTest, CreateAndInitializeDevice)
{
	std::shared_ptr<OpenNIDevice> device = std::make_shared<OpenNIDevice>("TestOpenNI");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	EXPECT_FALSE(device->isInitialized());
	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a OpenNI device plugged in?";
	EXPECT_TRUE(device->isInitialized());
}

TEST(OpenNIDeviceTest, Name)
{
	std::shared_ptr<OpenNIDevice> device = std::make_shared<OpenNIDevice>("TestOpenNI");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	EXPECT_EQ("TestOpenNI", device->getName());
	EXPECT_TRUE(device->initialize()) << "Initialization failed.  Is a OpenNI device plugged in?";
	EXPECT_EQ("TestOpenNI", device->getName());
}

TEST(OpenNIDeviceTest, CreateDevicesWithSameName)
{
	std::shared_ptr<OpenNIDevice> device1 = std::make_shared<OpenNIDevice>("OpenNI");
	ASSERT_TRUE(device1 != nullptr) << "Device creation failed.";
	ASSERT_TRUE(device1->initialize()) << "Initialization failed.  Is a OpenNI device plugged in?";

	std::shared_ptr<OpenNIDevice> device2 = std::make_shared<OpenNIDevice>("OpenNI");
	ASSERT_TRUE(device2 != nullptr) << "Device creation failed.";
	ASSERT_FALSE(device2->initialize()) << "Initialization succeeded despite duplicate name.";
}

TEST(OpenNIDeviceTest, InputConsumer)
{
	std::shared_ptr<OpenNIDevice> device = std::make_shared<OpenNIDevice>("TestOpenNI");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a OpenNI device plugged in?";

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
	EXPECT_GE(consumer->m_numTimesReceivedInput, 10);
	EXPECT_LE(consumer->m_numTimesReceivedInput, 120);

	EXPECT_TRUE(consumer->m_lastReceivedInput.images().hasData("color"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.images().hasData("depth"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.images().hasData("depth_xyz"));
}

TEST(OpenNIDeviceTest, OutputProducer)
{
	std::shared_ptr<OpenNIDevice> device = std::make_shared<OpenNIDevice>("TestOpenNI");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a OpenNI device plugged in?";

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
