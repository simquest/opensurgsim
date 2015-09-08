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
/// Tests for the RawMultiAxisDevice class.

#include <memory>
#include <string>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <gtest/gtest.h>
#include "SurgSim/Devices/MultiAxis/RawMultiAxisDevice.h"
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Testing/MockInputOutput.h"

using SurgSim::Devices::RawMultiAxisDevice;
using SurgSim::Devices::RawMultiAxisScaffold;
using SurgSim::DataStructures::DataGroup;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Matrix44d;
using SurgSim::Testing::MockInputOutput;

TEST(RawMultiAxisDeviceTest, CreateUninitializedDevice)
{
	std::shared_ptr<RawMultiAxisDevice> device = std::make_shared<RawMultiAxisDevice>("TestRawMultiAxis");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
}

TEST(RawMultiAxisDeviceTest, CreateAndInitializeDevice)
{
	std::shared_ptr<RawMultiAxisDevice> device = std::make_shared<RawMultiAxisDevice>("TestRawMultiAxis");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	EXPECT_FALSE(device->isInitialized());
	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a RawMultiAxis device plugged in?";
	EXPECT_TRUE(device->isInitialized());
}

TEST(RawMultiAxisDeviceTest, Name)
{
	std::shared_ptr<RawMultiAxisDevice> device = std::make_shared<RawMultiAxisDevice>("TestRawMultiAxis");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	EXPECT_EQ("TestRawMultiAxis", device->getName());
	EXPECT_TRUE(device->initialize()) << "Initialization failed.  Is a RawMultiAxis device plugged in?";
	EXPECT_EQ("TestRawMultiAxis", device->getName());
}

TEST(RawMultiAxisDeviceTest, Factory)
{
	std::shared_ptr<SurgSim::Input::DeviceInterface> device;
	ASSERT_NO_THROW(device = SurgSim::Input::DeviceInterface::getFactory().create(
								 "SurgSim::Devices::RawMultiAxisDevice", "Device"));
	EXPECT_NE(nullptr, device);
}


static void testCreateDeviceSeveralTimes(bool doSleep)
{
	for (int i = 0;  i < 6;  ++i)
	{
		std::shared_ptr<RawMultiAxisDevice> device = std::make_shared<RawMultiAxisDevice>("TestRawMultiAxis");
		ASSERT_TRUE(device != nullptr) << "Device creation failed.";
		ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a RawMultiAxis device plugged in?";
		if (doSleep)
		{
			boost::this_thread::sleep_until(boost::chrono::steady_clock::now() + boost::chrono::milliseconds(100));
		}
		// the device will be destroyed here
	}
}

TEST(RawMultiAxisDeviceTest, CreateDeviceSeveralTimes)
{
	testCreateDeviceSeveralTimes(true);
}

TEST(RawMultiAxisDeviceTest, CreateSeveralDevices)
{
	std::shared_ptr<RawMultiAxisDevice> device1 = std::make_shared<RawMultiAxisDevice>("RawMultiAxis1");
	ASSERT_TRUE(device1 != nullptr) << "Device creation failed.";
	ASSERT_TRUE(device1->initialize()) << "Initialization failed.  Is a RawMultiAxis device plugged in?";

	// We can't check what happens with the scaffolds, since those are no longer a part of the device's API...

	std::shared_ptr<RawMultiAxisDevice> device2 = std::make_shared<RawMultiAxisDevice>("RawMultiAxis2");
	ASSERT_TRUE(device2 != nullptr) << "Device creation failed.";
	if (! device2->initialize())
	{
		std::cerr << "[Warning: second RawMultiAxis controller did not come up; is it plugged in?]" << std::endl;
	}
}

TEST(RawMultiAxisDeviceTest, CreateDevicesWithSameName)
{
	std::shared_ptr<RawMultiAxisDevice> device1 = std::make_shared<RawMultiAxisDevice>("RawMultiAxis");
	ASSERT_TRUE(device1 != nullptr) << "Device creation failed.";
	ASSERT_TRUE(device1->initialize()) << "Initialization failed.  Is a RawMultiAxis device plugged in?";

	std::shared_ptr<RawMultiAxisDevice> device2 = std::make_shared<RawMultiAxisDevice>("RawMultiAxis");
	ASSERT_TRUE(device2 != nullptr) << "Device creation failed.";
	ASSERT_FALSE(device2->initialize()) << "Initialization succeeded despite duplicate name.";
}

// Create a string representation from an int.
// C++11 adds std::to_string() to do this for various types, but VS2010 only half-supports that.
template <typename T>
inline std::string makeString(T value)
{
	std::ostringstream out;
	out << value;
	return out.str();
}

TEST(RawMultiAxisDeviceTest, CreateAllDevices)
{
	std::vector<std::shared_ptr<RawMultiAxisDevice>> devices;

	for (int i = 1;  ;  ++i)
	{
		std::string name = "RawMultiAxis" + makeString(i);
		std::shared_ptr<RawMultiAxisDevice> device = std::make_shared<RawMultiAxisDevice>(name);
		ASSERT_TRUE(device != nullptr) << "Device creation failed.";
		if (! device->initialize())
		{
			break;
		}
		devices.emplace_back(std::move(device));
	}

	std::cout << devices.size() << " devices initialized." << std::endl;
	ASSERT_GT(devices.size(), 0U) << "Initialization failed.  Is a RawMultiAxis device plugged in?";
}

TEST(RawMultiAxisDeviceTest, InputConsumer)
{
	std::shared_ptr<RawMultiAxisDevice> device = std::make_shared<RawMultiAxisDevice>("TestRawMultiAxis");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a RawMultiAxis device plugged in?";

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
	// (A RawMultiAxis device updates internally at 60Hz, but our code currently runs at 100Hz to reduce latency.)
	boost::this_thread::sleep_until(boost::chrono::steady_clock::now() + boost::chrono::milliseconds(1000));

	EXPECT_TRUE(device->removeInputConsumer(consumer));

	// Removing the same input consumer again should fail.
	EXPECT_FALSE(device->removeInputConsumer(consumer));

	// Check the number of invocations.
	EXPECT_EQ(1, consumer->m_numTimesInitializedInput);
	EXPECT_GE(consumer->m_numTimesReceivedInput, 90);
	EXPECT_LE(consumer->m_numTimesReceivedInput, 110);

	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasData(SurgSim::DataStructures::Names::POSE));
	EXPECT_TRUE(consumer->m_lastReceivedInput.booleans().hasData(SurgSim::DataStructures::Names::BUTTON_1));
	EXPECT_TRUE(consumer->m_lastReceivedInput.booleans().hasData(SurgSim::DataStructures::Names::BUTTON_2));
	EXPECT_TRUE(consumer->m_lastReceivedInput.booleans().hasData(SurgSim::DataStructures::Names::BUTTON_3));
	EXPECT_TRUE(consumer->m_lastReceivedInput.booleans().hasData(SurgSim::DataStructures::Names::BUTTON_4));
}

TEST(RawMultiAxisDeviceTest, OutputProducer)
{
	std::shared_ptr<RawMultiAxisDevice> device = std::make_shared<RawMultiAxisDevice>("TestRawMultiAxis");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a RawMultiAxis device plugged in?";

	std::shared_ptr<MockInputOutput> producer = std::make_shared<MockInputOutput>();
	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);

	EXPECT_FALSE(device->removeOutputProducer(producer));
	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);

	EXPECT_TRUE(device->setOutputProducer(producer));

	// Sleep for a second, to see how many times the producer is invoked.
	// (A RawMultiAxis device is does not request any output.)
	boost::this_thread::sleep_until(boost::chrono::steady_clock::now() + boost::chrono::milliseconds(1000));

	EXPECT_TRUE(device->removeOutputProducer(producer));

	// Removing the same input producer again should fail.
	EXPECT_FALSE(device->removeOutputProducer(producer));

	// Check the number of invocations.
	EXPECT_GE(producer->m_numTimesRequestedOutput, 90);
	EXPECT_LE(producer->m_numTimesRequestedOutput, 110);
}
