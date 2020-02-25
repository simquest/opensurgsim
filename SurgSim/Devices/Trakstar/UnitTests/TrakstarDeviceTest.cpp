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
/// Tests for the TrakstarDevice class.

#include <memory>
#include <string>

#include <boost/thread.hpp>
#include <boost/chrono.hpp>

#include <gtest/gtest.h>

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/OptionalValue.h"
#include "SurgSim/Devices/Trakstar/TrakstarDevice.h"
#include "SurgSim/Input/InputConsumerInterface.h"
#include "SurgSim/Testing/MockInputOutput.h"

using SurgSim::Devices::TrakstarDevice;
using SurgSim::DataStructures::DataGroup;
using SurgSim::Input::InputConsumerInterface;
using SurgSim::Testing::MockInputOutput;

TEST(TrakstarDeviceTest, CreateAndInitializeDevice)
{
	std::shared_ptr<TrakstarDevice> device = std::make_shared<TrakstarDevice>("Trakstar");
	ASSERT_TRUE(nullptr != device) << "Device creation failed.";
	EXPECT_FALSE(device->isInitialized());
	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a Trakstar device plugged in?";
	EXPECT_TRUE(device->isInitialized());
}

TEST(TrakstarDeviceTest, GettersAndSetters)
{
	std::shared_ptr<TrakstarDevice> device = std::make_shared<TrakstarDevice>("Trakstar");
	ASSERT_TRUE(nullptr != device) << "Device creation failed.";

	EXPECT_NO_THROW(device->setSensorId((unsigned short)(0)));
	EXPECT_NO_THROW(device->setMeasurementRate(100.0));
	SurgSim::DataStructures::OptionalValue<double> optionalMeasurementRate;
	optionalMeasurementRate.setValue(100.0);
	EXPECT_NO_THROW(device->setOptionalMeasurementRate(optionalMeasurementRate));
	EXPECT_EQ("Trakstar", device->getName());

	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a Trakstar device plugged in?";

	EXPECT_EQ("Trakstar", device->getName());
	EXPECT_EQ(optionalMeasurementRate, device->getOptionalMeasurementRate());
	ASSERT_ANY_THROW(device->setSensorId((unsigned short)(0))) << "setSensorId should throw after initialization.";
	ASSERT_ANY_THROW(device->setMeasurementRate(80.0)) << "setMeasurementRate should throw after initialization.";
	ASSERT_ANY_THROW(device->setOptionalMeasurementRate(optionalMeasurementRate)) <<
		"setOptionalMeasurementRate should throw after initialization.";
}

TEST(TrakstarDeviceTest, CreateDevicesWithSameName)
{
	std::shared_ptr<TrakstarDevice> device1 = std::make_shared<TrakstarDevice>("Trakstar");
	ASSERT_TRUE(nullptr != device1) << "Device creation failed.";
	device1->setSensorId((unsigned short)(0));
	ASSERT_TRUE(device1->initialize()) << "Initialization failed.  Is a Trakstar device plugged in?";

	std::shared_ptr<TrakstarDevice> device2 = std::make_shared<TrakstarDevice>("Trakstar");
	ASSERT_TRUE(nullptr != device2) << "Device creation failed.";
	device2->setSensorId((unsigned short)(1));
	ASSERT_ANY_THROW(device2->initialize()) << "Initialization succeeded despite duplicate name.";
}

TEST(TrakstarDeviceTest, CreateDevicesWithSameId)
{
	std::shared_ptr<TrakstarDevice> device1 = std::make_shared<TrakstarDevice>("Trakstar");
	ASSERT_TRUE(nullptr != device1) << "Device creation failed.";
	device1->setSensorId((unsigned short)(0));
	ASSERT_TRUE(device1->initialize()) << "Initialization failed.  Is a Trakstar device plugged in?";

	std::shared_ptr<TrakstarDevice> device2 = std::make_shared<TrakstarDevice>("Trakstar2");
	ASSERT_TRUE(nullptr != device2) << "Device creation failed.";
	device2->setSensorId((unsigned short)(0));
	ASSERT_ANY_THROW(device2->initialize()) << "Initialization succeeded despite duplicate ID.";
}

TEST(TrakstarDeviceTest, CreateDevicesWithDifferentMeasurementRates)
{
	std::shared_ptr<TrakstarDevice> device1 = std::make_shared<TrakstarDevice>("Trakstar");
	ASSERT_TRUE(nullptr != device1) << "Device creation failed.";
	device1->setSensorId((unsigned short)(0));
	device1->setMeasurementRate(100.0);
	ASSERT_TRUE(device1->initialize()) << "Initialization failed.  Is a Trakstar device plugged in?";

	std::shared_ptr<TrakstarDevice> device2 = std::make_shared<TrakstarDevice>("Trakstar2");
	ASSERT_TRUE(nullptr != device2) << "Device creation failed.";
	device2->setSensorId((unsigned short)(1));
	device2->setMeasurementRate(90.0);
	ASSERT_ANY_THROW(device2->initialize()) << "Initialization succeeded despite different measurement rate.";
}

TEST(TrakstarDeviceTest, InputConsumer)
{
	std::shared_ptr<TrakstarDevice> device = std::make_shared<TrakstarDevice>("Trakstar");
	ASSERT_TRUE(nullptr != device) << "Device creation failed.";
	EXPECT_TRUE(device->initialize()) << "Initialization failed.  Is a Trakstar device plugged in?";

	std::shared_ptr<MockInputOutput> consumer = std::make_shared<MockInputOutput>();
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);
	EXPECT_TRUE(device->addInputConsumer(consumer));

	// Sleep for one second, to see how many times the consumer is invoked.
	// (Trakstar default device sample rate is 240FPS.)
	// (The thread to poll data out of Trakstar is running at default 240Hz.)
	boost::this_thread::sleep_until(boost::chrono::steady_clock::now() + boost::chrono::milliseconds(1000));

	EXPECT_TRUE(device->removeInputConsumer(consumer));

	// Check the number of invocations.
	EXPECT_GE(consumer->m_numTimesReceivedInput, 120);
	EXPECT_LE(consumer->m_numTimesReceivedInput, 360);

	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().isValid());
}
