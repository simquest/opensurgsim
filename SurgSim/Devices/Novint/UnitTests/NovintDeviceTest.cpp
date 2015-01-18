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
/// Tests for the NovintDevice class.

#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <gtest/gtest.h>
#include <memory>
#include <string>

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Devices/Novint/NovintDevice.h"
#include "SurgSim/Framework/Clock.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Testing/MockInputOutput.h"

using SurgSim::Device::NovintDevice;
using SurgSim::DataStructures::DataGroup;
using SurgSim::Framework::Clock;
using SurgSim::Math::Matrix44d;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Testing::MockInputOutput;

// Define common device names used in the Novint device tests.
// These initialization names should be set to two of the names used in devices.yaml (or be empty strings to just get
// the first device). The serial number should be a serial number for one of the attached Falcons (or be an empty
// string to just get the first device.)
const char* const NOVINT_TEST_DEVICE_NAME = "FALCON_1";
const char* const NOVINT_TEST_DEVICE_NAME_2 = "FALCON_2";
const char* const NOVINT_TEST_DEVICE_SERIAL_NUMBER = "14QAVEFF";

TEST(NovintDeviceTest, CreateAndInitializeDeviceByName)
{
	std::shared_ptr<NovintDevice> device = std::make_shared<NovintDevice>("TestFalcon");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	
	std::string initializationName = "";
	EXPECT_FALSE(device->getInitializationName(&initializationName));
	device->setInitializationName(NOVINT_TEST_DEVICE_NAME);
	ASSERT_TRUE(device->getInitializationName(&initializationName));
	EXPECT_EQ(NOVINT_TEST_DEVICE_NAME, initializationName);

	std::string serialNumber;
	EXPECT_FALSE(device->getSerialNumber(&serialNumber));
	EXPECT_ANY_THROW(device->setSerialNumber(""));

	EXPECT_FALSE(device->isInitialized());
	EXPECT_EQ("TestFalcon", device->getName());

	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a Novint device plugged in?";
	EXPECT_TRUE(device->isInitialized());
	EXPECT_EQ("TestFalcon", device->getName());

	const double positionScale = 2.0;
	device->setPositionScale(positionScale);
	EXPECT_EQ(positionScale, device->getPositionScale());

	const double orientationScale = 2.0;
	device->setOrientationScale(orientationScale);
	EXPECT_EQ(orientationScale, device->getOrientationScale());

	EXPECT_TRUE(device->finalize());
}
TEST(NovintDeviceTest, CreateAndInitializeDeviceBySerialNumber)
{
	std::shared_ptr<NovintDevice> device = std::make_shared<NovintDevice>("TestFalcon");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";

	std::string serialNumber = "";
	EXPECT_FALSE(device->getSerialNumber(&serialNumber));
	device->setSerialNumber(NOVINT_TEST_DEVICE_SERIAL_NUMBER);
	ASSERT_TRUE(device->getSerialNumber(&serialNumber));
	EXPECT_EQ(NOVINT_TEST_DEVICE_SERIAL_NUMBER, serialNumber);

	std::string initializationName;
	EXPECT_FALSE(device->getInitializationName(&initializationName));
	EXPECT_ANY_THROW(device->setInitializationName(""));

	EXPECT_FALSE(device->isInitialized());
	EXPECT_EQ("TestFalcon", device->getName());

	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a Novint device plugged in?";
	EXPECT_TRUE(device->isInitialized());
	EXPECT_EQ("TestFalcon", device->getName());

	const double positionScale = 2.0;
	device->setPositionScale(positionScale);
	EXPECT_EQ(positionScale, device->getPositionScale());

	const double orientationScale = 2.0;
	device->setOrientationScale(orientationScale);
	EXPECT_EQ(orientationScale, device->getOrientationScale());

	EXPECT_TRUE(device->finalize());
}

TEST(NovintDeviceTest, CreateAndInitializeDefaultDevices)
{
	std::shared_ptr<NovintDevice> device = std::make_shared<NovintDevice>("TestFalcon");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	EXPECT_FALSE(device->isInitialized());
	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a Novint device plugged in?";
	EXPECT_TRUE(device->isInitialized());

	std::string initializationName;
	EXPECT_FALSE(device->getInitializationName(&initializationName));
	std::string serialNumber;
	EXPECT_FALSE(device->getSerialNumber(&serialNumber));

	std::shared_ptr<NovintDevice> device2 = std::make_shared<NovintDevice>("TestFalcon2");
	ASSERT_TRUE(device2 != nullptr) << "Device creation failed.";
	EXPECT_FALSE(device2->isInitialized());
	if (!device2->initialize())
	{
		std::cerr << "[Warning: second Novint did not come up; is it plugged in?]" << std::endl;
	}
}

static void testCreateDeviceSeveralTimes(bool doSleep)
{
	for (int i = 0;  i < 6;  ++i)
	{
		std::shared_ptr<NovintDevice> device = std::make_shared<NovintDevice>("TestFalcon");
		ASSERT_TRUE(device != nullptr) << "Device creation failed.";
		ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a Novint device plugged in?";
		if (doSleep)
		{
			boost::this_thread::sleep_until(Clock::now() + boost::chrono::milliseconds(100));
		}
		// the device will be destroyed here
	}
}

TEST(NovintDeviceTest, CreateDeviceSeveralTimes)
{
	testCreateDeviceSeveralTimes(true);
}

TEST(NovintDeviceTest, CreateTwoDevices)
{
	std::shared_ptr<NovintDevice> device1 = std::make_shared<NovintDevice>("Novint1");
	ASSERT_TRUE(device1 != nullptr) << "Device creation failed.";
	device1->setInitializationName(NOVINT_TEST_DEVICE_NAME);
	ASSERT_TRUE(device1->initialize()) << "Initialization failed.  Is a Novint device plugged in?";

	std::shared_ptr<NovintDevice> device2 = std::make_shared<NovintDevice>("Novint2");
	ASSERT_TRUE(device2 != nullptr) << "Device creation failed.";
	device2->setInitializationName(NOVINT_TEST_DEVICE_NAME_2);
	if (!device2->initialize())
	{
		std::cerr << "[Warning: second Novint did not come up; is it plugged in?]" << std::endl;
	}
}

TEST(NovintDeviceTest, CreateDevicesWithSameInitializationName)
{
	if (NOVINT_TEST_DEVICE_NAME != "")
	{
		std::shared_ptr<NovintDevice> device1 = std::make_shared<NovintDevice>("Novint1");
		ASSERT_TRUE(device1 != nullptr) << "Device creation failed.";
		device1->setInitializationName(NOVINT_TEST_DEVICE_NAME);
		ASSERT_TRUE(device1->initialize()) << "Initialization failed.  Is a Novint device plugged in?";

		std::shared_ptr<NovintDevice> device2 = std::make_shared<NovintDevice>("Novint2");
		ASSERT_TRUE(device2 != nullptr) << "Device creation failed.";
		device2->setInitializationName(NOVINT_TEST_DEVICE_NAME);
		ASSERT_FALSE(device2->initialize()) << "Initialization succeeded despite duplicate initialization name.";
	}
}

TEST(NovintDeviceTest, CreateDevicesWithSameSerialNumber)
{
	if (NOVINT_TEST_DEVICE_SERIAL_NUMBER != "")
	{
		std::shared_ptr<NovintDevice> device1 = std::make_shared<NovintDevice>("Novint1");
		ASSERT_TRUE(device1 != nullptr) << "Device creation failed.";
		device1->setSerialNumber(NOVINT_TEST_DEVICE_SERIAL_NUMBER);
		ASSERT_TRUE(device1->initialize()) << "Initialization failed.  Is a Novint device plugged in?";

		std::shared_ptr<NovintDevice> device2 = std::make_shared<NovintDevice>("Novint2");
		ASSERT_TRUE(device2 != nullptr) << "Device creation failed.";
		device2->setSerialNumber(NOVINT_TEST_DEVICE_SERIAL_NUMBER);
		ASSERT_FALSE(device2->initialize()) << "Initialization succeeded despite duplicate initialization name.";
	}
}

TEST(NovintDeviceTest, InputConsumer)
{
	std::shared_ptr<NovintDevice> device = std::make_shared<NovintDevice>("TestFalcon");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	device->setInitializationName(NOVINT_TEST_DEVICE_NAME);
	EXPECT_TRUE(device->initialize()) << "Initialization failed.  Is a Novint device plugged in?";

	std::shared_ptr<MockInputOutput> consumer = std::make_shared<MockInputOutput>();
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);

	EXPECT_FALSE(device->removeInputConsumer(consumer));
	EXPECT_EQ(0, consumer->m_numTimesReceivedInput);

	EXPECT_TRUE(device->addInputConsumer(consumer));

	// Adding the same input consumer again should fail.
	EXPECT_FALSE(device->addInputConsumer(consumer));

	// Sleep for a second, to see how many times the consumer is invoked.
	// (A Novint device is supposed to run at 1KHz.)
	boost::this_thread::sleep_until(Clock::now() + boost::chrono::milliseconds(10000));

	EXPECT_TRUE(device->removeInputConsumer(consumer));

	// Removing the same input consumer again should fail.
	EXPECT_FALSE(device->removeInputConsumer(consumer));

	// Check the number of invocations.
	EXPECT_GE(consumer->m_numTimesReceivedInput, 10*700);
	EXPECT_LE(consumer->m_numTimesReceivedInput, 10*1300);

	EXPECT_TRUE(consumer->m_lastReceivedInput.poses().hasData(SurgSim::DataStructures::Names::POSE));
	EXPECT_TRUE(consumer->m_lastReceivedInput.booleans().hasData(SurgSim::DataStructures::Names::BUTTON_1));
}

TEST(NovintDeviceTest, OutputProducer)
{
	std::shared_ptr<NovintDevice> device = std::make_shared<NovintDevice>("TestFalcon");
	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	device->setInitializationName(NOVINT_TEST_DEVICE_NAME);
	EXPECT_TRUE(device->initialize()) << "Initialization failed.  Is a Novint device plugged in?";

	std::shared_ptr<MockInputOutput> producer = std::make_shared<MockInputOutput>();
	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);

	EXPECT_FALSE(device->removeOutputProducer(producer));
	EXPECT_EQ(0, producer->m_numTimesRequestedOutput);

	EXPECT_TRUE(device->setOutputProducer(producer));

	// Sleep for a second, to see how many times the producer is invoked.
	// (A Novint Falcon device is supposed to run at 1KHz.)
	boost::this_thread::sleep_until(Clock::now() + boost::chrono::milliseconds(10000));

	EXPECT_TRUE(device->removeOutputProducer(producer));

	// Removing the same input producer again should fail.
	EXPECT_FALSE(device->removeOutputProducer(producer));

	// Check the number of invocations.
	EXPECT_GE(producer->m_numTimesRequestedOutput, 10*700);
	EXPECT_LE(producer->m_numTimesRequestedOutput, 10*1300);
}
