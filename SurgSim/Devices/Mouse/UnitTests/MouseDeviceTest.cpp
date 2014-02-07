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
/// Tests for the MouseDevice class.
#include <gtest/gtest.h>

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Devices/Mouse/MouseDevice.h"
#include "SurgSim/Devices/Mouse/MouseScaffold.h"
#include "SurgSim/Input/InputConsumerInterface.h"

namespace SurgSim
{
namespace Device
{

using SurgSim::Device::MouseDevice;
using SurgSim::DataStructures::DataGroup;

struct TestListener : public SurgSim::Input::InputConsumerInterface
{
	virtual void initializeInput(const std::string& device, const DataGroup& inputData) override {}
	virtual void handleInput(const std::string& device, const DataGroup& inputData) override
	{
		m_lastReceivedInput = inputData;
	}

	DataGroup m_lastReceivedInput;
};

class MouseDeviceTest : public ::testing::Test
{
public:
	static void update(std::shared_ptr<MouseDevice> device)
	{
		device->m_scaffold->updateDevice(0, 0, 0, 0, 0);
	}
};


TEST_F(MouseDeviceTest, CreateInitializeAndDestroyDevice)
{
	std::shared_ptr<MouseDevice> device = std::make_shared<MouseDevice>("TestMouse");

	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	EXPECT_FALSE(device->isInitialized());

	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a Mouse device plugged in?";
	EXPECT_TRUE(device->isInitialized());

	ASSERT_TRUE(device->finalize()) << "Device finalization failed";
	EXPECT_FALSE(device->isInitialized());

	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a Mouse device plugged in?";
	EXPECT_TRUE(device->isInitialized());

	ASSERT_TRUE(device->finalize()) << "Device finalization failed";
	EXPECT_FALSE(device->isInitialized());
}

TEST_F(MouseDeviceTest, InputConsumer)
{
	std::shared_ptr<MouseDevice> device = std::make_shared<MouseDevice>("TestMouse");
	device->initialize();

	std::shared_ptr<TestListener> consumer = std::make_shared<TestListener>();
	EXPECT_TRUE(device->addInputConsumer(consumer));

	MouseDeviceTest::update(device);

	EXPECT_TRUE(consumer->m_lastReceivedInput.booleans().hasData("button1"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.booleans().hasData("button2"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.booleans().hasData("button3"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.scalars().hasData("x"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.scalars().hasData("y"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.integers().hasData("scrollDeltaX"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.integers().hasData("scrollDeltaY"));

	device->finalize();
}


};  // namespace Device
};  // namespace SurgSim
