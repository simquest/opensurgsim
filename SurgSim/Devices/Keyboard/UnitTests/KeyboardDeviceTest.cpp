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
/// Tests for the KeyboardDevice class.
#include <gtest/gtest.h>

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Devices/Keyboard/KeyboardDevice.h"
#include "SurgSim/Devices/Keyboard/KeyboardScaffold.h"
#include "SurgSim/Input/InputConsumerInterface.h"
#include "SurgSim/Testing/MockInputOutput.h"

namespace SurgSim
{
namespace Devices
{

using SurgSim::Devices::KeyboardDevice;
using SurgSim::DataStructures::DataGroup;
using SurgSim::Testing::MockInputOutput;

class KeyboardDeviceTest : public ::testing::Test
{
public:
	static void update(std::shared_ptr<KeyboardDevice> device)
	{
		device->m_scaffold->updateDevice(0, 0);
	}
};


TEST_F(KeyboardDeviceTest, InitializeDevice)
{
	std::shared_ptr<KeyboardDevice> device = std::make_shared<KeyboardDevice>("TestKeyboard");

	ASSERT_TRUE(device != nullptr) << "Device creation failed.";
	EXPECT_FALSE(device->isInitialized());

	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is a Keyboard device plugged in?";
	EXPECT_TRUE(device->isInitialized());
}

TEST_F(KeyboardDeviceTest, InputConsumer)
{
	std::shared_ptr<KeyboardDevice> device = std::make_shared<KeyboardDevice>("TestKeyboard");
	device->initialize();

	std::shared_ptr<MockInputOutput> consumer = std::make_shared<MockInputOutput>();
	EXPECT_TRUE(device->addInputConsumer(consumer));

	KeyboardDeviceTest::update(device);

	EXPECT_TRUE(consumer->m_lastReceivedInput.integers().hasData("key"));
	EXPECT_TRUE(consumer->m_lastReceivedInput.integers().hasData("modifierMask"));
}

TEST_F(KeyboardDeviceTest, NoTwoKeyboards)
{
	std::shared_ptr<KeyboardDevice> device1 = std::make_shared<KeyboardDevice>("TestKeyboard1");
	std::shared_ptr<KeyboardDevice> device2 = std::make_shared<KeyboardDevice>("TestKeyboard2");

	EXPECT_NO_THROW(device1->initialize());
	EXPECT_ANY_THROW(device2->initialize());
}

};  // namespace Devices
};  // namespace SurgSim
