// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

/** @file
 * Tests for the InputComponent class.
 */

#include <gtest/gtest.h>
#include <memory>
#include <string>
#include "yaml-cpp/yaml.h"

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Input/InputComponent.h"
#include "SurgSim/Input/UnitTests/TestDevice.h"

using SurgSim::Input::InputComponent;


TEST(InputComponentTest, CanConstruct)
{
	EXPECT_NO_THROW(InputComponent input("Input"); input.setDeviceName("InputDevice"));
}

TEST(InputComponentTest, Accessors)
{
	InputComponent input("Input");
	input.setDeviceName("InputDevice");
	EXPECT_EQ("Input", input.getName());
	EXPECT_EQ("InputDevice", input.getDeviceName());

	EXPECT_TRUE(input.getToDeviceTransform().isApprox(SurgSim::Math::RigidTransform3d::Identity()));
	EXPECT_TRUE(input.getToElementTransform().isApprox(SurgSim::Math::RigidTransform3d::Identity()));
}

TEST(InputComponentTest, GetData)
{
	auto input = std::make_shared<InputComponent>("Input");
	SurgSim::DataStructures::DataGroup data;
	ASSERT_NO_THROW(input->getData(&data)) << "Getting data from an unconnected InputComponent should not assert.";
	EXPECT_TRUE(data.isEmpty()) << "The data from an unconnected InputComponent should be empty.";

	TestDevice device("MyTestDevice");
	device.addInputConsumer(input);
	ASSERT_NO_THROW(input->getData(&data));
	EXPECT_FALSE(data.isEmpty());
}

TEST(InputComponentTest, Serialization)
{
	auto input = std::make_shared<InputComponent>("Input");
	input->setDeviceName("InputDevice");

	// Encode
	YAML::Node node;
	EXPECT_NO_THROW(node = YAML::convert<SurgSim::Framework::Component>::encode(*input););

	// Decode
	std::shared_ptr<InputComponent> newInput;
	EXPECT_NO_THROW(newInput = std::dynamic_pointer_cast<InputComponent>(
									node.as<std::shared_ptr<SurgSim::Framework::Component>>()););

	// Verify
	EXPECT_EQ(input->getDeviceName(), newInput->getDeviceName());
}

