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
/// Tests for the DeviceUtilities.

#include <memory>
#include <string>
#include <gtest/gtest.h>

#include "SurgSim/Devices/DeviceUtilities.h"
#include "SurgSim/Devices/IdentityPoseDevice/IdentityPoseDevice.h"
#include "SurgSim/Input/DeviceInterface.h"

TEST(DeviceInterfaceTests, CreateDevice)
{
	const std::string name = "name";
	std::vector<std::string> types;
	EXPECT_EQ(nullptr, SurgSim::Devices::createDevice(types, name));

	types.push_back("DoNotHave");
	EXPECT_EQ(nullptr, SurgSim::Devices::createDevice(types, name));

	types.push_back("SurgSim::Devices::IdentityPoseDevice");
	auto device = SurgSim::Devices::createDevice(types, name);
	ASSERT_NE(nullptr, device);
	EXPECT_NE(nullptr, std::dynamic_pointer_cast<SurgSim::Devices::IdentityPoseDevice>(device));
	EXPECT_EQ(name, device->getName());
}
