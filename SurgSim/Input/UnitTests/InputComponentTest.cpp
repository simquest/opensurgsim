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

/** @file
 * Tests for the InputComponent class.
 */

#include <memory>
#include <string>
#include <gtest/gtest.h>
#include "SurgSim/Input/InputComponent.h"
#include "SurgSim/DataStructures/DataGroup.h"

using SurgSim::Input::InputComponent;
using SurgSim::DataStructures::DataGroup;

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
}

TEST(InputComponentTest, NotConnected)
{
	InputComponent input("Input");
	input.setDeviceName("InputDevice");
	DataGroup dataGroup;
	EXPECT_THROW(input.getData(&dataGroup), SurgSim::Framework::AssertionFailure);
	EXPECT_FALSE(input.isDeviceConnected());
}

