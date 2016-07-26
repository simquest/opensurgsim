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
 * Tests for the OutputComponent class.
 */

#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Input/OutputComponent.h"

using SurgSim::Input::OutputComponent;
using SurgSim::DataStructures::DataGroup;

TEST(OutputComponentTest, CanConstruct)
{
	EXPECT_NO_THROW(OutputComponent output("Output"); output.setDeviceName("OutputDevice"));
}

TEST(OutputComponentTest, Accessors)
{
	OutputComponent output("Output");
	output.setDeviceName("OutputDevice");
	EXPECT_EQ("Output", output.getName());
	EXPECT_EQ("OutputDevice", output.getDeviceName());

	EXPECT_TRUE(output.getConnect());
	output.setConnect(false);
	EXPECT_FALSE(output.getConnect());
	output.setConnect(true);

	SurgSim::DataStructures::DataGroup actualData;
	ASSERT_FALSE(output.requestOutput("", &actualData));

	SurgSim::DataStructures::DataGroupBuilder builder;
	std::string name = "aBool";
	builder.addBoolean(name);
	DataGroup dataGroup = builder.createData();
	dataGroup.booleans().set(name, true);
	output.setData(dataGroup);
	bool result = false;
	ASSERT_TRUE(output.requestOutput("", &actualData));
	ASSERT_TRUE(actualData.booleans().get(name, &result));
	EXPECT_TRUE(result);
}

TEST(OutputComponentTest, Serialization)
{
	auto output = std::make_shared<OutputComponent>("Output");
	output->setDeviceName("OutputDevice");

	// Encode
	YAML::Node node;
	EXPECT_NO_THROW(node = YAML::convert<SurgSim::Framework::Component>::encode(*output););

	// Decode
	std::shared_ptr<OutputComponent> newOutput;
	EXPECT_NO_THROW(newOutput = std::dynamic_pointer_cast<OutputComponent>(
									node.as<std::shared_ptr<SurgSim::Framework::Component>>()););

	// Verify
	EXPECT_EQ(output->getDeviceName(), newOutput->getDeviceName());
}

