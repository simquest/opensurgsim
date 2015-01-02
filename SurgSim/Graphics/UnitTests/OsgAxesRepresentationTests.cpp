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

#include <gtest/gtest.h>

#include "SurgSim/Graphics/OsgAxesRepresentation.h"
#include "SurgSim/Framework/FrameworkConvert.h"


namespace SurgSim
{
namespace Graphics
{

TEST(OsgAxesRepresentationTests, Serialization)
{
	using SurgSim::Framework::Component;

	std::shared_ptr<Component> axes = std::make_shared<OsgAxesRepresentation>("axes");
	axes->setValue("Size", 12.0);

	YAML::Node node = YAML::convert<Component>::encode(*axes);

	std::shared_ptr<Component> component = node.as<std::shared_ptr<Component>>();

	ASSERT_NE(nullptr, component);

	std::shared_ptr<OsgAxesRepresentation> newAxes = std::dynamic_pointer_cast<OsgAxesRepresentation>(component);

	ASSERT_NE(nullptr, newAxes);

	EXPECT_DOUBLE_EQ(12.0, newAxes->getValue<double>("Size"));
}

}
}

