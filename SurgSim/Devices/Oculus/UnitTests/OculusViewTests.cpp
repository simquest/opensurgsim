// This file is a part of the OpenSurgSim project.
// Copyright 2012-2013, SimQuest Solutions Inc.
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
/// Tests for the OculusView class.

#include <gtest/gtest.h>

#include "SurgSim/Devices/Oculus/OculusView.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Input/InputComponent.h"

using SurgSim::Device::OculusView;

TEST(OculusViewTests, InitTest)
{
	EXPECT_NO_THROW(SurgSim::Device::OculusView("OculusView"));
}

TEST(OculusViewTests, SetAndGetInputComponent)
{
	auto view = std::make_shared<SurgSim::Device::OculusView>("OculusView");
	auto inputComponent = std::make_shared<SurgSim::Input::InputComponent>("InputComponent");

	EXPECT_NO_THROW(view->setInputComponent(inputComponent));
	EXPECT_EQ(inputComponent, view->getInputComponent());
}

TEST(OculusViewTests, Serialization)
{
	auto view = std::make_shared<SurgSim::Device::OculusView>("test name");
	std::shared_ptr<SurgSim::Framework::Component> inputComponent =
		std::make_shared<SurgSim::Input::InputComponent>("InputComponent");
	view->setValue("InputComponent", inputComponent);

	/// Serialize
	YAML::Node node;
	EXPECT_NO_THROW(node = YAML::convert<SurgSim::Framework::Component>::encode(*view););

	/// Deserialize
	std::shared_ptr<OculusView> newView;
	EXPECT_NO_THROW(newView = std::dynamic_pointer_cast<OculusView>(
		node.as<std::shared_ptr<SurgSim::Framework::Component>>()));
	EXPECT_NE(nullptr, newView);
	EXPECT_NE(nullptr, newView->getInputComponent());
	EXPECT_EQ(inputComponent->getName(), newView->getInputComponent()->getName());
}
