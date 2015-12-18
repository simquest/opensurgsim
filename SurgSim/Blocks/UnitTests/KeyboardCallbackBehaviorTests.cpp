// This file is a part of the OpenSurgSim project.
// Copyright 2015, SimQuest Solutions Inc.
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
/// Tests for the KeyboardCallbackBehavior class.

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "SurgSim/Blocks/KeyboardCallbackBehavior.h"
#include "SurgSim/DataStructures/DataStructuresConvert.h"
#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Input/InputComponent.h"
#include "SurgSim/Input/OutputComponent.h"

using SurgSim::Blocks::KeyboardCallbackBehavior;
using SurgSim::Devices::KeyCode;
using SurgSim::Framework::Component;
using SurgSim::Input::InputComponent;
using SurgSim::Input::OutputComponent;

TEST(KeyboardCallbackBehavior, Constructor)
{
	EXPECT_NO_THROW(KeyboardCallbackBehavior behavior("KeyboardCallbackBehavior"));
}

TEST(KeyboardCallbackBehavior, InputComponentTests)
{
	auto keyboardCallbackBehavior =
		std::make_shared<KeyboardCallbackBehavior>("KeyboardCallbackBehavior");
	{
		auto invalidInputComponent = std::make_shared<OutputComponent>("InvalidInputComponent");
		EXPECT_ANY_THROW(keyboardCallbackBehavior->setInputComponent(invalidInputComponent));
	}

	{
		auto inputComponent = std::make_shared<InputComponent>("InputComponent");

		EXPECT_NO_THROW(keyboardCallbackBehavior->setInputComponent(inputComponent));
		EXPECT_EQ(inputComponent, keyboardCallbackBehavior->getInputComponent());
	}
}

TEST(KeyboardCallbackBehavior, Serialization)
{
	auto keyboardCallbackBehavior = std::make_shared<KeyboardCallbackBehavior>("KeyboardCallbackBehavior");
	std::shared_ptr<Component> inputComponent = std::make_shared<InputComponent>("InputComponent");

	int keyValue = static_cast<int>(KeyCode::KEY_A);
	keyboardCallbackBehavior->setValue("ActionKey", keyValue);
	keyboardCallbackBehavior->setValue("InputComponent", inputComponent);

	YAML::Node node;
	EXPECT_NO_THROW(node = YAML::convert<Component>::encode(*keyboardCallbackBehavior));
	EXPECT_TRUE(node.IsMap());
	EXPECT_EQ(5, node[keyboardCallbackBehavior->getClassName()].size());

	std::shared_ptr<KeyboardCallbackBehavior> newKeyboardCallbackBehavior;
	EXPECT_NO_THROW(newKeyboardCallbackBehavior = std::dynamic_pointer_cast<KeyboardCallbackBehavior>(
															node.as<std::shared_ptr<Component>>()));
	ASSERT_NE(nullptr, newKeyboardCallbackBehavior);
	EXPECT_NE(nullptr, newKeyboardCallbackBehavior->getValue<std::shared_ptr<InputComponent>>("InputComponent"));

	int newKeyValue = newKeyboardCallbackBehavior->getValue<int>("ActionKey");
	EXPECT_EQ(newKeyValue, keyValue);
}
