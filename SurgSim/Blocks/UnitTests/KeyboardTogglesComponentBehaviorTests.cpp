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
/// Tests for the KeyboardTogglesComponentBehavior class.

#include <algorithm>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "SurgSim/Blocks/KeyboardTogglesComponentBehavior.h"
#include "SurgSim/DataStructures/DataStructuresConvert.h"
#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Graphics/OsgBoxRepresentation.h"
#include "SurgSim/Input/InputComponent.h"
#include "SurgSim/Input/OutputComponent.h"

using SurgSim::Blocks::KeyboardTogglesComponentBehavior;
using SurgSim::Devices::KeyCode;
using SurgSim::Framework::Component;
using SurgSim::Graphics::OsgBoxRepresentation;
using SurgSim::Graphics::Representation;
using SurgSim::Input::InputComponent;
using SurgSim::Input::OutputComponent;

TEST(KeyboardTogglesComponentBehavior, Constructor)
{
	EXPECT_NO_THROW(KeyboardTogglesComponentBehavior behavior("KeyboardTogglesComponentBehavior"));
}

TEST(KeyboardTogglesComponentBehavior, InputComponentTests)
{
	auto keyboardTogglesComponentBehavior =
		std::make_shared<KeyboardTogglesComponentBehavior>("KeyboardTogglesComponentBehavior");
	{
		auto invalidInputComponent = std::make_shared<OutputComponent>("InvalidInputComponent");
		EXPECT_ANY_THROW(keyboardTogglesComponentBehavior->setInputComponent(invalidInputComponent));
	}

	{
		auto inputComponent = std::make_shared<InputComponent>("InputComponent");

		EXPECT_NO_THROW(keyboardTogglesComponentBehavior->setInputComponent(inputComponent));
		EXPECT_EQ(inputComponent, keyboardTogglesComponentBehavior->getInputComponent());
	}
}

TEST(KeyboardTogglesComponentBehavior, RegistrationTests)
{
	auto keyboardTogglesComponentBehavior =
		std::make_shared<KeyboardTogglesComponentBehavior>("KeyboardTogglesComponentBehavior");
	{
		auto graphics = std::make_shared<OsgBoxRepresentation>("Graphics");
		auto graphics2 = std::make_shared<OsgBoxRepresentation>("Graphics2");
		auto graphics3 = std::make_shared<OsgBoxRepresentation>("Graphics3");

		keyboardTogglesComponentBehavior->registerKey(KeyCode::KEY_A, graphics);
		keyboardTogglesComponentBehavior->registerKey(KeyCode::KEY_A, graphics2);
		keyboardTogglesComponentBehavior->registerKey(KeyCode::KEY_B, graphics3);

		auto keyMap = keyboardTogglesComponentBehavior->getKeyboardRegistry();
		auto keyAPair = keyMap.find(KeyCode::KEY_A);
		auto keyBPair = keyMap.find(KeyCode::KEY_B);

		EXPECT_EQ(2u, keyAPair->second.size());
		EXPECT_EQ(1u, keyBPair->second.size());

		EXPECT_NE(std::end(keyAPair->second), keyAPair->second.find(graphics));
		EXPECT_NE(std::end(keyAPair->second), keyAPair->second.find(graphics2));
		EXPECT_NE(std::end(keyBPair->second), keyBPair->second.find(graphics3));
	}
}

TEST(KeyboardTogglesComponentBehavior, SetAndGetKeyboardRegisterTypeTest)
{
	auto keyboardTogglesComponentBehavior =
		std::make_shared<KeyboardTogglesComponentBehavior>("KeyboardTogglesComponentBehavior");
	std::shared_ptr<Representation> graphics1 = std::make_shared<OsgBoxRepresentation>("graphics1");
	std::shared_ptr<Representation> graphics2 = std::make_shared<OsgBoxRepresentation>("graphics2");
	std::shared_ptr<Representation> graphics3 = std::make_shared<OsgBoxRepresentation>("graphics3");

	std::unordered_set<std::shared_ptr<Component>> set1;
	std::unordered_set<std::shared_ptr<Component>> set2;

	set1.insert(graphics1);
	set2.insert(graphics2);
	set2.insert(graphics3);

	KeyboardTogglesComponentBehavior::KeyboardRegistryType keyMap;
	keyMap[KeyCode::KEY_A] = set1;
	keyMap[KeyCode::KEY_B] = set2;

	EXPECT_NO_THROW(keyboardTogglesComponentBehavior->setKeyboardRegistry(keyMap));

	auto retrievedKeyMap = keyboardTogglesComponentBehavior->getKeyboardRegistry();
	EXPECT_EQ(keyMap.size(), retrievedKeyMap.size());
	for (auto it = std::begin(keyMap); it != std::end(keyMap); ++it)
	{
		auto componentSet = retrievedKeyMap.find(it->first)->second;
		EXPECT_EQ(it->second.size(), componentSet.size());
		for (auto item = std::begin(it->second); item != std::end(it->second); ++item)
		{
			auto match = std::find_if(std::begin(componentSet), std::end(componentSet),
				[&item](const std::shared_ptr<Component> rep)
			{
				return rep->getName() == (*item)->getName();
			});
			EXPECT_NE(std::end(componentSet), match);
		}
	}
}

TEST(KeyboardTogglesComponentBehavior, Serialization)
{
	auto keyboardTogglesComponentBehavior =
		std::make_shared<KeyboardTogglesComponentBehavior>("KeyboardTogglesComponentBehavior");

	std::shared_ptr<SurgSim::Framework::Component> inputComponent = std::make_shared<InputComponent>("InputComponent");
	std::shared_ptr<Representation> graphics1 = std::make_shared<OsgBoxRepresentation>("graphics1");
	std::shared_ptr<Representation> graphics2 = std::make_shared<OsgBoxRepresentation>("graphics2");

	std::unordered_set<std::shared_ptr<Component>> set1;
	std::unordered_set<std::shared_ptr<Component>> set2;

	set1.insert(graphics1);
	set2.insert(graphics2);

	KeyboardTogglesComponentBehavior::KeyboardRegistryType keyMap;
	keyMap[KeyCode::KEY_A] = set1;
	keyMap[KeyCode::KEY_B] = set2;

	keyboardTogglesComponentBehavior->setValue("KeyboardRegistry", keyMap);
	keyboardTogglesComponentBehavior->setValue("InputComponent", inputComponent);

	YAML::Node node;
	EXPECT_NO_THROW(node = YAML::convert<SurgSim::Framework::Component>::encode(*keyboardTogglesComponentBehavior));
	EXPECT_TRUE(node.IsMap());
	EXPECT_EQ(5, node[keyboardTogglesComponentBehavior->getClassName()].size());

	std::shared_ptr<KeyboardTogglesComponentBehavior> newKeyboardTogglesComponentBehavior;
	EXPECT_NO_THROW(newKeyboardTogglesComponentBehavior = std::dynamic_pointer_cast<KeyboardTogglesComponentBehavior>(
															node.as<std::shared_ptr<SurgSim::Framework::Component>>()));
	ASSERT_NE(nullptr, newKeyboardTogglesComponentBehavior);
	EXPECT_NE(nullptr, newKeyboardTogglesComponentBehavior->getValue<
			std::shared_ptr<InputComponent>>("InputComponent"));

	// Make sure every registered representation in the original 'keyMap' is present in the de-serialized keyMap.
	auto retrievedKeyMap = newKeyboardTogglesComponentBehavior->getValue<
							KeyboardTogglesComponentBehavior::KeyboardRegistryType>("KeyboardRegistry");

	EXPECT_EQ(keyMap.size(), retrievedKeyMap.size());
	for (auto it = std::begin(keyMap); it != std::end(keyMap); ++it)
	{
		auto componentSet = retrievedKeyMap.find(it->first)->second;
		EXPECT_EQ(it->second.size(), componentSet.size());
		for (auto item = std::begin(it->second); item != std::end(it->second); ++item)
		{
			auto match = std::find_if(std::begin(componentSet), std::end(componentSet),
									  [&item](const std::shared_ptr<Component> rep)
									  {
										return rep->getName() == (*item)->getName();
									  });
			 EXPECT_NE(std::end(componentSet), match);
		}
	}
}
