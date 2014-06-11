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
/// Tests for the KeyboardTogglesGraphicsBehavior class.

#include <algorithm>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "SurgSim/Blocks/KeyboardTogglesGraphicsBehavior.h"
#include "SurgSim/DataStructures/DataStructuresConvert.h"
#include "SurgSIm/Framework/FrameworkConvert.h"
#include "SurgSim/Graphics/Material.h"
#include "SurgSim/Graphics/Representation.h"
#include "SurgSim/Input/InputComponent.h"

using SurgSim::Blocks::KeyboardTogglesGraphicsBehavior;
using SurgSim::Graphics::Representation;
using SurgSim::Input::InputComponent;

class MockComponent : public SurgSim::Framework::Component
{
public:
	explicit MockComponent(const std::string& name) : Component(name) {}
	SURGSIM_CLASSNAME(MockComponent);
	virtual bool doInitialize() override {return true;}
	virtual bool doWakeUp() override {return true;}
};
SURGSIM_REGISTER(SurgSim::Framework::Component, MockComponent);

class MockRepresentation : public SurgSim::Graphics::Representation
{
public:
	explicit MockRepresentation(const std::string& name) : SurgSim::Graphics::Representation(name) {}
	SURGSIM_CLASSNAME(MockRepresentation);
	virtual void setVisible(bool visible) override {}
	virtual bool isVisible() const override {return true;}
	virtual void update(double dt) override {}
	virtual bool setMaterial(std::shared_ptr<SurgSim::Graphics::Material> material) override {return true;}
	virtual std::shared_ptr<SurgSim::Graphics::Material> getMaterial() const override {return nullptr;}
	virtual void clearMaterial() override {}
};
SURGSIM_REGISTER(SurgSim::Framework::Component, MockRepresentation);

TEST(KeyboardTogglesGraphicsBehavior, Constructor)
{
	EXPECT_NO_THROW(KeyboardTogglesGraphicsBehavior keyboardTogglesGraphicsBehavior("KeyboardTogglesGraphicsBehavior"));
}

TEST(KeyboardTogglesGraphicsBehavior, InputComponentTests)
{
	auto keyboardTogglesGraphicsBehavior =
		std::make_shared<KeyboardTogglesGraphicsBehavior>("KeyboardTogglesGraphicsBehavior");
	{
		SCOPED_TRACE("Set invalid input component");
		auto mockComponent = std::make_shared<MockComponent>("MockComponent");
		EXPECT_ANY_THROW(keyboardTogglesGraphicsBehavior->setInputComponent(mockComponent));
	}

	{
		SCOPED_TRACE("Set valid input component");
		auto inputComponent = std::make_shared<InputComponent>("InputComponent");

		EXPECT_NO_THROW(keyboardTogglesGraphicsBehavior->setInputComponent(inputComponent));
		EXPECT_EQ(inputComponent, keyboardTogglesGraphicsBehavior->getInputComponent());
	}
}

TEST(KeyboardTogglesGraphicsBehavior, RegistrationTests)
{
	auto keyboardTogglesGraphicsBehavior =
		std::make_shared<KeyboardTogglesGraphicsBehavior>("KeyboardTogglesGraphicsBehavior");
	{
		SCOPED_TRACE("Register invalid graphics representaiton");
		auto mockComponent = std::make_shared<MockComponent>("MockComponent");
		EXPECT_FALSE(keyboardTogglesGraphicsBehavior->registerKey(SurgSim::Device::KeyCode::KEY_A, mockComponent));
	}

	{
		SCOPED_TRACE("Register valid graphics representation");
		auto mockGraphics = std::make_shared<MockRepresentation>("MockGraphics");
		auto mockGraphics2 = std::make_shared<MockRepresentation>("MockGraphics2");
		auto mockGraphics3 = std::make_shared<MockRepresentation>("MockGraphics3");

		EXPECT_TRUE(keyboardTogglesGraphicsBehavior->registerKey(SurgSim::Device::KeyCode::KEY_A, mockGraphics));
		EXPECT_TRUE(keyboardTogglesGraphicsBehavior->registerKey(SurgSim::Device::KeyCode::KEY_A, mockGraphics2));
		EXPECT_TRUE(keyboardTogglesGraphicsBehavior->registerKey(SurgSim::Device::KeyCode::KEY_B, mockGraphics3));
	}
}

TEST(KeyboardTogglesGraphicsBehavior, SetAndGetKeyboardRegisterTypeTest)
{
	auto keyboardTogglesGraphicsBehavior =
		std::make_shared<KeyboardTogglesGraphicsBehavior>("KeyboardTogglesGraphicsBehavior");
	std::shared_ptr<Representation> mockGraphics1 = std::make_shared<MockRepresentation>("MockGraphics1");
	std::shared_ptr<Representation> mockGraphics2 = std::make_shared<MockRepresentation>("MockGraphics2");
	std::shared_ptr<Representation> mockGraphics3 = std::make_shared<MockRepresentation>("MockGraphics3");

	std::unordered_set<std::shared_ptr<Representation>> set1;
	std::unordered_set<std::shared_ptr<Representation>> set2;

	set1.insert(mockGraphics1);
	set2.insert(mockGraphics2);
	set2.insert(mockGraphics3);

	KeyboardTogglesGraphicsBehavior::KeyboardRegisterType keyMap;
	keyMap.insert(KeyboardTogglesGraphicsBehavior::KeyboardRegisterType::value_type(SurgSim::Device::KeyCode::KEY_A,
																					set1));
	keyMap.insert(KeyboardTogglesGraphicsBehavior::KeyboardRegisterType::value_type(SurgSim::Device::KeyCode::KEY_B,
																					set2));

	EXPECT_NO_THROW(keyboardTogglesGraphicsBehavior->setKeyboardRegister(keyMap));
	EXPECT_EQ(keyMap, keyboardTogglesGraphicsBehavior->getKeyboardRegister());
}

TEST(KeyboardTogglesGraphicsBehavior, Serialization)
{
	auto keyboardTogglesGraphicsBehavior =
		std::make_shared<KeyboardTogglesGraphicsBehavior>("KeyboardTogglesGraphicsBehavior");

	std::shared_ptr<SurgSim::Framework::Component> inputComponent = std::make_shared<InputComponent>("InputComponent");
	std::shared_ptr<Representation> mockGraphics1 = std::make_shared<MockRepresentation>("MockGraphics1");
	std::shared_ptr<Representation> mockGraphics2 = std::make_shared<MockRepresentation>("MockGraphics2");

	std::unordered_set<std::shared_ptr<Representation>> set1;
	std::unordered_set<std::shared_ptr<Representation>> set2;

	set1.insert(mockGraphics1);
	set2.insert(mockGraphics2);

	KeyboardTogglesGraphicsBehavior::KeyboardRegisterType keyMap;
	keyMap.insert(KeyboardTogglesGraphicsBehavior::KeyboardRegisterType::value_type(SurgSim::Device::KeyCode::KEY_A,
																					set1));
	keyMap.insert(KeyboardTogglesGraphicsBehavior::KeyboardRegisterType::value_type(SurgSim::Device::KeyCode::KEY_B,
																					set2));

	keyboardTogglesGraphicsBehavior->setValue("KeyboardRegister", keyMap);
	keyboardTogglesGraphicsBehavior->setValue("InputComponent", inputComponent);

	YAML::Node node;
	EXPECT_NO_THROW(node = YAML::convert<SurgSim::Framework::Component>::encode(*keyboardTogglesGraphicsBehavior));
	EXPECT_TRUE(node.IsMap());
	EXPECT_EQ(4, node[keyboardTogglesGraphicsBehavior->getClassName()].size());

	std::shared_ptr<KeyboardTogglesGraphicsBehavior> newKeyboardTogglesGraphicsBehavior;
	EXPECT_NO_THROW(newKeyboardTogglesGraphicsBehavior = std::dynamic_pointer_cast<KeyboardTogglesGraphicsBehavior>(
															node.as<std::shared_ptr<SurgSim::Framework::Component>>()));

	EXPECT_NE(nullptr, newKeyboardTogglesGraphicsBehavior->getValue<std::shared_ptr<InputComponent>>("InputComponent"));

	// Make sure every registered representation in the original 'keyMap' is present in the de-serialized keyMap.
	auto retrievedKeyMap = newKeyboardTogglesGraphicsBehavior->getValue<
							KeyboardTogglesGraphicsBehavior::KeyboardRegisterType>("KeyboardRegister");

	EXPECT_EQ(keyMap.size(), retrievedKeyMap.size());
	for (auto it = std::begin(keyMap); it != std::end(keyMap); ++it)
	{
		auto representationSet = retrievedKeyMap.find(it->first)->second;
		for (auto item = std::begin(it->second); item != std::end(it->second); ++item)
		{
			auto match = std::find_if(std::begin(representationSet), std::end(representationSet),
									  [&item](const std::shared_ptr<Representation> rep)
									  {
										return rep->getName() == (*item)->getName();
									  });
			 EXPECT_NE(std::end(representationSet), match);
		}
	}
}