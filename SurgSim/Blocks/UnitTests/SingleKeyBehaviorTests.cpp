// This file is a part of the OpenSurgSim project.
// Copyright 2013 - 2016, SimQuest Solutions Inc.
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
/// Tests for the SingleKeyBehavior class.

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "SurgSim/Blocks/SingleKeyBehavior.h"
#include "SurgSim/Devices/Keyboard/KeyCode.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Input/InputComponent.h"

class MockSingleKeyBehavior : public SurgSim::Blocks::SingleKeyBehavior
{
public:
	explicit MockSingleKeyBehavior(const std::string& name) : SingleKeyBehavior(name)
	{
	}

protected:
	void onKey() override
	{
	}
};

TEST(SingleKeyBehaviorTest, Constructor)
{
	std::shared_ptr<SurgSim::Blocks::SingleKeyBehavior> singleKeyBehavior;
	ASSERT_NO_THROW(singleKeyBehavior = std::make_shared<MockSingleKeyBehavior>("SingleKeyBehavior"));
	EXPECT_EQ(nullptr, singleKeyBehavior->getInputComponent());
	EXPECT_EQ(SurgSim::Devices::KeyCode::NONE, singleKeyBehavior->getKey());
}

TEST(SingleKeyBehaviorTest, SetAndGetKey)
{
	auto behavior = std::make_shared<MockSingleKeyBehavior>("MockSingleKeyBehavior");
	int key = SurgSim::Devices::KeyCode::KEY_0;

	EXPECT_NO_THROW(behavior->setKey(key));
	int retrievedKey;
	EXPECT_NO_THROW(retrievedKey = behavior->getKey());
	EXPECT_EQ(key, retrievedKey);
}

TEST(SingleKeyBehaviorTest, WakeUp)
{
	auto input = std::make_shared<SurgSim::Input::InputComponent>("input");
	int key = SurgSim::Devices::KeyCode::KEY_1;
	{
		auto behavior = std::make_shared<MockSingleKeyBehavior>("MockSingleKeyBehavior");
		EXPECT_FALSE(behavior->doWakeUp());
	}

	{
		auto behavior = std::make_shared<MockSingleKeyBehavior>("MockSingleKeyBehavior");
		behavior->setInputComponent(input);
		EXPECT_FALSE(behavior->doWakeUp());
	}

	{
		auto behavior = std::make_shared<MockSingleKeyBehavior>("MockSingleKeyBehavior");
		behavior->setKey(key);
		EXPECT_FALSE(behavior->doWakeUp());
	}

	{
		auto behavior = std::make_shared<MockSingleKeyBehavior>("MockSingleKeyBehavior");
		behavior->setKey(key);
		behavior->setInputComponent(input);
		EXPECT_TRUE(behavior->doWakeUp());
	}
}

TEST(SingleKeyBehaviorTest, AccessibleValues)
{
	auto behavior = std::make_shared<MockSingleKeyBehavior>("MockSingleKeyBehavior");
	std::shared_ptr<SurgSim::Framework::Component> input = std::make_shared<SurgSim::Input::InputComponent>("input");
	int key = SurgSim::Devices::KeyCode::KEY_0;

	EXPECT_NO_THROW(behavior->setValue("ActionKey", key));
	EXPECT_NO_THROW(behavior->setValue("InputComponent", input));
	int retrievedKey;
	std::shared_ptr<SurgSim::Framework::Component> retrievedInputComponent;

	EXPECT_NO_THROW(retrievedKey = behavior->getValue<int>("ActionKey"));
	EXPECT_NO_THROW(retrievedInputComponent =
						behavior->getValue<std::shared_ptr<SurgSim::Input::InputComponent>>("InputComponent"));

	EXPECT_EQ(key, retrievedKey);
	EXPECT_EQ(input, retrievedInputComponent);
}
