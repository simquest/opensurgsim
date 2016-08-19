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


#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "SurgSim/Blocks/KeyBehavior.h"
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/DataStructures/NamedData.h"
#include "SurgSim/Devices/Keyboard/KeyCode.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Testing/MockInputComponent.h"

namespace SurgSim
{
namespace Blocks
{
class MockKeyBehavior : public KeyBehavior
{
public:
	explicit MockKeyBehavior(const std::string& name) : KeyBehavior(name) {}

	MOCK_METHOD1(onKeyDown, void(int));
	MOCK_METHOD1(onKeyUp, void(int));
};

class KeyBehaviorTest: public testing::Test
{
public:

	void SetUp()
	{
		builder.addInteger(DataStructures::Names::KEY);
		data = builder.createData();
		inputComponent = std::make_shared<Testing::MockInputComponent>("Input");
		inputComponent->setData(data);
		runtime = std::make_shared<Framework::Runtime>();
		element = std::make_shared<Framework::BasicSceneElement>("Element");
		runtime->getScene()->addSceneElement(element);
		element->addComponent(inputComponent);
	}

	DataStructures::DataGroupBuilder builder;
	DataStructures::DataGroup data;
	std::shared_ptr<Testing::MockInputComponent> inputComponent;
	std::shared_ptr<Framework::Runtime> runtime;
	std::shared_ptr<Framework::SceneElement> element;
};

TEST_F(KeyBehaviorTest, Init)
{
	std::shared_ptr<MockKeyBehavior> behavior;
	std::shared_ptr<Framework::Component> component = inputComponent;
	ASSERT_NO_THROW(behavior = std::make_shared<MockKeyBehavior>("Behavior"));
	EXPECT_NO_THROW(behavior->setValue("InputComponent", component));
	auto input = behavior->getValue<std::shared_ptr<Input::InputComponent>>("InputComponent");
	EXPECT_EQ(inputComponent.get(), input.get());
}

TEST_F(KeyBehaviorTest, SetAndGetInputComponent)
{
	auto behavior = std::make_shared<MockKeyBehavior>("MockSingleKeyBehavior");
	auto input = std::make_shared<Input::InputComponent>("input");

	EXPECT_NO_THROW(behavior->setInputComponent(input));
	std::shared_ptr<Input::InputComponent> retrievedInputComponent;
	EXPECT_NO_THROW(retrievedInputComponent = behavior->getInputComponent());
	EXPECT_EQ(input, retrievedInputComponent);
}

TEST_F(KeyBehaviorTest, SimpleTrigger)
{
	auto behavior = std::make_shared<MockKeyBehavior>("Behavior");
	EXPECT_CALL(*behavior, onKeyDown(Devices::KEY_A));
	EXPECT_CALL(*behavior, onKeyUp(Devices::KEY_A));
	behavior->setInputComponent(inputComponent);
	element->addComponent(behavior);
	data.integers().set(DataStructures::Names::KEY, Devices::KeyCode::KEY_A);
	inputComponent->setData(data);
	behavior->update(0.0);

	// Shouldn't trigger another KeyDown ..
	data.integers().set(DataStructures::Names::KEY, Devices::KeyCode::KEY_A);
	inputComponent->setData(data);
	behavior->update(0.0);

	data.integers().set(DataStructures::Names::KEY, Devices::KeyCode::NONE);
	inputComponent->setData(data);
	behavior->update(0.0);
	// Shouldn't trigger another KeyUp ..
	behavior->update(0.0);
}

TEST_F(KeyBehaviorTest, FromOneKeyToAnother)
{
	auto behavior = std::make_shared<MockKeyBehavior>("Behavior");
	EXPECT_CALL(*behavior, onKeyDown(Devices::KEY_A));
	EXPECT_CALL(*behavior, onKeyDown(Devices::KEY_B));
	EXPECT_CALL(*behavior, onKeyUp(Devices::KEY_A));
	EXPECT_CALL(*behavior, onKeyUp(Devices::KEY_B));
	behavior->setInputComponent(inputComponent);
	element->addComponent(behavior);
	data.integers().set(DataStructures::Names::KEY, Devices::KeyCode::KEY_A);
	inputComponent->setData(data);
	behavior->update(0.0);
	data.integers().set(DataStructures::Names::KEY, Devices::KeyCode::KEY_B);
	inputComponent->setData(data);
	behavior->update(0.0);
	data.integers().set(DataStructures::Names::KEY, Devices::KeyCode::NONE);
	inputComponent->setData(data);
	behavior->update(0.0);
	// Shouldn't trigger another KeyUp ..
	behavior->update(0.0);
}

TEST_F(KeyBehaviorTest, Registry)
{
	EXPECT_TRUE(KeyBehavior::registerKey(Devices::KeyCode::KEY_A, "TestFunctionality"));
	EXPECT_TRUE(KeyBehavior::registerKey(Devices::KeyCode::KEY_B, "OtherFunctionality"));

	EXPECT_FALSE(KeyBehavior::registerKey(Devices::KeyCode::KEY_A, "TestFunctionality"));

	KeyBehavior::unregisterKey(Devices::KeyCode::KEY_A);
	EXPECT_TRUE(KeyBehavior::registerKey(Devices::KeyCode::KEY_A, "TestFunctionality"));

	KeyBehavior::logMap();

}






}
}
