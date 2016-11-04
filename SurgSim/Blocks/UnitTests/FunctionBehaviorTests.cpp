// This file is a part of the OpenSurgSim project.
// Copyright 2016, SimQuest Solutions Inc.
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

#include <iostream>
#include <gtest/gtest.h>

#include "SurgSim/Blocks/FunctionBehavior.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/Runtime.h"

namespace {

void f1(double dt) { std::cout << dt; }
void f2(double dt, double a) { std::cout << dt + a; }

};

namespace SurgSim
{
namespace Blocks
{

TEST(FunctionBehaviorTest, Constructor)
{
	EXPECT_NO_THROW(FunctionBehavior behavior("Behavior"));
}

TEST(FunctionBehaviorTest, GetSetTargetManagerType)
{
	auto behavior = std::make_shared<FunctionBehavior>("Behavior");
	EXPECT_EQ(Framework::MANAGER_TYPE_BEHAVIOR, behavior->getTargetManagerType());

	EXPECT_THROW(behavior->setTargetManagerType(-1), Framework::AssertionFailure);
	EXPECT_NO_THROW(behavior->setTargetManagerType(Framework::MANAGER_TYPE_PHYSICS));
	EXPECT_EQ(Framework::MANAGER_TYPE_PHYSICS, behavior->getTargetManagerType());

	auto runtime = std::make_shared<Framework::Runtime>();
	behavior->setFunction([](double dt) { std::cout << dt; });
	behavior->initialize(runtime);

	EXPECT_THROW(behavior->setTargetManagerType(Framework::MANAGER_TYPE_GRAPHICS), Framework::AssertionFailure);
}

TEST(FunctionBehaviorTest, SetFunction)
{
	auto runtime = std::make_shared<Framework::Runtime>();
	{
		auto behavior = std::make_shared<FunctionBehavior>("Behavior");
		EXPECT_FALSE(behavior->initialize(runtime));
	}
	{
		auto behavior = std::make_shared<FunctionBehavior>("Behavior");
		behavior->setFunction([](double dt) { std::cout << dt; });
		EXPECT_TRUE(behavior->initialize(runtime));
	}
}

TEST(FunctionBehaviorTest, Update)
{
	size_t numUpdates = 0;
	auto behavior = std::make_shared<FunctionBehavior>("Behavior");
	behavior->setFunction([&numUpdates](double dt) { numUpdates++; });

	auto runtime = std::make_shared<Framework::Runtime>();
	behavior->initialize(runtime);
	behavior->wakeUp();

	behavior->update(1.0);
	behavior->update(1.0);
	EXPECT_EQ(2, numUpdates);
}

TEST(FunctionBehaviorTest, ExampleUsage)
{
	auto behavior1 = std::make_shared<SurgSim::Blocks::FunctionBehavior>("Behavior 1");
	EXPECT_NO_THROW(behavior1->setTargetManagerType(SurgSim::Framework::MANAGER_TYPE_BEHAVIOR));
	EXPECT_NO_THROW(behavior1->setFunction(f1));

	auto behavior2 = std::make_shared<SurgSim::Blocks::FunctionBehavior>("Behavior 2");
	EXPECT_NO_THROW(behavior2->setTargetManagerType(SurgSim::Framework::MANAGER_TYPE_GRAPHICS));
	EXPECT_NO_THROW(behavior2->setFunction(std::bind(f2, std::placeholders::_1, 2.0)));

	auto behavior3 = std::make_shared<SurgSim::Blocks::FunctionBehavior>("Behavior 3");
	EXPECT_NO_THROW(behavior3->setTargetManagerType(SurgSim::Framework::MANAGER_TYPE_PHYSICS));
	EXPECT_NO_THROW(behavior3->setFunction([](double dt) { std::cout << dt; }));

	auto element = std::make_shared<SurgSim::Framework::BasicSceneElement>("Element");
	EXPECT_NO_THROW(element->addComponent(behavior1));
	EXPECT_NO_THROW(element->addComponent(behavior2));
	EXPECT_NO_THROW(element->addComponent(behavior3));
}

};
};
