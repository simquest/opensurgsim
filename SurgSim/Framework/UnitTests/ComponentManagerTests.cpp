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


///\file ComponentManagerTests.cpp test the basic functionality of the component manager
///		 mostly through a mock manager that exposes the private interface and implements
///		 the simplest version of the abstract interface.

#include <gtest/gtest.h>

#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Framework/ComponentManager.h>


#include <SurgSim/Framework/UnitTests/MockObjects.h>

using SurgSim::Framework::Runtime;
using SurgSim::Framework::Component;

TEST(ComponentManagerTests, TestInternalAddRemove)
{
	std::shared_ptr<Component> mock1 = std::make_shared<MockComponent>("Component1");
	std::shared_ptr<Component> mock2 = std::make_shared<MockComponent>("Component2");
	std::shared_ptr<Component> invalid = std::make_shared<MockBehavior>("Behavior1");

	MockManager manager;
	EXPECT_EQ(0u, manager.getComponents().size());


	// Basic case should be able to add mockcomponent
	EXPECT_TRUE(manager.testTryAddComponent(mock1));
	EXPECT_EQ(1u, manager.getComponents().size());
	EXPECT_TRUE(manager.testTryAddComponent(mock2));
	EXPECT_EQ(2u, manager.getComponents().size());

	// Should not be able to add behavior
	EXPECT_FALSE(manager.testTryAddComponent(invalid));
	EXPECT_EQ(2u, manager.getComponents().size());

	// Should not be able to add duplicate
	EXPECT_FALSE(manager.testTryAddComponent(mock2));
	EXPECT_EQ(2u, manager.getComponents().size());

	// Test the removals
	EXPECT_FALSE(manager.testTryRemoveComponent(invalid));
	EXPECT_EQ(2u, manager.getComponents().size());

	EXPECT_TRUE(manager.testTryRemoveComponent(mock1));
	EXPECT_EQ(1u, manager.getComponents().size());

	EXPECT_FALSE(manager.testTryRemoveComponent(mock1));
	EXPECT_EQ(1u, manager.getComponents().size());

	EXPECT_TRUE(manager.testTryRemoveComponent(mock2));
	EXPECT_EQ(0u, manager.getComponents().size());

	// Add after remove
	EXPECT_TRUE(manager.testTryAddComponent(mock1));
	EXPECT_EQ(1u, manager.getComponents().size());

}

TEST(ComponentManagerTests, SimpleAddRemoveComponentTest)
{
	std::shared_ptr<Component> mock1 = std::make_shared<MockComponent>("Component1");
	std::shared_ptr<Component> mock2 = std::make_shared<MockComponent>("Component2");
	std::shared_ptr<Component> invalid = std::make_shared<MockBehavior>("Behavior1");

	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();
	MockManager manager;
	manager.setRuntime(runtime);
	manager.enqueueAddComponent(mock1);
	manager.enqueueAddComponent(mock1);

	manager.testProcessComponents();

	EXPECT_EQ(1u, manager.getComponents().size());

	manager.enqueueRemoveComponent(mock1);
	manager.testProcessComponents();
	EXPECT_EQ(0u, manager.getComponents().size());

}

TEST(ComponentManagerTests, CompundAddRemoveComponentTest)
{
	std::shared_ptr<Component> mock1 = std::make_shared<MockComponent>("Component1");
	std::shared_ptr<Component> mock2 = std::make_shared<MockComponent>("Component2");
	std::shared_ptr<Component> invalid = std::make_shared<MockBehavior>("Behavior1");

	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();
	MockManager manager;
	manager.setRuntime(runtime);
	manager.enqueueAddComponent(mock1);
	manager.enqueueAddComponent(mock2);
	manager.enqueueAddComponent(invalid);
	manager.testProcessComponents();

	EXPECT_EQ(2u, manager.getComponents().size());
	manager.enqueueRemoveComponent(mock1);
	manager.enqueueAddComponent(mock2);
	manager.enqueueRemoveComponent(invalid);
	manager.testProcessComponents();

	EXPECT_EQ(1u, manager.getComponents().size());
}