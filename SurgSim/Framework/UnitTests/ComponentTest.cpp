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
#include <SurgSim/Framework/Scene.h>
#include <SurgSim/Framework/Component.h>

#include <SurgSim/Framework/UnitTests/MockObjects.h>

using SurgSim::Framework::Component;
using SurgSim::Framework::Scene;
using SurgSim::Framework::Runtime;

TEST(ComponentTests, Constructor)
{
	ASSERT_NO_THROW({MockComponent component("Component");});
}

TEST(ComponentTests, SetAndGetSceneElementTest)
{
	std::shared_ptr<Component> mock1 = std::make_shared<MockComponent>("Component");

	std::shared_ptr<MockSceneElement> element1(new MockSceneElement("one"));

	mock1->setSceneElement(element1);

	EXPECT_EQ(element1, mock1->getSceneElement());
	
}

TEST(ComponentTests, SetAndGetSceneTest)
{
	std::shared_ptr<Component> mock1 = std::make_shared<MockComponent>("Component");

	std::shared_ptr<Scene> scene = std::make_shared<Scene>();

	mock1->setScene(scene);

	EXPECT_EQ(scene, mock1->getScene());
	
}

