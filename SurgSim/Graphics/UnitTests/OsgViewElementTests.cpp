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
/// Tests for the OsgViewElement class.

#include <SurgSim/Graphics/UnitTests/MockObjects.h>
#include <SurgSim/Graphics/UnitTests/MockOsgObjects.h>

#include <SurgSim/Graphics/OsgView.h>
#include <SurgSim/Graphics/OsgViewElement.h>

#include <gtest/gtest.h>

namespace SurgSim
{
namespace Graphics
{

TEST(OsgViewElementTests, InitTest)
{
	ASSERT_NO_THROW({std::shared_ptr<ViewElement> viewElement = std::make_shared<OsgViewElement>("test name");});

	std::shared_ptr<ViewElement> viewElement = std::make_shared<OsgViewElement>("test name");

	std::shared_ptr<OsgView> osgView = std::dynamic_pointer_cast<OsgView>(viewElement->getView());
	EXPECT_NE(nullptr, osgView);
}

TEST(OsgViewElementTests, ViewTest)
{
	std::shared_ptr<ViewElement> element = std::make_shared<OsgViewElement>("test name");

	/// Setting an OsgView should succeed
	std::shared_ptr<View> osgView = std::make_shared<OsgView>("test osg view");
	EXPECT_TRUE(element->setView(osgView));
	EXPECT_EQ(osgView, element->getView());

	/// Any other View should fail
	std::shared_ptr<View> mockView = std::make_shared<MockView>("test mock view");

	EXPECT_FALSE(element->setView(mockView));
	EXPECT_NE(mockView, element->getView());
	EXPECT_EQ(osgView, element->getView());
}

TEST(OsgViewElementTests, enableKeyboardTest)
{
	std::shared_ptr<OsgViewElement> element = std::make_shared<OsgViewElement>("test name");

	EXPECT_TRUE(element->enableKeyboardDevice(true));
	EXPECT_TRUE(element->enableKeyboardDevice(false));
}

};  // namespace Graphics
};  // namespace SurgSim
