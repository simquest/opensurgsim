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
/// Tests for the OsgView class.

#include "SurgSim/Graphics/UnitTests/MockObjects.h"
#include "SurgSim/Graphics/UnitTests/MockOsgObjects.h"

#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Graphics/OsgView.h"

#include <gtest/gtest.h>

#include <random>

namespace SurgSim
{
namespace Graphics
{

TEST(OsgViewTests, InitTest)
{
	ASSERT_NO_THROW({std::shared_ptr<View> view = std::make_shared<OsgView>("test name");});

	std::shared_ptr<View> view = std::make_shared<OsgView>("test name");

	EXPECT_EQ("test name", view->getName());

	EXPECT_EQ(nullptr, view->getCamera());

	std::array<int, 2> position = view->getPosition();
	EXPECT_EQ(0, position[0]);
	EXPECT_EQ(0, position[1]);

	std::array<int, 2> dimensions = view->getDimensions();
	EXPECT_EQ(1024, dimensions[0]);
	EXPECT_EQ(768, dimensions[1]);

	EXPECT_TRUE(view->isWindowBorderEnabled());
}

TEST(OsgViewTests, PositionAndDimensionsTest)
{
	std::shared_ptr<OsgView> osgView = std::make_shared<OsgView>("test name");
	std::shared_ptr<View> view = osgView;

	std::default_random_engine generator;
	std::uniform_int_distribution<int> distribution(0, 1000);

	std::array<int, 2> position = {distribution(generator), distribution(generator)};
	std::array<int, 2> dimensions = {distribution(generator), distribution(generator)};

	/// Set position and check that it set correctly
	view->setPosition(position);

	auto test = view->getPosition();

	EXPECT_EQ(position, test);

	/// Set dimensions and check that it set correctly
	view->setDimensions(dimensions);

	test = view->getDimensions();

	EXPECT_EQ(dimensions, test);

	/// The window border should be enabled initially
	EXPECT_TRUE(view->isWindowBorderEnabled());
	/// Disable the window border
	view->setWindowBorderEnabled(false);
	EXPECT_FALSE(view->isWindowBorderEnabled());
}

TEST(OsgViewTests, CameraTest)
{
	std::shared_ptr<View> view = std::make_shared<OsgView>("test name");

	std::shared_ptr<Camera> camera = std::make_shared<OsgCamera>("test camera");

	/// Set the camera and check that it set correctly
	EXPECT_NO_THROW(view->setCamera(camera));
	EXPECT_EQ(camera, view->getCamera());

	std::shared_ptr<Camera> mockCamera = std::make_shared<MockCamera>("non-osg camera");

	/// Try to set a camera that does not derive from OsgCamera
	EXPECT_ANY_THROW(view->setCamera(mockCamera));
	EXPECT_EQ(camera, view->getCamera());
}

}  // namespace Graphics
}  // namespace SurgSim
