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

#include <SurgSim/Graphics/UnitTests/MockObjects.h>
#include <SurgSim/Graphics/UnitTests/MockOsgObjects.h>

#include <gtest/gtest.h>

#include <random>

using SurgSim::Graphics::Camera;
using SurgSim::Graphics::Manager;
using SurgSim::Graphics::View;
using SurgSim::Graphics::OsgCamera;
using SurgSim::Graphics::OsgView;

TEST(OsgViewTests, InitTest)
{
	ASSERT_NO_THROW({std::shared_ptr<View> view = std::make_shared<OsgView>("test name");});

	std::shared_ptr<View> view = std::make_shared<OsgView>("test name");

	EXPECT_EQ("test name", view->getName());

	EXPECT_EQ(nullptr, view->getCamera());

	int x, y;
	view->getPosition(&x, &y);
	EXPECT_EQ(0, x);
	EXPECT_EQ(0, y);

	int width, height;
	view->getDimensions(&width, &height);
	EXPECT_EQ(800, width);
	EXPECT_EQ(600, height);
}

TEST(OsgViewTests, PositionAndDimensionsTest)
{
	std::shared_ptr<OsgView> osgView = std::make_shared<MockOsgView>("test name");
	std::shared_ptr<View> view = std::make_shared<MockOsgView>("test name");

	std::default_random_engine generator;
	std::uniform_int_distribution<int> distribution(0, 1000);

	int x = distribution(generator);
	int y = distribution(generator);
	int width = distribution(generator);
	int height = distribution(generator);

	/// Set position and check that it set correctly
	EXPECT_TRUE(view->setPosition(x, y));

	int testX, testY;
	view->getPosition(&testX, &testY);

	EXPECT_EQ(x, testX);
	EXPECT_EQ(y, testY);

	/// Set dimensions and check that it set correctly
	EXPECT_TRUE(view->setDimensions(width, height));

	int testWidth, testHeight;
	view->getDimensions(&testWidth, &testHeight);

	EXPECT_EQ(width, testWidth);
	EXPECT_EQ(height, testHeight);
}

TEST(OsgViewTests, CameraTest)
{
	std::shared_ptr<View> view = std::make_shared<OsgView>("test name");

	std::shared_ptr<Camera> camera = std::make_shared<OsgCamera>("test camera");

	/// Set the camera and check that it set correctly
	EXPECT_TRUE(view->setCamera(camera));
	EXPECT_EQ(camera, view->getCamera());

	std::shared_ptr<Camera> mockCamera = std::make_shared<MockCamera>("non-osg camera");

	/// Try to set a camera that does not derive from OsgCamera
	EXPECT_FALSE(view->setCamera(mockCamera));
	EXPECT_EQ(camera, view->getCamera());
}

TEST(OsgViewTests, UpdateTest)
{
	std::shared_ptr<MockOsgView> mockOsgView = std::make_shared<MockOsgView>("test view");
	std::shared_ptr<View> view = mockOsgView;

	std::shared_ptr<Camera> camera = std::make_shared<OsgCamera>("test camera");
	view->setCamera(camera);

	/// Make sure that we can successfully initialize, which setups up the window
	EXPECT_TRUE(mockOsgView->initialize());

	EXPECT_EQ(0u, mockOsgView->getNumUpdates());
	EXPECT_EQ(0.0, mockOsgView->getSumDt());

	double sumDt = 0.0;
	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(0.0, 1.0);

	/// Do 10 updates with random dt and check each time that the number of updates and sum of dt are correct.
	for (int i = 1; i <= 10; ++i)
	{
		double dt = distribution(generator);
		sumDt += dt;

		view->update(dt);

		EXPECT_EQ(i, mockOsgView->getNumUpdates());
		EXPECT_EQ(sumDt, mockOsgView->getSumDt());
	}
}
