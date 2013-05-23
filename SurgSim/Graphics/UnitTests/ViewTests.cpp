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
/// Tests for the View class.

#include <SurgSim/Graphics/UnitTests/MockObjects.h>

#include <gtest/gtest.h>

#include <random>

using SurgSim::Graphics::Camera;
using SurgSim::Graphics::View;

TEST(ViewTests, InitTest)
{
	ASSERT_NO_THROW({std::shared_ptr<View> view = std::make_shared<MockView>("test name");});
}

TEST(ViewTests, NameTest)
{
	std::shared_ptr<View> view = std::make_shared<MockView>("test name");

	EXPECT_EQ("test name", view->getName());
}

TEST(ViewTests, PositionAndDimensionsTest)
{
	std::shared_ptr<View> view = std::make_shared<MockView>("test name");

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

TEST(ViewTests, CameraTest)
{
	std::shared_ptr<View> view = std::make_shared<MockView>("test name");

	std::shared_ptr<Camera> camera = std::make_shared<MockCamera>("test camera");

	/// Set the camera and check that it set correctly
	EXPECT_TRUE(view->setCamera(camera));

	EXPECT_EQ(camera, view->getCamera());
}

TEST(ViewTests, UpdateTest)
{
	std::shared_ptr<MockView> mockView = std::make_shared<MockView>("test name");
	std::shared_ptr<View> view = mockView;

	EXPECT_EQ(0, mockView->getNumUpdates());
	EXPECT_EQ(0.0, mockView->getSumDt());

	double sumDt = 0.0;
	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(0.0, 1.0);

	/// Do 10 updates with random dt and check each time that the number of updates and sum of dt are correct.
	for (int i = 1; i <= 10; ++i)
	{
		double dt = distribution(generator);
		sumDt += dt;

		view->update(dt);
		EXPECT_EQ(i, mockView->getNumUpdates());
		EXPECT_LT(fabs(sumDt - mockView->getSumDt()), Eigen::NumTraits<double>::dummy_precision());
	}
}
