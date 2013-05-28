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

#include <SurgSim/Graphics/OsgManager.h>
#include <SurgSim/Graphics/OsgView.h>
#include <SurgSim/Graphics/OsgViewElement.h>

#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Framework/Scene.h>

#include <gtest/gtest.h>

using SurgSim::Framework::Runtime;
using SurgSim::Framework::Scene;
using SurgSim::Graphics::OsgManager;
using SurgSim::Graphics::OsgView;
using SurgSim::Graphics::OsgViewElement;
using SurgSim::Graphics::View;
using SurgSim::Graphics::ViewElement;

TEST(OsgViewElementTests, InitTest)
{
	ASSERT_NO_THROW({std::shared_ptr<ViewElement> viewElement = std::make_shared<OsgViewElement>("test name");});

	std::shared_ptr<ViewElement> viewElement = std::make_shared<OsgViewElement>("test name");

	std::shared_ptr<OsgView> osgView = std::dynamic_pointer_cast<OsgView>(viewElement->getView());
	EXPECT_NE(nullptr, osgView);
}

TEST(OsgViewElementTests, StartUpTest)
{
	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();
	std::shared_ptr<OsgManager> manager = std::make_shared<OsgManager>();

	runtime->addManager(manager);

	std::shared_ptr<Scene> scene = std::make_shared<Scene>();
	runtime->setScene(scene);

	/// Add a graphics component to the scene
	std::shared_ptr<OsgViewElement> viewElement = std::make_shared<OsgViewElement>("test element");
	scene->addSceneElement(viewElement);

	/// Run the thread
	runtime->start();
	EXPECT_TRUE(manager->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));

	/// Get the GraphicsWindow to check that it is updated correctly
	std::shared_ptr<OsgView> osgView = std::dynamic_pointer_cast<OsgView>(viewElement->getView());
	ASSERT_NE(nullptr, osgView);
	osgViewer::GraphicsWindow* osgWindow = dynamic_cast<osgViewer::GraphicsWindow*>(
		osgView->getOsgView()->getCamera()->getGraphicsContext());
	ASSERT_NE(nullptr, osgWindow);
	int testX, testY, testWidth, testHeight;

	/// Window should initially be at (0, 0) and 800 x 600
	osgWindow->getWindowRectangle(testX, testY, testWidth, testHeight);
	EXPECT_EQ(0, testX);
	EXPECT_EQ(0, testY);
	EXPECT_EQ(800, testWidth);
	EXPECT_EQ(600, testHeight);

	/// Move the window to (100, 200)
	viewElement->getView()->setPosition(100, 200);
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));

	/// Check that the window position was updated
	osgWindow->getWindowRectangle(testX, testY, testWidth, testHeight);
	EXPECT_EQ(100, testX);
	EXPECT_EQ(200, testY);
	EXPECT_EQ(800, testWidth);
	EXPECT_EQ(600, testHeight);

	/// Resize the window to 400 x 800
	viewElement->getView()->setDimensions(400, 500);
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));

	/// Check that the window dimensions were updated
	osgWindow->getWindowRectangle(testX, testY, testWidth, testHeight);
	EXPECT_EQ(100, testX);
	EXPECT_EQ(200, testY);
	EXPECT_EQ(400, testWidth);
	EXPECT_EQ(500, testHeight);

	runtime->stop();
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
