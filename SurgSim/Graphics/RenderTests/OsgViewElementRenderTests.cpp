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
/// Render Tests for the OsgViewElement class.

#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgView.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Graphics/OsgBoxRepresentation.h"
#include "SurgSim/Blocks/PoseInterpolator.h"

#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/BehaviorManager.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"

#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"

#include <gtest/gtest.h>

using SurgSim::Framework::Runtime;
using SurgSim::Framework::Scene;

using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::makeRigidTransform;

namespace SurgSim
{
namespace Graphics
{

// Note: This tests the ViewElement, don't derive from RenderTest


TEST(OsgViewElementRenderTests, MoveAndResizeWindowTest)
{
	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();
	std::shared_ptr<OsgManager> manager = std::make_shared<OsgManager>();

	runtime->addManager(manager);

	std::shared_ptr<Scene> scene = runtime->getScene();

	/// Add a graphics component to the scene
	std::shared_ptr<OsgViewElement> viewElement = std::make_shared<OsgViewElement>("test element");
	scene->addSceneElement(viewElement);

	/// Set initial position to (50, 60), dimensions to 200 x 100 and disable the window border

	std::array<int, 2> position = {50, 60};
	std::array<int, 2> dimensions = {200, 100};

	viewElement->getView()->setPosition(position);
	viewElement->getView()->setDimensions(dimensions);
	viewElement->getView()->setWindowBorderEnabled(false);

	/// Run the thread
	runtime->start();
	EXPECT_TRUE(manager->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	/// Move the window to (100, 200)
	position[0] = 100;
	position[1] = 200;

	viewElement->getView()->setPosition(position);
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	/// Enable the window border and resize the window to 400 x 500
	viewElement->getView()->setWindowBorderEnabled(true);
	dimensions[0] = 400;
	dimensions[1] = 500;
	viewElement->getView()->setDimensions(dimensions);
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	runtime->stop();
}

// Just an empty screen but it should be fullscreen
TEST(OsgViewElementRenderTest, FullScreenView)
{
	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();
	std::shared_ptr<OsgManager> manager = std::make_shared<OsgManager>();

	runtime->addManager(manager);

	std::shared_ptr<Scene> scene = runtime->getScene();

	/// Add a graphics component to the scene
	std::shared_ptr<OsgViewElement> viewElement = std::make_shared<OsgViewElement>("test element");
	scene->addSceneElement(viewElement);

	auto osgView = std::static_pointer_cast<OsgView>(viewElement->getView());
	std::array<int, 2> dimensions = { 0, 0 };
	osgView->setDimensions(dimensions);
	EXPECT_EQ(0, osgView->getDimensions()[0]);
	EXPECT_EQ(0, osgView->getDimensions()[1]);
	osgView->setFullScreen(true);
	// The actual value depends on the system, it's hard to test for it here
	EXPECT_NE(0, osgView->getDimensions()[0]);
	EXPECT_NE(0, osgView->getDimensions()[1]);


	runtime->start();
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	runtime->stop();

}

TEST(OsgViewElementRenderTest, StereoView)
{
	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();
	std::shared_ptr<OsgManager> manager = std::make_shared<OsgManager>();

	runtime->addManager(manager);
	runtime->addManager(std::make_shared<SurgSim::Framework::BehaviorManager>());

	std::shared_ptr<Scene> scene = runtime->getScene();

	/// Add a graphics component to the scene
	std::shared_ptr<OsgViewElement> viewElement = std::make_shared<OsgViewElement>("view");

	auto boxElement = std::make_shared<SurgSim::Framework::BasicSceneElement>("box");

	RigidTransform3d pose =
		makeRigidTransform(Vector3d(1.0, 1.0, 1.0), Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 1.0, 0.0));
	viewElement->setPose(pose);
	scene->addSceneElement(viewElement);

	auto box = std::make_shared<OsgBoxRepresentation>("box");
	box->setSizeXYZ(0.1, 0.1, 0.2);
	boxElement->addComponent(box);

	RigidTransform3d from =
		makeRigidTransform(Vector3d(0.2, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 1.0, 0.0));
	RigidTransform3d to =
		makeRigidTransform(Vector3d(-0.2, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 1.0, 0.0));
	auto interpolator = std::make_shared<SurgSim::Blocks::PoseInterpolator>("interpolator");

	interpolator->setDuration(2.0);
	interpolator->setStartingPose(from);
	interpolator->setEndingPose(to);
	interpolator->setPingPong(true);
	interpolator->setTarget(boxElement);

	boxElement->addComponent(interpolator);

	scene->addSceneElement(boxElement);

	auto osgView = std::static_pointer_cast<OsgView>(viewElement->getView());
	osgView->setStereoMode(View::STEREO_MODE_HORIZONTAL_SPLIT);
	osgView->setDisplayType(View::DISPLAY_TYPE_MONITOR);
	osgView->setEyeSeparation(0.06);
	osgView->setScreenWidth(0.486918);
	osgView->setScreenHeight(0.273812);
	osgView->setScreenDistance(1.0);


	runtime->start();
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	runtime->stop();
}

// Verify a bug fix on the OsgView that had to do with the deallocation of the mouse and
// keyboard handler.
TEST(OsgViewElementRenderTest, KeyBoardMouseTest)
{
	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();
	std::shared_ptr<OsgManager> manager = std::make_shared<OsgManager>();

	runtime->addManager(manager);
	runtime->addManager(std::make_shared<SurgSim::Framework::BehaviorManager>());

	std::shared_ptr<Scene> scene = runtime->getScene();

	/// Add a graphics component to the scene
	std::shared_ptr<OsgViewElement> viewElement = std::make_shared<OsgViewElement>("view");
	scene->addSceneElement(viewElement);

	auto osgView = std::static_pointer_cast<OsgView>(viewElement->getView());
	osgView->enableKeyboardDevice(true);
	osgView->enableMouseDevice(true);

	runtime->start();
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	runtime->stop();
}

}  // namespace Graphics
}  // namespace SurgSim
