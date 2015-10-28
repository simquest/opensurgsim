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

#include "SurgSim/Framework/FrameworkConvert.h"

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

typedef std::array<int, 2> CoordinateType;
using SurgSim::Math::Vector3d;

void expectEqual(std::shared_ptr<OsgView> expected, std::shared_ptr<OsgView> actual)
{
	if (expected->getCamera() == nullptr)
	{
		EXPECT_EQ(nullptr, actual->getCamera());
	}
	else
	{
		EXPECT_NE(nullptr, actual->getCamera());
		EXPECT_EQ(boost::any_cast<std::shared_ptr<Camera>>(expected->getValue("Camera"))->getName(),
				  boost::any_cast<std::shared_ptr<Camera>>(actual->getValue("Camera"))->getName());
	}
	EXPECT_EQ(boost::any_cast<CoordinateType>(expected->getValue("Position")),
			  boost::any_cast<CoordinateType>(actual->getValue("Position")));
	EXPECT_EQ(boost::any_cast<CoordinateType>(expected->getValue("Dimensions")),
			  boost::any_cast<CoordinateType>(actual->getValue("Dimensions")));
	EXPECT_EQ(boost::any_cast<bool>(expected->getValue("WindowBorder")),
			  boost::any_cast<bool>(actual->getValue("WindowBorder")));
	EXPECT_EQ(boost::any_cast<int>(expected->getValue("StereoMode")),
			  boost::any_cast<int>(actual->getValue("StereoMode")));
	EXPECT_EQ(boost::any_cast<int>(expected->getValue("DisplayType")),
			  boost::any_cast<int>(actual->getValue("DisplayType")));
	EXPECT_EQ(boost::any_cast<bool>(expected->getValue("FullScreen")),
			  boost::any_cast<bool>(actual->getValue("FullScreen")));
	EXPECT_EQ(boost::any_cast<int>(expected->getValue("TargetScreen")),
			  boost::any_cast<int>(actual->getValue("TargetScreen")));
	EXPECT_EQ(boost::any_cast<double>(expected->getValue("EyeSeparation")),
			  boost::any_cast<double>(actual->getValue("EyeSeparation")));
	EXPECT_EQ(boost::any_cast<double>(expected->getValue("ScreenDistance")),
			  boost::any_cast<double>(actual->getValue("ScreenDistance")));
	EXPECT_EQ(boost::any_cast<double>(expected->getValue("ScreenWidth")),
			  boost::any_cast<double>(actual->getValue("ScreenWidth")));
	EXPECT_EQ(boost::any_cast<double>(expected->getValue("ScreenHeight")),
			  boost::any_cast<double>(actual->getValue("ScreenHeight")));
	EXPECT_EQ(boost::any_cast<bool>(expected->getValue("CameraManipulatorEnabled")),
			  boost::any_cast<bool>(actual->getValue("CameraManipulatorEnabled")));
	EXPECT_TRUE(boost::any_cast<Vector3d>(expected->getValue("CameraPosition")).isApprox(
				boost::any_cast<Vector3d>(actual->getValue("CameraPosition"))));
	EXPECT_TRUE(boost::any_cast<Vector3d>(expected->getValue("CameraLookAt")).isApprox(
				boost::any_cast<Vector3d>(actual->getValue("CameraLookAt"))));
	EXPECT_EQ(boost::any_cast<bool>(expected->getValue("OsgMapUniforms")),
			  boost::any_cast<bool>(actual->getValue("OsgMapUniforms")));
	EXPECT_EQ(boost::any_cast<bool>(expected->getValue("KeyboardDeviceEnabled")),
			  boost::any_cast<bool>(actual->getValue("KeyboardDeviceEnabled")));
	EXPECT_EQ(boost::any_cast<bool>(expected->getValue("MouseDeviceEnabled")),
			  boost::any_cast<bool>(actual->getValue("MouseDeviceEnabled")));
}

TEST(OsgViewTests, Serialization)
{
	{
		SCOPED_TRACE("Serialize with default values");

		std::shared_ptr<OsgView> view = std::make_shared<OsgView>("test name");

		/// Serialize
		YAML::Node node;
		EXPECT_NO_THROW(node = YAML::convert<SurgSim::Framework::Component>::encode(*view););

		/// Deserialize
		std::shared_ptr<OsgView> newView;
		EXPECT_NO_THROW(newView = std::dynamic_pointer_cast<OsgView>(
			node.as<std::shared_ptr<SurgSim::Framework::Component>>()));
		EXPECT_NE(nullptr, newView);

		// Verify
		expectEqual(view, newView);
	}

	{
		SCOPED_TRACE("Serialize with user-defined values");

		std::shared_ptr<OsgView> view = std::make_shared<OsgView>("test name");
		std::shared_ptr<OsgCamera> camera = std::make_shared<OsgCamera>("test camera");

		CoordinateType position;
		position[0] = 100;
		position[1] = 100;
		view->setValue("Position", position);
		CoordinateType dimensions;
		dimensions[0] = 200;
		dimensions[1] = 200;
		view->setValue("Dimensions", dimensions);
		view->setValue("WindowBorder", true);
		view->setValue("Camera", std::dynamic_pointer_cast<SurgSim::Framework::Component>(camera));
		view->setValue("StereoMode", static_cast<int>(SurgSim::Graphics::View::STEREO_MODE_QUAD_BUFFER));
		view->setValue("DisplayType", static_cast<int>(SurgSim::Graphics::View::DISPLAY_TYPE_HMD));
		view->setValue("FullScreen", true);
		view->setValue("TargetScreen", 3);
		view->setValue("EyeSeparation", 0.123);
		view->setValue("ScreenDistance", 2.123);
		view->setValue("ScreenWidth", 1.1);
		view->setValue("ScreenHeight", 1.1);
		view->setValue("CameraManipulatorEnabled", true);
		view->setValue("CameraPosition", Vector3d(1.5, 1.5, 1.5));
		view->setValue("CameraLookAt", Vector3d(10.5, 10.5, 10.5));
		view->setValue("OsgMapUniforms", true);

		/// Serialize
		YAML::Node node;
		EXPECT_NO_THROW(node = YAML::convert<SurgSim::Framework::Component>::encode(*view););

		/// Deserialize
		std::shared_ptr<OsgView> newView;
		EXPECT_NO_THROW(newView = std::dynamic_pointer_cast<OsgView>(
			node.as<std::shared_ptr<SurgSim::Framework::Component>>()));
		EXPECT_NE(nullptr, newView);

		// Verify
		expectEqual(view, newView);
		YAML::Node cameraNode;
		EXPECT_NO_THROW(cameraNode = YAML::convert<SurgSim::Framework::Component>::encode(*camera););
		EXPECT_EQ(cameraNode[camera->getClassName()]["Id"].as<std::string>(),
				  node[view->getClassName()]["Camera"][camera->getClassName()]["Id"].as<std::string>());
	}
}


TEST(OsgViewTests, OnlyOneDevice)
{
	std::shared_ptr<OsgView> view1 = std::make_shared<OsgView>("One");
	std::shared_ptr<OsgView> view2 = std::make_shared<OsgView>("Two");

	view1->enableKeyboardDevice(true);
	view1->enableMouseDevice(true);
	EXPECT_ANY_THROW(view2->enableKeyboardDevice(true));
	EXPECT_ANY_THROW(view2->enableMouseDevice(true));
}

}  // namespace Graphics
}  // namespace SurgSim
