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

#include <gtest/gtest.h>

#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"

#include "SurgSim/Graphics/Material.h"
#include "SurgSim/Graphics/OsgBoxRepresentation.h"
#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Graphics/OsgGroup.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgRenderTarget.h"
#include "SurgSim/Graphics/OsgScreenSpaceQuadRepresentation.h"
#include "SurgSim/Graphics/OsgTexture.h"
#include "SurgSim/Graphics/OsgTexture2d.h"
#include "SurgSim/Graphics/OsgTextureRectangle.h"
#include "SurgSim/Graphics/OsgUniform.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Graphics/RenderTests/RenderTest.h"
#include "SurgSim/Graphics/View.h"

#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"

#include "SurgSim/Testing/MathUtilities.h"

using SurgSim::Framework::BasicSceneElement;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;

namespace SurgSim
{
namespace Graphics
{

struct OsgScreenSpaceQuadRenderTests : public RenderTest
{

};


TEST_F(OsgScreenSpaceQuadRenderTests, InitTest)
{
	std::shared_ptr<OsgScreenSpaceQuadRepresentation> quad =
		std::make_shared<OsgScreenSpaceQuadRepresentation>("Screen Quad");

	viewElement->addComponent(quad);

	/// Run the thread
	runtime->start();
	EXPECT_TRUE(graphicsManager->isInitialized());
	EXPECT_TRUE(viewElement->isInitialized());
	quad->setSize(100, 100);
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	auto dimensions = viewElement->getView()->getDimensions();

	SurgSim::Math::Vector3d startPosition(0.0, 0.0, 0.0);
	SurgSim::Math::Vector3d endPosition(dimensions[0], dimensions[1], 0.0);

	SurgSim::Math::Vector2d startSize(0.0, 0.0);
	SurgSim::Math::Vector2d endSize(200, 200);

	int numSteps = 100;
	for (int i = 0; i < numSteps; ++i)
	{
		/// Calculate t in [0.0, 1.0]
		double t = static_cast<double>(i) / numSteps;
		RigidTransform3d currentPose = SurgSim::Testing::interpolatePose(
										   Vector3d::Identity(), Vector3d::Identity(),
										   startPosition, endPosition, t);

		quad->setLocalPose(currentPose);

		SurgSim::Math::Vector2d size = SurgSim::Testing::interpolate(startSize, endSize, t);
		quad->setSize(size.x(), size.y());

		boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / numSteps));
	}

}

TEST_F(OsgScreenSpaceQuadRenderTests, TextureTest)
{
	std::string checkerTexturePath = applicationData->findFile("Textures/CheckerBoard.png");
	std::string rectangleTexturePath = applicationData->findFile("Textures/Rectangle.png");

	EXPECT_NE("", checkerTexturePath) << "Could not find checker texture shader!";
	EXPECT_NE("", rectangleTexturePath) << "Could not find rectangle texture!";

	std::shared_ptr<OsgTexture2d> checkerTexture = std::make_shared<OsgTexture2d>();
	EXPECT_TRUE(checkerTexture->loadImage(checkerTexturePath));

	std::shared_ptr<OsgTextureRectangle> rectTexture = std::make_shared<OsgTextureRectangle>();
	EXPECT_TRUE(rectTexture->loadImage(rectangleTexturePath));


	std::shared_ptr<OsgScreenSpaceQuadRepresentation> quad1 =
		std::make_shared<OsgScreenSpaceQuadRepresentation>("Screen Quad 1");

	auto dimensions = viewElement->getView()->getDimensions();

	quad1->setSize(256, 256);
	quad1->setLocalPose(SurgSim::Math::makeRigidTransform(
							Quaterniond::Identity(),
							Vector3d(dimensions[0] - 256, dimensions[1] - 256, -0.2)));
	EXPECT_TRUE(quad1->setTexture(checkerTexture));
	viewElement->addComponent(quad1);


	std::shared_ptr<OsgScreenSpaceQuadRepresentation> quad2 =
		std::make_shared<OsgScreenSpaceQuadRepresentation>("Screen Quad 2");

	int width, height;
	rectTexture->getSize(&width, &height);
	EXPECT_TRUE(quad2->setTexture(rectTexture));
	quad2->setSize(width, height);
	viewElement->addComponent(quad2);

	/// Run the thread
	runtime->start();
	EXPECT_TRUE(graphicsManager->isInitialized());
	EXPECT_TRUE(viewElement->isInitialized());

	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
}



// Should show two rotating cubes, one in the middle of the screen being rendered normally, the
// other one in the top right hand corner, being rendered onto a texture mapped on a quad
TEST_F(OsgScreenSpaceQuadRenderTests, RenderTextureTest)
{

	auto defaultCamera = viewElement->getCamera();
	auto camera = std::make_shared<OsgCamera>("RenderPass");
	camera->setProjectionMatrix(defaultCamera->getProjectionMatrix());
	camera->setRenderGroupReference("RenderPass");
	camera->setGroupReference(SurgSim::Graphics::Representation::DefaultGroupName);

	auto dimensions = viewElement->getView()->getDimensions();

	std::shared_ptr<OsgRenderTarget2d> renderTargetOsg =
		std::make_shared<OsgRenderTarget2d>(dimensions[0], dimensions[1], 1.0, 2, true);
	camera->setRenderTarget(renderTargetOsg);

	viewElement->addComponent(camera);

	int screenWidth = dimensions[0];
	int screenHeight = dimensions[1];

	int width = dimensions[0] / 3;
	int height = dimensions[1] / 3;

	std::shared_ptr<ScreenSpaceQuadRepresentation> quad;
	quad = makeQuad("Color1", width, height, screenWidth - width, screenHeight - height);
	quad->setTexture(renderTargetOsg->getColorTargetOsg(0));
	viewElement->addComponent(quad);

	quad = makeQuad("Color2", width, height, screenWidth - width, screenHeight - height * 2);
	quad->setTexture(renderTargetOsg->getColorTargetOsg(1));
	viewElement->addComponent(quad);

	quad = makeQuad("Depth", width, height, 0.0, screenHeight - height);
	quad->setTexture(renderTargetOsg->getDepthTargetOsg());
	viewElement->addComponent(quad);

	Quaterniond quat = Quaterniond::Identity();
	RigidTransform3d startPose = SurgSim::Math::makeRigidTransform(quat, Vector3d(0.0, 0.0, -0.2));
	quat = SurgSim::Math::makeRotationQuaternion(M_PI, Vector3d::UnitY().eval());
	RigidTransform3d endPose = SurgSim::Math::makeRigidTransform(quat, Vector3d(0.0, 0.0, -0.2));

	auto box = std::make_shared<OsgBoxRepresentation>("Graphics");
	box->setSizeXYZ(0.05, 0.05, 0.05);
	box->setGroupReference("RenderPass");
	auto boxElement1 = std::make_shared<BasicSceneElement>("Box 1");
	boxElement1->addComponent(box);
	boxElement1->setPose(startPose);
	scene->addSceneElement(boxElement1);

	box = std::make_shared<OsgBoxRepresentation>("Graphics");
	box->setSizeXYZ(0.05, 0.05, 0.05);
	auto boxElement2 = std::make_shared<BasicSceneElement>("Box 2");
	boxElement2->addComponent(box);
	boxElement2->setPose(startPose);
	scene->addSceneElement(boxElement2);

	/// Run the thread
	runtime->start();

	int numSteps = 100;
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	for (int i = 0; i < numSteps; ++i)
	{
		double t = static_cast<double>(i) / numSteps;
		boxElement1->setPose(SurgSim::Testing::interpolate<RigidTransform3d>(startPose, endPose, t));
		boxElement2->setPose(SurgSim::Testing::interpolate<RigidTransform3d>(endPose, startPose, t));
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / 100));
	}

	graphicsManager->dumpDebugInfo();
}

}; // namespace Graphics
}; // namespace SurgSim
