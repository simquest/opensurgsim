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
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/RigidTransform.h>

#include <SurgSim/Framework/ApplicationData.h>
#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Framework/Scene.h>
#include <SurgSim/Graphics/OsgUniform.h>
#include <SurgSim/Graphics/Material.h>
#include <SurgSim/Graphics/OsgCamera.h>
#include <SurgSim/Graphics/OsgGroup.h>
#include <SurgSim/Graphics/OsgManager.h>
#include <SurgSim/Graphics/OsgViewElement.h>
#include <SurgSim/Graphics/OsgTexture2d.h>
#include <SurgSim/Graphics/OsgTextureRectangle.h>
#include <SurgSim/Graphics/View.h>
#include <SurgSim/Graphics/OsgBoxRepresentation.h>
#include <SurgSim/Graphics/OsgMaterial.h>
#include <SurgSim/Graphics/OsgRenderTarget.h>

#include <SurgSim/Testing/MathUtilities.h>

#include <SurgSim/Graphics/OsgScreenSpaceQuadRepresentation.h>

using SurgSim::Math::Vector3d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;

namespace SurgSim
{
namespace Graphics
{

struct OsgScreenSpaceQuadRenderTests : public ::testing::Test
{
	virtual void SetUp()
	{
		runtime = std::make_shared<SurgSim::Framework::Runtime>();
		graphicsManager = std::make_shared<SurgSim::Graphics::OsgManager>();

		runtime->addManager(graphicsManager);

		scene = std::make_shared<SurgSim::Framework::Scene>();
		runtime->setScene(scene);

		viewElement = std::make_shared<OsgViewElement>("view element");
		viewElement->getView()->setPosition(100,100);
		viewElement->getView()->setWindowBorderEnabled(true);
		scene->addSceneElement(viewElement);

	}

	virtual void TearDown()
	{
		runtime->stop();
	}

	std::shared_ptr<ScreenSpaceQuadRepresentation> makeQuad(
		const std::string& name,
		int width,
		int height,
		int x,
		int y)
	{
		std::shared_ptr<OsgScreenSpaceQuadRepresentation> quad =
			std::make_shared<OsgScreenSpaceQuadRepresentation>(name, viewElement->getView());
		quad->setSize(width,height);
		Quaterniond quat;
		quat = SurgSim::Math::makeRotationQuaternion<double,Eigen::DontAlign>(0.0,Vector3d::UnitY());
		quad->setInitialPose(SurgSim::Math::makeRigidTransform(quat, Vector3d(x,y,-0.2)));
		return quad;
	}

	std::shared_ptr<SurgSim::Framework::Runtime> runtime;
	std::shared_ptr<OsgManager> graphicsManager;
	std::shared_ptr<SurgSim::Framework::Scene> scene;
	std::shared_ptr<OsgViewElement> viewElement;

protected:


};


TEST_F(OsgScreenSpaceQuadRenderTests, InitTest)
{
	std::shared_ptr<OsgScreenSpaceQuadRepresentation> quad =
		std::make_shared<OsgScreenSpaceQuadRepresentation>("Screen Quad", viewElement->getView());

	viewElement->addComponent(quad);

	/// Run the thread
	runtime->start();
	EXPECT_TRUE(graphicsManager->isInitialized());
	EXPECT_TRUE(viewElement->isInitialized());
	quad->setSize(100,100);
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	int width,height;
	viewElement->getView()->getDimensions(&width, &height);


	SurgSim::Math::Vector3d startPosition(0.0,0.0,0.0);
	SurgSim::Math::Vector3d endPosition(width,height,0.0);

	SurgSim::Math::Vector2d startSize(0.0,0.0);
	SurgSim::Math::Vector2d endSize(200,200);

	int numSteps = 100;
	for (int i = 0; i < numSteps; ++i)
	{
		/// Calculate t in [0.0, 1.0]
		double t = static_cast<double>(i) / numSteps;
		RigidTransform3d currentPose = SurgSim::Testing::interpolatePose(
			Vector3d::Identity(), Vector3d::Identity(),
			startPosition, endPosition, t);

		quad->setPose(currentPose);

		SurgSim::Math::Vector2d size = SurgSim::Testing::interpolate(startSize,endSize,t);
		quad->setSize(size.x(), size.y());

		boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / numSteps));
	}

}

TEST_F(OsgScreenSpaceQuadRenderTests, TextureTest)
{
	std::vector<std::string> paths;
	paths.push_back("Data/OsgScreenSpaceQuadRenderTests");
	SurgSim::Framework::ApplicationData data(paths);

	std::string checkerTexturePath = data.findFile("CheckerBoard.png");
	std::string rectangleTexturePath = data.findFile("Rectangle.png");

	EXPECT_NE("", checkerTexturePath) << "Could not find checker texture shader!";
	EXPECT_NE("", rectangleTexturePath) << "Could not find rectangle texture!";

	std::shared_ptr<OsgTexture2d> checkerTexture = std::make_shared<OsgTexture2d>();
	EXPECT_TRUE(checkerTexture->loadImage(checkerTexturePath));

	std::shared_ptr<OsgTextureRectangle> rectTexture = std::make_shared<OsgTextureRectangle>();
	EXPECT_TRUE(rectTexture->loadImage(rectangleTexturePath));


	std::shared_ptr<OsgScreenSpaceQuadRepresentation> quad1 =
		std::make_shared<OsgScreenSpaceQuadRepresentation>("Screen Quad 1", viewElement->getView());

	quad1->setSize(256,256);
	quad1->setInitialPose(SurgSim::Math::makeRigidTransform(
		Quaterniond::Identity(),
		Vector3d(800-256,600-256,-0.2)));
	EXPECT_TRUE(quad1->setTexture(checkerTexture));
	viewElement->addComponent(quad1);


	std::shared_ptr<OsgScreenSpaceQuadRepresentation> quad2 =
		std::make_shared<OsgScreenSpaceQuadRepresentation>("Screen Quad 2", viewElement->getView());

	int width, height;
	rectTexture->getSize(&width, &height);
	EXPECT_TRUE(quad2->setTexture(rectTexture));
	quad2->setSize(width,height);
	viewElement->addComponent(quad2);

	/// Run the thread
	runtime->start();
	EXPECT_TRUE(graphicsManager->isInitialized());
	EXPECT_TRUE(viewElement->isInitialized());

	boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
}



// Should show two rotating cubes, one in the middle of the screen being rendered normally, the
// other one in the top right hand corner, being rendered onto a texture mapped on a quad
TEST_F(OsgScreenSpaceQuadRenderTests, RenderTextureTest)
{

	auto defaultCamera = graphicsManager->getDefaultCamera();
	auto camera = std::make_shared<OsgCamera>("Texture");
	camera->setViewMatrix(defaultCamera->getViewMatrix());
	camera->setProjectionMatrix(defaultCamera->getProjectionMatrix());

	int width, height;
	viewElement->getView()->getDimensions(&width,&height);


	std::shared_ptr<OsgRenderTarget2d> renderTargetOsg =
		std::make_shared<OsgRenderTarget2d>(width,height, 1.0, 2, true);

	camera->setRenderTarget(renderTargetOsg);

	viewElement->addComponent(camera);

	int screenWidth = 800;
	int screenHeight = 600;

	width = width/3;
	height = height/3;

	std::shared_ptr<ScreenSpaceQuadRepresentation> quad;
	quad = makeQuad("Color1", width, height, screenWidth - width, screenHeight - height);
	quad->setTexture(renderTargetOsg->getColorTargetOsg(0));
	viewElement->addComponent(quad);

	quad = makeQuad("Color2", width, height, screenWidth - width, screenHeight - height*2);
	quad->setTexture(renderTargetOsg->getColorTargetOsg(1));
	viewElement->addComponent(quad);

	quad = makeQuad("Depth", width, height, 0.0, screenHeight - height);
	quad->setTexture(renderTargetOsg->getDepthTargetOsg());
	viewElement->addComponent(quad);



	Quaterniond quat = Quaterniond::Identity();
	RigidTransform3d startPose = SurgSim::Math::makeRigidTransform(quat,Vector3d(0.0,0.0,-0.2));
	quat = SurgSim::Math::makeRotationQuaternion<double,Eigen::DontAlign>(M_PI,Vector3d::UnitY());
	RigidTransform3d endPose = SurgSim::Math::makeRigidTransform(quat, Vector3d(0.0,0.0,-0.2));

	auto boxRepresentation1 = std::make_shared<OsgBoxRepresentation>("Box Representation");
	boxRepresentation1->setSize(0.05,0.05,0.05);
	boxRepresentation1->setPose(startPose);
	auto group = std::make_shared<OsgGroup>("RenderPass");
	group->add(boxRepresentation1);
	camera->setGroup(group);
	viewElement->addComponent(group);

	auto boxRepresentation = std::make_shared<OsgBoxRepresentation>("Box Representation");
	boxRepresentation->setSize(0.05,0.05,0.05);
	viewElement->addComponent(boxRepresentation);

	/// Run the thread
	runtime->start();

	int numSteps = 1000;
	boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
	for (int i = 0; i < numSteps; ++i)
	{
		double t = static_cast<double>(i) / numSteps;
		boxRepresentation->setPose(SurgSim::Testing::interpolate<RigidTransform3d>(startPose, endPose, t));
		boxRepresentation1->setPose(SurgSim::Testing::interpolate<RigidTransform3d>(endPose, startPose, t));
		camera->setViewMatrix(defaultCamera->getViewMatrix());
		camera->setProjectionMatrix(defaultCamera->getProjectionMatrix());

		boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / 100));
	}
}

}; // namespace Graphics
}; // namespace SurgSim
