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
#include <SurgSim/Blocks/SphereElement.h>
#include <SurgSim/Graphics/OsgManager.h>
#include <SurgSim/Graphics/OsgCamera.h>
#include <SurgSim/Graphics/OsgScreenSpaceQuadRepresentation.h>
#include <SurgSim/Graphics/OsgRenderPass.h>
#include <SurgSim/Graphics/OsgRenderTarget.h>
#include <SurgSim/Graphics/OsgBoxRepresentation.h>
#include <SurgSim/Graphics/OsgGroup.h>

#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/RigidTransform.h>

#include <SurgSim/Graphics/RenderTests/RenderTest.h>
#include <SurgSim/Testing/MathUtilities.h>

using SurgSim::Math::Quaterniond;
using SurgSim::Math::Vector3d;
using SurgSim::Math::RigidTransform3d;


namespace SurgSim
{
namespace Graphics
{

struct OsgRenderPassRenderTests: public RenderTest
{

};

TEST_F(OsgRenderPassRenderTests, SimpleTest)
{
	auto defaultCamera = graphicsManager->getDefaultCamera();
	auto renderPass = std::make_shared<OsgRenderPass>("RenderPass");

	auto camera = renderPass->getCamera();
	camera->setViewMatrix(defaultCamera->getViewMatrix());
	camera->setProjectionMatrix(defaultCamera->getProjectionMatrix());

	int width, height;
	viewElement->getView()->getDimensions(&width,&height);


	std::shared_ptr<OsgRenderTarget2d> renderTargetOsg =
		std::make_shared<OsgRenderTarget2d>(width,height, 1.0, 2, true);
	renderPass->setRenderTarget(renderTargetOsg);

	viewElement->addComponent(renderPass->getCamera());

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
	renderPass->getGroup()->add(boxRepresentation1);

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


