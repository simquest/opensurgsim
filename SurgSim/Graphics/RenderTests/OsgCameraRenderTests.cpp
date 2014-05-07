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
#include "SurgSim/Blocks/SphereElement.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Graphics/OsgScreenSpaceQuadRepresentation.h"
#include "SurgSim/Graphics/OsgRenderTarget.h"
#include "SurgSim/Graphics/OsgBoxRepresentation.h"
#include "SurgSim/Graphics/OsgGroup.h"
#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgShader.h"
#include "SurgSim/Graphics/OsgUniform.h"

#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/RigidTransform.h"

#include "SurgSim/Graphics/RenderTests/RenderTest.h"
#include "SurgSim/Testing/MathUtilities.h"

using SurgSim::Framework::BasicSceneElement;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector3f;
using SurgSim::Math::RigidTransform3d;

std::string vertexShaderSource =
	"uniform vec3 ambientColor;\n"
	"uniform vec3 otherColor;\n"
	"varying vec4 color;\n"
	"void main(void)\n"
	"{\n"
	"   vec3 lightVector =  normalize(vec3(1.0,-1.0,1.0));\n"
	"	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;\n"
// This is not a realy lighting calculation ... just faking something ...
	"	color.rgb = otherColor * max(dot(gl_Normal, lightVector), 0.0) + ambientColor;\n"
	"	color.a = 1.0;\n"
	"}";
std::string fragmentShaderSource =
	"varying vec4 color;\n"
	"void main(void)\n"
	"{\n"
	"	gl_FragColor = color;\n"
	"}";


namespace SurgSim
{
namespace Graphics
{

struct OsgCameraRenderTests: public RenderTest
{

};

TEST_F(OsgCameraRenderTests, PassTest)
{
	auto camera = viewElement->getCamera();
	auto renderPass = std::make_shared<SurgSim::Graphics::OsgCamera>("RenderPass");
	renderPass->setGroup(std::make_shared<SurgSim::Graphics::OsgGroup>("RenderPass Group"));
	renderPass ->setProjectionMatrix(camera->getProjectionMatrix());

	int width, height;
	viewElement->getView()->getDimensions(&width,&height);

	std::shared_ptr<OsgRenderTarget2d> renderTargetOsg =
		std::make_shared<OsgRenderTarget2d>(width,height, 1.0, 2, true);
	renderPass->setRenderTarget(renderTargetOsg);
	renderPass->setRenderOrder(Camera::RENDER_ORDER_PRE_RENDER, 0);

	auto shader = std::make_shared<OsgShader>();
	shader->setFragmentShaderSource(fragmentShaderSource);
	shader->setVertexShaderSource(vertexShaderSource);

	auto material1 = std::make_shared<OsgMaterial>();
	auto material2 = std::make_shared<OsgMaterial>();

	material1->setShader(shader);
	material2->setShader(shader);

	renderPass->setMaterial(material2);


	auto uniform = std::make_shared<OsgUniform<Vector3f>>("ambientColor");
	uniform->set(Vector3f(0.2,0.2,0.2));
	material1->addUniform(uniform);

	uniform = std::make_shared<OsgUniform<Vector3f>>("otherColor");
	uniform->set(Vector3f(1.0,0.0,0.0));
	material1->addUniform(uniform);

	uniform = std::make_shared<OsgUniform<Vector3f>>("otherColor");
	uniform->set(Vector3f(0.0,1.0,0.0));
	material2->addUniform(uniform);


	viewElement->addComponent(renderPass);

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
	RigidTransform3d startPose = SurgSim::Math::makeRigidTransform(quat,Vector3d(0.0, 0.0, -0.2));
	quat = SurgSim::Math::makeRotationQuaternion<double>(M_PI,Vector3d::UnitY());
	RigidTransform3d endPose = SurgSim::Math::makeRigidTransform(quat, Vector3d(0.0, 0.0, -0.2));

	auto boxRepresentation1 = std::make_shared<OsgBoxRepresentation>("Box Representation 1");
	boxRepresentation1->setSizeXYZ(0.05, 0.05, 0.05);
	renderPass->getGroup()->add(boxRepresentation1);
	auto boxElement1 = std::make_shared<BasicSceneElement>("Box Element 1");
	boxElement1->addComponent(boxRepresentation1);
	boxElement1->setPose(startPose);
	scene->addSceneElement(boxElement1);

	auto boxRepresentation2 = std::make_shared<OsgBoxRepresentation>("Box Representation 2");
	boxRepresentation2->setSizeXYZ(0.05, 0.05, 0.05);
	boxRepresentation2->setMaterial(material1);
	auto boxElement2 = std::make_shared<BasicSceneElement>("Box Element 2");
	boxElement2->addComponent(boxRepresentation2);
	boxElement2->setPose(startPose);
	scene->addSceneElement(boxElement2);

	/// Run the thread
	runtime->start();

	int numSteps = 1000;
	boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
	for (int i = 0; i < numSteps; ++i)
	{
		double t = static_cast<double>(i) / numSteps;
		boxElement1->setPose(SurgSim::Testing::interpolate<RigidTransform3d>(endPose, startPose, t));
		boxElement2->setPose(SurgSim::Testing::interpolate<RigidTransform3d>(startPose, endPose, t));
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / 100));
	}
}

}; // namespace Graphics
}; // namespace SurgSim


