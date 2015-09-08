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
#include "SurgSim/Graphics/OsgProgram.h"
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
	auto defaultCamera = viewElement->getCamera();
	auto renderPass = std::make_shared<OsgCamera>("RenderPass");

	renderPass->setProjectionMatrix(defaultCamera->getProjectionMatrix());
	renderPass->setRenderGroupReference("RenderPass");
	renderPass->setGroupReference(SurgSim::Graphics::Representation::DefaultGroupName);

	std::array<int, 2> dimensions = viewElement->getView()->getDimensions();

	auto renderTargetOsg = std::make_shared<OsgRenderTarget2d>(dimensions[0], dimensions[1], 1.0, 2, true);
	renderPass->setRenderTarget(renderTargetOsg);
	renderPass->setRenderOrder(Camera::RENDER_ORDER_PRE_RENDER, 0);

	auto program = std::make_shared<OsgProgram>();
	program->setFragmentShaderSource(fragmentShaderSource);
	program->setVertexShaderSource(vertexShaderSource);

	auto material1 = std::make_shared<OsgMaterial>("material1");
	auto material2 = std::make_shared<OsgMaterial>("material2");

	material1->setProgram(program);
	material2->setProgram(program);

	viewElement->addComponent(material1);
	viewElement->addComponent(material2);

	renderPass->setMaterial(material2);


	auto uniform = std::make_shared<OsgUniform<Vector3f>>("ambientColor");
	uniform->set(Vector3f(0.2, 0.2, 0.2));
	material1->addUniform(uniform);

	uniform = std::make_shared<OsgUniform<Vector3f>>("otherColor");
	uniform->set(Vector3f(1.0, 0.0, 0.0));
	material1->addUniform(uniform);

	uniform = std::make_shared<OsgUniform<Vector3f>>("otherColor");
	uniform->set(Vector3f(0.0, 1.0, 0.0));
	material2->addUniform(uniform);


	viewElement->addComponent(renderPass);

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
	box->setMaterial(material1);

	auto boxElement2 = std::make_shared<BasicSceneElement>("Box 2");
	boxElement2->addComponent(box);
	boxElement2->setPose(startPose);
	scene->addSceneElement(boxElement2);

	/// Run the thread
	runtime->start();

	int numSteps = 100;
	boost::this_thread::sleep(boost::posix_time::milliseconds(500));
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


