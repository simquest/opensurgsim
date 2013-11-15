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

#include <memory>

#include <SurgSim/Physics/PhysicsManager.h>
#include <SurgSim/Framework/ApplicationData.h>
#include <SurgSim/Framework/BehaviorManager.h>
#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Framework/Scene.h>
#include <SurgSim/Framework/Log.h>

#include <SurgSim/Graphics/Light.h>
#include <SurgSim/Graphics/OsgBoxRepresentation.h>
#include <SurgSim/Graphics/OsgCamera.h>
#include <SurgSim/Graphics/OsgGroup.h>
#include <SurgSim/Graphics/OsgLight.h>
#include <SurgSim/Graphics/OsgManager.h>
#include <SurgSim/Graphics/OsgMaterial.h>
#include <SurgSim/Graphics/OsgRenderTarget.h>
#include <SurgSim/Graphics/OsgShader.h>
#include <SurgSim/Graphics/OsgUniform.h>
#include <SurgSim/Graphics/OsgTexture2d.h>
#include <SurgSim/Graphics/OsgView.h>
#include <SurgSim/Graphics/OsgViewElement.h>
#include <SurgSim/Graphics/RenderPass.h>

#include <SurgSim/Blocks/BasicSceneElement.h>

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>

using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4d;
using SurgSim::Math::Matrix44f;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Framework::Logger;

using SurgSim::Graphics::OsgUniform;
using SurgSim::Graphics::OsgTexture2d;

#include <osg/Matrix>
#include <osg/Camera>

namespace
{

std::unordered_map<std::string, std::shared_ptr<SurgSim::Graphics::OsgMaterial>> materials;

std::shared_ptr<SurgSim::Graphics::OsgMaterial> loadMaterial(
	const SurgSim::Framework::ApplicationData& data,
	const std::string& name)
{
	std::string vertexShaderName = name+".vert";
	std::string fragmentShaderName = name+".frag";

	std::string filename;

	auto shader(std::make_shared<SurgSim::Graphics::OsgShader>());
	bool success = true;
	filename = data.findFile(vertexShaderName);
	if (filename == "")
	{
		SURGSIM_LOG_WARNING(Logger::getDefaultLogger()) << "Could not find vertex shader " << vertexShaderName;
		return nullptr;
	}
	success = success && shader->loadVertexShaderSource(filename);

	filename = data.findFile(fragmentShaderName);
	if (filename == "")
	{
		SURGSIM_LOG_WARNING(Logger::getDefaultLogger()) << "Could not find fragment shader " << fragmentShaderName;
		return nullptr;
	}
	success = success && shader->loadFragmentShaderSource(filename);

	std::shared_ptr<SurgSim::Graphics::OsgMaterial> material;
	if (success)
	{
		material = std::make_shared<SurgSim::Graphics::OsgMaterial>();
		material->setShader(shader);
	}
	return material;

}

std::shared_ptr<SurgSim::Graphics::ViewElement> createView(const std::string& name, int x, int y, int width, int height)
{
	using SurgSim::Graphics::OsgViewElement;

	std::shared_ptr<OsgViewElement> viewElement = std::make_shared<OsgViewElement>(name);
	viewElement->getView()->setPosition(x, y);
	viewElement->getView()->setDimensions(width, height);
	viewElement->getView();
	viewElement->enableManipulator(true);
	viewElement->setManipulatorParameters(SurgSim::Math::Vector3d(3.0,3.0,3.0), SurgSim::Math::Vector3d(0.0,0.0,0.0));

	auto light = std::make_shared<SurgSim::Graphics::OsgLight>("Main Light");
	light->setAmbientColor(Vector4d(0.5,0.5,0.5,1.0));
	light->setDiffuseColor(Vector4d(0.5,0.5,0.5,1.0));
	light->setSpecularColor(Vector4d(0.8,0.8,0.8,1.0));
	light->setInitialPose(SurgSim::Math::makeRigidTransform(Quaterniond::Identity(),Vector3d(10.0,10.0,10.0)));

	viewElement->addComponent(light);

	return viewElement;
}

std::shared_ptr<SurgSim::Graphics::RenderPass> lightMapPass()
{
	auto pass = std::make_shared<SurgSim::Graphics::RenderPass>("shadowing");
	auto renderTarget = std::make_shared<SurgSim::Graphics::OsgRenderTarget2d>(1024, 1024, 1.0, 1, false);
	pass->setRenderTarget(renderTarget);
	pass->setRenderOrder(SurgSim::Graphics::Camera::RENDER_ORDER_PRE_RENDER, 0);
	pass->setMaterial(materials["depthMap"]);
	return pass;
}

std::shared_ptr<SurgSim::Graphics::RenderPass> shadowCastPass()
{
	auto pass = std::make_shared<SurgSim::Graphics::RenderPass>("shadowed");
	auto renderTarget = std::make_shared<SurgSim::Graphics::OsgRenderTarget2d>(1024, 1024, 1.0, 1, false);
	pass->setRenderTarget(renderTarget);
	pass->setRenderOrder(SurgSim::Graphics::Camera::RENDER_ORDER_PRE_RENDER, 1);
	pass->setMaterial(materials["shadowedVertexColors"]);
	return pass;
}

class SimpleBox : public SurgSim::Blocks::BasicSceneElement
{
public:
	SimpleBox(const std::string& name) : BasicSceneElement(name)
	{
		m_box = std::make_shared<SurgSim::Graphics::OsgBoxRepresentation>(getName()+" Graphics");
		m_box->setInitialPose(RigidTransform3d::Identity());
		//m_box->setMaterial(materials["default"]);
		m_box->addGroupReference("shadowing");
		m_box->addGroupReference("shadowed");
	}

	virtual bool doInitialize() override
	{
		addComponent(m_box);
		return true;
	}

	void setSize(double width, double height, double length)
	{
		m_box->setSize(width, height, length);
	}

	void setPose(const RigidTransform3d& pose)
	{
		m_box->setPose(pose);
	}

private:
	std::shared_ptr<SurgSim::Graphics::OsgBoxRepresentation> m_box;
};


}

std::shared_ptr<SurgSim::Framework::Scene> createScene(std::shared_ptr<SurgSim::Graphics::OsgManager> graphicsManager)
{
	auto scene = std::make_shared<SurgSim::Framework::Scene>();
	auto box = std::make_shared<SimpleBox>("Plane");
	box->setSize(3,0.01,3);
	box->setPose(RigidTransform3d::Identity());
	scene->addSceneElement(box);

	box = std::make_shared<SimpleBox>("Box 1");
	box->setSize(1.0,0.5,0.25);
	box->setPose(makeRigidTransform(Quaterniond::Identity(), Vector3d(-0.5,0.0,-0.5)));
	scene->addSceneElement(box);

	std::shared_ptr<SurgSim::Graphics::ViewElement> viewElement = createView("View", 0, 0, 1024, 768);
	scene->addSceneElement(viewElement);

	auto pass1 = lightMapPass();
	pass1->setView(viewElement->getView());
	pass1->showColorTarget(0,0,256,256);

	auto pass2 = shadowCastPass();
	pass2->setView(viewElement->getView());
	pass2->showColorTarget(1024-256,0,256,256);

	auto light = std::dynamic_pointer_cast<SurgSim::Graphics::Light>
				 (viewElement->getComponents<SurgSim::Graphics::Light>()[0]);
	auto camera = pass1->getCamera();
	camera->setViewMatrix(SurgSim::Math::makeViewMatrix(Vector3d(-3,3,-3), Vector3d(0,0,0), Vector3d(0,1,0)));

	camera = pass2->getCamera();
	camera->setViewMatrix(SurgSim::Math::makeViewMatrix(Vector3d(3,3,3), Vector3d(0,0,0), Vector3d(0,1,0)));
	

	// Connect pass1 parameters to pass2 parameters
	// Pass1 Camera becomes lightViewCamera in Pass2

	auto osgCamera = (std::dynamic_pointer_cast<SurgSim::Graphics::OsgCamera>(pass1->getCamera()))->getOsgCamera();

	osg::Matrixd m1 = osgCamera->getViewMatrix();
	SurgSim::Math::Matrix44d m2 = pass1->getCamera()->getViewMatrix();

	auto lightViewMatrix = std::make_shared<OsgUniform<Matrix44f>>("oss_lightViewMatrix");
	lightViewMatrix->set(pass1->getCamera()->getViewMatrix().cast<float>());
	pass2->getMaterial()->addUniform(lightViewMatrix);

	auto lightProjectionMatrix = std::make_shared<OsgUniform<Matrix44f>>("oss_lightProjectionMatrix");
	lightProjectionMatrix->set(pass1->getCamera()->getProjectionMatrix().cast<float>());
	pass2->getMaterial()->addUniform(lightProjectionMatrix);

	// Need to send the inverse view matrix of the main camera
	auto inverseViewMatrix = std::make_shared<OsgUniform<Matrix44f>>("oss_inverseViewMatrix");
	inverseViewMatrix->set(pass2->getCamera()->getViewMatrix().inverse().cast<float>());
	pass2->getMaterial()->addUniform(inverseViewMatrix);

	auto lightDepthTexture = 
		std::make_shared<SurgSim::Graphics::OsgUniform<std::shared_ptr<SurgSim::Graphics::OsgTexture2d>>>("oss_encodedLightDepthMap");
	lightDepthTexture->set(std::dynamic_pointer_cast<OsgTexture2d>(pass1->getRenderTarget()->getColorTarget(0)));
	pass2->getMaterial()->addUniform(lightDepthTexture);

	scene->addSceneElement(pass1);
	scene->addSceneElement(pass2);

	return scene;
}


int main(int argc, char* argv[])
{
	const SurgSim::Framework::ApplicationData data("config.txt");

	materials["basicLit"] = loadMaterial(data, "Shaders/basic_lit");
	materials["basicUnlit"] = loadMaterial(data, "Shaders/basic_unlit");
	materials["depthMap"] = loadMaterial(data, "Shaders/depth_map");
	materials["shadowedVertexColors"] = loadMaterial(data, "Shaders/shadowmap_vertexcolor");
	materials["default"] = materials["basicLit"];

	auto runtime(std::make_shared<SurgSim::Framework::Runtime>());
	auto graphicsManager = std::make_shared<SurgSim::Graphics::OsgManager>();

	graphicsManager->getDefaultCamera()->setInitialPose(
		SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), Vector3d(0.5, 0.5, 0.5)));
	
	runtime->addManager(graphicsManager);
	runtime->addManager(std::make_shared<SurgSim::Framework::BehaviorManager>());

	runtime->setScene(createScene(graphicsManager));
	runtime->execute();

	return 0;
}