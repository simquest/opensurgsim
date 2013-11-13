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
#include <SurgSim/Graphics/OsgShader.h>
#include <SurgSim/Graphics/OsgViewElement.h>
#include <SurgSim/Graphics/OsgView.h>

#include <SurgSim/Blocks/BasicSceneElement.h>

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>

using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Framework::Logger;

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
	viewElement->setManipulatorParameters(SurgSim::Math::Vector3d(8.0,4.0,8.0), SurgSim::Math::Vector3d(0.0,0.0,0.0));

	std::shared_ptr<SurgSim::Graphics::Group> group = std::make_shared<SurgSim::Graphics::OsgGroup>("Light");
	auto light = std::make_shared<SurgSim::Graphics::OsgLight>("Main Light");
	light->setAmbientColor(Vector4d(0.5,0.5,0.5,1.0));
	light->setDiffuseColor(Vector4d(0.5,0.5,0.5,1.0));
	light->setSpecularColor(Vector4d(0.8,0.8,0.8,1.0));
	light->setInitialPose(SurgSim::Math::makeRigidTransform(Quaterniond::Identity(),Vector3d(10.0,10.0,10.0)));

	viewElement->addComponent(light); 

	return viewElement;
}

class SimpleBox : public SurgSim::Blocks::BasicSceneElement
{
public:
	SimpleBox(const std::string& name) : BasicSceneElement(name)
	{
		m_box = std::make_shared<SurgSim::Graphics::OsgBoxRepresentation>(getName()+" Graphics");
		m_box->setInitialPose(RigidTransform3d::Identity());
		m_box->setMaterial(materials["default"]);
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

void createScene(std::shared_ptr<SurgSim::Framework::Runtime> runtime)
{
	auto scene = std::make_shared<SurgSim::Framework::Scene>();
	auto box = std::make_shared<SimpleBox>("Plane");
	box->setSize(10,0.01,10);
	box->setPose(RigidTransform3d::Identity());
	scene->addSceneElement(box);

	box = std::make_shared<SimpleBox>("Box 1");
	box->setSize(1.0,2.0,3.0);
	box->setPose(RigidTransform3d::Identity());
	scene->addSceneElement(box);

	scene->addSceneElement(createView("View", 0, 0, 1023, 768));
	runtime->setScene(scene);
}


int main(int argc, char* argv[])
{
	const SurgSim::Framework::ApplicationData data("config.txt");

	materials["default"] = loadMaterial(data, "Shaders/basic_lit");
	materials["defaultNoShade"] = loadMaterial(data, "Shaders/basic_unlit");

	auto runtime(std::make_shared<SurgSim::Framework::Runtime>());
	auto graphicsManager = std::make_shared<SurgSim::Graphics::OsgManager>();

	graphicsManager->getDefaultCamera()->setInitialPose(
		SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), Vector3d(0.0, 0.5, 5.0)));
	graphicsManager->getDefaultCamera()->setMaterial(materials["default"]);

	runtime->addManager(graphicsManager);
	runtime->addManager(std::make_shared<SurgSim::Framework::BehaviorManager>());

	createScene(runtime);

	runtime->execute();

	return 0;
}
