// This file is a part of the OpenSurgSim project.
// Copyright 2013-2020, SimQuest Solutions Inc.
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

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Graphics/OsgBoxRepresentation.h"
#include "SurgSim/Graphics/Material.h"
#include "SurgSim/Graphics/Uniform.h"
#include "SurgSim/Blocks/PostprocessingView.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Blocks/PoseInterpolator.h"
#include "SurgSim//Framework/BehaviorManager.h"
#include "SurgSim//Graphics/OsgMeshRepresentation.h"
#include "SurgSim//Graphics/OsgProgram.h"
#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgLight.h"
#include "SurgSim/Graphics/OsgView.h"


using SurgSim::Math::Vector3d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;

namespace SurgSim
{
namespace Blocks
{


TEST(PostprocessingViewTest, BasicScene)
{
	auto runtime = std::make_shared<Framework::Runtime>("config.txt");
	auto manager = std::make_shared<Graphics::OsgManager>();
	runtime->addManager(manager);
	manager->setRate(140);
	runtime->addManager(std::make_shared<Framework::BehaviorManager>());

	auto scene = runtime->getScene();

	auto display = std::make_shared<Blocks::PostprocessingView>("Display");
	//auto display = std::make_shared<Graphics::OsgViewElement>("Display");
	display->enableManipulator(false);

	auto pose =
		Math::makeRigidTransform(Vector3d(2.0, 2.0, 2.0), Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 1.0, 0.0));
	display->setPose(pose);

	scene->addSceneElement(display);

	{

		auto element = std::make_shared<Framework::BasicSceneElement>("Graphics");
		//element->setPose(Math::makeRigidTranslation(Math::Vector3d(0.0, 0.0, 0.0)));
		auto box = std::make_shared<Graphics::OsgBoxRepresentation>("Box");

		RigidTransform3d from =
			Math::makeRigidTransform(Vector3d(0.2, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 1.0, 0.0));
		RigidTransform3d to =
			Math::makeRigidTransform(Vector3d(-0.2, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 1.0, 0.0));
		auto interpolator = std::make_shared<SurgSim::Blocks::PoseInterpolator>("interpolator");

		interpolator->setDuration(2.0);
		interpolator->setStartingPose(from);
		interpolator->setEndingPose(to);
		interpolator->setPingPong(true);
		interpolator->setTarget(element);

		element->addComponent(box);
		element->addComponent(interpolator);
		scene->addSceneElement(element);
	}

	auto material = display->getPostProcessingMaterial();

	float exposure = 1.0;
	float sign = -0.1;

	runtime->start();
	for (int i = 0; i < 200; ++i)
	{
		boost::this_thread::sleep(boost::posix_time::milliseconds(30));
		exposure = exposure + sign;
		if (exposure < 0) {
			exposure = 0.0;
			sign = sign * -1;
		}
		else if (exposure > 2.0)
		{
			exposure = 2.0;
			sign = sign * -1;
		}
		material->setValue("exposure", exposure);
	}

	runtime->stop();


}

TEST(PostprocessingViewTest, OverExposureTest)
{

	auto runtime = std::make_shared<Framework::Runtime>("config.txt");
	auto manager = std::make_shared<Graphics::OsgManager>();
	runtime->addManager(manager);
	manager->setRate(140);
	runtime->addManager(std::make_shared<Framework::BehaviorManager>());

	auto scene = runtime->getScene();

	std::string textureFilename;
	ASSERT_TRUE(runtime->getApplicationData()->tryFindFile("Textures/wound_deformable.png",
		&textureFilename));

	// Create a triangle mesh for visualizing the surface of the finite element model
	auto graphics = std::make_shared<SurgSim::Graphics::OsgMeshRepresentation>("Mesh");
	graphics->loadMesh("Geometry/wound_deformable_with_texture.ply");

	// Create material to transport the Textures

	auto material = std::make_shared<SurgSim::Graphics::OsgMaterial>("material");
	auto program = SurgSim::Graphics::loadProgram(*runtime->getApplicationData(), "Shaders/ds_mapping_material");
	ASSERT_TRUE(program != nullptr);
	material->setProgram(program);
	material->addUniform("vec4","diffuseColor", SurgSim::Math::Vector4f(0.8, 0.8, 0.8, 1.0));
	material->addUniform("vec4", "specularColor", SurgSim::Math::Vector4f(0.01, 0.01, 0.01, 1.0));
	material->addUniform("float", "shininess", 32.0f);

	{
		auto texture = std::make_shared<SurgSim::Graphics::OsgTexture2d>();
		texture->loadImage(textureFilename);
		auto uniform =
			std::make_shared<SurgSim::Graphics::OsgTextureUniform<SurgSim::Graphics::OsgTexture2d>>("diffuseMap");
		uniform->set(texture);
		material->addUniform(uniform);
	}

	{
		auto texture = std::make_shared<SurgSim::Graphics::OsgTexture2d>();
		std::string blackTexture;
		ASSERT_TRUE(runtime->getApplicationData()->tryFindFile("Textures/black.png", &blackTexture));

		texture->loadImage(blackTexture);
		auto uniform =
			std::make_shared<SurgSim::Graphics::OsgTextureUniform<SurgSim::Graphics::OsgTexture2d>>("shadowMap");
		uniform->set(texture);
		uniform->setMinimumTextureUnit(8);
		material->addUniform(uniform);
	}


	auto sceneElement = std::make_shared<SurgSim::Framework::BasicSceneElement>("Wound");
	sceneElement->addComponent(graphics);

	auto light = std::make_shared<SurgSim::Graphics::OsgLight>("Light");
	light->setDiffuseColor(SurgSim::Math::Vector4d(7.0, 7.0, 7.0, 1.0));
	light->setSpecularColor(SurgSim::Math::Vector4d(0.8, 0.8, 0.8, 1.0));
	light->setLightGroupReference(SurgSim::Graphics::Representation::DefaultGroupName);

	scene->addSceneElement(sceneElement);

	auto viewElement = std::make_shared<Blocks::PostprocessingView>("Display");
	viewElement->enableManipulator(false);

	viewElement->getView()->setManipulatorParameters({ 0,0.5,0 }, { 0,0,0 });

	viewElement->getCamera()->setAmbientColor(SurgSim::Math::Vector4d(0.2, 0.2, 0.2, 1.0));
	viewElement->getCamera()->setMaterial(material);
	viewElement->addComponent(material);
	viewElement->addComponent(light);

	scene->addSceneElement(viewElement);

	auto postprocess = viewElement->getPostProcessingMaterial();

	float exposure = 1.0;
	float sign = -0.01;
	postprocess->setValue("gamma", 1.0f);

	runtime->start();
	for (int i = 0; i < 300; ++i)
	{
		boost::this_thread::sleep(boost::posix_time::milliseconds(30));
		exposure = exposure + sign;
		if (exposure < 0) {
			exposure = 0.0;
			sign = sign * -1;
		}
		else if (exposure > 2.0)
		{
			exposure = 2.0;
			sign = sign * -1;
		}
		postprocess->setValue("exposure", exposure);
	}
	runtime->stop();

}

}
}
