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

#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Graphics/OsgAxesRepresentation.h"
#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Graphics/OsgLight.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgProgram.h"
#include "SurgSim/Graphics/OsgSphereRepresentation.h"
#include "SurgSim/Graphics/OsgUniform.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Graphics/RenderTests/RenderTest.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/Vector.h"

#include <boost/filesystem.hpp>

#include <gtest/gtest.h>

#include <random>


using SurgSim::Framework::Runtime;
using SurgSim::Framework::Scene;
using SurgSim::Framework::SceneElement;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::makeRigidTransform;

namespace SurgSim
{

namespace Graphics
{

std::shared_ptr<Program> loadExampleProgram(const SurgSim::Framework::ApplicationData& data)
{
	std::shared_ptr<Program> program = std::make_shared<OsgProgram>();

	std::string vertexShaderPath = data.findFile("OsgShaderRenderTests/shader.vert");
	std::string geometryShaderPath = data.findFile("OsgShaderRenderTests/shader.geom");
	std::string fragmentShaderPath = data.findFile("OsgShaderRenderTests/shader.frag");

	EXPECT_NE("", vertexShaderPath) << "Could not find vertex shader!";
	EXPECT_NE("", geometryShaderPath) << "Could not find geometry shader!";
	EXPECT_NE("", fragmentShaderPath) << "Could not find fragment shader!";

	program->loadVertexShader(vertexShaderPath);
	program->loadGeometryShader(geometryShaderPath);
	program->loadFragmentShader(fragmentShaderPath);

	return program;
}


std::shared_ptr<Material> createShinyMaterial(const SurgSim::Framework::ApplicationData& data)
{
	auto material = std::make_shared<SurgSim::Graphics::OsgMaterial>("material");
	auto program = SurgSim::Graphics::loadProgram(data, "Shaders/material");
	material->setProgram(program);

	std::shared_ptr<SurgSim::Graphics::UniformBase>
	uniform = std::make_shared<OsgUniform<SurgSim::Math::Vector4f>>("diffuseColor");
	material->addUniform(uniform);

	uniform = std::make_shared<OsgUniform<SurgSim::Math::Vector4f>>("specularColor");
	material->addUniform(uniform);

	uniform = std::make_shared<OsgUniform<float>>("shininess");
	material->addUniform(uniform);

	return material;
}

struct OsgProgramRenderTests : public RenderTest
{

};

/// Pops up a window with a sphere colored by its normals and its mirror along the x-axis is also drawn using the
/// geometry shader
TEST_F(OsgProgramRenderTests, SphereShaderTest)
{
	/// Add the sphere representation to the view element, no need to make another scene element
	std::shared_ptr<SphereRepresentation> sphereRepresentation =
		std::make_shared<OsgSphereRepresentation>("sphere representation");
	sphereRepresentation->setRadius(0.25);
	sphereRepresentation->setLocalPose(makeRigidTransform(Quaterniond::Identity(), Vector3d(0.25, 0.0, -1.0)));

	/// Add a material to the sphere
	std::shared_ptr<OsgMaterial> material = std::make_shared<OsgMaterial>("material");
	std::shared_ptr<Program> program = loadExampleProgram(*applicationData);

	material->setProgram(program);
	sphereRepresentation->setMaterial(material);

	viewElement->addComponent(sphereRepresentation);
	viewElement->addComponent(material);

	/// Run the thread
	runtime->start();
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	runtime->stop();
}

TEST_F(OsgProgramRenderTests, ShinyShaderTest)
{
	/// Add the sphere representation to the view element, no need to make another scene element
	auto sceneElement = std::make_shared<SurgSim::Framework::BasicSceneElement>("Sphere");
	std::shared_ptr<SphereRepresentation> sphereRepresentation =
		std::make_shared<OsgSphereRepresentation>("sphere representation");
	sphereRepresentation->setRadius(0.25);

	auto material = createShinyMaterial(*runtime->getApplicationData());
	material->setValue("diffuseColor", SurgSim::Math::Vector4f(0.8, 0.8, 0.1, 1.0));
	material->setValue("specularColor", SurgSim::Math::Vector4f(1.0, 1.0, 0.4, 1.0));
	material->setValue("shininess", 64.0f);
	sphereRepresentation->setMaterial(material);
	sceneElement->addComponent(material);
	sceneElement->addComponent(sphereRepresentation);
	sceneElement->addComponent(std::make_shared<SurgSim::Graphics::OsgAxesRepresentation>("axes"));
	scene->addSceneElement(sceneElement);


	sceneElement = std::make_shared<SurgSim::Framework::BasicSceneElement>("Light");
	auto light = std::make_shared<SurgSim::Graphics::OsgLight>("Light");
	light->setDiffuseColor(SurgSim::Math::Vector4d(0.8, 0.8, 0.8, 1.0));
	light->setSpecularColor(SurgSim::Math::Vector4d(0.8, 0.8, 0.8, 1.0));
	light->setLightGroupReference(SurgSim::Graphics::Representation::DefaultGroupName);
	sceneElement->addComponent(light);
	sceneElement->addComponent(std::make_shared<SurgSim::Graphics::OsgAxesRepresentation>("axes"));
	sceneElement->setPose(makeRigidTransform(Quaterniond::Identity(), Vector3d(-2.0, -2.0, -4.0)));

	scene->addSceneElement(sceneElement);

	viewElement->setPose(
		makeRigidTransform(Vector3d(0.0, 0.0, -2.0), Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 1.0, 0.0)));
	viewElement->addComponent(std::make_shared<SurgSim::Graphics::OsgAxesRepresentation>("axes"));


	/// Run the thread
	runtime->start();
	boost::this_thread::sleep(boost::posix_time::milliseconds(500));
	runtime->stop();
}

TEST_F(OsgProgramRenderTests, TexturedShinyShaderTest)
{
	// The textured Sphere
	std::shared_ptr<SphereRepresentation> sphereRepresentation =
		std::make_shared<OsgSphereRepresentation>("sphere representation");
	sphereRepresentation->setRadius(0.25);

	auto material = std::make_shared<OsgMaterial>("material");
	auto program = SurgSim::Graphics::loadProgram(*runtime->getApplicationData(), "Shaders/ds_mapping_material");
	ASSERT_TRUE(program != nullptr);
	material->setProgram(program);

	std::shared_ptr<SurgSim::Graphics::UniformBase>
	uniform = std::make_shared<OsgUniform<SurgSim::Math::Vector4f>>("diffuseColor");
	material->addUniform(uniform);

	uniform = std::make_shared<OsgUniform<SurgSim::Math::Vector4f>>("specularColor");
	material->addUniform(uniform);

	uniform = std::make_shared<OsgUniform<float>>("shininess");
	material->addUniform(uniform);

	material->setValue("diffuseColor", SurgSim::Math::Vector4f(0.8, 0.8, 0.1, 1.0));
	material->setValue("specularColor", SurgSim::Math::Vector4f(1.0, 1.0, 0.4, 1.0));
	material->setValue("shininess", 1.0f);

	// Provide a texture for the diffuse color
	std::string filename;
	EXPECT_TRUE(runtime->getApplicationData()->tryFindFile("Textures/checkered.png", &filename));
	auto texture = std::make_shared<SurgSim::Graphics::OsgTexture2d>();
	texture->loadImage(filename);
	auto textureUniform =
		std::make_shared<OsgTextureUniform<OsgTexture2d>>("diffuseMap");
	textureUniform->set(texture);
	material->addUniform(textureUniform);

	// Provide a fake shadow map, it's all black so no shadow contribution
	EXPECT_TRUE(runtime->getApplicationData()->tryFindFile("Textures/black.png", &filename));
	texture = std::make_shared<SurgSim::Graphics::OsgTexture2d>();
	texture->loadImage(filename);
	textureUniform = std::make_shared<OsgTextureUniform<OsgTexture2d>>("shadowMap");
	textureUniform->set(texture);
	textureUniform->setMinimumTextureUnit(8);
	material->addUniform(textureUniform);

	sphereRepresentation->setMaterial(material);

	auto sceneElement = std::make_shared<SurgSim::Framework::BasicSceneElement>("Sphere");
	sceneElement->addComponent(sphereRepresentation);
	sceneElement->addComponent(material);
	sceneElement->addComponent(std::make_shared<SurgSim::Graphics::OsgAxesRepresentation>("axes"));

	scene->addSceneElement(sceneElement);

	sceneElement = std::make_shared<SurgSim::Framework::BasicSceneElement>("Light");
	auto light = std::make_shared<SurgSim::Graphics::OsgLight>("Light");
	light->setDiffuseColor(SurgSim::Math::Vector4d(0.8, 0.8, 0.8, 1.0));
	light->setSpecularColor(SurgSim::Math::Vector4d(0.8, 0.8, 0.8, 1.0));
	light->setLightGroupReference(SurgSim::Graphics::Representation::DefaultGroupName);
	sceneElement->addComponent(light);
	sceneElement->addComponent(std::make_shared<SurgSim::Graphics::OsgAxesRepresentation>("axes"));
	sceneElement->setPose(makeRigidTransform(Quaterniond::Identity(), Vector3d(-2.0, -2.0, -4.0)));
	scene->addSceneElement(sceneElement);

	viewElement->enableManipulator(true);
	viewElement->getCamera()->setAmbientColor(SurgSim::Math::Vector4d(0.2, 0.2, 0.2, 1.0));

	viewElement->setPose(makeRigidTransform(Vector3d(0.0, 0.0, -2.0),
											Vector3d(0.0, 0.0, 0.0),
											Vector3d(0.0, 1.0, 0.0)));

	viewElement->addComponent(std::make_shared<SurgSim::Graphics::OsgAxesRepresentation>("axes"));

	runtime->start();
	boost::this_thread::sleep(boost::posix_time::milliseconds(500));
	runtime->stop();
}

};  // namespace Graphics

};  // namespace SurgSim
