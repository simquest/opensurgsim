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
#include "SurgSim/Graphics/OsgBoxRepresentation.h"
#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Graphics/OsgLight.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgProgram.h"
#include "SurgSim/Graphics/OsgSceneryRepresentation.h"
#include "SurgSim/Graphics/OsgSphereRepresentation.h"
#include "SurgSim/Graphics/OsgUniform.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Graphics/OsgSceneryRepresentation.h"
#include "SurgSim/Graphics/OsgRenderTarget.h"
#include "SurgSim/Graphics/RenderTests/RenderTest.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/Vector.h"

#include <boost/filesystem.hpp>

#include <gtest/gtest.h>

#include <random>
#include <array>


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

	std::string vertexShaderPath   = data.findFile("OsgShaderRenderTests/shader.vert");
	std::string geometryShaderPath = data.findFile("OsgShaderRenderTests/shader_axis_mirrored.geom");
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

std::shared_ptr<OsgTextureCubeMap> loadAxisCubeMap(
	const SurgSim::Framework::ApplicationData& data,
	const std::string& prefix)
{
	std::array<std::string, 6> filenames;

	bool success = data.tryFindFile(prefix + "NegativeX.png", &filenames[0]);
	success = data.tryFindFile(prefix + "PositiveX.png", &filenames[1]) && success;
	success = data.tryFindFile(prefix + "NegativeY.png", &filenames[2]) && success;
	success = data.tryFindFile(prefix + "PositiveY.png", &filenames[3]) && success;
	success = data.tryFindFile(prefix + "NegativeZ.png", &filenames[4]) && success;
	success = data.tryFindFile(prefix + "PositiveZ.png", &filenames[5]) && success;

	EXPECT_TRUE(success) << "One or more files are missing";

	auto result = std::make_shared<OsgTextureCubeMap>();
	result->loadImageFaces(filenames[0], filenames[1], filenames[2], filenames[3], filenames[4], filenames[5]);
	return result;
}

void add2DTexture(std::shared_ptr<OsgMaterial> material,
				  const std::string& uniform,
				  int unit,
				  const std::string& filename)
{
	std::string path;
	EXPECT_TRUE(Framework::Runtime::getApplicationData()->tryFindFile(filename, &path));
	auto texture = std::make_shared<SurgSim::Graphics::OsgTexture2d>();
	texture->loadImage(path);
	auto textureUniform = std::make_shared<OsgTextureUniform<OsgTexture2d>>(uniform);
	textureUniform->set(texture);
	textureUniform->setMinimumTextureUnit(unit);
	material->addUniform(textureUniform);
}


struct OsgProgramRenderTests : public RenderTest
{
	void SetUp()
	{
		RenderTest::SetUp();

		// Light
		auto sceneElement = std::make_shared<SurgSim::Framework::BasicSceneElement>("Light");
		auto light = std::make_shared<SurgSim::Graphics::OsgLight>("Light");
		light->setDiffuseColor(SurgSim::Math::Vector4d(0.8, 0.8, 0.8, 1.0));
		light->setSpecularColor(SurgSim::Math::Vector4d(0.8, 0.8, 0.8, 1.0));
		light->setLightGroupReference(SurgSim::Graphics::Representation::DefaultGroupName);
		sceneElement->addComponent(light);
		sceneElement->addComponent(std::make_shared<SurgSim::Graphics::OsgAxesRepresentation>("axes"));
		sceneElement->setPose(makeRigidTransform(Quaterniond::Identity(), Vector3d(-2.0, -2.0, -4.0)));
		scene->addSceneElement(sceneElement);

		// Camera
		viewElement->getCamera()->setAmbientColor(SurgSim::Math::Vector4d(0.1, 0.1, 0.1, 1.0));
		viewElement->setPose(
			makeRigidTransform(Vector3d(0.0, 0.0, -2.0), Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 1.0, 0.0)));
		viewElement->enableManipulator(true);
	}

	void run(size_t time = 500)
	{
		// Action
		runtime->start();
		boost::this_thread::sleep(boost::posix_time::milliseconds(time));
		runtime->stop();
	}
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

	run();
}

TEST_F(OsgProgramRenderTests, Shiny)
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

	run();
}

TEST_F(OsgProgramRenderTests, TexturedShiny)
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

	run();
}



TEST_F(OsgProgramRenderTests, Metal)
{
	// Multiple objects used for testing the shader, utilize if needed
	std::shared_ptr<SphereRepresentation> sphere = std::make_shared<OsgSphereRepresentation>("sphere");
	sphere->setRadius(0.25);

	std::shared_ptr<BoxRepresentation> cube = std::make_shared<OsgBoxRepresentation>("box");

	auto scenery = std::make_shared<OsgSceneryRepresentation>("scenery");
	scenery->loadModel("OsgShaderRenderTests/L_forcep.obj");

	// Assign the object used for testing to the representation
	auto representation = std::dynamic_pointer_cast<Representation>(sphere);

	auto material = std::make_shared<OsgMaterial>("material");
	auto program = SurgSim::Graphics::loadProgram(*runtime->getApplicationData(), "Shaders/s_mapping_metal");
	ASSERT_TRUE(program != nullptr);
	material->setProgram(program);

	material->addUniform("vec4", "specularColor");
	material->setValue("specularColor", SurgSim::Math::Vector4f(1.0, 1.0, 1.0, 1.0));

	material->addUniform("float", "shininess");
	material->setValue("shininess", 1024.0f);

	material->addUniform("float", "specularPercent");
	material->setValue("specularPercent", 1.0f);

	material->addUniform("float", "diffusePercent");
	material->setValue("diffusePercent", 0.0f);

	std::string filename;
	// Provide a fake shadow map, it's all black so no shadow contribution
	{
		EXPECT_TRUE(runtime->getApplicationData()->tryFindFile("Textures/black.png", &filename));
		auto texture = std::make_shared<SurgSim::Graphics::OsgTexture2d>();
		texture->loadImage(filename);
		auto textureUniform = std::make_shared<OsgTextureUniform<OsgTexture2d>>("shadowMap");
		textureUniform->set(texture);
		textureUniform->setMinimumTextureUnit(8);
		material->addUniform(textureUniform);
	}

	{
		// Provide the Diffuse environment map
		// Axis map is used for testing mapping
		// auto texture = loadAxisCubeMap(*runtime->getApplicationData(), "Textures/");
		EXPECT_TRUE(runtime->getApplicationData()->tryFindFile(
						"Textures/CubeMap_reflection_diffuse.png", &filename));
		auto texture = std::make_shared<OsgTextureCubeMap>();
		texture->loadImage(filename);
		material->addUniform("samplerCube", "diffuseEnvMap");
		material->setValue("diffuseEnvMap", texture);
	}


	{
		// Provide the Specular environment map
		// Axis map is used for testing mapping
		// auto texture = loadAxisCubeMap(*runtime->getApplicationData(), "Textures/");
		EXPECT_TRUE(runtime->getApplicationData()->tryFindFile(
						"Textures/CubeMap_reflection_specular.png", &filename));
		auto texture = std::make_shared<OsgTextureCubeMap>();
		texture->loadImage(filename);
		material->addUniform("samplerCube", "specularEnvMap");
		material->setValue("specularEnvMap", texture);
	}

	representation->setMaterial(material);

	auto sceneElement = std::make_shared<SurgSim::Framework::BasicSceneElement>("Sphere");
	sceneElement->addComponent(representation);
	sceneElement->addComponent(material);
	sceneElement->addComponent(std::make_shared<SurgSim::Graphics::OsgAxesRepresentation>("axes"));
	scene->addSceneElement(sceneElement);

	run();
}

TEST_F(OsgProgramRenderTests, NormalMap)
{
	// Assign the object used for testing to the representation
	auto graphics = std::make_shared<OsgSceneryRepresentation>("scenery");
	graphics->loadModel("Geometry/cube.osgt");

	auto representation = std::dynamic_pointer_cast<Representation>(graphics);
	representation->setGenerateTangents(true);

	auto material = std::make_shared<OsgMaterial>("material");
	auto program = SurgSim::Graphics::loadProgram(*runtime->getApplicationData(), "Shaders/dns_mapping_material");
	ASSERT_TRUE(program != nullptr);
	material->setProgram(program);

	material->addUniform("vec4", "specularColor");
	material->setValue("specularColor", SurgSim::Math::Vector4f(1.0, 1.0, 1.0, 1.0));

	material->addUniform("vec4", "diffuseColor");
	material->setValue("diffuseColor", SurgSim::Math::Vector4f(1.0, 1.0, 1.0, 1.0));

	material->addUniform("float", "shininess");
	material->setValue("shininess", 1.0f);

	std::string filename;

	add2DTexture(material, "shadowMap", 8, "Textures/black.png");
	add2DTexture(material, "diffuseMap", 0, "Textures/checkered.png");
	add2DTexture(material, "normalMap", 1, "Textures/bricks.png");
	representation->setMaterial(material);

	auto sceneElement = std::make_shared<SurgSim::Framework::BasicSceneElement>("Graphics");
	sceneElement->addComponent(representation);
	sceneElement->addComponent(material);
	sceneElement->addComponent(std::make_shared<SurgSim::Graphics::OsgAxesRepresentation>("axes"));
	scene->addSceneElement(sceneElement);


	viewElement->setPose(makeRigidTransform(Vector3d(0.5, 0.5, -0.5),
											Vector3d(0.0, 0.0, 0.0),
											Vector3d(0.0, 1.0, 0.0)));

	run();
}

TEST_F(OsgProgramRenderTests, BlurShader)
{
	auto element = std::make_shared<Framework::BasicSceneElement>("Graphics");

	auto texture1 = std::make_shared<Graphics::OsgTexture2d>();
	std::string filename;
	ASSERT_TRUE(Runtime::getApplicationData()->tryFindFile("OsgScreenSpaceQuadRenderTests/CheckerBoard.png", &filename));
	texture1->loadImage(filename);


	auto texture2 = std::make_shared<Graphics::OsgTexture2d>();
	ASSERT_TRUE(Runtime::getApplicationData()->tryFindFile("Textures/checkered.png", &filename));
	texture2->loadImage(filename);

	auto material = std::make_shared<OsgMaterial>("Material");
	auto program = SurgSim::Graphics::loadProgram(*runtime->getApplicationData(), "Shaders/horizontalBlurPass");
	ASSERT_TRUE(program != nullptr);
	material->setProgram(program);

	material->addUniform("float", "width");
	material->setValue("width", 1024.0f);
	material->addUniform("float", "blurRadius");
	material->setValue("blurRadius", 16.0f);
	material->getProgram()->setGlobalScope(true);

	auto graphics = std::make_shared<Graphics::OsgScreenSpaceQuadRepresentation>("Quad");
	graphics->setSize(1024, 1024);
	graphics->setLocation(0, 0);
	graphics->setTexture(texture1);
	graphics->setGroupReference("BlurrPass");
	element->addComponent(graphics);

	auto passCamera = std::make_shared<Graphics::OsgCamera>("BlurrPassCamera");
	auto osgCamera = passCamera->getOsgCamera();
	passCamera->setRenderGroupReference("BlurrPass");
	passCamera->setGroupReference(Graphics::Representation::DefaultGroupName);
	osgCamera->setViewport(0, 0, 1024, 1024);
	osgCamera->setProjectionMatrixAsOrtho2D(0, 1024, 0, 1024);
	passCamera->setMaterial(material);

	auto renderTarget = std::make_shared<Graphics::OsgRenderTarget2d>(1024, 1024, 1.0, 1, false);
	passCamera->setRenderTarget(renderTarget);
	element->addComponent(passCamera);


	graphics = std::make_shared<Graphics::OsgScreenSpaceQuadRepresentation>("DebugQuad");
	graphics->setLocation(512, 512);
	graphics->setSize(256, 256);
	graphics->setTexture(passCamera->getRenderTarget()->getColorTarget(0));
	element->addComponent(graphics);

	element->addComponent(material);

	scene->addSceneElement(element);

	run();
}

};  // namespace Graphics

};  // namespace SurgSim
