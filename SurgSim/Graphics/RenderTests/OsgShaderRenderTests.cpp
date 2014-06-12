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

/// \file
/// Render Tests for the OsgShader class.


#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Graphics/OsgAxesRepresentation.h"
#include "SurgSim/Graphics/OsgLight.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgShader.h"
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

std::shared_ptr<Shader> loadExampleShader(const SurgSim::Framework::ApplicationData& data)
{
	std::shared_ptr<Shader> shader = std::make_shared<OsgShader>();

	std::string vertexShaderPath = data.findFile("OsgShaderRenderTests/shader.vert");
	std::string geometryShaderPath = data.findFile("OsgShaderRenderTests/shader.geom");
	std::string fragmentShaderPath = data.findFile("OsgShaderRenderTests/shader.frag");

	EXPECT_NE("", vertexShaderPath) << "Could not find vertex shader!";
	EXPECT_NE("", geometryShaderPath) << "Could not find geometry shader!";
	EXPECT_NE("", fragmentShaderPath) << "Could not find fragment shader!";

	shader->loadVertexShaderSource(vertexShaderPath);
	shader->loadGeometryShaderSource(geometryShaderPath);
	shader->loadFragmentShaderSource(fragmentShaderPath);

	return shader;
}

std::shared_ptr<Material> loadMaterial(const SurgSim::Framework::ApplicationData& data,
									   const std::string& shaderName)
{
	SCOPED_TRACE("Load Material");

	auto shader = std::make_shared<SurgSim::Graphics::OsgShader>();

	std::string filename;
	EXPECT_TRUE(data.tryFindFile(shaderName + ".vert", &filename));
	shader->loadVertexShaderSource(filename);

	EXPECT_TRUE(data.tryFindFile(shaderName + ".frag", &filename));
	shader->loadFragmentShaderSource(filename);

	auto material = std::make_shared<SurgSim::Graphics::OsgMaterial>();
	material->setShader(shader);

	return material;
}

std::shared_ptr<Material> createShinyMaterial(const SurgSim::Framework::ApplicationData& data)
{
	auto material = loadMaterial(data, "Shaders/material");
	std::shared_ptr<SurgSim::Graphics::UniformBase>
	uniform = std::make_shared<OsgUniform<SurgSim::Math::Vector4f>>("diffuseColor");
	material->addUniform(uniform);

	uniform = std::make_shared<OsgUniform<SurgSim::Math::Vector4f>>("specularColor");
	material->addUniform(uniform);

	uniform = std::make_shared<OsgUniform<float>>("shinyness");
	material->addUniform(uniform);

	return material;
}

struct OsgShaderRenderTests : public RenderTest
{

};

/// Pops up a window with a sphere colored by its normals and its mirror along the x-axis is also drawn using the
/// geometry shader
TEST_F(OsgShaderRenderTests, SphereShaderTest)
{
	/// Add the sphere representation to the view element, no need to make another scene element
	std::shared_ptr<SphereRepresentation> sphereRepresentation =
		std::make_shared<OsgSphereRepresentation>("sphere representation");
	sphereRepresentation->setRadius(0.25);
	sphereRepresentation->setLocalPose(makeRigidTransform(Quaterniond::Identity(), Vector3d(0.25, 0.0, -1.0)));

	/// Add a shader to the sphere
	std::shared_ptr<OsgMaterial> material = std::make_shared<OsgMaterial>();
	std::shared_ptr<Shader> shader = loadExampleShader(*applicationData);

	material->setShader(shader);
	sphereRepresentation->setMaterial(material);

	viewElement->addComponent(sphereRepresentation);

	/// Run the thread
	runtime->start();
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	runtime->stop();

}

TEST_F(OsgShaderRenderTests, SpecificShaderTest)
{
	/// Add the sphere representation to the view element, no need to make another scene element
	auto sceneElement = std::make_shared<SurgSim::Framework::BasicSceneElement>("Sphere");
	std::shared_ptr<SphereRepresentation> sphereRepresentation =
		std::make_shared<OsgSphereRepresentation>("sphere representation");
	sphereRepresentation->setRadius(0.25);

	auto material = createShinyMaterial(*runtime->getApplicationData());
	material->setValue("diffuseColor", SurgSim::Math::Vector4f(0.8, 0.8, 0.1, 1.0));
	material->setValue("specularColor", SurgSim::Math::Vector4f(1.0, 1.0, 0.4, 1.0));
	material->setValue("shininess", 1.0f);
	sphereRepresentation->setMaterial(material);
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

	sphereRepresentation = std::make_shared<OsgSphereRepresentation>("debug");
	sphereRepresentation->setRadius(0.01);
	sceneElement->addComponent(sphereRepresentation);

	scene->addSceneElement(sceneElement);

	//viewElement->enableManipulator(true);

	viewElement->setPose(
		makeRigidTransform(Vector3d(0.0, 0.0, -2.0), Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 1.0, 0.0)));
	viewElement->addComponent(std::make_shared<SurgSim::Graphics::OsgAxesRepresentation>("axes"));


	/// Run the thread
// 	runtime->start();
// 	boost::this_thread::sleep(boost::posix_time::milliseconds(10000));
// 	runtime->stop();
	runtime->execute();

}

};  // namespace Graphics

};  // namespace SurgSim
