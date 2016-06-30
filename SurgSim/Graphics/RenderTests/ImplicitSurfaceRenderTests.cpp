// This file is a part of the OpenSurgSim project.
// Copyright 2015, SimQuest Solutions Inc.
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

#include <memory>
#include <vector>

#include "SurgSim/Blocks/ImplicitSurface.h"
#include "SurgSim/Blocks/PoseInterpolator.h"
#include "SurgSim/Blocks/ShadowMapping.h"
#include "SurgSim/DataStructures/Vertices.h"
#include "SurgSim/Framework/BehaviorManager.h"
#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Graphics/OsgAxesRepresentation.h"
#include "SurgSim/Graphics/OsgBoxRepresentation.h"
#include "SurgSim/Graphics/Camera.h"
#include "SurgSim/Graphics/OsgLight.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgMeshRepresentation.h"
#include "SurgSim/Graphics/OsgPointCloudRepresentation.h"
#include "SurgSim/Graphics/OsgSphereRepresentation.h"
#include "SurgSim/Graphics/OsgView.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Graphics/RenderTests/RenderTest.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::makeRigidTranslation;

namespace SurgSim
{
namespace Graphics
{

class ImplicitSurfaceRenderTests : public RenderTest
{

};

TEST_F(ImplicitSurfaceRenderTests, PointSpriteFluid)
{
	Math::Vector4f diffuseColor(0.83f, 0.0f, 0.0f, 1.0f);
	Math::Vector4f specularColor(0.8f, 0.8f, 0.8f, 1.0f);

	std::array<int, 2> dimensions = {1280, 720};
	viewElement->getView()->setDimensions(dimensions);
	viewElement->getCamera()->setPerspectiveProjection(45, 1.7, 0.01, 10.0);
	viewElement->getCamera()->setAmbientColor(Math::Vector4d(0.2, 0.2, 0.2, 1.0));


	auto interpolator = std::make_shared<Blocks::PoseInterpolator>("Interpolator");
	RigidTransform3d from = makeRigidTransform(
								Vector3d(0.5, 0.0, -0.5),
								Vector3d(0.0, 0.0, 0.0),
								Vector3d(0.0, 1.0, 0.0));
	RigidTransform3d to = makeRigidTransform(
							  Vector3d(-0.5, 0.0, -0.5),
							  Vector3d(0.0, 0.0, 0.0),
							  Vector3d(0.0, 1.0, 0.0));
	interpolator->setTarget(viewElement);
	interpolator->setStartingPose(from);
	interpolator->setDuration(5.0);
	interpolator->setEndingPose(to);
	interpolator->setPingPong(true);

	viewElement->setPose(from);
	viewElement->addComponent(interpolator);

	auto light = std::make_shared<Graphics::OsgLight>("Light");
	light->setDiffuseColor(Math::Vector4d(1.0, 1.0, 1.0, 1.0));
	light->setSpecularColor(Math::Vector4d(0.8, 0.8, 0.8, 1.0));
	light->setLightGroupReference(SurgSim::Graphics::Representation::DefaultGroupName);

	auto lightElement = std::make_shared<Framework::BasicSceneElement>("LightElement");
	lightElement->setPose(makeRigidTransform(
			Math::Vector3d(0.5, 0.5, 0.5),
	Math::Vector3d(0.0 ,0.0, 0.0),
	Math::Vector3d(0.0, 1.0, 0.0)));
	lightElement->addComponent(light);
	scene->addSceneElement(lightElement);

	auto axes = std::make_shared<Graphics::OsgAxesRepresentation>("Axes");
	lightElement->addComponent(axes);

	std::array<double, 6> lightProjection = { -2.0, 2.0, -2.0, 2.0, -1.0, 2.0 };
	scene->addSceneElements(Blocks::createShadowMapping(viewElement->getCamera(), light,
														4096, 1024, lightProjection, 0.002, 0.75, true, 4.0, false));

	scene->addSceneElements(Blocks::createImplicitSurfaceEffect(viewElement->getView(), light,
																scene->getSceneElement(Blocks::GROUP_SHADOW_CASTER),
																0.01f, 800.0f, 4.0,
																diffuseColor, specularColor,
																"Textures/CubeMap_reflection_diffuse.png", 0.9,
																"Textures/CubeMap_reflection_specular.png", 0.1,
																100.0f, false));

	auto cube = std::make_shared<Graphics::OsgBoxRepresentation>("Cube");
	cube->setSizeXYZ(0.1, 0.1, 0.1);

	auto element = std::make_shared<Framework::BasicSceneElement>("box");
	element->setPose(makeRigidTranslation(Math::Vector3d(0.0, 0.0, 0.25)));
	element->addComponent(cube);

	scene->addSceneElement(element);

	auto sphere = std::make_shared<Graphics::OsgSphereRepresentation>("Graphics");
	sphere->setRadius(0.1);

	auto material = Graphics::buildMaterial("Shaders/material.vert", "Shaders/material.frag");
	material->addUniform("vec4", "diffuseColor");
	material->setValue("diffuseColor", diffuseColor);
	material->addUniform("vec4", "specularColor");
	material->setValue("specularColor", specularColor);
	material->addUniform("float", "shininess");
	material->setValue("shininess", 10.0f);
	sphere->setMaterial(material);

	element = std::make_shared<Framework::BasicSceneElement>("Sphere");
	element->setPose(makeRigidTranslation(Math::Vector3d(0.25, 0.0, 0.0)));
	element->addComponent(sphere);
	element->addComponent(material);

	scene->addSceneElement(element);

	// Create the point cloud
	auto mesh = std::make_shared<Graphics::OsgMeshRepresentation>("Mesh");
	mesh->loadMesh("Geometry/sphere.ply");

	auto graphics = std::make_shared<Graphics::OsgPointCloudRepresentation>("Cloud");

	for (const auto& vertex : mesh->getMesh()->getVertices())
	{
		graphics->getVertices()->addVertex(Graphics::PointCloud::VertexType(vertex));
	}

	graphics->setGroupReference(Blocks::GROUP_IMPLICIT_SURFACE);

	auto sceneElement = std::make_shared<Framework::BasicSceneElement>("PointSprites");
	sceneElement->addComponent(graphics);

	scene->addSceneElement(sceneElement);


	runtime->start();
	boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
	runtime->stop();
}

TEST_F(ImplicitSurfaceRenderTests, StereoFluid)
{
	Math::Vector4f diffuseColor(0.83f, 0.0f, 0.0f, 1.0f);
	Math::Vector4f specularColor(0.8f, 0.8f, 0.8f, 1.0f);

	std::array<int, 2> dimensions = {1280, 720};
	viewElement->getView()->setDimensions(dimensions);
	viewElement->getCamera()->setPerspectiveProjection(45, 1.7, 0.01, 10.0);
	viewElement->getCamera()->setAmbientColor(Math::Vector4d(0.2, 0.2, 0.2, 1.0));

	auto osgView = std::static_pointer_cast<OsgView>(viewElement->getView());
	osgView->setStereoMode(View::STEREO_MODE_HORIZONTAL_SPLIT);
	osgView->setDisplayType(View::DISPLAY_TYPE_MONITOR);
	osgView->setScreenWidth(0.0631);
	osgView->setScreenHeight(0.071);
	osgView->setEyeSeparation(0.06);
	osgView->setScreenDistance(0.10);

	auto interpolator = std::make_shared<Blocks::PoseInterpolator>("Interpolator");
	RigidTransform3d from = makeRigidTransform(
			Vector3d(2.0, 0.0, -2.0),
			Vector3d(0.0, 0.0, 0.0),
			Vector3d(0.0, 1.0, 0.0));
	RigidTransform3d to = makeRigidTransform(
			Vector3d(-2.0, 0.0, -2.0),
			Vector3d(0.0, 0.0, 0.0),
			Vector3d(0.0, 1.0, 0.0));
	interpolator->setTarget(viewElement);
	interpolator->setStartingPose(from);
	interpolator->setDuration(5.0);
	interpolator->setEndingPose(to);
	interpolator->setPingPong(true);

	viewElement->setPose(from);
	viewElement->addComponent(interpolator);

	auto light = std::make_shared<Graphics::OsgLight>("Light");
	light->setDiffuseColor(Math::Vector4d(1.0, 1.0, 1.0, 1.0));
	light->setSpecularColor(Math::Vector4d(0.8, 0.8, 0.8, 1.0));
	light->setLightGroupReference(SurgSim::Graphics::Representation::DefaultGroupName);

	auto lightElement = std::make_shared<Framework::BasicSceneElement>("LightElement");
	lightElement->setPose(makeRigidTransform(
			Math::Vector3d(0.5, 0.5, 0.5),
			Math::Vector3d(0.0 ,0.0, 0.0),
			Math::Vector3d(0.0, 1.0, 0.0)));
	lightElement->addComponent(light);
	scene->addSceneElement(lightElement);

	auto axes = std::make_shared<Graphics::OsgAxesRepresentation>("Axes");
	lightElement->addComponent(axes);

	std::array<double, 6> lightProjection = { -2.0, 2.0, -2.0, 2.0, -1.0, 2.0 };
	scene->addSceneElements(Blocks::createShadowMapping(viewElement->getCamera(), light,
														4096, 1024, lightProjection, 0.002, 0.75, true, 4.0, false));

	scene->addSceneElements(Blocks::createImplicitSurfaceEffect(viewElement->getView(), light,
																scene->getSceneElement(Blocks::GROUP_SHADOW_CASTER),
																0.01f, 800.0f, 4.0,
																diffuseColor, specularColor,
																"Textures/CubeMap_reflection_diffuse.png", 0.9,
																"Textures/CubeMap_reflection_specular.png", 0.1,
																100.0f, false));

	auto cube = std::make_shared<Graphics::OsgBoxRepresentation>("Cube");
	cube->setSizeXYZ(0.1, 0.1, 0.1);

	auto element = std::make_shared<Framework::BasicSceneElement>("box");
	element->setPose(makeRigidTranslation(Math::Vector3d(0.0, 0.0, 0.25)));
	element->addComponent(cube);

	scene->addSceneElement(element);

	auto sphere = std::make_shared<Graphics::OsgSphereRepresentation>("Graphics");
	sphere->setRadius(0.1);

	auto material = Graphics::buildMaterial("Shaders/material.vert", "Shaders/material.frag");
	material->addUniform("vec4", "diffuseColor");
	material->setValue("diffuseColor", diffuseColor);
	material->addUniform("vec4", "specularColor");
	material->setValue("specularColor", specularColor);
	material->addUniform("float", "shininess");
	material->setValue("shininess", 10.0f);
	sphere->setMaterial(material);

	element = std::make_shared<Framework::BasicSceneElement>("Sphere");
	element->setPose(makeRigidTranslation(Math::Vector3d(0.25, 0.0, 0.0)));
	element->addComponent(sphere);
	element->addComponent(material);

	scene->addSceneElement(element);

	// Create the point cloud
	auto mesh = std::make_shared<Graphics::OsgMeshRepresentation>("Mesh");
	mesh->loadMesh("Geometry/sphere.ply");

	auto graphics = std::make_shared<Graphics::OsgPointCloudRepresentation>("Cloud");

	for (const auto& vertex : mesh->getMesh()->getVertices())
	{
		graphics->getVertices()->addVertex(Graphics::PointCloud::VertexType(vertex));
	}

	graphics->setGroupReference(Blocks::GROUP_IMPLICIT_SURFACE);

	auto sceneElement = std::make_shared<Framework::BasicSceneElement>("PointSprites");
	sceneElement->addComponent(graphics);

	scene->addSceneElement(sceneElement);


	runtime->start();
	boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
	runtime->stop();
}

}; // namespace Graphics
}; // namespace SurgSim
