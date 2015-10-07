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
#include "SurgSim/DataStructures/Vertices.h"
#include "SurgSim/Framework/BehaviorManager.h"
#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Graphics/OsgBoxRepresentation.h"
#include "SurgSim/Graphics/Camera.h"
#include "SurgSim/Graphics/OsgLight.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgMeshRepresentation.h"
#include "SurgSim/Graphics/OsgPointCloudRepresentation.h"
#include "SurgSim/Graphics/OsgView.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Graphics/RenderTests/RenderTest.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"

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
	std::array<int, 2> dimensions = {1280, 720};
	viewElement->getView()->setDimensions(dimensions);
	viewElement->getCamera()->setPerspectiveProjection(45, 1.7, 0.01, 10.0);
	viewElement->enableManipulator(true);


	auto light = std::make_shared<Graphics::OsgLight>("Light");
	light->setDiffuseColor(Math::Vector4d(1.0, 1.0, 1.0, 1.0));
	light->setSpecularColor(Math::Vector4d(0.8, 0.8, 0.8, 1.0));
	light->setLightGroupReference(SurgSim::Graphics::Representation::DefaultGroupName);

	auto lightElement = std::make_shared<Framework::BasicSceneElement>("LightElement");
	lightElement->setPose(makeRigidTranslation(Math::Vector3d(1.0, 1.0, 1.0)));
	lightElement->addComponent(light);
	scene->addSceneElement(lightElement);

	std::vector<std::shared_ptr<Framework::SceneElement>> surface =
			Blocks::createImplicitSurfaceEffect(viewElement->getView(), viewElement->getCamera(), light, 0.01f, 800.0f,
												1024, Math::Vector4f(0.3, 0.0, 0.05, 1.0),
												Math::Vector4f(0.4, 0.4, 0.4, 1.0), 10, false);

	for (auto element : surface)
	{
		scene->addSceneElement(element);
	}

	auto cube = std::make_shared<Graphics::OsgBoxRepresentation>("Cube");
	cube->setSizeXYZ(0.1, 0.1, 0.1);

	auto element = std::make_shared<Framework::BasicSceneElement>("box");
	element->setPose(makeRigidTranslation(Math::Vector3d(0.0, 0.0, 0.25)));
	element->addComponent(cube);

	scene->addSceneElement(element);

	// Create the point cloud
	auto mesh = std::make_shared<Graphics::OsgMeshRepresentation>("Mesh");
	mesh->loadMesh("Geometry/sphere.ply");

	auto graphics = std::make_shared<Graphics::OsgPointCloudRepresentation>("Cloud");

	// graphics->setLocalPose(makeRigidTranslation(Math::Vector3d(0.0, 0.0, 0.0)));
	for (const auto& vertex : mesh->getMesh()->getVertices())
	{
		graphics->getVertices()->addVertex(Graphics::PointCloud::VertexType(vertex));
	}

	graphics->addGroupReference(Blocks::GROUP_IMPLICIT_SURFACE);

	auto sceneElement = std::make_shared<Framework::BasicSceneElement>("PointSprites");
	sceneElement->addComponent(graphics);

	scene->addSceneElement(sceneElement);


	runtime->start();
	boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
	runtime->stop();
}

}; // namespace Graphics
}; // namespace SurgSim
