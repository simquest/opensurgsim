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

// This test uses the Stanford Bunny from http://graphics.stanford.edu/data/3Dscanrep/

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
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::Math::makeRigidTranslation;

namespace SurgSim
{
namespace Graphics
{

TEST(ImplicitSurfaceRenderTests, PointSpriteFluid)
{
	auto runtime = std::make_shared<Framework::Runtime>("config.txt");
	auto graphicsManager = std::make_shared<Graphics::OsgManager>();

	runtime->addManager(graphicsManager);
	runtime->addManager(std::make_shared<Framework::BehaviorManager>());

	auto scene = runtime->getScene();

	auto viewElement = std::make_shared<Graphics::OsgViewElement>("view element");
	std::array<int, 2> position = {100, 100};
	//viewElement->getView()->setPosition(position);
	//viewElement->getView()->setWindowBorderEnabled(true);
	viewElement->getView()->setFullScreen(true);

	scene->addSceneElement(viewElement);

	viewElement->enableManipulator(true);


	auto light = std::make_shared<Graphics::OsgLight>("Light");
	light->setDiffuseColor(Math::Vector4d(1.0, 1.0, 1.0, 1.0));
	light->setSpecularColor(Math::Vector4d(0.8, 0.8, 0.8, 1.0));
	light->setLightGroupReference(SurgSim::Graphics::Representation::DefaultGroupName);
	light->setLocalPose(makeRigidTranslation(Math::Vector3d(-0.5, 1.5, -5.0)));

	auto lightElement = std::make_shared<Framework::BasicSceneElement>("LightElement");
	lightElement->addComponent(light);
	scene->addSceneElement(lightElement);

	std::vector<std::shared_ptr<Framework::SceneElement>> surface =
		Blocks::createImplicitSurfaceEffect(viewElement->getView(), viewElement->getCamera(), light, 0.01f, 800.0f, 1024,
										Math::Vector4f(0.3, 0.0, 0.05, 1.0), Math::Vector4f(1.0, 1.0, 1.0, 1.0),
										100, false);

	for (auto element : surface)
	{
		scene->addSceneElement(element);
	}

	auto cube = std::make_shared<Graphics::OsgBoxRepresentation>("Cube");
	cube->setSizeXYZ(0.1, 0.1, 0.1);

	auto element = std::make_shared<Framework::BasicSceneElement>("box");
	element->addComponent(cube);

	scene->addSceneElement(element);

	// Create the point cloud
	auto bunny = std::make_shared<Graphics::OsgMeshRepresentation>("Bunny");
	bunny->loadMesh("Geometry/stanford_bunny.ply");

	auto graphics = std::make_shared<Graphics::OsgPointCloudRepresentation>("Cloud");

	graphics->setLocalPose(makeRigidTranslation(Math::Vector3d(0.01, -0.1, -0.25)));
	for (const auto& vertex : bunny->getMesh()->getVertices())
	{
		graphics->getVertices()->addVertex(Graphics::PointCloud::VertexType(vertex));
	}

	graphics->addGroupReference(Blocks::GROUP_IMPLICIT_SURFACE);

	auto sceneElement = std::make_shared<Framework::BasicSceneElement>("PointSprites");
	sceneElement->addComponent(graphics);

	scene->addSceneElement(sceneElement);


	runtime->start();
	boost::this_thread::sleep(boost::posix_time::milliseconds(100000));
	runtime->stop();
}

}; // namespace Graphics
}; // namespace SurgSim
