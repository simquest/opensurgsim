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
#include "SurgSim/Graphics/OsgBoxRepresentation.h"
#include "SurgSim/Graphics/OsgLight.h"
#include "SurgSim/Graphics/OsgMeshRepresentation.h"
#include "SurgSim/Graphics/OsgPointCloudRepresentation.h"
#include "SurgSim/Graphics/RenderTests/RenderTest.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Testing/MathUtilities.h"

using SurgSim::Math::makeRigidTranslation;

namespace SurgSim
{
namespace Graphics
{

struct FluidRenderTests : public RenderTest
{
protected:

};

TEST_F(FluidRenderTests, PointSpriteFluid)
{
	viewElement->enableManipulator(true);

	auto light = std::make_shared<Graphics::OsgLight>("Light");
	light->setDiffuseColor(Math::Vector4d(1.0, 1.0, 1.0, 1.0));
	light->setSpecularColor(Math::Vector4d(0.8, 0.8, 0.8, 1.0));
	light->setLightGroupReference(SurgSim::Graphics::Representation::DefaultGroupName);
	light->setLocalPose(makeRigidTranslation(Math::Vector3d(-0.5, 1.5, -5.0)));

	auto sphereRadius = std::make_shared<Graphics::OsgUniform<float>>("SphereRadius");
	sphereRadius->set(0.01f);

	auto sphereScale = std::make_shared<Graphics::OsgUniform<float>>("SphereScale");
	sphereScale->set(800.0f);

	auto textureSize = std::make_shared<Graphics::OsgUniform<int>>("TextureSize");
	textureSize->set(1024);

	auto color = std::make_shared<Graphics::OsgUniform<Math::Vector4f>>("Color");
	color->set(Math::Vector4f(0.3, 0.0, 0.05, 1.0));

	std::vector<std::shared_ptr<Framework::SceneElement>> surface =
		Blocks::createImplicitSurface(viewElement->getCamera(), light, sphereRadius, sphereScale, textureSize,
									  color, false);

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
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	graphicsManager->dumpDebugInfo();
	boost::this_thread::sleep(boost::posix_time::milliseconds(100000));
	runtime->stop();
}

}; // namespace Graphics
}; // namespace SurgSim
