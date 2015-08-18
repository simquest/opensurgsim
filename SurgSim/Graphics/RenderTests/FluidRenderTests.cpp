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

#include "SurgSim/Blocks/ImplicitFluidRendering.h"
#include "SurgSim/DataStructures/Vertices.h"
#include "SurgSim/Graphics/OsgBoxRepresentation.h"
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
//	void createPointSpriteSpherePass(const float& sphereRadius, const Math::Vector4f& color)
//	{
//		// Create material to transport the Textures for the point sprite
//		auto material = std::make_shared<Graphics::OsgMaterial>("psMaterial");
//		auto program = SurgSim::Graphics::loadProgram(*runtime->getApplicationData(), "Shaders/pointsplat/sphere");
//		ASSERT_TRUE(program != nullptr);
//		material->setProgram(program);

//		auto texture = std::make_shared<Graphics::OsgTexture2d>();
//		texture->setIsPointSprite(true);
//		auto pointSpriteUniform = std::make_shared<Graphics::OsgTextureUniform<Graphics::OsgTexture2d>>("pointsprite");
//		pointSpriteUniform->set(texture);
//		material->addUniform(pointSpriteUniform);

//		material->addUniform("float", "sphereRadius");
//		material->setValue("sphereRadius", sphereRadius);
//		material->addUniform("float", "sphereScale");
//		material->setValue("sphereScale", 200.0f);
//		material->addUniform("vec4", "color");
//		material->setValue("color", color);

//		viewElement->getCamera()->setMaterial(material);
//		viewElement->addComponent(material);
//	}
};

TEST_F(FluidRenderTests, PointSpriteFluid)
{
	viewElement->enableManipulator(true);
	viewElement->getView()->setTargetScreen(1);
// 	viewElement->setPose(SurgSim::Math::makeRigidTransform(Math::Vector3d(1.0, 1.0, 1.0), Math::Vector3d(0.0, 0.0, 0.0),
// 						 Math::Vector3d(0.0, 1.0, 0.0)));
	//createPointSpriteSpherePass(0.01f, Math::Vector4f(1.0, 0.0, 0.0, 1.0));
	Blocks::creatFluidRenderingPass(0.01f, viewElement, scene);

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

	graphics->addGroupReference("DepthPass");

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
