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

#include "SurgSim/DataStructures/Vertices.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgMeshRepresentation.h"
#include "SurgSim/Graphics/OsgPointCloudRepresentation.h"
#include "SurgSim/Graphics/OsgProgram.h"
#include "SurgSim/Graphics/OsgRenderTarget.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Graphics/RenderPass.h"
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
	void createPointSpriteSpherePass(const float& sphereRadius, const Math::Vector4f& color)
	{
		// Create material to transport the Textures for the point sprite
		auto material = std::make_shared<Graphics::OsgMaterial>("psMaterial");
		auto program = SurgSim::Graphics::loadProgram(*runtime->getApplicationData(), "Shaders/pointsplat/sphere");
		ASSERT_TRUE(program != nullptr);
		material->setProgram(program);

		auto texture = std::make_shared<Graphics::OsgTexture2d>();
		texture->setIsPointSprite(true);
		auto pointSpriteUniform = std::make_shared<Graphics::OsgTextureUniform<Graphics::OsgTexture2d>>("pointsprite");
		pointSpriteUniform->set(texture);
		material->addUniform(pointSpriteUniform);

		material->addUniform("float", "sphereRadius");
		material->setValue("sphereRadius", sphereRadius);
		material->addUniform("float", "sphereScale");
		material->setValue("sphereScale", 100.0f);
		material->addUniform("vec4", "color");
		material->setValue("color", color);

		viewElement->getCamera()->setMaterial(material);
		viewElement->addComponent(material);
	}

	void createPointSpriteSphereDepthPass(const float& sphereRadius)
	{
		auto renderPass = std::make_shared<RenderPass>("DepthPass");
		renderPass->getCamera()->setProjectionMatrix(viewElement->getCamera()->getProjectionMatrix());
		renderPass->getCamera()->setRenderGroupReference("DepthPass");

		std::array<int, 2> dimensions = viewElement->getView()->getDimensions();

		auto renderTarget = std::make_shared<OsgRenderTarget2d>(dimensions[0], dimensions[1], 1.0, 1, true);
		renderPass->setRenderTarget(renderTarget);

		auto material = std::make_shared<Graphics::OsgMaterial>("depthPassMaterial");
		auto program = Graphics::loadProgram(*runtime->getApplicationData(),
													  "Shaders/pointsplat/sphere_depth");
		ASSERT_TRUE(program != nullptr);
		material->setProgram(program);

		auto texture = std::make_shared<Graphics::OsgTexture2d>();
		texture->setIsPointSprite(true);
		auto pointSpriteUniform = std::make_shared<Graphics::OsgTextureUniform<Graphics::OsgTexture2d>>("psDepth");
		pointSpriteUniform->set(texture);
		material->addUniform(pointSpriteUniform);

		material->addUniform("float", "sphereRadius");
		material->setValue("sphereRadius", sphereRadius);
		material->addUniform("float", "sphereScale");
		material->setValue("sphereScale", 100.0f);

		renderPass->setMaterial(material);
		viewElement->addComponent(renderPass->getCamera());
		viewElement->addComponent(material);

		int screenWidth = dimensions[0];
		int screenHeight = dimensions[1];

		int width = dimensions[0] / 3;
		int height = dimensions[1] / 3;

		auto element = std::make_shared<Framework::BasicSceneElement>("debug");
		scene->addSceneElement(element);
		auto quad = makeQuad("Depth", width, height, 0.0, screenHeight - height);
		quad->setTexture(renderTarget->getDepthTarget());
		element->addComponent(quad);

		// Normal Pass
		auto nRenderPass = std::make_shared<RenderPass>("NormalPass");
		nRenderPass->getCamera()->setProjectionMatrix(viewElement->getCamera()->getProjectionMatrix());
		nRenderPass->getCamera()->setRenderGroupReference("NormalPass");

		auto nRenderTarget = std::make_shared<OsgRenderTarget2d>(dimensions[0], dimensions[1], 1.0, 1, false);
		nRenderPass->setRenderTarget(nRenderTarget);

		auto normalMat = std::make_shared<Graphics::OsgMaterial>("normalPassMaterial");
		auto normalProg = Graphics::loadProgram(*runtime->getApplicationData(),
												"Shaders/pointsplat/sphere_normal");
		SURGSIM_ASSERT(normalProg != nullptr);
		normalMat->setProgram(normalProg);

		normalMat->addUniform("sampler2D", "depthMap");
		normalMat->setValue("depthMap", renderTarget->getDepthTarget());
		normalMat->addUniform("float", "texelSize");
		normalMat->setValue("texelSize", 0.1f);

		nRenderPass->setMaterial(normalMat);
		viewElement->addComponent(renderPass->getCamera());
		viewElement->addComponent(normalMat);

		auto normalElement = std::make_shared<Framework::BasicSceneElement>("normal");
		scene->addSceneElement(normalElement);
		auto normQuad = makeQuad("Normal", width, height, screenWidth - width, screenHeight - height);
		normQuad->setTexture(nRenderTarget->getColorTarget(0));
		normalElement->addComponent(normQuad);
	}
};

TEST_F(FluidRenderTests, PointSpriteDepth)
{
	viewElement->enableManipulator(true);
	createPointSpriteSpherePass(0.01f, Math::Vector4f(1.0, 0.0, 0.0, 1.0));
	createPointSpriteSphereDepthPass(0.01f);

	// Create the point cloud
	auto bunny = std::make_shared<Graphics::OsgMeshRepresentation>("Bunny");
	bunny->loadMesh("Geometry/stanford_bunny.ply");

	auto graphics = std::make_shared<Graphics::OsgPointCloudRepresentation>("Cloud");
	graphics->setPointSize(3.0f);

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
	boost::this_thread::sleep(boost::posix_time::milliseconds(100000));
	runtime->stop();
}

}; // namespace Graphics
}; // namespace SurgSim
