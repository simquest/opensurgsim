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
#include "SurgSim/Framework/Framework.h"
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
#include "SurgSim/Blocks/GraphicsUtilities.h"

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
		material->setValue("sphereScale", 200.0f);
		material->addUniform("vec4", "color");
		material->setValue("color", color);

// 		viewElement->getCamera()->setMaterial(material);
// 		viewElement->addComponent(material);
	}

	void createPointSpriteSphereFluidPass(const float& sphereRadius)
	{
		auto copier =  std::make_shared<Framework::TransferPropertiesBehavior>("Copier");
		copier->setTargetManagerType(SurgSim::Framework::MANAGER_TYPE_GRAPHICS);
		viewElement->addComponent(copier);

		std::array<int, 2> dimensions = viewElement->getView()->getDimensions();
		int screenWidth = dimensions[0];
		int screenHeight = dimensions[1];
		int width = dimensions[0] / 3;
		int height = dimensions[1] / 3;

		// Depth Pass //
		auto depthPass = std::make_shared<RenderPass>("DepthPass");
		depthPass->getCamera()->setRenderGroupReference("DepthPass");

		copier->connect(viewElement->getPoseComponent(), "Pose", depthPass->getCamera(), "LocalPose");
		copier->connect(viewElement->getCamera(), "ProjectionMatrix", depthPass->getCamera() , "ProjectionMatrix");

		auto renderTarget = std::make_shared<OsgRenderTarget2d>(1024, 1024, 1.0, 0, true);
		depthPass->setRenderTarget(renderTarget);

		auto material = std::make_shared<Graphics::OsgMaterial>("depthPassMaterial");
		auto program = Graphics::loadProgram(*runtime->getApplicationData(), "Shaders/pointsplat/sphere_depth");
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
        material->setValue("sphereScale", 800.0f);
		depthPass->setMaterial(material);

		depthPass->showDepthTarget(0.0, screenHeight - height, width, height);

		scene->addSceneElement(depthPass);


		// Normal Pass //
		auto normalPass = std::make_shared<RenderPass>("NormalPass");
		normalPass->getCamera()->setGroupReference(Graphics::Representation::DefaultGroupName);

		auto camera = std::dynamic_pointer_cast<SurgSim::Graphics::OsgCamera>(normalPass->getCamera());
		auto osgCamera = camera->getOsgCamera();
		osgCamera->setViewport(0, 0, 1024, 1024);
		camera->setOrthogonalProjection(0, 1024, 0, 1024, -1.0, 1.0);

		camera->setRenderOrder(Graphics::Camera::RENDER_ORDER_PRE_RENDER, 3);

		auto nRenderTarget = std::make_shared<OsgRenderTarget2d>(1024, 1024, 1.0, 1, false);
		normalPass->setRenderTarget(nRenderTarget);

		material = std::make_shared<Graphics::OsgMaterial>("NormalPassMaterial");
		program = Graphics::loadProgram(*runtime->getApplicationData(), "Shaders/pointsplat/sphere_normal");
		SURGSIM_ASSERT(program != nullptr);
		program->setGlobalScope(true);
		material->setProgram(program);

		material->addUniform("sampler2D", "depthMap");
		material->setValue("depthMap", renderTarget->getDepthTarget());
		material->getUniform("depthMap")->setValue("MinimumTextureUnit", static_cast<size_t>(8));
		material->addUniform("float", "texelSize");
		material->setValue("texelSize", static_cast<float>(1.0 / 1024.0));
		material->addUniform("mat4", "inverseProjectionMatrix");

// 		SurgSim::Math::Matrix44f inverse = viewElement->getCamera()->getProjectionMatrix().cast<float>();
// 		material->setValue("inverseProjectionMatrix", inverse);

		copier->connect(viewElement->getCamera(), "FloatInverseProjectionMatrix", material, "inverseProjectionMatrix");

		normalPass->setMaterial(material);

		texture = std::make_shared<SurgSim::Graphics::OsgTexture2d>();
		texture->loadImage("Textures/checkered.png");

		// Quad
		auto graphics = std::make_shared<Graphics::OsgScreenSpaceQuadRepresentation>("Quad`");
		graphics->setSize(1024, 1024);
		graphics->setLocation(0, 0);
		graphics->setGroupReference("NormalPass");
		normalPass->addComponent(graphics);

		normalPass->showColorTarget(screenWidth - width, screenHeight - height, width, height);

		camera->getOsgCamera()->setClearColor(osg::Vec4(0.8, 0.0, 0.0, 1.0));


		scene->addSceneElement(normalPass);
	}
};

TEST_F(FluidRenderTests, PointSpriteFluid)
{
	viewElement->enableManipulator(true);
	createPointSpriteSpherePass(0.01f, Math::Vector4f(1.0, 0.0, 0.0, 1.0));
	createPointSpriteSphereFluidPass(0.01f);

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
