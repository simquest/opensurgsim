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

#include "ImplicitSurface.h"

#include "SurgSim/Framework/PoseComponent.h"
#include "SurgSim/Framework/TransferPropertiesBehavior.h"
#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgRenderTarget.h"
#include "SurgSim/Graphics/OsgScreenSpaceQuadRepresentation.h"
#include "SurgSim/Graphics/OsgTexture2d.h"
#include "SurgSim/Graphics/OsgTextureUniform.h"
#include "SurgSim/Graphics/RenderPass.h"
#include "SurgSim/Graphics/View.h"

namespace SurgSim
{
namespace Blocks
{
std::shared_ptr<Graphics::RenderPass> createDepthPass(
	std::shared_ptr<Framework::TransferPropertiesBehavior> copier,
	std::shared_ptr<Graphics::OsgViewElement> viewElement,
	const float& sphereRadius,
	const float& sphereScale,
	int textureSize, bool debug)
{
	auto renderPass = std::make_shared<Graphics::RenderPass>("ImplicitSurfaceDepthPass");
	renderPass->getCamera()->setRenderGroupReference(GROUP_IMPLICIT_SURFACE);
	renderPass->getCamera()->setRenderOrder(Graphics::Camera::RENDER_ORDER_PRE_RENDER, 0);

	copier->connect(viewElement->getPoseComponent(), "Pose", renderPass->getCamera(), "LocalPose");
	copier->connect(viewElement->getCamera(), "ProjectionMatrix", renderPass->getCamera() , "ProjectionMatrix");

	auto renderTarget = std::make_shared<Graphics::OsgRenderTarget2d>(textureSize, textureSize, 1.0, 0, true);
	renderPass->setRenderTarget(renderTarget);

	auto material = Graphics::buildMaterial("Shaders/implicit_surface/depth.vert", "Shaders/implicit_surface/depth.frag");

	auto texture = std::make_shared<Graphics::OsgTexture2d>();
	texture->setIsPointSprite(true);
	auto pointSpriteUniform = std::make_shared<Graphics::OsgTextureUniform<Graphics::OsgTexture2d>>("PointSpriteDepth");
	pointSpriteUniform->set(texture);
	material->addUniform(pointSpriteUniform);

	material->addUniform("float", "sphereRadius");
	material->setValue("sphereRadius", sphereRadius);
	material->addUniform("float", "sphereScale");
	material->setValue("sphereScale", sphereScale);
	renderPass->setMaterial(material);

	if(debug)
	{
		renderPass->showDepthTarget(0, 0, 256, 256);
	}

	return renderPass;
}

std::shared_ptr<Graphics::RenderPass> createNormalPass(
	std::shared_ptr<Framework::TransferPropertiesBehavior> copier,
	std::shared_ptr<Graphics::Camera> camera,
	std::shared_ptr<Graphics::Texture> depthMap,
	int textureSize, bool debug)
{
	auto renderPass = std::make_shared<Graphics::RenderPass>("ImplicitSurfaceNormalPass");
	renderPass->getCamera()->setRenderGroupReference("ImplicitSurfaceNormalPass");
	renderPass->getCamera()->setGroupReference(Graphics::Representation::DefaultGroupName);

	auto renderCamera = std::dynamic_pointer_cast<Graphics::OsgCamera>(renderPass->getCamera());
	auto osgCamera = renderCamera->getOsgCamera();
	osgCamera->setViewport(0, 0, textureSize, textureSize);
	renderCamera->setOrthogonalProjection(0, textureSize, 0, textureSize, -1.0, 1.0);

	renderCamera->setRenderOrder(Graphics::Camera::RENDER_ORDER_PRE_RENDER, 1);

	auto renderTarget = std::make_shared<Graphics::OsgRenderTarget2d>(textureSize, textureSize, 1.0, 1, false);
	renderPass->setRenderTarget(renderTarget);

	std::shared_ptr<Graphics::OsgMaterial> material = Graphics::buildMaterial("Shaders/implicit_surface/normal.vert", "Shaders/implicit_surface/normal.frag");

	material->addUniform("sampler2D", "depthMap");
	material->setValue("depthMap", depthMap);
	material->getUniform("depthMap")->setValue("MinimumTextureUnit", static_cast<size_t>(8));
	material->addUniform("float", "texelSize");
	material->setValue("texelSize", static_cast<float>(1.0 / textureSize));
	material->addUniform("mat4", "inverseProjectionMatrix");

	copier->connect(camera, "FloatInverseProjectionMatrix", material, "inverseProjectionMatrix");

	renderPass->setMaterial(material);

	// Quad
	auto graphics = std::make_shared<Graphics::OsgScreenSpaceQuadRepresentation>("Quad");
	graphics->setSize(textureSize, textureSize);
	graphics->setLocation(0, 0);
	graphics->setGroupReference("ImplicitSurfaceNormalPass");
	renderPass->addComponent(graphics);

	if(debug)
	{
		renderPass->showColorTarget(256, 0, 256, 256);
	}

	return renderPass;
}

std::shared_ptr<Framework::SceneElement> createShadingQuad(
	std::shared_ptr<Framework::TransferPropertiesBehavior> copier,
	std::shared_ptr<Graphics::Camera> camera,
	std::shared_ptr<Graphics::Texture> depthMap,
	std::shared_ptr<Graphics::Texture> normalMap,
	const Math::Vector4f& color,
	int width, int height)
{
	auto element = std::make_shared<Framework::BasicSceneElement>("ImplicitSurfaceShading");

	auto material = Graphics::buildMaterial("Shaders/implicit_surface/shading.vert", "Shaders/implicit_surface/shading.frag");
	material->addUniform("sampler2D", "depthMap");
	material->setValue("depthMap", depthMap);
	material->getUniform("depthMap")->setValue("MinimumTextureUnit", static_cast<size_t>(8));
	material->addUniform("sampler2D", "normalMap");
	material->setValue("normalMap", normalMap);
	material->getUniform("normalMap")->setValue("MinimumTextureUnit", static_cast<size_t>(9));
	material->addUniform("vec4", "color");
	material->setValue("color", color);
	material->addUniform("mat4", "inverseProjectionMatrix");

	copier->connect(camera, "FloatInverseProjectionMatrix", material, "inverseProjectionMatrix");
	
	auto renderCamera = std::make_shared<Graphics::OsgCamera>("Camera");
	renderCamera->setViewport(0, 0, width, height);
	renderCamera->getOsgCamera()->setProjectionMatrixAsOrtho2D(0, width, 0, height);
	renderCamera->getOsgCamera()->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	renderCamera->getOsgCamera()->setClearMask(GL_NONE);
	renderCamera->setRenderOrder(Graphics::Camera::RENDER_ORDER_POST_RENDER, 0);
	renderCamera->setRenderGroupReference("ImplicitSurfaceShading");
	element->addComponent(renderCamera);
	
	auto graphics = std::make_shared<Graphics::OsgScreenSpaceQuadRepresentation>("Graphics");
	graphics->setSize(width , height);
	graphics->setLocation(0, 0);
	graphics->setMaterial(material);
	graphics->setGroupReference("ImplicitSurfaceShading");
	element->addComponent(graphics);
	element->addComponent(material);
	return element;
}

std::vector<std::shared_ptr<Framework::SceneElement>> createImplicitSurface(const float& sphereRadius, const float& sphereScale, const int& textureSize, const Math::Vector4f& color, std::shared_ptr<Graphics::OsgViewElement> viewElement, bool debug)
{
	auto copier =  std::make_shared<Framework::TransferPropertiesBehavior>("Copier");
	copier->setTargetManagerType(SurgSim::Framework::MANAGER_TYPE_GRAPHICS);
	viewElement->addComponent(copier);

	std::array<int, 2> dimensions = viewElement->getView()->getDimensions();
	int screenWidth = dimensions[0];
	int screenHeight = dimensions[1];

	auto depthPass = createDepthPass(copier, viewElement, sphereRadius, sphereScale, 1024, debug);

	auto normalPass = createNormalPass(copier, viewElement->getCamera(), depthPass->getRenderTarget()->getDepthTarget(), 1024, debug);


	// Shading Pass //

	auto shader = createShadingQuad(copier, viewElement->getCamera(),
									depthPass->getRenderTarget()->getDepthTarget(),
									normalPass->getRenderTarget()->getColorTarget(0),
									color, screenWidth, screenHeight);

	std::vector<std::shared_ptr<Framework::SceneElement>> result;
	result.push_back(depthPass);
	result.push_back(normalPass);
	result.push_back(shader);

	return result;
}
} // Blocks
} // SurgSim
