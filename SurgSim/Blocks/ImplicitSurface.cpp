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
#include "SurgSim/Graphics/OsgLight.h"
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
		std::shared_ptr<Graphics::Camera> camera,
		float sphereRadius,
		float sphereScale,
		int textureSize,
		bool debug)
{
	auto renderPass = std::make_shared<Graphics::RenderPass>("ImplicitSurfaceDepthPass");
	renderPass->getCamera()->setRenderGroupReference(GROUP_IMPLICIT_SURFACE);
	renderPass->getCamera()->setRenderOrder(Graphics::Camera::RENDER_ORDER_PRE_RENDER, 0);

	copier->connect(camera, "Pose", renderPass->getCamera(), "LocalPose");
	copier->connect(camera, "ProjectionMatrix", renderPass->getCamera() , "ProjectionMatrix");

	auto renderTarget = std::make_shared<Graphics::OsgRenderTarget2d>(textureSize, textureSize, 1.0, 0, true);
	renderPass->setRenderTarget(renderTarget);

	auto material = Graphics::buildMaterial("Shaders/implicit_surface/depth.vert",
											"Shaders/implicit_surface/depth.frag");

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
		int textureSize,
		bool debug)
{
	auto renderPass = std::make_shared<Graphics::RenderPass>("ImplicitSurfaceNormalPass");
	renderPass->getCamera()->setOrthogonalProjection(0, textureSize, 0, textureSize, -1.0, 1.0);
	renderPass->getCamera()->setRenderOrder(Graphics::Camera::RENDER_ORDER_PRE_RENDER, 1);

	auto renderTarget = std::make_shared<Graphics::OsgRenderTarget2d>(textureSize, textureSize, 1.0, 1, false);
	renderPass->setRenderTarget(renderTarget);

	auto material = Graphics::buildMaterial("Shaders/implicit_surface/normal.vert",
											"Shaders/implicit_surface/normal.frag");

	material->addUniform("sampler2D", "depthMap");
	material->setValue("depthMap", depthMap);
	material->getUniform("depthMap")->setValue("MinimumTextureUnit", static_cast<size_t>(8));
	material->addUniform("float", "texelSize");
	material->setValue("texelSize", static_cast<float>(1.0 / textureSize));
	material->addUniform("mat4", "mainInverseProjectionMatrix");

	copier->connect(camera, "FloatInverseProjectionMatrix", material, "mainInverseProjectionMatrix");

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

std::shared_ptr<Graphics::RenderPass> createShadingPass(
	std::shared_ptr<Framework::TransferPropertiesBehavior> copier,
	std::shared_ptr<Graphics::Camera> camera,
	std::shared_ptr<Graphics::Light> light,
	std::shared_ptr<Graphics::Texture> depthMap,
	std::shared_ptr<Graphics::Texture> normalMap,
	const Math::Vector4f& diffuseColor,
	const Math::Vector4f& specularColor,
	float shininess,
	int width, int height)
{
	auto renderPass = std::make_shared<Graphics::RenderPass>("ImplicitSurfaceShadingPass");

	auto renderCamera = std::dynamic_pointer_cast<Graphics::OsgCamera>(renderPass->getCamera());
	renderCamera->setViewport(0, 0, width, height);
	renderCamera->getOsgCamera()->setProjectionMatrixAsOrtho2D(0, width, 0, height);
	renderCamera->getOsgCamera()->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	renderCamera->getOsgCamera()->setClearMask(GL_NONE);
	renderCamera->setRenderOrder(Graphics::Camera::RENDER_ORDER_POST_RENDER, 0);

	auto material = Graphics::buildMaterial("Shaders/implicit_surface/shading.vert",
											"Shaders/implicit_surface/shading.frag");
	material->addUniform("sampler2D", "depthMap");
	material->setValue("depthMap", depthMap);
	material->getUniform("depthMap")->setValue("MinimumTextureUnit", static_cast<size_t>(8));
	material->addUniform("sampler2D", "normalMap");
	material->setValue("normalMap", normalMap);
	material->getUniform("normalMap")->setValue("MinimumTextureUnit", static_cast<size_t>(9));
	material->addUniform("vec3", "light");
	material->setValue("light", light->getPose().translation().cast<float>().eval());
	material->addUniform("vec4", "diffuseColor");
	material->setValue("diffuseColor", diffuseColor);
	material->addUniform("vec4", "specularColor");
	material->setValue("specularColor", specularColor);
	material->addUniform("float", "shininess");
	material->setValue("shininess", shininess);
	material->addUniform("mat4", "mainInverseProjectionMatrix");

	copier->connect(camera, "FloatInverseProjectionMatrix", material, "mainInverseProjectionMatrix");

	renderPass->setMaterial(material);

	auto graphics = std::make_shared<Graphics::OsgScreenSpaceQuadRepresentation>("Graphics");
	graphics->setSize(width , height);
	graphics->setLocation(0, 0);
	graphics->setGroupReference("ImplicitSurfaceShadingPass");
	renderPass->addComponent(graphics);

	return renderPass;
}

std::vector<std::shared_ptr<Framework::SceneElement>> createImplicitSurface(
			std::shared_ptr<Framework::Component> camera,
			std::shared_ptr<Framework::Component> light,
			float sphereRadius,
			float sphereScale,
			int textureSize,
			const Math::Vector4f& diffuseColor,
			const Math::Vector4f& specularColor,
			float shininess,
			bool showDebug)
{
	SURGSIM_ASSERT(camera != nullptr) << "Camera can't be nullptr.";
	SURGSIM_ASSERT(light != nullptr) << "Light can't be nullptr.";
	auto osgCamera = Framework::checkAndConvert<Graphics::OsgCamera>(camera, "SurgSim::Graphics::OsgCamera");
	auto osgLight = Framework::checkAndConvert<Graphics::OsgLight>(light, "SurgSim::Graphics::OsgLight");

	auto copier =  std::make_shared<Framework::TransferPropertiesBehavior>("Copier");
	copier->setTargetManagerType(SurgSim::Framework::MANAGER_TYPE_GRAPHICS);

	int x, y, width, height;
	osgCamera->getViewport(&x, &y, &width, &height);

	auto depthPass = createDepthPass(copier, osgCamera, sphereRadius, sphereScale, textureSize, showDebug);

	auto normalPass = createNormalPass(copier, osgCamera, depthPass->getRenderTarget()->getDepthTarget(),
									   textureSize, showDebug);

	auto shadingPass = createShadingPass(copier, osgCamera, osgLight,
									depthPass->getRenderTarget()->getDepthTarget(),
									normalPass->getRenderTarget()->getColorTarget(0),
									diffuseColor, specularColor, shininess, width, height);

	depthPass->addComponent(copier);

	std::vector<std::shared_ptr<Framework::SceneElement>> result;
	result.push_back(depthPass);
	result.push_back(normalPass);
	result.push_back(shadingPass);

	return result;
}
} // Blocks
} // SurgSim
