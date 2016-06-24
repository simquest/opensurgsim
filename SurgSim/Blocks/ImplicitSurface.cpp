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

#include "SurgSim/Blocks/GraphicsUtilities.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/Runtime.h"
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
std::shared_ptr<Graphics::Camera> createBlurPass(
		std::shared_ptr<Graphics::RenderPass> depthPass,
		int textureWidth,
		int textureHeight,
		double blurRadius,
		std::vector<std::shared_ptr<Framework::SceneElement>>* elements,
		bool debug)
{
	float floatRadius = static_cast<float>(blurRadius);

	std::shared_ptr<Graphics::Camera> previousCamera = depthPass->getCamera();

	// Horizontal Pass
	{
		auto renderPass = std::make_shared<Graphics::RenderPass>("ImplicitSurfaceHorizontalBlurPass");
		renderPass->getCamera()->setOrthogonalProjection(0, textureWidth, 0, textureHeight, -1.0, 1.0);

		auto renderTarget = std::make_shared<Graphics::OsgRenderTarget2d>(textureWidth, textureHeight, 1.0, 0, true);
		renderPass->setRenderTarget(renderTarget);
		renderPass->getCamera()->setRenderOrder(Graphics::Camera::RENDER_ORDER_PRE_RENDER, 1);

		auto material = Graphics::buildMaterial("Shaders/gauss_blur_horizontal.vert", "Shaders/bilateral_blur.frag");
		material->addUniform("sampler2D", "texture");
		material->setValue("texture", previousCamera->getRenderTarget()->getDepthTarget());
		material->addUniform("float", "width");
		material->setValue("width", static_cast<float>(textureWidth));
		material->addUniform("float", "blurRadius");
		material->setValue("blurRadius", floatRadius);
		renderPass->setMaterial(material);

		// Quad
		auto graphics = std::make_shared<Graphics::OsgScreenSpaceQuadRepresentation>("Quad");
		graphics->setSize(textureWidth, textureHeight);
		graphics->setLocation(0, 0);
		graphics->setGroupReference("ImplicitSurfaceHorizontalBlurPass");
		renderPass->addComponent(graphics);

		if(debug)
		{
			renderPass->showDepthTarget(0, 256, 256, 256);
		}
		previousCamera = renderPass->getCamera();
		elements->push_back(renderPass);
	}

	// Vertical Pass
	{
		auto renderPass = std::make_shared<Graphics::RenderPass>("ImplicitSurfaceVerticalBlurPass");
		renderPass->getCamera()->setOrthogonalProjection(0, textureWidth, 0, textureHeight, -1.0, 1.0);

		auto renderTarget = std::make_shared<Graphics::OsgRenderTarget2d>(textureWidth, textureHeight, 1.0, 0, true);
		renderPass->setRenderTarget(renderTarget);
		renderPass->getCamera()->setRenderOrder(Graphics::Camera::RENDER_ORDER_PRE_RENDER, 2);

		auto material = Graphics::buildMaterial("Shaders/gauss_blur_vertical.vert", "Shaders/bilateral_blur.frag");
		material->addUniform("sampler2D", "texture");
		material->setValue("texture", previousCamera->getRenderTarget()->getDepthTarget());
		material->addUniform("float", "height");
		material->setValue("height", static_cast<float>(textureWidth));
		material->addUniform("float", "blurRadius");
		material->setValue("blurRadius", floatRadius);
		renderPass->setMaterial(material);

		// Quad
		auto graphics = std::make_shared<Graphics::OsgScreenSpaceQuadRepresentation>("Quad");
		graphics->setSize(textureWidth, textureHeight);
		graphics->setLocation(0, 0);
		graphics->setGroupReference("ImplicitSurfaceVerticalBlurPass");
		renderPass->addComponent(graphics);

		if(debug)
		{
			renderPass->showDepthTarget(256, 256, 256, 256);
		}
		previousCamera = renderPass->getCamera();
		elements->push_back(renderPass);
	}

	return previousCamera;
}
std::shared_ptr<Graphics::RenderPass> createDepthPass(
		float sphereRadius,
		float sphereScale,
		int textureWidth,
		int textureHeight,
		bool debug)
{
	auto renderPass = std::make_shared<Graphics::RenderPass>("ImplicitSurfaceDepthPass");
	renderPass->getCamera()->setRenderGroupReference(GROUP_IMPLICIT_SURFACE);

	auto renderTarget = std::make_shared<Graphics::OsgRenderTarget2d>(textureWidth, textureHeight, 1.0, 0, true);
	renderPass->setRenderTarget(renderTarget);
	renderPass->getCamera()->setRenderOrder(Graphics::Camera::RENDER_ORDER_PRE_RENDER, 0);

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
		std::shared_ptr<Graphics::Camera> camera,
		std::shared_ptr<Graphics::Texture> depthMap,
		int textureWidth,
		int textureHeight,
		bool debug)
{
	auto renderPass = std::make_shared<Graphics::RenderPass>("ImplicitSurfaceNormalPass");
	renderPass->getCamera()->setOrthogonalProjection(0, textureWidth, 0, textureHeight, -1.0, 1.0);

	auto renderTarget = std::make_shared<Graphics::OsgRenderTarget2d>(textureWidth, textureHeight, 1.0, 1, false);
	renderPass->setRenderTarget(renderTarget);
	renderPass->getCamera()->setRenderOrder(Graphics::Camera::RENDER_ORDER_PRE_RENDER, 3);

	auto material = Graphics::buildMaterial("Shaders/implicit_surface/normal.vert",
											"Shaders/implicit_surface/normal.frag");

	material->addUniform("sampler2D", "depthMap");
	material->setValue("depthMap", depthMap);
	material->getUniform("depthMap")->setValue("MinimumTextureUnit", static_cast<size_t>(8));
	material->addUniform("float", "texelSize");
	material->setValue("texelSize", static_cast<float>(1.0 / textureWidth));

	renderPass->setMaterial(material);

	// Quad
	auto graphics = std::make_shared<Graphics::OsgScreenSpaceQuadRepresentation>("Quad");
	graphics->setSize(textureWidth, textureHeight);
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
	std::shared_ptr<Graphics::View> view,
	std::shared_ptr<Graphics::Camera> camera,
	std::shared_ptr<Graphics::Light> light,
	std::shared_ptr<Graphics::Texture> depthMap,
	std::shared_ptr<Graphics::Texture> normalMap,
	const Math::Vector4f& diffuseColor,
	const Math::Vector4f& specularColor,
	const std::string diffuseEnvMap,
	float diffusePct,
	const std::string specularEnvMap,
	float specularPct,
	float shininess)
{
	std::array<int, 2> dimensions = view->getDimensions();

	auto renderPass = std::make_shared<Graphics::RenderPass>("ImplicitSurfaceShadingPass");

	auto renderCamera = std::dynamic_pointer_cast<Graphics::OsgCamera>(renderPass->getCamera());
	renderCamera->setAmbientColor(camera->getAmbientColor());
	renderCamera->getOsgCamera()->setProjectionMatrixAsOrtho2D(0, dimensions[0], 0, dimensions[1]);
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
	Blocks::enable2DTexture(material, "shadowMap", Graphics::SHADOW_TEXTURE_UNIT, "Textures/black.png");

	std::string filename;
	{
		// The Diffuse environment map
		SURGSIM_ASSERT(Framework::Runtime::getApplicationData()->tryFindFile(diffuseEnvMap, &filename))
				<< "Could not load diffuse environment map " << diffuseEnvMap;
		auto texture = std::make_shared<SurgSim::Graphics::OsgTextureCubeMap>();
		texture->loadImage(filename);
		material->addUniform("samplerCube", "diffuseEnvMap");
		material->setValue("diffuseEnvMap", texture);
		material->addUniform("float", "diffusePercent");
		material->setValue("diffusePercent", diffusePct);
	}

	{
		// The Specular environment map
		SURGSIM_ASSERT(Framework::Runtime::getApplicationData()->tryFindFile(specularEnvMap, &filename))
				<< "Could not load specular environment map " << specularEnvMap;
		auto texture = std::make_shared<SurgSim::Graphics::OsgTextureCubeMap>();
		texture->loadImage(filename);
		material->addUniform("samplerCube", "specularEnvMap");
		material->setValue("specularEnvMap", texture);
		material->addUniform("float", "specularPercent");
		material->setValue("specularPercent", specularPct);
	}

	renderPass->setMaterial(material);

	auto graphics = std::make_shared<Graphics::OsgScreenSpaceQuadRepresentation>("Graphics");
	graphics->setSize(dimensions[0], dimensions[1]);
	graphics->setLocation(0, 0);
	graphics->setGroupReference("ImplicitSurfaceShadingPass");
	renderPass->addComponent(graphics);

	return renderPass;
}

std::vector<std::shared_ptr<Framework::SceneElement>> createImplicitSurfaceEffect(
			std::shared_ptr<Framework::Component> view,
			std::shared_ptr<Framework::Component> light,
			float sphereRadius,
			float sphereScale,
			float blurRadius,
			const Math::Vector4f& diffuseColor,
			const Math::Vector4f& specularColor,
			const std::string diffuseEnvMap,
			float diffusePct,
			const std::string specularEnvMap,
			float specularPct,
			float shininess,
			bool showDebug)
{
	SURGSIM_ASSERT(view != nullptr) << "View can't be nullptr.";
	SURGSIM_ASSERT(light != nullptr) << "Light can't be nullptr.";
	auto graphicsView = Framework::checkAndConvert<Graphics::View>(view, "SurgSim::Graphics::View");
	auto osgCamera = Framework::checkAndConvert<Graphics::OsgCamera>(graphicsView->getCamera(),
																	 "SurgSim::Graphics::OsgCamera");
	auto osgLight = Framework::checkAndConvert<Graphics::OsgLight>(light, "SurgSim::Graphics::OsgLight");

	auto copier =  std::make_shared<Framework::TransferPropertiesBehavior>("Copier");
	copier->setTargetManagerType(SurgSim::Framework::MANAGER_TYPE_GRAPHICS);

	std::vector<std::shared_ptr<Framework::SceneElement>> result;

	auto dimensions = graphicsView->getDimensions();

	switch (graphicsView->getStereoMode())
	{
		case  Graphics::View::STEREO_MODE_VERTICAL_SPLIT:
			dimensions[0] /= 2;
			break;
		case Graphics::View::STEREO_MODE_HORIZONTAL_SPLIT:
			dimensions[1] /= 2;
			break;
		default:
			break;
	}

	auto depthPass = createDepthPass(sphereRadius, sphereScale, dimensions[0], dimensions[1], showDebug);

	auto blurPass = createBlurPass(depthPass, dimensions[0], dimensions[1], blurRadius, &result, showDebug);

	auto normalPass = createNormalPass(osgCamera, blurPass->getRenderTarget()->getDepthTarget(),
									   dimensions[0], dimensions[1], showDebug);

	auto shadingPass = createShadingPass(graphicsView, osgCamera, osgLight,
									blurPass->getRenderTarget()->getDepthTarget(),
									normalPass->getRenderTarget()->getColorTarget(0),
									diffuseColor, specularColor,
									diffuseEnvMap, diffusePct, specularEnvMap, specularPct, shininess);

	depthPass->addComponent(copier);

	result.push_back(depthPass);
	result.push_back(normalPass);
	result.push_back(shadingPass);

	return result;
}
} // Blocks
} // SurgSim
