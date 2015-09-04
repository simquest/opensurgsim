// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest Solutions Inc.
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

#include "SurgSim/Blocks/ShadowMapping.h"

#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Graphics/OsgScreenSpaceQuadRepresentation.h"
#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgRenderTarget.h"
#include "SurgSim/Graphics/Program.h"
#include "SurgSim/Graphics/RenderPass.h"
#include "SurgSim/Graphics/OsgScreenSpacePass.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/TransferPropertiesBehavior.h"
#include "SurgSim/Framework/PoseComponent.h"
#include "SurgSim/Blocks/GraphicsUtilities.h"
#include "SurgSim/Graphics/OsgLight.h"

#include <osg/PolygonMode>

namespace SurgSim
{

namespace Blocks
{

std::shared_ptr<Graphics::Camera> setupBlurPasses(
	std::shared_ptr<Graphics::RenderPass> previousPass,
	int textureSize,
	double blurRadius,
	bool debug,
	std::vector<std::shared_ptr<Framework::SceneElement>>* elements)
{
	std::vector<std::shared_ptr<Framework::SceneElement>> result;
	float shadowMapSize = static_cast<float>(textureSize);
	float floatRadius = static_cast<float>(blurRadius);

	std::shared_ptr<Graphics::Camera> previous = previousPass->getCamera();

	// Horizontal Pass
	{
		auto pass = std::make_shared<Graphics::RenderPass>("HorizontalBlurPass");
		pass->getCamera()->setOrthogonalProjection(0, textureSize, 0, textureSize, -1.0, 1.0);

		auto renderTarget = std::make_shared<Graphics::OsgRenderTarget2d>(textureSize, textureSize, 1.0, 1, false);
		pass->setRenderTarget(renderTarget);
		pass->setRenderOrder(Graphics::Camera::RENDER_ORDER_PRE_RENDER, 2);

		// Material
		auto material = Graphics::buildMaterial("Shaders/gauss_blur_horizontal.vert",
												"Shaders/gauss_blur.frag");
		material->getProgram()->setGlobalScope(true);
		material->addUniform("float", "width");
		material->setValue("width", shadowMapSize);
		material->addUniform("float", "blurRadius");
		material->setValue("blurRadius", floatRadius);
		pass->setMaterial(material);

		// Quad
		auto graphics = std::make_shared<Graphics::OsgScreenSpaceQuadRepresentation>("Quad");
		graphics->setSize(textureSize, textureSize);
		graphics->setLocation(0, 0);
		graphics->setTexture(previous->getRenderTarget()->getColorTarget(0));
		graphics->setGroupReference("HorizontalBlurPass");
		pass->addComponent(graphics);

		if (debug)
		{
			pass->showColorTarget(512, 0, 256, 256);
		}
		previous = pass->getCamera();
		elements->push_back(pass);
	}

	// Vertical Pass
	{
		auto pass = std::make_shared<Graphics::RenderPass>("VerticalBlurPass");
		pass->getCamera()->setOrthogonalProjection(0, textureSize, 0, textureSize, -1.0, 1.0);

		auto renderTarget = std::make_shared<Graphics::OsgRenderTarget2d>(textureSize, textureSize, 1.0, 1, false);
		pass->setRenderTarget(renderTarget);
		pass->setRenderOrder(Graphics::Camera::RENDER_ORDER_PRE_RENDER, 3);

		// Material
		auto material = Graphics::buildMaterial("Shaders/gauss_blur_vertical.vert",
												"Shaders/gauss_blur.frag");
		material->getProgram()->setGlobalScope(true);
		material->addUniform("float", "height");
		material->setValue("height", shadowMapSize);
		material->addUniform("float", "blurRadius");
		material->setValue("blurRadius", floatRadius);
		pass->setMaterial(material);

		// Quad
		auto graphics = std::make_shared<Graphics::OsgScreenSpaceQuadRepresentation>("Quad");
		graphics->setSize(textureSize, textureSize);
		graphics->setLocation(0, 0);
		graphics->setTexture(previousPass->getRenderTarget()->getColorTarget(0));
		graphics->setGroupReference("VerticalBlurPass");
		pass->addComponent(graphics);

		if (debug)
		{
			pass->showColorTarget(756, 0, 256, 256);
		}
		elements->push_back(pass);
	}

	return previous;
}

/// Create the pass that renders the scene from the view of the light source
/// the identifier GROUP_SHADOW_CASTER is used in all graphic objects to mark them as used
/// in this pass
std::shared_ptr<Graphics::RenderPass> createLightMapPass(int textureSize, bool debug)
{
	auto pass = std::make_shared<Graphics::RenderPass>(GROUP_SHADOW_CASTER);
	auto renderTarget = std::make_shared<Graphics::OsgRenderTarget2d>(textureSize, textureSize, 1.0, 0, true);
	pass->setRenderTarget(renderTarget);
	pass->setRenderOrder(Graphics::Camera::RENDER_ORDER_PRE_RENDER, 0);
	std::dynamic_pointer_cast<Graphics::OsgCamera>(pass->getCamera())->getOsgCamera()->setReferenceFrame(
		osg::Transform::ABSOLUTE_RF);

	auto material = Graphics::buildMaterial("Shaders/depth_map.vert", "Shaders/depth_map.frag");
	material->getProgram()->setGlobalScope(true);
	pass->setMaterial(material);

	if (debug)
	{
		pass->showDepthTarget(0, 0, 256, 256);
	}

	return pass;
}

/// Create the pass that renders shadowed pixels into the scene,
/// the identifier  GROUPD_SHADOW_RECEIVER is used in all graphics objects to mark them
/// as used in this pass
std::shared_ptr<Graphics::RenderPass> createShadowMapPass(int textureSize, bool debug)
{
	auto pass = std::make_shared<Graphics::RenderPass>(GROUP_SHADOW_RECEIVER);
	auto renderTarget = std::make_shared<Graphics::OsgRenderTarget2d>(textureSize, textureSize, 1.0, 1, false);
	pass->setRenderTarget(renderTarget);
	pass->setRenderOrder(Graphics::Camera::RENDER_ORDER_PRE_RENDER, 1);
	std::dynamic_pointer_cast<Graphics::OsgCamera>(pass->getCamera())->getOsgCamera()->setReferenceFrame(
		osg::Transform::ABSOLUTE_RF);


	auto material = Graphics::buildMaterial("Shaders/shadow_map.vert", "Shaders/shadow_map.frag");
	material->getProgram()->setGlobalScope(true);
	pass->setMaterial(material);

	if (debug)
	{
		pass->showColorTarget(256, 0, 256, 256);
	}

	return pass;
}


std::vector<std::shared_ptr<Framework::SceneElement>> createShadowMapping(
			std::shared_ptr<Framework::Component> camera,
			std::shared_ptr<Framework::Component> light,
			int depthTextureSize,
			int shadowTextureSize,
			std::array<double, 6> lightCameraProjection,
			bool useBlur,
			double blurRadius,
			bool showDebug)
{
	SURGSIM_ASSERT(camera != nullptr) << "Camera can't be nullptr.";
	SURGSIM_ASSERT(light != nullptr) << "Light can't be nullptr.";

	std::vector<std::shared_ptr<Framework::SceneElement>> result;

	auto osgCamera = Framework::checkAndConvert<Graphics::OsgCamera>(camera, "SurgSim::Graphics::OsgCamera");
	auto osgLight = Framework::checkAndConvert<Graphics::OsgLight>(light, "SurgSim::Graphics::OsgLight");

	auto lightMapPass = createLightMapPass(depthTextureSize, showDebug);
	result.push_back(lightMapPass);

	lightMapPass->getCamera()->setValue("OrthogonalProjection", lightCameraProjection);

	auto cameraNode = std::dynamic_pointer_cast<Graphics::OsgCamera>(lightMapPass->getCamera())->getOsgCamera();
	cameraNode->getOrCreateStateSet()->setAttributeAndModes(
		new osg::PolygonMode(osg::PolygonMode::BACK, osg::PolygonMode::FILL), osg::StateAttribute::ON);

	auto copier = std::make_shared<Framework::TransferPropertiesBehavior>("Copier");
	copier->setTargetManagerType(Framework::MANAGER_TYPE_GRAPHICS);

	// connect the light pose and the light map camera pose, so when the light moves,
	// this camera will move as well
	// Stapling has the light and the camera on the same element, the view, take the view element and connect the pose
	// to the light map pass
	copier->connect(osgLight, "Pose", lightMapPass->getPoseComponent(), "Pose");

	auto shadowMapPass = createShadowMapPass(shadowTextureSize, showDebug);
	result.push_back(shadowMapPass);

	shadowMapPass->getMaterial()->addUniform("mat4", "lightViewMatrix");
	copier->connect(lightMapPass->getCamera(), "FloatViewMatrix",
					shadowMapPass->getMaterial(), "lightViewMatrix");

	auto shadowMaterial = shadowMapPass->getMaterial();
	// The projection matrix of the camera used to render the light map
	shadowMaterial->addUniform("mat4", "lightProjectionMatrix");
	copier->connect(lightMapPass->getCamera(), "FloatProjectionMatrix",
					shadowMaterial, "lightProjectionMatrix");

	// Get the result of the lightMapPass and pass it on to the shadowMapPass, because it is used
	// in a pass we ask the system to use a higher than normal texture unit (in this case 8) for
	// this texture, this prevents the texture from being overwritten by other textures

	shadowMaterial->addUniform("sampler2D", "depthMap");
	shadowMaterial->setValue("depthMap", lightMapPass->getRenderTarget()->getDepthTarget());
	shadowMaterial->getUniform("depthMap")->setValue("MinimumTextureUnit", static_cast<size_t>(8));

	// Make the camera in the shadowMapPass follow the main camera that is being used to render the
	// whole scene
	auto shadowCamera = shadowMapPass->getCamera();
	copier->connect(osgCamera, "ProjectionMatrix", shadowCamera , "ProjectionMatrix");
	copier->connect(osgCamera, "Pose", shadowCamera, "LocalPose");

	// Put the result of the last pass into the main camera to make it accessible
	auto material = std::make_shared<Graphics::OsgMaterial>("camera material");
	material->addUniform("sampler2D", "shadowMap");

	std::shared_ptr<Graphics::Texture> texture;

	if (useBlur)
	{
		auto blurrPass = setupBlurPasses(
							 shadowMapPass,
							 shadowTextureSize,
							 static_cast<float>(blurRadius),
							 showDebug,
							 &result);
		texture = blurrPass->getRenderTarget()->getColorTarget(0);
	}
	else
	{
		texture = shadowCamera->getRenderTarget()->getColorTarget(0);
	}

	material->setValue("shadowMap", texture);
	material->getUniform("shadowMap")->setValue("MinimumTextureUnit", static_cast<size_t>(8));

	osgCamera->getSceneElement()->addComponent(material);
	osgCamera->setMaterial(material);

	lightMapPass->addComponent(copier);

	return result;
}

}
}
