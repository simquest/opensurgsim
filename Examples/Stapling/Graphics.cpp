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

#include "Examples/Stapling/Graphics.h"

#include "SurgSim/Graphics/Graphics.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/PoseComponent.h"
#include "SurgSim/Framework/TransferPropertiesBehavior.h"
#include "SurgSim/Blocks/GraphicsUtilities.h"
#include "SurgSim/Math/Vector.h"

#include <string>

using namespace SurgSim;

using SurgSim::Math::Vector4f;


std::shared_ptr<SurgSim::Graphics::OsgMaterial> createMaterial(
	const std::string& materialName,
	const std::string& shaderName)
{

	auto program = SurgSim::Graphics::loadProgram(*Framework::Runtime::getApplicationData(), shaderName);

	std::shared_ptr<SurgSim::Graphics::OsgMaterial> material;
	if (program != nullptr)
	{
		material = std::make_shared<SurgSim::Graphics::OsgMaterial>(materialName);
		material->setProgram(program);
	}

	return material;
}

std::shared_ptr<SurgSim::Graphics::ScreenSpaceQuadRepresentation> makeDebugQuad(
	const std::string& name,
	std::shared_ptr<SurgSim::Graphics::Texture> texture,
	double x, double y, double width, double height)
{
	auto result = std::make_shared<SurgSim::Graphics::OsgScreenSpaceQuadRepresentation>(name);
	result->setTexture(texture);
	result->setSize(width, height);
	result->setLocation(x, y);
	return result;
}

std::unordered_map<std::string, std::shared_ptr<SurgSim::Graphics::OsgMaterial>>
		createMaterials(std::shared_ptr<Framework::Scene> scene)
{
	std::unordered_map<std::string, std::shared_ptr<Graphics::OsgMaterial>> result;
	auto element = std::make_shared<Framework::BasicSceneElement>("Materials");
	scene->addSceneElement(element);

	// Skin
	auto material = Blocks::createNormalMappedMaterial("skin",
					Vector4f(1.0, 1.0, 1.0, 1.0), Vector4f(0.4, 0.4, 0.4, 1.0), 10.0, "", "");
	result[material->getName()] = material;
	element->addComponent(material);

	// Staple
	material = Blocks::createPlainMaterial("staple", Vector4f(0.5, 0.5, 0.5, 1.0), Vector4f(0.4, 0.4, 0.4, 1.0), 2.0);
	result[material->getName()] = material;
	element->addComponent(material);

	// Plain Normal Map
	material = Blocks::createNormalMappedMaterial("normalmapped",
			   Vector4f(1.0, 1.0, 1.0, 1.0), Vector4f(1.0, 1.0, 1.0, 1.0), 1.0, "", "");
	result[material->getName()] = material;
	element->addComponent(material);

	// Textured
	material = Blocks::createTexturedMaterial("textured",
			   Vector4f(0.8, 0.8, 0.8, 1.0), Vector4f(0.5, 0.5, 0.5, 1.0), 1.0);
	result[material->getName()] = material;
	element->addComponent(material);

	// Shadow Mapping, don't add materials to scene, they get added in the renderpass
	material = createMaterial("depthMap", "Shaders/depth_map");
	material->getProgram()->setGlobalScope(true);
	result[material->getName()] = material;

	material = createMaterial("shadowMap", "Shaders/shadow_map");
	material->getProgram()->setGlobalScope(true);
	result[material->getName()] = material;
	// element->addComponent(material);

	// ShadowMap placeholder
	material = std::make_shared<Graphics::OsgMaterial>("placeholder");
	Blocks::enable2DTexture(material, "shadowMap", Graphics::SHADOW_TEXTURE_UNIT, "Textures/black.png");
	result[material->getName()] = material;
	element->addComponent(material);

	return result;
}

std::shared_ptr<SurgSim::Graphics::RenderPass> createPass(
	Materials materials,
	const std::string& passName,
	const std::string& materialName)
{
	auto pass = std::make_shared<SurgSim::Graphics::RenderPass>(passName);
	auto renderTarget = std::make_shared<SurgSim::Graphics::OsgRenderTarget2d>(1024, 1024, 1.0, 1, false);
	pass->setRenderTarget(renderTarget);
	pass->setRenderOrder(SurgSim::Graphics::Camera::RENDER_ORDER_PRE_RENDER, 1);
	auto material = materials[materialName];
	SURGSIM_ASSERT(material != nullptr);
	pass->setMaterial(material);
	return pass;
}

std::shared_ptr<Graphics::RenderPass> setupBlurPasses(
	Materials materials,
	std::shared_ptr<Framework::Scene> scene,
	std::shared_ptr<Graphics::RenderPass> previousPass)
{
	// Input of Horizontal and Vertical Blur passes is sampler2D 'texture'
	// Output is result of rendertarget
	// Set up the horizontal blur pass
	auto horizontalBlur = createPass(materials, "HorizontalBlur", "horizontalBlur");
	horizontalBlur->setRenderOrder(SurgSim::Graphics::Camera::RENDER_ORDER_PRE_RENDER, 2);
	scene->addSceneElement(horizontalBlur);

	// Put the result of the last pass into the first blur pass to make it accessible
	auto material =  horizontalBlur->getMaterial();
	material->setValue("texture", previousPass->getRenderTarget()->getColorTarget(0));

	auto quad = std::make_shared<Graphics::OsgScreenSpaceQuadRepresentation>("Quad");
	quad->clearGroupReferences();
	quad->addGroupReference("HorizontalBlur");
	quad->setSize(1024, 1024);
	quad->setTexture(previousPass->getRenderTarget()->getColorTarget(0));
	horizontalBlur->addComponent(quad);

	// Set up the vertical blurr pass
	auto verticalBlur = createPass(materials, "VerticalBlur", "verticalBlur");
	verticalBlur->setRenderOrder(SurgSim::Graphics::Camera::RENDER_ORDER_PRE_RENDER, 3);
	scene->addSceneElement(verticalBlur);

	material = verticalBlur->getMaterial();
	material->setValue("texture", verticalBlur->getRenderTarget()->getColorTarget(0));

	quad = std::make_shared<Graphics::OsgScreenSpaceQuadRepresentation>("Quad");
	quad->clearGroupReferences();
	quad->addGroupReference("VerticalBlur");
	quad->setSize(1024, 1024);
	quad->setTexture(horizontalBlur->getRenderTarget()->getColorTarget(0));
	verticalBlur->addComponent(quad);

	// Take the output of previousPass and put it into horizontalBlur
	// route output of horizontalPass and route it into verticalPass

	return verticalBlur;
}


void setupShadowMapping(std::unordered_map<std::string, std::shared_ptr<SurgSim::Graphics::OsgMaterial>>
						materials, std::shared_ptr<Framework::Scene> scene)
{
	auto viewElement = scene->getSceneElement("View");
	auto lightMapPass = createPass(materials, "shadowing", "depthMap");
	auto osgCamera = std::dynamic_pointer_cast<Graphics::OsgCamera>(lightMapPass->getCamera());
	osgCamera->getOsgCamera()->setProjectionMatrixAsPerspective(100, 1.0, 0.01, 10.0);

	// Setup shadow map pass by hand ...
	auto shadowMapPass = createPass(materials, "shadowed", "shadowMap");
	auto shadowCamera = std::make_shared<Graphics::OsgCamera>("ShadowCamera");
	shadowCamera->setRenderGroupReference("shadowing");

	auto shadowMaterial = materials["shadowMap"];
	shadowCamera->setMaterial(shadowMaterial);
	auto renderTarget = std::make_shared<SurgSim::Graphics::OsgRenderTarget2d>(1024, 1024, 1.0, 1, false);
	shadowCamera->setRenderTarget(renderTarget);
	shadowCamera->setRenderOrder(SurgSim::Graphics::Camera::RENDER_ORDER_PRE_RENDER, 1);

	auto copier = std::make_shared<Framework::TransferPropertiesBehavior>("Copier");
	copier->setTargetManagerType(Framework::MANAGER_TYPE_GRAPHICS);
	viewElement->addComponent(copier);

	// connect the light pose and the light map camera pose, so when the light moves,
	// this camera will move as well
	// Stapling has the light and the camera on the same element, the view, take the view element and connect the pose
	// to the light map pass
	auto lightElement = scene->getSceneElement("Light");
	copier->connect(lightElement->getPoseComponent(), "Pose", lightMapPass->getPoseComponent(), "Pose");

	shadowMapPass->getMaterial()->addUniform("mat4", "lightViewMatrix");
	copier->connect(lightMapPass->getCamera(), "FloatViewMatrix",
					shadowMapPass->getMaterial(), "lightViewMatrix");

	// The projection matrix of the camera used to render the light map
	shadowMaterial->addUniform("mat4", "lightProjectionMatrix");
	copier->connect(lightMapPass->getCamera(), "FloatProjectionMatrix",
					shadowMaterial, "lightProjectionMatrix");

	// Get the result of the lightMapPass and pass it on to the shadowMapPass, because it is used
	// in a pass we ask the system to use a higher than normal texture unit (in this case 8) for
	// this texture, this prevents the texture from being overwritten by other textures

	shadowMaterial->addUniform("sampler2D", "encodedLightDepthMap");
	shadowMaterial->setValue("encodedLightDepthMap", lightMapPass->getRenderTarget()->getColorTarget(0));
	shadowMaterial->getUniform("encodedLightDepthMap")->setValue("MinimumTextureUnit", static_cast<size_t>(8));

	// Make the camera in the shadowMapPass follow the main camera that is being used to render the
	// whole scene
	auto mainCamera = std::dynamic_pointer_cast<Graphics::OsgCamera>(viewElement->getComponent("Camera"));
	SURGSIM_ASSERT(mainCamera != nullptr);
	copier->connect(mainCamera, "ProjectionMatrix", shadowCamera , "ProjectionMatrix");

	// Put the result of the last pass into the main camera to make it accessible
	auto material = std::make_shared<SurgSim::Graphics::OsgMaterial>("camera material");

	material->addUniform("sampler2D", "shadowMap");
	material->setValue("shadowMap", shadowCamera->getRenderTarget()->getColorTarget(0));
	material->getUniform("shadowMap")->setValue("MinimumTextureUnit", static_cast<size_t>(8));
	mainCamera->setMaterial(material);
	viewElement->addComponent(material);

	// Needs to be last to delay setting up of uniforms
	scene->addSceneElement(lightMapPass);
	viewElement->addComponent(shadowCamera);
	viewElement->addComponent(shadowMaterial);

	//scene->addSceneElement(shadowMapPass);

	auto debug = std::make_shared<SurgSim::Framework::BasicSceneElement>("debug");
	debug->addComponent(
		makeDebugQuad("light", lightMapPass->getRenderTarget()->getColorTarget(0), 0, 0, 256, 256));
	debug->addComponent(
		makeDebugQuad("shadow", shadowCamera->getRenderTarget()->getColorTarget(0), 1024 - 256, 0, 256, 256));
	scene->addSceneElement(debug);
}