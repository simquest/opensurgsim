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

#include "SurgSim/Blocks/GraphicsUtilities.h"

#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Graphics/OsgRepresentation.h"
#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgRenderTarget.h"
#include "SurgSim/Graphics/OsgScreenSpaceQuadRepresentation.h"
#include "SurgSim/Graphics/OsgTexture2d.h"
#include "SurgSim/Graphics/OsgUniform.h"
#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Graphics/OsgTextureUniform.h"
#include "SurgSim/Graphics/OsgProgram.h"
#include "SurgSim/Graphics/OsgUniformFactory.h"
#include "SurgSim/Graphics/RenderPass.h"

#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/TransferPropertiesBehavior.h"
#include "SurgSim/Framework/PoseComponent.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "osg/PolygonMode"
namespace SurgSim
{
namespace Blocks
{

void enable2DTexture(
	std::shared_ptr<Graphics::OsgMaterial> material,
	const std::string& uniform, int unit, const std::string& filename, bool repeat)
{
	if (!filename.empty())
	{
		std::string path;
		SURGSIM_ASSERT(Framework::Runtime::getApplicationData()->tryFindFile(filename, &path))
				<< "Could not find texture file " << filename;
		auto texture = std::make_shared<Graphics::OsgTexture2d>();
		texture->loadImage(path);
		auto textureUniform = std::make_shared<Graphics::OsgTextureUniform<Graphics::OsgTexture2d>>(uniform);

		if (repeat)
		{
			auto osgTexture = texture->getOsgTexture();
			osgTexture->setWrap(osg::Texture::WRAP_R, osg::Texture::REPEAT);
			osgTexture->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
			osgTexture->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
		}

		textureUniform->set(texture);
		textureUniform->setMinimumTextureUnit(unit);
		material->addUniform(textureUniform);
	}
	else
	{
		// If there is no texture provided, just set the texture unit on the material
		material->addUniform("int", uniform);
		material->setValue(uniform, unit);
	}
}

std::shared_ptr<SurgSim::Graphics::OsgMaterial> createPlainMaterial(
	const std::string& name,
	SurgSim::Math::Vector4f diffuseColor,
	SurgSim::Math::Vector4f specularColor,
	float shininess)
{
	auto material = std::make_shared<Graphics::OsgMaterial>(name);

	auto program = Graphics::loadProgram(*Framework::Runtime::getApplicationData(), "Shaders/s_mapping_material");
	SURGSIM_ASSERT(program != nullptr) << "Could not load program" << "Shaders/s_mapping_material";
	material->setProgram(program);

	material->addUniform("vec4", "specularColor");
	material->setValue("specularColor", specularColor);

	material->addUniform("vec4", "diffuseColor");
	material->setValue("diffuseColor", diffuseColor);

	material->addUniform("float", "shininess");
	material->setValue("shininess", shininess);

	return material;
}

std::shared_ptr<SurgSim::Graphics::OsgMaterial> createTexturedMaterial(
	const std::string& name,
	SurgSim::Math::Vector4f diffuseColor,
	SurgSim::Math::Vector4f specularColor,
	float shininess,
	const std::string& diffuseMap)
{
	auto material = std::make_shared<Graphics::OsgMaterial>(name);

	auto program = Graphics::loadProgram(*Framework::Runtime::getApplicationData(), "Shaders/ds_mapping_material");
	SURGSIM_ASSERT(program != nullptr) << "Could not load program" << "Shaders/ds_mapping_material";
	material->setProgram(program);

	material->addUniform("vec4", "specularColor");
	material->setValue("specularColor", specularColor);

	material->addUniform("vec4", "diffuseColor");
	material->setValue("diffuseColor", diffuseColor);

	material->addUniform("float", "shininess");
	material->setValue("shininess", shininess);

	enable2DTexture(material, "diffuseMap", Graphics::DIFFUSE_TEXTURE_UNIT, diffuseMap, false);

	return material;
}

std::shared_ptr<Graphics::OsgMaterial> createNormalMappedMaterial(
	const std::string& name,
	SurgSim::Math::Vector4f diffuseColor,
	SurgSim::Math::Vector4f specularColor,
	float shininess,
	const std::string& diffuseMap,
	const std::string& normalMap)
{
	auto material = std::make_shared<Graphics::OsgMaterial>(name);

	auto program = Graphics::loadProgram(*Framework::Runtime::getApplicationData(), "Shaders/dns_mapping_material");
	SURGSIM_ASSERT(program != nullptr) << "Could not load program" << "Shaders/dns_mapping_material";

	// Prepare vertex attributes
	auto osgProgram = program->getOsgProgram();
	osgProgram->addBindAttribLocation("tangent", Graphics::TANGENT_VERTEX_ATTRIBUTE_ID);
	osgProgram->addBindAttribLocation("bitangent", Graphics::BITANGENT_VERTEX_ATTRIBUTE_ID);

	material->setProgram(program);

	material->addUniform("vec4", "specularColor");
	material->setValue("specularColor", specularColor);

	material->addUniform("vec4", "diffuseColor");
	material->setValue("diffuseColor", diffuseColor);

	material->addUniform("float", "shininess");
	material->setValue("shininess", shininess);

	enable2DTexture(material, "diffuseMap", Graphics::DIFFUSE_TEXTURE_UNIT, diffuseMap, false);
	enable2DTexture(material, "normalMap", Graphics::NORMAL_TEXTURE_UNIT, normalMap, false);

	return material;
}

void applyMaterials(std::shared_ptr<SurgSim::Framework::Scene> scene, std::string materialFilename,
					const Materials& materials)
{
	static SurgSim::Graphics::OsgUniformFactory uniformFactory;
	YAML::Node nodes;
	if (Framework::tryLoadNode(materialFilename, &nodes))
	{
		for (auto node = nodes.begin(); node != nodes.end(); ++node)
		{
			auto name = node->begin()->first.as<std::string>();
			auto materialName = node->begin()->second["Material"].as<std::string>();
			auto found = name.find("/");
			auto elementName = name.substr(0, found);
			auto componentName = name.substr(found + 1);

			auto component = std::dynamic_pointer_cast<Graphics::OsgRepresentation>
							 (scene->getComponent(elementName, componentName));

			if (component != nullptr)
			{

				auto materialIt = materials.find(materialName);

				SURGSIM_ASSERT(materialIt != materials.end())
						<< "Could not find material " << materialName << " in the prebuilt materials.";

				auto material = materialIt->second;
				component->setMaterial(material);

				auto propertyNodes = node->begin()->second["Properties"];
				for (auto nodeIt = propertyNodes.begin(); nodeIt != propertyNodes.end(); ++nodeIt)
				{
					auto rawUniform = uniformFactory.create(
										  (*nodeIt)[0].as<std::string>(), (*nodeIt)[1].as<std::string>()
									  );
					SURGSIM_ASSERT(rawUniform != nullptr)
							<< "Could not create uniform " << (*nodeIt)[1].as<std::string>() << " of type "
							<< (*nodeIt)[0].as<std::string>() << ".";
					auto uniform = std::dynamic_pointer_cast<Graphics::OsgUniformBase>(rawUniform);
					uniform->set((*nodeIt)[2]);
					component->addUniform(uniform);
				}
			}
		}
	}
	else
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getLogger("GraphicsUtilities"))
				<< "Could not find material definitions, visuals are going to be compromised.";
	}
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

std::shared_ptr<SurgSim::Graphics::RenderPass> createPass(
	std::unordered_map<std::string, std::shared_ptr<Graphics::OsgMaterial>> materials,
	const std::string& passName,
	const std::string& materialName)
{
	auto pass = std::make_shared<SurgSim::Graphics::RenderPass>(passName);
	auto renderTarget = std::make_shared<Graphics::OsgRenderTarget2d>(1024, 1024, 1.0, 1, false);
	pass->setRenderTarget(renderTarget);
	pass->setRenderOrder(SurgSim::Graphics::Camera::RENDER_ORDER_PRE_RENDER, 0);
	SURGSIM_ASSERT(materials[materialName] != nullptr);
	materials[materialName]->getProgram()->setGlobalScope(true);
	pass->setMaterial(materials[materialName]);
	return pass;
}

std::shared_ptr<Graphics::Camera> setupBlurPasses(
	std::unordered_map<std::string, std::shared_ptr<Graphics::OsgMaterial>> materials,
	std::shared_ptr<Framework::Scene> scene,
	std::shared_ptr<Graphics::RenderPass> previousPass)
{
	float size = 1024.0;
	float blurRadius = 6.0;

	auto element = std::make_shared<Framework::BasicSceneElement>("BlurPass");
	std::shared_ptr<Graphics::Camera> previous = previousPass->getCamera();

	// Horizontal Pass
	{
		// Material
		auto material = materials["horizontalBlur"];
		material->addUniform("float", "width");
		material->setValue("width", size);
		material->addUniform("float", "blurRadius");
		material->setValue("blurRadius", blurRadius);
		material->getProgram()->setGlobalScope(true);
		element->addComponent(material);

		// Camera
		auto camera = std::make_shared<Graphics::OsgCamera>("HorizontalCamera");
		auto osgCamera = camera->getOsgCamera();
		camera->setRenderGroupReference("HorizontalBlur");
		camera->setGroupReference(Graphics::Representation::DefaultGroupName);
		osgCamera->setViewport(0, 0, 1024, 1024);
		camera->setOrthogonalProjection(0, 1024, 0, 1024, -1.0, 1.0);
		camera->setRenderOrder(Graphics::Camera::RENDER_ORDER_PRE_RENDER, 3);
		camera->setMaterial(material);

		auto renderTarget = std::make_shared<Graphics::OsgRenderTarget2d>(1024, 1024, 1.0, 1, false);
		camera->setRenderTarget(renderTarget);
		element->addComponent(camera);

		// Quad
		auto graphics = std::make_shared<Graphics::OsgScreenSpaceQuadRepresentation>("HQuad");
		graphics->setSize(1024, 1024);
		graphics->setLocation(0, 0);
		graphics->setTexture(previous->getRenderTarget()->getColorTarget(0));
		graphics->setGroupReference("HorizontalBlur");
		element->addComponent(graphics);

		// Debug Quad
		graphics = std::make_shared<Graphics::OsgScreenSpaceQuadRepresentation>("HDebugQuad");
		graphics->setLocation(512, 0);
		graphics->setSize(256, 256);
		graphics->setTexture(camera->getRenderTarget()->getColorTarget(0));
		//element->addComponent(graphics);
		previous = camera;
	}

	// Vertical Pass
	{
		// Material
		auto material = materials["verticalBlur"];
		material->addUniform("float", "height");
		material->setValue("height", size);
		material->addUniform("float", "blurRadius");
		material->setValue("blurRadius", blurRadius);
		material->getProgram()->setGlobalScope(true);
		element->addComponent(material);

		// Camera
		auto camera = std::make_shared<Graphics::OsgCamera>("VerticalCamera");
		auto osgCamera = camera->getOsgCamera();
		camera->setRenderGroupReference("VerticalBlur");
		camera->setGroupReference(Graphics::Representation::DefaultGroupName);
		osgCamera->setViewport(0, 0, 1024, 1024);
		camera->setOrthogonalProjection(0, 1024, 0, 1024, -1.0, 1.0);
		camera->setRenderOrder(Graphics::Camera::RENDER_ORDER_PRE_RENDER, 4);
		camera->setMaterial(material);

		auto renderTarget = std::make_shared<Graphics::OsgRenderTarget2d>(1024, 1024, 1.0, 1, false);
		camera->setRenderTarget(renderTarget);
		element->addComponent(camera);

		// Quad
		auto graphics = std::make_shared<Graphics::OsgScreenSpaceQuadRepresentation>("VQuad");
		graphics->setSize(1024, 1024);
		graphics->setLocation(0, 0);
		graphics->setTexture(previous->getRenderTarget()->getColorTarget(0));
		graphics->setGroupReference("VerticalBlur");
		element->addComponent(graphics);

		// Debug Quad
		graphics = std::make_shared<Graphics::OsgScreenSpaceQuadRepresentation>("VDebugQuad");
		graphics->setLocation(768, 0);
		graphics->setSize(256, 256);
		graphics->setTexture(camera->getRenderTarget()->getColorTarget(0));
		//element->addComponent(graphics);
		previous = camera;
	}

	scene->addSceneElement(element);

	return previous;
}

/// Create the pass that renders the scene from the view of the light source
/// the identifier 'shadowing' is used in all graphic objects to mark them as used
/// in this pass
std::shared_ptr<SurgSim::Graphics::RenderPass> createLightMapPass(Materials materials)
{
	auto pass = std::make_shared<SurgSim::Graphics::RenderPass>("shadowing");
	auto renderTarget = std::make_shared<SurgSim::Graphics::OsgRenderTarget2d>(2048, 2048, 1.0, 1, false);
	pass->setRenderTarget(renderTarget);
	pass->setRenderOrder(SurgSim::Graphics::Camera::RENDER_ORDER_PRE_RENDER, 0);
	materials["depthMap"]->getProgram()->setGlobalScope(true);
	pass->setMaterial(materials["depthMap"]);
	return pass;
}

/// Create the pass that renders shadowed pixels into the scene,
/// the identifier 'shadowed' can be used in all graphics objects to mark them
/// as used in this pass
std::shared_ptr<SurgSim::Graphics::RenderPass> createShadowMapPass(Materials materials)
{
	auto pass = std::make_shared<SurgSim::Graphics::RenderPass>("shadowed");
	auto renderTarget = std::make_shared<SurgSim::Graphics::OsgRenderTarget2d>(1024, 1024, 1.0, 1, false);
	pass->setRenderTarget(renderTarget);
	pass->setRenderOrder(SurgSim::Graphics::Camera::RENDER_ORDER_PRE_RENDER, 1);
	materials["shadowMap"]->getProgram()->setGlobalScope(true);
	pass->setMaterial(materials["shadowMap"]);
	return pass;
}


void setupShadowMapping(std::unordered_map<std::string, std::shared_ptr<Graphics::OsgMaterial>>materials,
						std::shared_ptr<Framework::Scene> scene)
{
	bool useBlur = true;
	auto viewElement = scene->getSceneElement("View");

	// TODO
	// Backface rendering for LightmapPass
	// Render shadow map to depthmap and remove encodei
	// Rename LightMapPass to ShadowMapPass to concur with RealtimeRendering Book
	// Optional utilize Blurr Pass
	// Fix shader to ignore access outside of texturemap
	// Figure out how to set up renderpass through serialization
	// Expose more camera settings in the Graphics::Camera

	auto debug = std::make_shared<SurgSim::Framework::BasicSceneElement>("Debug");
	scene->addSceneElement(debug);

	// Question how to get the bouding box of the view frustrum of the viewing camera into the
	// projection matrix of the lightmapPass Camera
	auto lightMapPass = createLightMapPass(materials);
	lightMapPass->getCamera()->setOrthogonalProjection(-3, 3, -3, 3, 4, 10);
	auto osgCamera = std::dynamic_pointer_cast<Graphics::OsgCamera>(lightMapPass->getCamera())->getOsgCamera();
	osgCamera->getOrCreateStateSet()->setAttributeAndModes(
		new osg::PolygonMode(osg::PolygonMode::BACK, osg::PolygonMode::FILL), osg::StateAttribute::ON);

	auto shadowMapPass = createShadowMapPass(materials);

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

	auto shadowMaterial = shadowMapPass->getMaterial();
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
	auto shadowCamera = shadowMapPass->getCamera();
	auto mainCamera = std::dynamic_pointer_cast<Graphics::OsgCamera>(viewElement->getComponent("Camera"));
	SURGSIM_ASSERT(mainCamera != nullptr);
	copier->connect(mainCamera, "ProjectionMatrix", shadowCamera , "ProjectionMatrix");
	copier->connect(mainCamera, "Pose", shadowMapPass->getPoseComponent(), "Pose");

	// Put the result of the last pass into the main camera to make it accessible
	auto material = std::make_shared<SurgSim::Graphics::OsgMaterial>("camera material");
	material->addUniform("sampler2D", "shadowMap");

	std::shared_ptr<Graphics::Texture> texture;
	auto blurrPass = setupBlurPasses(materials, scene, shadowMapPass);

	if (useBlur)
	{
		texture = blurrPass->getRenderTarget()->getColorTarget(0);
	}
	else
	{
		texture = shadowCamera->getRenderTarget()->getColorTarget(0);
	}
	material->setValue("shadowMap", texture);
	material->getUniform("shadowMap")->setValue("MinimumTextureUnit", static_cast<size_t>(8));

	mainCamera->setMaterial(material);
	viewElement->addComponent(material);

	// Needs to be last to delay setting up of uniforms
	scene->addSceneElement(lightMapPass);
	scene->addSceneElement(shadowMapPass);

// 	debug->addComponent(
// 		makeDebugQuad("light", lightMapPass->getRenderTarget()->getColorTarget(0), 0, 0, 256, 256));
// 	debug->addComponent(
// 		makeDebugQuad("shadow", shadowCamera->getRenderTarget()->getColorTarget(0), 256, 0, 256, 256));
}

}
}

