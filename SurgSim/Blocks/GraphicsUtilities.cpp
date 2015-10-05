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
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/PoseComponent.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Framework/TransferPropertiesBehavior.h"
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



}
}

