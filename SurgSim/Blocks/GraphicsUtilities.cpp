

#include "SurgSim/Blocks/GraphicsUtilities.h"

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Graphics/OsgRepresentation.h"
#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgTexture2d.h"
#include "SurgSim/Graphics/OsgUniform.h"
#include "SurgSim/Graphics/OsgTextureUniform.h"
#include "SurgSim/Graphics/OsgProgram.h"

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
	SURGSIM_ASSERT(program != nullptr) << "Could not load program" << "Shaders/s_mapping_material" ;
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
	SURGSIM_ASSERT(program != nullptr) << "Could not load program" << "Shaders/ds_mapping_material" ;
	material->setProgram(program);

	material->addUniform("vec4", "specularColor");
	material->setValue("specularColor", specularColor);

	material->addUniform("vec4", "diffuseColor");
	material->setValue("diffuseColor", diffuseColor);

	material->addUniform("float", "shininess");
	material->setValue("shininess", shininess);

	enable2DTexture(material, "shadowMap", Graphics::SHADOW_TEXTURE_UNIT, "Textures/black.png");
	enable2DTexture(material, "diffuseMap", Graphics::DIFFUSE_TEXTURE_UNIT, diffuseMap, false);

	return material;
}

std::shared_ptr<SurgSim::Graphics::OsgMaterial> createNormalMappedMaterial(
	const std::string& name,
	SurgSim::Math::Vector4f diffuseColor,
	SurgSim::Math::Vector4f specularColor,
	float shininess,
	const std::string& diffuseMap,
	const std::string& normalMap)
{
	auto material = std::make_shared<Graphics::OsgMaterial>(name);

	auto program = Graphics::loadProgram(*Framework::Runtime::getApplicationData(), "Shaders/dns_mapping_material");
	SURGSIM_ASSERT(program != nullptr) << "Could not load program" << "Shaders/dns_mapping_material" ;
	material->setProgram(program);

	material->addUniform("vec4", "specularColor");
	material->setValue("specularColor", specularColor);

	material->addUniform("vec4", "diffuseColor");
	material->setValue("diffuseColor", diffuseColor);

	material->addUniform("float", "shininess");
	material->setValue("shininess", shininess);

	enable2DTexture(material, "shadowMap", Graphics::SHADOW_TEXTURE_UNIT, "Textures/black.png");
	enable2DTexture(material, "diffuseMap", Graphics::DIFFUSE_TEXTURE_UNIT, diffuseMap, false);
	enable2DTexture(material, "normalMap", Graphics::NORMAL_TEXTURE_UNIT, normalMap, false);

	return material;
}

}
}

