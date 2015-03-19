

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

void add2DTexture(
	std::shared_ptr<Graphics::OsgMaterial> material,
	const std::string& uniform, int unit, const std::string& filename, bool repeat)
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

std::shared_ptr<Graphics::OsgMaterial> createTexturedMaterial(
	Math::Vector4f diffuseColor,
	Math::Vector4f specularColor,
	double shininess,
	const std::string& diffuseMap)
{
	auto material = std::make_shared<Graphics::OsgMaterial>("material");

	auto program = Graphics::loadProgram(*Framework::Runtime::getApplicationData(), "Shaders/ds_mapping_material");
	SURGSIM_ASSERT(program != nullptr) << "Could not load program" << "Shaders/ds_mapping_material" ;
	material->setProgram(program);

	material->addUniform("vec4", "specularColor");
	material->setValue("specularColor", specularColor);

	material->addUniform("vec4", "diffuseColor");
	material->setValue("diffuseColor", diffuseColor);

	material->addUniform("float", "shininess");
	material->setValue("shininess", shininess);

	add2DTexture(material, "shadowMap", Graphics::SHADOW_TEXTURE_UNIT, "Textures/black.png");
	if (diffuseMap.empty())
	{
		// If there is no texture provided, just use unit 0 for the diffuse map
		material->addUniform("int", "diffuseMap");
		material->setValue("diffuseMap", Graphics::DIFFUSE_TEXTURE_UNIT);
	}
	else
	{
		add2DTexture(material, "diffuseMap", Graphics::DIFFUSE_TEXTURE_UNIT, diffuseMap, false);
	}

	return material;
}

std::shared_ptr<Graphics::OsgMaterial> createNormalMappedMaterial(
	Math::Vector4f diffuseColor,
	Math::Vector4f specularColor,
	double shininess,
	const std::string& diffuseMap,
	const std::string& normalMap)
{
	auto material = std::make_shared<Graphics::OsgMaterial>("material");

	auto program = Graphics::loadProgram(*Framework::Runtime::getApplicationData(), "Shaders/dns_mapping_material");
	SURGSIM_ASSERT(program != nullptr) << "Could not load program" << "Shaders/dns_mapping_material" ;
	material->setProgram(program);

	material->addUniform("vec4", "specularColor");
	material->setValue("specularColor", specularColor);

	material->addUniform("vec4", "diffuseColor");
	material->setValue("diffuseColor", diffuseColor);

	material->addUniform("float", "shininess");
	material->setValue("shininess", shininess);

	add2DTexture(material, "shadowMap", Graphics::SHADOW_TEXTURE_UNIT, "Textures/black.png");
	if (diffuseMap.empty())
	{
		// If there is no texture provided, just use unit 0 for the diffuse map
		material->addUniform("int", "diffuseMap");
		material->setValue("diffuseMap", Graphics::DIFFUSE_TEXTURE_UNIT);
	}
	else
	{
		add2DTexture(material, "diffuseMap", Graphics::DIFFUSE_TEXTURE_UNIT, diffuseMap, false);
	}

	if (normalMap.empty())
	{
		// If there is no texture provided, just use unit 0 for the diffuse map
		material->addUniform("int", "normalMap");
		material->setValue("normalMap", Graphics::NORMAL_TEXTURE_UNIT);
	}
	else
	{
		add2DTexture(material, "normalMap", Graphics::NORMAL_TEXTURE_UNIT, normalMap, false);
	}

	return material;
}

}
}

