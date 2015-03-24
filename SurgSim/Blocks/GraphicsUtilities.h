#ifndef BURRHOLE_GRAPHICS_H
#define BURRHOLE_GRAPHICS_H

#include <memory>
#include <SurgSim/Math/Vector.h>

/// \file GraphicsUtilities.h
/// The code in here is supposed to help create materials correctly, a lot of it is boilerplate that is repeated over
/// for each material. It will have to be changed if the shaders being used change.
namespace SurgSim
{
namespace Graphics
{
class OsgMaterial;
}

namespace Blocks
{

/// Load and add a given texture to the material
/// \param material The material for adding the texture
/// \param uniform The name of the uniform to use
/// \param unit The texture unit to use
/// \param filename The file to use for the texture,
/// \param repeat whether to create the texture as repeating
void add2DTexture(std::shared_ptr<SurgSim::Graphics::OsgMaterial> material,
				  const std::string& uniform,
				  int unit,
				  const std::string& filename, bool repeat = false);

/// Create a basic textured material
/// \param name name of the material
/// \param diffuseColor Base diffuse color to use
/// \param specularColor Base specular color to use
/// \param shininess Phong shininess exponent
/// \param diffuseMap Diffuse texture map name to use, if the texture is embedded in the object
///        pass an empty string here, it has to occupy the correct texture unit though.
std::shared_ptr<SurgSim::Graphics::OsgMaterial> createTexturedMaterial(
	const std::string& name,
	SurgSim::Math::Vector4f diffuseColor,
	SurgSim::Math::Vector4f specularColor,
	float shininess,
	const std::string& diffuseMap = "");

/// Create a basic textured material
/// \param name name of the material
/// \param diffuseColor Base diffuse color to use
/// \param specularColor Base specular color to use
/// \param shininess Phong shininess exponent
/// \param diffuseMap Diffuse texture map name to use, if the texture is embedded in the object
///        pass an empty string here, it has to occupy the correct texture unit as defined by \sa DIFFUSE_TEXTURE_UNIT.
/// \param normalMap Normal texture map to use, if the texture is embedded in the object pass an empty string here,
///        it has to occupy the correct texture unit as defined by \sa NORMAL_TEXTURE_UNIT.
std::shared_ptr<SurgSim::Graphics::OsgMaterial> createNormalMappedMaterial(
	const std::string& name,
	SurgSim::Math::Vector4f diffuseColor,
	SurgSim::Math::Vector4f specularColor,
	float shininess,
	const std::string& diffuseMap = "",
	const std::string& normalMap = "");
}
}
#endif