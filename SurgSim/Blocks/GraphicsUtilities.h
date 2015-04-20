#ifndef SURGSIM_BLOCKS_GRAPHICSUTILITIES_H
#define SURGSIM_BLOCKS_GRAPHICSUTILITIES_H

#include <memory>
#include <unordered_map>
#include <string>
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
namespace Framework
{
class Scene;
}

namespace Blocks
{

typedef std::unordered_map<std::string, std::shared_ptr<SurgSim::Graphics::OsgMaterial>> Materials;

/// Load and add a given texture to the material
/// \param material The material for adding the texture
/// \param uniform The name of the uniform to use
/// \param unit The texture unit to use
/// \param filename The file to use for the texture,
/// \param repeat whether to create the texture as repeating
/// \note if the texture filename is empty a placeholder uniform will be created using the unit as a value
/// this us use objects with textures builtin without having to assign the texture to the material on creation
void enable2DTexture(std::shared_ptr<SurgSim::Graphics::OsgMaterial> material,
					 const std::string& uniform,
					 int unit,
					 const std::string& filename = "", bool repeat = false);

/// Create a basic material
/// \param name name of the material
/// \param diffuseColor Base diffuse color to use
/// \param specularColor Base specular color to use
/// \param shininess Phong shininess exponent
std::shared_ptr<SurgSim::Graphics::OsgMaterial> createPlainMaterial(
	const std::string& name,
	SurgSim::Math::Vector4f diffuseColor,
	SurgSim::Math::Vector4f specularColor,
	float shininess);

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

/// Reads a material file, iterates over the components listed up in the material file and applies the materials and
/// the appropriate material properties (if present) to the component, if the component is not found it will be ignored
/// The material file is a yaml file with the following format
/// <pre>
/// - <SceneElementName>/<ComponentName>
///     Material: <MaterialName>
///     Properties:
///         - [<GLSLUniformType>, <UniformName>, <YamlEncodedValue>]
///         - [<GLSLUniformType>, <UniformName>, <YamlEncodedValue>]
/// - <SceneElementName>/<ComponentName>
///     Material: ...
/// </pre>
/// The name of the SceneElement and the Component addressed need to be be separated by a '/' character.
/// For each of the properties a uniform is created with the given GLSL type, name, and the YAML node will be passed
/// to the uniform setter for conversion. If the type does not match what the appropriate GLSL type is, there will
/// be an error.
/// GLSLUniformType is e.g. vec3, vec4, float, mat4 ...
/// \param scene The scene to traverse for component lookup
/// \param materialFilename the YAML file that contains the descriptions
/// \param materials lookup table for all the materials that are available
void applyMaterials(std::shared_ptr<SurgSim::Framework::Scene> scene,
					std::string materialFilename,
					Materials materials);
}
}
#endif