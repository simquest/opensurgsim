// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
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

/// \file
/// Functions to get the OSG uniform type enum for a given type

#ifndef SURGSIM_GRAPHICS_OSGUNIFORMGLSLTYPES_H
#define SURGSIM_GRAPHICS_OSGUNIFORMGLSLTYPES_H

#include<memory>

#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{

namespace Graphics
{

class OsgTexture1d;
class OsgTexture2d;
class OsgTexture3d;
class OsgTextureCubeMap;
class OsgTextureRectangle;

/// Returns the OSG uniform type enum value for the template parameter type.
/// This template function is specialized to match each type with an enum value.
/// Any types for which the function is not specialized will return osg::Uniform::UNDEFINED.
/// \tparam	T	Data type
/// \return	OSG uniform type enum value
template <class T>
inline const std::string getGlslType()
{
	return "__undefined__";
}

template <>
inline const std::string getGlslType<float>()
{
	return "float";
}

template <>
inline const std::string getGlslType<double>()
{
	return "double";
}
template <>
inline const std::string getGlslType<int>()
{
	return "int";
}
template <>
inline const std::string getGlslType<unsigned int>()
{
	return "uint";
}
template <>
inline const std::string getGlslType<bool>()
{
	return "bool";
}

template <>
inline const std::string getGlslType<SurgSim::Math::Vector2f>()
{
	return "vec2";
}
template <>
inline const std::string getGlslType<SurgSim::Math::Vector3f>()
{
	return "vec3";
}
template <>
inline const std::string getGlslType<SurgSim::Math::Vector4f>()
{
	return "vec4";
}

template <>
inline const std::string getGlslType<SurgSim::Math::Vector2d>()
{
	return "dvec2";
}
template <>
inline const std::string getGlslType<SurgSim::Math::Vector3d>()
{
	return "dvec3";
}
template <>
inline const std::string getGlslType<SurgSim::Math::Vector4d>()
{
	return "dvec4";
}

template <>
inline const std::string getGlslType<SurgSim::Math::Matrix22f>()
{
	return "mat2x2";
}
template <>
inline const std::string getGlslType<SurgSim::Math::Matrix33f>()
{
	return "mat3x3";
}
template <>
inline const std::string getGlslType<SurgSim::Math::Matrix44f>()
{
	return "mat4x4";
}

template <>
inline const std::string getGlslType<SurgSim::Math::Matrix22d>()
{
	return "dmat2x2";
}
template <>
inline const std::string getGlslType<SurgSim::Math::Matrix33d>()
{
	return "dmat3x3";
}
template <>
inline const std::string getGlslType<SurgSim::Math::Matrix44d>()
{
	return "dmat4x4";
}

/// \note This name depends on the fact of OSS using integer types for the texture types if we introduce
/// a different type as the data type for the texture, the name has to be changed accordingly
template <>
inline const std::string getGlslType<std::shared_ptr<OsgTexture1d>>()
{
	return "sampler1D";
}

/// \note This name depends on the fact of OSS using integer types for the texture types if we introduce
/// a different type as the data type for the texture, the name has to be changed accordingly
template <>
inline const std::string getGlslType<std::shared_ptr<OsgTexture2d>>()
{
	return "sampler2D";
}

/// \note This name depends on the fact of OSS using integer types for the texture types if we introduce
/// a different type as the data type for the texture, the name has to be changed accordingly
template <>
inline const std::string getGlslType<std::shared_ptr<OsgTexture3d>>()
{
	return "sampler3D";
}

/// \note This name depends on the fact of OSS using integer types for the texture types if we introduce
/// a different type as the data type for the texture, the name has to be changed accordingly
template <>
inline const std::string getGlslType<std::shared_ptr<OsgTextureCubeMap>>()
{
	return "samplerCube";
}

template <>
inline const std::string getGlslType<std::shared_ptr<OsgTextureRectangle>>()
{
	return "sampler2DRect";
}

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_OSGUNIFORMGLSLTYPES_H