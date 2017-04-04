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

/// \file
/// Functions to get the OSG uniform type enum for a given type

#ifndef SURGSIM_GRAPHICS_OSGUNIFORMTYPES_H
#define SURGSIM_GRAPHICS_OSGUNIFORMTYPES_H

#include<memory>

#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Vector.h"

#include <osg/Uniform>

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
inline osg::Uniform::Type getOsgUniformType()
{
	return osg::Uniform::UNDEFINED;
}

template <>
inline osg::Uniform::Type getOsgUniformType<float>()
{
	return osg::Uniform::FLOAT;
}
template <>
inline osg::Uniform::Type getOsgUniformType<double>()
{
	return osg::Uniform::DOUBLE;
}
template <>
inline osg::Uniform::Type getOsgUniformType<int>()
{
	return osg::Uniform::INT;
}
template <>
inline osg::Uniform::Type getOsgUniformType<unsigned int>()
{
	return osg::Uniform::UNSIGNED_INT;
}
template <>
inline osg::Uniform::Type getOsgUniformType<bool>()
{
	return osg::Uniform::BOOL;
}

template <>
inline osg::Uniform::Type getOsgUniformType<SurgSim::Math::Vector2f>()
{
	return osg::Uniform::FLOAT_VEC2;
}
template <>
inline osg::Uniform::Type getOsgUniformType<SurgSim::Math::Vector3f>()
{
	return osg::Uniform::FLOAT_VEC3;
}
template <>
inline osg::Uniform::Type getOsgUniformType<SurgSim::Math::Vector4f>()
{
	return osg::Uniform::FLOAT_VEC4;
}

template <>
inline osg::Uniform::Type getOsgUniformType<SurgSim::Math::Vector2d>()
{
	return osg::Uniform::DOUBLE_VEC2;
}
template <>
inline osg::Uniform::Type getOsgUniformType<SurgSim::Math::Vector3d>()
{
	return osg::Uniform::DOUBLE_VEC3;
}
template <>
inline osg::Uniform::Type getOsgUniformType<SurgSim::Math::Vector4d>()
{
	return osg::Uniform::DOUBLE_VEC4;
}

template <>
inline osg::Uniform::Type getOsgUniformType<SurgSim::Math::Matrix22f>()
{
	return osg::Uniform::FLOAT_MAT2;
}
template <>
inline osg::Uniform::Type getOsgUniformType<SurgSim::Math::Matrix33f>()
{
	return osg::Uniform::FLOAT_MAT3;
}
template <>
inline osg::Uniform::Type getOsgUniformType<SurgSim::Math::Matrix44f>()
{
	return osg::Uniform::FLOAT_MAT4;
}

template<>
inline osg::Uniform::Type getOsgUniformType <SurgSim::Math::UnalignedMatrix44f>()
{
	return osg::Uniform::FLOAT_MAT4;
}

template <>
inline osg::Uniform::Type getOsgUniformType<SurgSim::Math::Matrix22d>()
{
	return osg::Uniform::DOUBLE_MAT2;
}
template <>
inline osg::Uniform::Type getOsgUniformType<SurgSim::Math::Matrix33d>()
{
	return osg::Uniform::DOUBLE_MAT3;
}
template <>
inline osg::Uniform::Type getOsgUniformType<SurgSim::Math::Matrix44d>()
{
	return osg::Uniform::DOUBLE_MAT4;
}

template <>
inline osg::Uniform::Type getOsgUniformType<std::shared_ptr<OsgTexture1d>>()
{
	return osg::Uniform::SAMPLER_1D;
}

template <>
inline osg::Uniform::Type getOsgUniformType<std::shared_ptr<OsgTexture2d>>()
{
	return osg::Uniform::SAMPLER_2D;
}

template <>
inline osg::Uniform::Type getOsgUniformType<std::shared_ptr<OsgTexture3d>>()
{
	return osg::Uniform::SAMPLER_3D;
}

template <>
inline osg::Uniform::Type getOsgUniformType<std::shared_ptr<OsgTextureCubeMap>>()
{
	return osg::Uniform::SAMPLER_CUBE;
}

template <>
inline osg::Uniform::Type getOsgUniformType<std::shared_ptr<OsgTextureRectangle>>()
{
	return osg::Uniform::SAMPLER_2D_RECT;
}

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_OSGUNIFORMTYPES_H