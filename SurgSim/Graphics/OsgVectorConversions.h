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
/// Conversions to and from OSG vector types

#ifndef SURGSIM_GRAPHICS_OSGVECTORCONVERSIONS_H
#define SURGSIM_GRAPHICS_OSGVECTORCONVERSIONS_H

#include "SurgSim/Math/Vector.h"

#include <osg/Vec2f>
#include <osg/Vec2d>
#include <osg/Vec3f>
#include <osg/Vec3d>
#include <osg/Vec4f>
#include <osg/Vec4d>

namespace SurgSim
{

namespace Graphics
{

/// Convert 2D vector of floats to OSG
inline osg::Vec2f toOsg(const SurgSim::Math::Vector2f& vector)
{
	osg::Vec2f osgVector;
	Eigen::Map<SurgSim::Math::Vector2f>(osgVector.ptr()) = vector;
	return osgVector;
}
/// Convert from OSG to 2D vector of floats
inline SurgSim::Math::Vector2f fromOsg(const osg::Vec2f& vector)
{
	return SurgSim::Math::Vector2f(vector.ptr());
}

/// Convert 2D vector of doubles to OSG
inline osg::Vec2d toOsg(const SurgSim::Math::Vector2d& vector)
{
	osg::Vec2d osgVector;
	Eigen::Map<SurgSim::Math::Vector2d>(osgVector.ptr()) = vector;
	return osgVector;
}
/// Convert from OSG to 2D vector of doubles
inline SurgSim::Math::Vector2d fromOsg(const osg::Vec2d& vector)
{
	return SurgSim::Math::Vector2d(vector.ptr());
}

/// Convert 3D vector of floats to OSG
inline osg::Vec3f toOsg(const SurgSim::Math::Vector3f& vector)
{
	osg::Vec3f osgVector;
	Eigen::Map<SurgSim::Math::Vector3f>(osgVector.ptr()) = vector;
	return osgVector;
}
/// Convert from OSG to 3D vector of floats
inline SurgSim::Math::Vector3f fromOsg(const osg::Vec3f& vector)
{
	return SurgSim::Math::Vector3f(vector.ptr());
}

/// Convert 3D vector of doubles to OSG
inline osg::Vec3d toOsg(SurgSim::Math::Vector3d vector)
{
	osg::Vec3d osgVector;
	Eigen::Map<SurgSim::Math::Vector3d>(osgVector.ptr()) = vector;
	return osgVector;
}
/// Convert from OSG to 3D vector of doubles
inline SurgSim::Math::Vector3d fromOsg(const osg::Vec3d& vector)
{
	return SurgSim::Math::Vector3d(vector.ptr());
}


/// Convert 4D vector of floats to OSG
inline osg::Vec4f toOsg(const SurgSim::Math::Vector4f& vector)
{
	osg::Vec4f osgVector;
	Eigen::Map<SurgSim::Math::Vector4f>(osgVector.ptr()) = vector;
	return osgVector;
}

/// Convert 4D vector of floats to OSG
inline osg::Vec4f toOsg(const SurgSim::Math::UnalignedVector4f& vector)
{
	osg::Vec4f osgVector;
	Eigen::Map<SurgSim::Math::Vector4f>(osgVector.ptr()) = vector;
	return osgVector;
}

/// Convert from OSG to 4D vector of floats
inline SurgSim::Math::Vector4f fromOsg(const osg::Vec4f& vector)
{
	return SurgSim::Math::Vector4f(vector.ptr());
}

/// Convert 4D vector of doubles to OSG
inline osg::Vec4d toOsg(const SurgSim::Math::Vector4d& vector)
{
	osg::Vec4d osgVector;
	Eigen::Map<SurgSim::Math::Vector4d>(osgVector.ptr()) = vector;
	return osgVector;
}

/// Convert 4D vector of doubles to OSG
inline osg::Vec4d toOsg(const SurgSim::Math::UnalignedVector4d& vector)
{
	osg::Vec4d osgVector;
	Eigen::Map<SurgSim::Math::UnalignedVector4d>(osgVector.ptr()) = vector;
	return osgVector;
}

/// Convert from OSG to 4D vector of doubles
inline SurgSim::Math::Vector4d fromOsg(const osg::Vec4d& vector)
{
	return SurgSim::Math::Vector4d(vector.ptr());
}

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_OSGVECTORCONVERSIONS_H
