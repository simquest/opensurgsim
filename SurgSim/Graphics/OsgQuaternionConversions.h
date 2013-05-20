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
/// Conversions to and from OSG quaternion types
///
/// Note that the Eigen quaternion stores the W component first, while OSG stores it last.
/// These conversions handle this difference in ordering.
///
/// Also note that OSG only has one Quat type, which uses double for the value type. Conversions are provided to and 
/// from this type for both SurgSim::Math::Quaternionf and SurgSim::Math::Quaterniond.

#ifndef SURGSIM_GRAPHICS_OSGQUATERNIONCONVERSIONS_H
#define SURGSIM_GRAPHICS_OSGQUATERNIONCONVERSIONS_H

#include <SurgSim/Math/Quaternion.h>

#include <osg/Quat>

namespace SurgSim
{

namespace Graphics
{

/// Convert quaternion of floats to OSG
inline osg::Quat toOsg(const SurgSim::Math::Quaternionf& quaternion)
{
	return osg::Quat(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());
}

/// Convert quaternion of doubles to OSG
inline osg::Quat toOsg(const SurgSim::Math::Quaterniond& quaternion)
{
	return osg::Quat(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());
}

/// Convert from OSG to either quaternion of floats or doubles
/// \tparam	T	Value type (float or double)
template <typename T> inline
Eigen::Quaternion<T,  Eigen::DontAlign> fromOsg(const osg::Quat& quaternion)
{
	return Eigen::Quaternion<T,  Eigen::DontAlign>(quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z());
}

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_OSGQUATERNIONCONVERSIONS_H
