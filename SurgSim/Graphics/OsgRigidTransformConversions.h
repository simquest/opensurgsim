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
/// Conversions to and from OSG rigid transform types
///
/// Note that OSG does not have a rigid transform type, so a quaternion, vector pair are used for the rotation and
/// translation components of the rigid transform.

#ifndef SURGSIM_GRAPHICS_OSGRIGIDTRANSFORMCONVERSIONS_H
#define SURGSIM_GRAPHICS_OSGRIGIDTRANSFORMCONVERSIONS_H

#include "SurgSim/Graphics/OsgQuaternionConversions.h"
#include "SurgSim/Graphics/OsgVectorConversions.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"

#include <osg/Quat>
#include <osg/Vec2>
#include <osg/Vec3>

namespace SurgSim
{

namespace Graphics
{

/// Convert 3D rigid body (isometric) transform, represented as floats, to OSG
inline std::pair<osg::Quat, osg::Vec3f> toOsg(const SurgSim::Math::RigidTransform3f& transform)
{
	SurgSim::Math::Quaternionf normalizedQuaternion = SurgSim::Math::Quaternionf(transform.linear()).normalized();
	return std::make_pair(toOsg(normalizedQuaternion), toOsg(SurgSim::Math::Vector3f(transform.translation())));
}
/// Convert 3D rigid body (isometric) transform, represented as doubles, to OSG
inline std::pair<osg::Quat, osg::Vec3d> toOsg(const SurgSim::Math::RigidTransform3d& transform)
{
	SurgSim::Math::Quaterniond normalizedQuaternion = SurgSim::Math::Quaterniond(transform.linear()).normalized();
	return std::make_pair(toOsg(normalizedQuaternion), toOsg(SurgSim::Math::Vector3d(transform.translation())));
}

/// Convert 3D rigid body (isometric) transform, represented as doubles, to OSG
inline std::pair<osg::Quat, osg::Vec3d> toOsg(const SurgSim::Math::UnalignedRigidTransform3d& transform)
{
	SurgSim::Math::Quaterniond normalizedQuaternion = SurgSim::Math::Quaterniond(transform.linear()).normalized();
	return std::make_pair(toOsg(normalizedQuaternion), toOsg(SurgSim::Math::Vector3d(transform.translation())));
}

/// Convert from OSG to 3D rigid body (isometric) transform, represented as floats
inline SurgSim::Math::RigidTransform3f fromOsg(const osg::Quat& rotation, const osg::Vec3f& translation)
{
	return SurgSim::Math::makeRigidTransform(fromOsg<float>(rotation), fromOsg(translation));
}
/// Convert from OSG to 3D rigid body (isometric) transform, represented as floats
inline SurgSim::Math::RigidTransform3f fromOsg(const std::pair<osg::Quat, osg::Vec3f>& transform)
{
	return fromOsg(transform.first, transform.second);
}
/// Convert from OSG to 3D rigid body (isometric) transform, represented as doubles
inline SurgSim::Math::RigidTransform3d fromOsg(const osg::Quat& rotation, const osg::Vec3d& translation)
{
	return SurgSim::Math::makeRigidTransform(fromOsg<double>(rotation), fromOsg(translation));
}
/// Convert from OSG to 3D rigid body (isometric) transform, represented as doubles
inline SurgSim::Math::RigidTransform3d fromOsg(const std::pair<osg::Quat, osg::Vec3d>& transform)
{
	return fromOsg(transform.first, transform.second);
}

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_OSGRIGIDTRANSFORMCONVERSIONS_H