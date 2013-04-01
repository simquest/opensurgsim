// This file is a part of the OpenSurgSim project.
// Copyright 2012-2013, SimQuest Solutions Inc.
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

/** @file
 *  Definitions of quaternion types.
 */

#ifndef SURGSIM_MATH_QUATERNION_H
#define SURGSIM_MATH_QUATERNION_H

#include <math.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace SurgSim
{
namespace Math
{

/// A quaternion of floats.
/// This type (and any structs that contain it) can be safely allocated via new.
typedef Eigen::Quaternion<float,  Eigen::DontAlign>  Quaternionf;

/// A quaternion of doubles.
/// This type (and any structs that contain it) can be safely allocated via new.
typedef Eigen::Quaternion<double, Eigen::DontAlign>  Quaterniond;



/// Create a quaternion rotation corresponding to the specified angle (in radians) and axis.
template <typename T, int MOpt>
inline Eigen::Quaternion<T> makeRotationQuaternion(const T& angle, const Eigen::Matrix<T, 3, 1, MOpt>& axis)
{
	return Eigen::Quaternion<T>(Eigen::AngleAxis<T>(angle, axis));
}

/// Get the angle (in radians) and axis corresponding to a quaternion's rotation.
template <typename T, int QOpt, int MOpt>
inline void computeAngleAndAxis(const Eigen::Quaternion<T, QOpt>& quaternion,
                                T& angle, Eigen::Matrix<T, 3, 1, MOpt>& axis)
{
	Eigen::AngleAxis<T> angleAxis(quaternion);
	angle = angleAxis.angle();
	axis = angleAxis.axis();
}

/// Get the angle corresponding to a quaternion's rotation, in radians.
/// If you don't care about the rotation axis, this is more efficient than computeAngleAndAxis().
template <typename T, int Opt>
inline T computeAngle(const Eigen::Quaternion<T, Opt>& quaternion)
{
	T w = std::abs(quaternion.w());
	if (w >= 1.0)
	{
		return static_cast<T>(0);
	}
	return static_cast<T>(2 * std::acos(w));
}

};  // namespace Math
};  // namespace SurgSim

#endif // SURGSIM_MATH_QUATERNION_H
