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

#ifndef SURGSIM_TESTING_MATHUTILITIES_H
#define SURGSIM_TESTING_MATHUTILITIES_H

#include <utility>

#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/Quaternion.h"

#include <Eigen/Geometry>

namespace SurgSim
{
namespace Testing
{
/// Does a linear interpolation between the start and the end dependent on t.
/// \tparam	T	The type of value to be interpolated.
/// \param	start	The start value.
/// \param	end  	The end value.
/// \param	t	 	The percentage for the interpolation.
/// \return	the interpolated value.
template <class T>
T interpolate(const T& start, const T& end, const double& t)
{
	return (1 - t) * start + t * end;
}

template <class T>
T interpolate(const std::pair<T, T>& values, const double& t)
{
	return interpolate<T>(values.first, values.second, t);
}

/// Specialized template to call the correct function for Quaterniond, might be superfluous,
/// delegates to the eigen interpolation function for Quaterniond
/// \param	start	The start quaternion.
/// \param	end  	The end quaternion.
/// \param	t	 	The percentage for the interpolation.
/// \return	the interpolated quaternion.
template <>
SurgSim::Math::Quaterniond interpolate<SurgSim::Math::Quaterniond>(
	const SurgSim::Math::Quaterniond& start,
	const SurgSim::Math::Quaterniond& end,
	const double& t);

/// Specialized template to call the correct function for RigidTransform3d, might be superfluous,
/// delegates to the eigen interpolation function for RigidTransform3d
/// \param	start	The start quaternion.
/// \param	end  	The end quaternion.
/// \param	t	 	The percentage for the interpolation.
/// \return	the interpolated RigidTransform3d.
template <>
SurgSim::Math::RigidTransform3d interpolate<SurgSim::Math::RigidTransform3d>(
	const SurgSim::Math::RigidTransform3d& start,
	const SurgSim::Math::RigidTransform3d& end,
	const double& t);

/// Does a linear interpolation on a pose, given Vector3d for angles and positions. The angles around the X, Y
/// and Z axis are being passed in a Vector3d for a terser expression.
/// \param	startAngles  	The start angles in the order X/Y/Z angle axis value in radians.
/// \param	endAngles	 	The end angles in the order X/Y/Z angle axis value in radians.
/// \param	startPosition	The start position.
/// \param	endPosition  	The end position.
/// \param	t			 	The percentage for the interpolation.
/// \return					The transform gained by interpolating and
/// 						assembling the rotation and position values.
SurgSim::Math::RigidTransform3d interpolatePose(
	const SurgSim::Math::Vector3d& startAngles,
	const SurgSim::Math::Vector3d& endAngles,
	const SurgSim::Math::Vector3d& startPosition,
	const SurgSim::Math::Vector3d& endPosition,
	const double& t);


}
}


#endif


