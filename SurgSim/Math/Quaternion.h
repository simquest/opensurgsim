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

/// \file
/// Definitions of quaternion types.

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
typedef Eigen::Quaternion<float>  Quaternionf;

/// A quaternion of doubles.
/// This type (and any structs that contain it) can be safely allocated via new.
typedef Eigen::Quaternion<double>  Quaterniond;



/// Create a quaternion rotation corresponding to the specified angle (in radians) and axis.
/// \tparam T the numeric data type used for arguments and the return value.  Can usually be deduced.
/// \tparam VOpt the option flags (alignment etc.) used for the axis vector argument.  Can be deduced.
/// \param angle the angle of the rotation, in radians.
/// \param axis the axis of the rotation.
/// \returns the rotation quaternion.
template <typename T, int VOpt>
inline Eigen::Quaternion<T> makeRotationQuaternion(const T& angle, const Eigen::Matrix<T, 3, 1, VOpt>& axis)
{
	return Eigen::Quaternion<T>(Eigen::AngleAxis<T>(angle, axis));
}

/// Quaternion negation (i.e. unary operator -)
/// \tparam T the numeric data type used for arguments and the return value.  Can usually be deduced.
/// \tparam QOpt the option flags (alignment etc.) used for the quaternion arguments.  Can be deduced.
/// \param q The quaternion to negate
/// \returns the negation of q (i.e. -q)
template <typename T, int QOpt>
inline Eigen::Quaternion<T, QOpt> negate(const Eigen::Quaternion<T, QOpt>& q)
{
	return Eigen::Quaternion<T, QOpt>(q.coeffs() * -1.0);
}

/// Get the angle (in radians) and axis corresponding to a quaternion's rotation.
/// \note Unit quaternions cover the unit sphere twice (q=-q). To make sure that the same rotation (q or -q)
/// \note returns the same Axis/Angle, we need a pi range for the angle.
/// \note We choose to enforce half the angle in the quadrant [0 +pi/2], which leads to an output angle in [0 +pi].
/// \tparam T the numeric data type used for arguments and the return value.  Can usually be deduced.
/// \tparam QOpt the option flags (alignment etc.) used for the quaternion argument.  Can be deduced.
/// \tparam VOpt the option flags (alignment etc.) used for the axis vector argument.  Can be deduced.
/// \param quaternion the rotation quaternion to inspect.
/// \param [out] angle the angle of the rotation in [0 +pi], in radians.
/// \param [out] axis the axis of the rotation.
template <typename T, int QOpt, int VOpt>
inline void computeAngleAndAxis(const Eigen::Quaternion<T, QOpt>& quaternion,
								T* angle, Eigen::Matrix<T, 3, 1, VOpt>* axis)
{
	using SurgSim::Math::negate;

	if (quaternion.w() >= T(0))
	{
		Eigen::AngleAxis<T> angleAxis(quaternion);
		*angle = angleAxis.angle();
		*axis = angleAxis.axis();
	}
	else
	{
		Eigen::AngleAxis<T> angleAxis(negate(quaternion));
		*angle = angleAxis.angle();
		*axis = angleAxis.axis();
	}
}

/// Get the angle corresponding to a quaternion's rotation, in radians.
/// \tparam T the numeric data type used for arguments and the return value.  Can usually be deduced.
/// \tparam QOpt the option flags (alignment etc.) used for the quaternion argument.  Can be deduced.
/// \param quaternion the rotation quaternion to inspect.
/// \returns the angle of the rotation within [0 +pi], in radians.
template <typename T, int QOpt>
inline T computeAngle(const Eigen::Quaternion<T, QOpt>& quaternion)
{
	using SurgSim::Math::negate;

	if (quaternion.w() >= T(0))
	{
		Eigen::AngleAxis<T> angleAxis(quaternion);
		return angleAxis.angle();
	}
	else
	{
		Eigen::AngleAxis<T> angleAxis(negate(quaternion));
		return angleAxis.angle();
	}
}

/// Get the vector corresponding to the rotation between 2 quaternions.
/// \tparam Derived the Quaternion type.  Can usually be deduced.
/// \tparam VOpt the option flags (alignment etc.) used for the vector argument.  Can be deduced.
/// \param end the quaternion after the rotation.
/// \param start the quaternion before the rotation.
/// \param[out] rotationVector a vector describing the rotation.
template <typename Derived, int VOpt>
void computeRotationVector(const Eigen::QuaternionBase<Derived>& end,
						   const Eigen::QuaternionBase<Derived>& start,
						   Eigen::Matrix<typename Derived::Scalar, 3, 1, VOpt>* rotationVector)
{
	typename Derived::Scalar angle;
	Eigen::Matrix<typename Derived::Scalar, 3, 1, VOpt> axis;
	computeAngleAndAxis((end * start.inverse()).normalized(), &angle, &axis);
	*rotationVector = angle * axis;
}

/// Get the vector corresponding to the rotation between transforms.
/// \tparam T the numeric data type used for arguments and the return value.  Can usually be deduced.
/// \tparam TOpt the option flags (alignment etc.) used for the transform arguments.  Can be deduced.
/// \tparam VOpt the option flags (alignment etc.) used for the vector argument.  Can be deduced.
/// \param end the transform after the rotation.
/// \param start the transform before the rotation.
/// \param[out] rotationVector a vector describing the rotation.
template <typename T, int TOpt, int VOpt>
void computeRotationVector(const Eigen::Transform<T, 3, Eigen::Isometry, TOpt>& end,
						   const Eigen::Transform<T, 3, Eigen::Isometry, TOpt>& start,
						   Eigen::Matrix<T, 3, 1, VOpt>* rotationVector)
{
	computeRotationVector(Eigen::Quaternion<T>(end.linear()), Eigen::Quaternion<T>(start.linear()), rotationVector);
}

/// Interpolate (slerp) between 2 quaternions
/// \tparam T the numeric data type used for arguments and the return value.  Can usually be deduced.
/// \tparam QOpt the option flags (alignment etc.) used for the quaternion arguments.  Can be deduced.
/// \param q0 The start quaternion (at time 0.0).
/// \param q1 The end   quaternion (at time 1.0).
/// \param t  The interpolation time requested. Within [0..1].
/// \returns the quaternion resulting in the slerp interpolation at time t, between q0 and q1.
/// \note t=0 => returns either q0 or -q0
/// \note t=1 => returns either q1 or -q1
/// \note 'Interpolate' has been created because slerp might not be enough in certain cases.
/// This gives room for correction and special future treatment
template <typename T, int QOpt>
inline Eigen::Quaternion<T, QOpt> interpolate(const Eigen::Quaternion<T, QOpt>& q0,
											  const Eigen::Quaternion<T, QOpt>& q1, T t)
{
	Eigen::Quaternion<T, QOpt> result;

	result = q0.slerp(t, q1);

	return result;
}

};  // namespace Math
};  // namespace SurgSim

#endif // SURGSIM_MATH_QUATERNION_H
