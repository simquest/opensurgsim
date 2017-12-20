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
/// Definitions of 2x2 and 3x3 rigid (isometric) transforms.

#ifndef SURGSIM_MATH_RIGIDTRANSFORM_H
#define SURGSIM_MATH_RIGIDTRANSFORM_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include "SurgSim/Math/Quaternion.h"

namespace SurgSim
{
namespace Math
{

/// A 2D rigid (isometric) transform, represented as floats.
/// This type (and any struct that contain it) can be safely allocated via new.
typedef Eigen::Transform<float,  2, Eigen::Isometry>  RigidTransform2f;
typedef Eigen::Transform<float, 2, Eigen::Isometry, Eigen::DontAlign>  UnalignedRigidTransform2f;

/// A 3D rigid (isometric) transform, represented as floats.
/// This type (and any struct that contain it) can be safely allocated via new.
typedef Eigen::Transform<float,  3, Eigen::Isometry>  RigidTransform3f;
typedef Eigen::Transform<float, 3, Eigen::Isometry, Eigen::DontAlign>  UnalignedRigidTransform3f;

/// A 2D rigid (isometric) transform, represented as doubles.
/// This type (and any struct that contain it) can be safely allocated via new.
typedef Eigen::Transform<float,  2, Eigen::Isometry>  RigidTransform2d;
typedef Eigen::Transform<float, 2, Eigen::Isometry, Eigen::DontAlign>  UnalignedRigidTransform2d;

/// A 3D rigid (isometric) transform, represented as doubles.
/// This type (and any struct that contain it) can be safely allocated via new.
typedef Eigen::Transform<double, 3, Eigen::Isometry>  RigidTransform3d;
typedef Eigen::Transform<double, 3, Eigen::Isometry, Eigen::DontAlign>  UnalignedRigidTransform3d;

/// Create a rigid transform using the specified rotation matrix and translation.
/// \tparam M the type used to describe the rotation matrix.  Can usually be deduced.
/// \tparam V the type used to describe the translation vector.  Can usually be deduced.
/// \param rotation the rotation matrix.
/// \param translation the translation vector.
/// \returns the transform with the specified rotation and translation.
template <typename M, typename V>
inline Eigen::Transform<typename M::Scalar, M::RowsAtCompileTime, Eigen::Isometry> makeRigidTransform(
	const Eigen::MatrixBase<M>& rotation, const Eigen::MatrixBase<V>& translation)
{
	Eigen::Transform<typename M::Scalar, M::RowsAtCompileTime, Eigen::Isometry> rigid;
	rigid.makeAffine();
	rigid.linear() = rotation;
	rigid.translation() = translation;
	return rigid;
}

/// Create a rigid transform using the specified rotation quaternion and translation.
/// \tparam Q the type used to describe the rotation quaternion.  Can usually be deduced.
/// \tparam V the type used to describe the translation vector.  Can usually be deduced.
/// \param rotation the rotation quaternion.
/// \param translation the translation vector.
/// \returns the transform with the specified rotation and translation.
template <typename Q, typename V>
inline Eigen::Transform<typename Q::Scalar, 3, Eigen::Isometry> makeRigidTransform(
	const Eigen::QuaternionBase<Q>& rotation, const Eigen::MatrixBase<V>& translation)
{
	Eigen::Transform<typename Q::Scalar, 3, Eigen::Isometry> rigid;
	rigid.makeAffine();
	rigid.linear() = rotation.matrix();
	rigid.translation() = translation;
	return rigid;
}

/// Make a rigid transform from a eye point a center view point and an up direction, does not check whether up is
/// colinear with eye-center
/// The original formula can be found at
/// http://www.opengl.org/sdk/docs/man2/xhtml/gluLookAt.xml
/// \tparam	typename T	T the numeric data type used for arguments and the return value. Can usually be deduced.
/// \tparam	int VOpt  	VOpt the option flags (alignment etc.) used for the axis vector argument.  Can be deduced.
/// \param	position   	The position of the object.
/// \param	center	The point to which the object should point.
/// \param	up	  	The up vector to be used for this calculation.
/// \return	a RigidTransform that locates the object at position rotated into the direction of center.
template <typename T, int VOpt>
inline Eigen::Transform<T, 3, Eigen::Isometry> makeRigidTransform(
	const Eigen::Matrix<T, 3, 1, VOpt>& position,
	const Eigen::Matrix<T, 3, 1, VOpt>& center,
	const Eigen::Matrix<T, 3, 1, VOpt>& up)
{

	Eigen::Transform<T, 3, Eigen::Isometry> rigid;
	rigid.makeAffine();

	Eigen::Matrix<T, 3, 1, VOpt> forward = (center - position).normalized();
	Eigen::Matrix<T, 3, 1, VOpt> side = (forward.cross(up)).normalized();
	Eigen::Matrix<T, 3, 1, VOpt> actualUp = side.cross(forward).normalized();

	typename Eigen::Transform<T, 3, Eigen::Isometry>::LinearMatrixType rotation;
	rotation << side[0], actualUp[0], -forward[0],
			 side[1], actualUp[1], -forward[1],
			 side[2], actualUp[2], -forward[2];

	rigid.linear() = rotation;
	rigid.translation() = position;

	return rigid;
}

/// Create a rigid transform using the identity rotation and the specified translation.
/// \tparam V the type used to describe the translation vector.  Can usually be deduced.
/// \param translation the translation vector.
/// \returns the transform with the identity rotation and the specified translation.
template <typename V>
inline Eigen::Transform<typename V::Scalar, V::SizeAtCompileTime, Eigen::Isometry> makeRigidTranslation(
	const Eigen::MatrixBase<V>& translation)
{
	EIGEN_STATIC_ASSERT_VECTOR_ONLY(Eigen::MatrixBase<V>);
	Eigen::Transform<typename V::Scalar, V::SizeAtCompileTime, Eigen::Isometry> rigid;
	rigid.makeAffine();
	rigid.linear().setIdentity();
	rigid.translation() = translation;
	return rigid;
}

/// Interpolate (slerp) between 2 rigid transformations
/// \tparam T the numeric data type used for arguments and the return value.  Can usually be deduced.
/// \tparam TOpt the option flags (alignment etc.) used for the Transform arguments.  Can be deduced.
/// \param t0 The start transform (at time 0.0).
/// \param t1 The end   transform (at time 1.0).
/// \param t  The interpolation time requested. Within [0..1].
/// \returns the transform resulting in the slerp interpolation at time t, between t0 and t1.
/// \note t=0 => returns t0
/// \note t=1 => returns t1
template <typename T, int TOpt>
inline Eigen::Transform<T, 3, Eigen::Isometry> interpolate(
	const Eigen::Transform<T, 3, Eigen::Isometry, TOpt>& t0,
	const Eigen::Transform<T, 3, Eigen::Isometry, TOpt>& t1,
	T t)
{
	Eigen::Transform<T, 3, Eigen::Isometry> transform;
	transform.makeAffine();
	transform.translation() = t0.translation() * (static_cast<T>(1.0) - t) + t1.translation() * t;
	{
		Eigen::Quaternion<T> q0(t0.linear());
		Eigen::Quaternion<T> q1(t1.linear());
		q0.normalize();
		q1.normalize();
		transform.linear() = interpolate(q0, q1, t).matrix();
	}
	return transform;
}

};  // namespace Math
};  // namespace SurgSim

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(SurgSim::Math::RigidTransform3d);

#endif  // SURGSIM_MATH_RIGIDTRANSFORM_H
