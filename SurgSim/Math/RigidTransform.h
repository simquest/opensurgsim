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
/// Definitions of 2x2 and 3x3 rigid-body (isometric) transforms.

#ifndef SURGSIM_MATH_RIGIDTRANSFORM_H
#define SURGSIM_MATH_RIGIDTRANSFORM_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace SurgSim
{
namespace Math
{

/// A 2D rigid body (isometric) transform, represented as floats.
/// This type (and any struct that contain it) can be safely allocated via new.
typedef Eigen::Transform<float,  2, Eigen::Isometry, Eigen::DontAlign>  RigidTransform2f;

/// A 3D rigid body (isometric) transform, represented as floats.
/// This type (and any struct that contain it) can be safely allocated via new.
typedef Eigen::Transform<float,  3, Eigen::Isometry, Eigen::DontAlign>  RigidTransform3f;

/// A 2D rigid body (isometric) transform, represented as doubles.
/// This type (and any struct that contain it) can be safely allocated via new.
typedef Eigen::Transform<double, 2, Eigen::Isometry, Eigen::DontAlign>  RigidTransform2d;

/// A 3D rigid body (isometric) transform, represented as doubles.
/// This type (and any struct that contain it) can be safely allocated via new.
typedef Eigen::Transform<double, 3, Eigen::Isometry, Eigen::DontAlign>  RigidTransform3d;


/// Create a rigid-body transform using the specified rotation matrix and translation.
template <typename T, int N, int MOpt, int VOpt>
inline Eigen::Transform<T, N, Eigen::Isometry> makeRigidTransform(const Eigen::Matrix<T, N, N, MOpt>& rotation,
	const Eigen::Matrix<T, N, 1, VOpt>& translation)
{
	Eigen::Transform<T, N, Eigen::Isometry> rigid;
	rigid.makeAffine();
	rigid.linear() = rotation;
	rigid.translation() = translation;
	return rigid;
}

/// Create a rigid-body transform using the specified rotation quaternion and translation.
template <typename T, int QOpt, int VOpt>
inline Eigen::Transform<T, 3, Eigen::Isometry> makeRigidTransform(const Eigen::Quaternion<T, QOpt>& rotation,
	const Eigen::Matrix<T, 3, 1, VOpt>& translation)
{
	Eigen::Transform<T, 3, Eigen::Isometry> rigid;
	rigid.makeAffine();
	rigid.linear() = rotation.matrix();
	rigid.translation() = translation;
	return rigid;
}

};  // namespace Math
};  // namespace SurgSim

#endif  // SURGSIM_MATH_RIGIDTRANSFORM_H
