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
 *  Definitions of small fixed-size square matrix types.
 */

#ifndef SURGSIM_MATH_MATRIX_H
#define SURGSIM_MATH_MATRIX_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU> 		// needed for determinant() and inverse()

namespace SurgSim
{
namespace Math
{

/// A 2x2 matrix of floats.
/// This type (and any struct that contain it) can be safely allocated via new.
typedef Eigen::Matrix<float,  2, 2, Eigen::DontAlign | Eigen::RowMajor>  Matrix22f;

/// A 3x3 matrix of floats.
/// This type (and any struct that contain it) can be safely allocated via new.
typedef Eigen::Matrix<float,  3, 3, Eigen::DontAlign | Eigen::RowMajor>  Matrix33f;

/// A 4x4 matrix of floats.
/// This type (and any struct that contain it) can be safely allocated via new.
typedef Eigen::Matrix<float,  4, 4, Eigen::DontAlign | Eigen::RowMajor>  Matrix44f;

/// A 2x2 matrix of doubles.
/// This type (and any struct that contain it) can be safely allocated via new.
typedef Eigen::Matrix<double, 2, 2, Eigen::DontAlign | Eigen::RowMajor>  Matrix22d;

/// A 3x3 matrix of doubles.
/// This type (and any struct that contain it) can be safely allocated via new.
typedef Eigen::Matrix<double, 3, 3, Eigen::DontAlign | Eigen::RowMajor>  Matrix33d;

/// A 4x4 matrix of doubles.
/// This type (and any struct that contain it) can be safely allocated via new.
typedef Eigen::Matrix<double, 4, 4, Eigen::DontAlign | Eigen::RowMajor>  Matrix44d;



/// A 2x2 matrix of floats, with vectorized (SIMD) operations enabled if possible.
/// Note that special alignment restrictions may apply to this type; see Eigen documentation for details.
typedef Eigen::Matrix<float,  2, 2, Eigen::AutoAlign | Eigen::RowMajor>  Matrix22fv;

/// A 3x3 matrix of floats, with vectorized (SIMD) operations enabled if possible.
/// Note that special alignment restrictions may apply to this type; see Eigen documentation for details.
typedef Eigen::Matrix<float,  3, 3, Eigen::AutoAlign | Eigen::RowMajor>  Matrix33fv;

/// A 4x4 matrix of floats, with vectorized (SIMD) operations enabled if possible.
/// Note that special alignment restrictions may apply to this type; see Eigen documentation for details.
typedef Eigen::Matrix<float,  4, 4, Eigen::AutoAlign | Eigen::RowMajor>  Matrix44fv;

/// A 2x2 matrix of doubles, with vectorized (SIMD) operations enabled if possible.
/// Note that special alignment restrictions may apply to this type; see Eigen documentation for details.
typedef Eigen::Matrix<double, 2, 2, Eigen::AutoAlign | Eigen::RowMajor>  Matrix22dv;

/// A 3x3 matrix of doubles, with vectorized (SIMD) operations enabled if possible.
/// Note that special alignment restrictions may apply to this type; see Eigen documentation for details.
typedef Eigen::Matrix<double, 3, 3, Eigen::AutoAlign | Eigen::RowMajor>  Matrix33dv;

/// A 4x4 matrix of doubles, with vectorized (SIMD) operations enabled if possible.
/// Note that special alignment restrictions may apply to this type; see Eigen documentation for details.
typedef Eigen::Matrix<double, 4, 4, Eigen::AutoAlign | Eigen::RowMajor>  Matrix44dv;



/// Create a rotation matrix corresponding to the specified angle (in radians) and axis.
template <typename T, int MOpt>
inline Eigen::Matrix<T, 3, 3> makeRotationMatrix(const T& angle, const Eigen::Matrix<T, 3, 1, MOpt>& axis)
{
	return Eigen::AngleAxis<T>(angle, axis).toRotationMatrix();
}

/// Get the angle (in radians) and axis corresponding to a rotation matrix.
template <typename T, int MOpt, int VOpt>
inline void computeAngleAndAxis(const Eigen::Matrix<T, 3, 3, MOpt>& matrix,
                                T& angle, Eigen::Matrix<T, 3, 1, VOpt>& axis)
{
	Eigen::AngleAxis<T> angleAxis(matrix);
	angle = angleAxis.angle();
	axis = angleAxis.axis();
}

/// Get the angle corresponding to a quaternion's rotation, in radians.
template <typename T, int Opt>
inline T computeAngle(const Eigen::Matrix<T, 3, 3, Opt>& matrix)
{
	// TODO(bert): there has to be a more efficient way...
	Eigen::AngleAxis<T> angleAxis(matrix);
	return angleAxis.angle();
}

};  // namespace Math
};  // namespace SurgSim

#endif  // SURGSIM_MATH_MATRIX_H
