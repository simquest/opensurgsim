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
/// Declarations of isValid(), isSubnormal() and setSubnormalToZero().

#ifndef SURGSIM_MATH_VALID_H
#define SURGSIM_MATH_VALID_H

#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace SurgSim
{
namespace Math
{

/// Check if a <code>float</code> value is valid.
/// Zero, subnormal and normal numbers are valid; infinities and NaNs are not.
/// \param value the value to check.
/// \returns true if valid, false if not.
inline bool isValid(float value);

/// Check if a <code>double</code> value is valid.
/// Zero, subnormal and normal numbers are valid; infinities and NaNs are not.
/// \param value the value to check.
/// \returns true if valid, false if not.
inline bool isValid(double value);

/// Check if a matrix or a vector is valid.
/// These quantities are valid if all of their elements are valid.
/// Zero, subnormal and normal numbers are valid; infinities and NaNs are not.
/// \tparam T the base type used to describe the matrix or vector.  Can usually be deduced.
/// \param value the matrix or vector value to check.
/// \returns true if valid, false if not.
template <typename T>
inline bool isValid(const Eigen::DenseBase<T>& value);

/// Check if a quaternion is valid.
/// Quaternions are valid if all of their components are valid.
/// Zero, subnormal and normal numbers are valid; infinities and NaNs are not.
/// \tparam T the base type used to describe the quaternion.  Can usually be deduced.
/// \param value the quaternion value to check.
/// \returns true if valid, false if not.
template <typename T>
inline bool isValid(const Eigen::QuaternionBase<T>& value);

/// Check if an angle/axis 3D rotation is valid.
/// Angle/axis rotations are valid if the angle and the axis components are valid.
/// Zero, subnormal and normal numbers are valid; infinities and NaNs are not.
/// \tparam T the scalar type used to describe the rotation.  Can usually be deduced.
/// \param value the rotation value to check.
/// \returns true if valid, false if not.
template <typename T>
inline bool isValid(const Eigen::AngleAxis<T>& value);

/// Check if a 2D rotation is valid.
/// 2D rotations are valid if the rotation angle is valid.
/// Zero, subnormal and normal numbers are valid; infinities and NaNs are not.
/// \tparam T the scalar type used to describe the rotation.  Can usually be deduced.
/// \param value the rotation value to check.
/// \returns true if valid, false if not.
template <typename T>
inline bool isValid(const Eigen::Rotation2D<T>& value);

/// Check if a transform is valid.
/// Transforms are valid if all of their components are valid.
/// Zero, subnormal and normal numbers are valid; infinities and NaNs are not.
/// \tparam T the scalar type used to describe the transform.  Can usually be deduced.
/// \tparam D the dimension used to describe the transform.  Can usually be deduced.
/// \tparam M the mode value used to describe the transform.  Can usually be deduced.
/// \tparam O the options value used to describe the transform.  Can usually be deduced.
/// \param value the transform value to check.
/// \returns true if valid, false if not.
template <typename T, int D, int M, int O>
inline bool isValid(const Eigen::Transform<T, D, M, O>& value);



/// Check if a <code>float</code> value is subnormal.
/// Subnormal values have absolute values in the range <code>std::numeric_limits&lt;float&gt;::%denorm_min() &lt;=
/// x &lt; std::numeric_limits&lt;float&gt;::%min()</code>, and can result in very slow floating point calculations
/// under some conditions.
/// \param value the value to check.
/// \returns true if subnormal; false if not (normal, zero, infinite or NaN).
inline bool isSubnormal(float value);

/// Check if a <code>double</code> value is subnormal.
/// Subnormal values have absolute values in the range <code>std::numeric_limits&lt;double&gt;::%denorm_min() &lt;=
/// x &lt; std::numeric_limits&lt;double&gt;::%min()</code>, and can result in very slow floating point
/// calculations under some conditions.
/// \param value the value to check.
/// \returns true if subnormal; false if not (normal, zero, infinite or NaN).
inline bool isSubnormal(double value);

/// Check if a matrix or a vector contains any subnormal floating-point values.
/// Subnormal values have absolute values in the range <code>std::numeric_limits&lt;T&gt;::%denorm_min() &lt;= x
/// &lt; std::numeric_limits&lt;T&gt;::%min()</code>, and can result in very slow floating point calculations under
/// some conditions.
/// \tparam T the base type used to describe the matrix or vector.  Can usually be deduced.
/// \param value the matrix or vector value to check.
/// \returns true if any value is subnormal; false if none are (i.e. each is normal, zero, infinite or NaN).
template <typename T>
inline bool isSubnormal(const Eigen::DenseBase<T>& value);

/// Check if a quaternion contains any subnormal floating-point values.
/// Subnormal values have absolute values in the range <code>std::numeric_limits&lt;T&gt;::%denorm_min() &lt;= x
/// &lt; std::numeric_limits&lt;T&gt;::%min()</code>, and can result in very slow floating point calculations under
/// some conditions.
/// \tparam T the base type used to describe the quaternion.  Can usually be deduced.
/// \param value the quaternion value to check.
/// \returns true if any value is subnormal; false if none are (i.e. each is normal, zero, infinite or NaN).
template <typename T>
inline bool isSubnormal(const Eigen::QuaternionBase<T>& value);

/// Check if an angle/axis 3D rotation contains any subnormal floating-point values.
/// Subnormal values have absolute values in the range <code>std::numeric_limits&lt;T&gt;::%denorm_min() &lt;= x
/// &lt; std::numeric_limits&lt;T&gt;::%min()</code>, and can result in very slow floating point calculations under
/// some conditions.
/// \tparam T the scalar type used to describe the rotation.  Can usually be deduced.
/// \param value the rotation value to check.
/// \returns true if any value is subnormal; false if none are (i.e. each is normal, zero, infinite or NaN).
template <typename T>
inline bool isSubnormal(const Eigen::AngleAxis<T>& value);

/// Check if a 2D rotation is described by an angle that is subnormal.
/// Subnormal values have absolute values in the range <code>std::numeric_limits&lt;T&gt;::%denorm_min() &lt;= x
/// &lt; std::numeric_limits&lt;T&gt;::%min()</code>, and can result in very slow floating point calculations under
/// some conditions.
/// \tparam T the scalar type used to describe the rotation.  Can usually be deduced.
/// \param value the 2D rotation value to check.
/// \returns true if the angle is subnormal; false if not (normal, zero, infinite or NaN).
template <typename T>
inline bool isSubnormal(const Eigen::Rotation2D<T>& value);

/// Check if a transform contains any subnormal floating-point values.
/// Subnormal values have absolute values in the range <code>std::numeric_limits&lt;T&gt;::%denorm_min() &lt;= x
/// &lt; std::numeric_limits&lt;T&gt;::%min()</code>, and can result in very slow floating point calculations under
/// some conditions.
/// \tparam T the scalar type used to describe the transform.  Can usually be deduced.
/// \tparam D the dimension used to describe the transform.  Can usually be deduced.
/// \tparam M the mode value used to describe the transform.  Can usually be deduced.
/// \tparam O the options value used to describe the transform.  Can usually be deduced.
/// \param value the transform value to check.
/// \returns true if any value is subnormal; false if none are (i.e. each is normal, zero, infinite or NaN).
template <typename T, int D, int M, int O>
inline bool isSubnormal(const Eigen::Transform<T, D, M, O>& value);



/// If the <code>float</code> value is subnormal, set it to zero.
/// Subnormal values have absolute values in the range <code>std::numeric_limits&lt;float&gt;::%denorm_min() &lt;=
/// x &lt; std::numeric_limits&lt;float&gt;::%min()</code>, and can result in very slow floating point calculations
/// under some conditions.
/// \param [in,out] value the value to check and possibly modify.
/// \returns true if the value was modified.
inline bool setSubnormalToZero(float* value);

/// If the <code>double</code> value is subnormal, set it to zero.
/// Subnormal values have absolute values in the range <code>std::numeric_limits&lt;double&gt;::%denorm_min() &lt;=
/// x &lt; std::numeric_limits&lt;double&gt;::%min()</code>, and can result in very slow floating point calculations
/// under some conditions.
/// \param [in,out] value the value to check and possibly modify.
/// \returns true if the value was modified.
inline bool setSubnormalToZero(double* value);

/// Set all subnormal values in a matrix or a vector to zero.
/// Subnormal values have absolute values in the range <code>std::numeric_limits&lt;T&gt;::%denorm_min() &lt;= x
/// &lt; std::numeric_limits&lt;T&gt;::%min()</code>, and can result in very slow floating point calculations under
/// some conditions.
/// \tparam T the base type used to describe the matrix or vector.  Can usually be deduced.
/// \param [in,out] value the matrix or vector value to check and possibly modify.
/// \returns true if any value was modified.
template <typename T>
inline bool setSubnormalToZero(Eigen::DenseBase<T>* value);

/// Set all subnormal values in a quaternion to zero.
/// Subnormal values have absolute values in the range <code>std::numeric_limits&lt;T&gt;::%denorm_min() &lt;= x
/// &lt; std::numeric_limits&lt;T&gt;::%min()</code>, and can result in very slow floating point calculations under
/// some conditions.
/// \tparam T the base type used to describe the quaternion.  Can usually be deduced.
/// \param [in,out] value the quaternion value to check and possibly modify.
/// \returns true if any value was modified.
template <typename T>
inline bool setSubnormalToZero(Eigen::QuaternionBase<T>* value);

/// Set all subnormal values in an angle/axis 3D rotation to zero.
/// Subnormal values have absolute values in the range <code>std::numeric_limits&lt;T&gt;::%denorm_min() &lt;= x
/// &lt; std::numeric_limits&lt;T&gt;::%min()</code>, and can result in very slow floating point calculations under
/// some conditions.
/// \tparam T the scalar type used to describe the rotation.  Can usually be deduced.
/// \param [in,out] value the rotation value to check and possibly modify.
/// \returns true if any value was modified.
template <typename T>
inline bool setSubnormalToZero(Eigen::AngleAxis<T>* value);

/// If the angle of a 2D rotation is subnormal, set it to zero.
/// Subnormal values have absolute values in the range <code>std::numeric_limits&lt;T&gt;::%denorm_min() &lt;= x
/// &lt; std::numeric_limits&lt;T&gt;::%min()</code>, and can result in very slow floating point calculations under
/// some conditions.
/// \tparam T the scalar type used to describe the rotation.  Can usually be deduced.
/// \param [in,out] value the rotation value to check and possibly modify.
/// \returns true if the value was modified.
template <typename T>
inline bool setSubnormalToZero(Eigen::Rotation2D<T>* value);

/// Set all subnormal values in a transform to zero.
/// Subnormal values have absolute values in the range <code>std::numeric_limits&lt;T&gt;::%denorm_min() &lt;= x
/// &lt; std::numeric_limits&lt;T&gt;::%min()</code>, and can result in very slow floating point calculations under
/// some conditions.
/// \tparam T the base type used to describe the transform.  Can usually be deduced.
/// \param [in,out] value the transform value to check and possibly modify.
/// \returns true if any value was modified.
template <typename T, int D, int M, int O>
inline bool setSubnormalToZero(Eigen::Transform<T, D, M, O>* value);

};  // namespace Math
};  // namespace SurgSim


#include <SurgSim/Math/Valid-inl.h>


#endif  // SURGSIM_MATH_VALID_H
