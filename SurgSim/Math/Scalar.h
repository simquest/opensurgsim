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

#ifndef SURGSIM_MATH_SCALAR_H
#define SURGSIM_MATH_SCALAR_H

namespace SurgSim
{
namespace Math
{
/// Clamp any values within an epsilon window to a maximum or minimum value.
/// \note max is tested first, so if the value < min + epsilon and value >
/// max - epsilon, value will be set to max.
/// \tparam T underlying type
/// \param value [in/out] the value to be clamped
/// \param min the minimum value for the clamp
/// \param max the maximum value for the clamp
/// \param epsilon definition of the epsilon window.
template <class T>
T clamp(T value, T min, T max, T epsilon);

/// define a custom template unary functor to execute the clamp operation
/// over an Eigen matrix structure. The operation clamps based on an epsilon
/// interval. For values outside the interval min to max, the clamp proceeds
/// as expected. However, values within the interval that lie within epsilon
/// of an endpoint are also clamped to th endpoint.
///
/// \tparam T the type over which the operator is defined.
template<typename T>
class clampOperator
{
public:
	/// Constructor.
	///
	/// \param min minimum value for the clamp interval
	/// \param max maximum value for the clamp interval
	/// \param epsilon values within epsilon of an interval endpoint
	/// are clamped to the interval regardless of if they are within the interval or not
	clampOperator(const T& min, const T& max, const T& epsilon) : m_min(min), m_max(max),
		m_epsilon(epsilon) {}

	/// Execute the clamp operator
	/// \param x the value to clamp based
	/// \return the x clamped using the minimum, maximum and epsilon specified in he class constructor
	const T operator()(const T& x) const;

private:
	/// The minimum value of the interval
	T m_min;

	/// The maximum value of the interval
	T m_max;

	/// The closeness parameter for the clamp. Values within epsilon of an interval endpoint
	/// are clamped to the interval regardless of if they are within the interval or not
	T m_epsilon;
};

};
};

#include "SurgSim/Math/Scalar-inl.h"

#endif // SURGSIM_MATH_SCALAR_H
