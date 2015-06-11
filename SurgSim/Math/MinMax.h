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

#ifndef SURGSIM_MATH_MINMAX_H
#define SURGSIM_MATH_MINMAX_H

namespace SurgSim
{
namespace Math
{
/// Calculate the minimum and maximum of two values
/// \tparam T underlying type
/// \param a1 the first value
/// \param a2 the second value
/// \param min [out] the minimum of a1 and a2
/// \param max [out] the maximum of a1 and a2
template <class T> void MinMax(const T a1, const  T a2, T& min, T& max);

/// Calculate the minimum and maximum of three values
/// \tparam T underlying type
/// \param a1 the first value
/// \param a2 the second value
/// \param a3 the third value
/// \param minVal [out] the minimum of a1, a2 and a3
/// \param maxVal [out] the maximum of a1, a2 and a3
template <class T> void MinMax(const T a1, const  T a2, const  T a3, T& minVal, T& maxVal);

/// Calculate the minimum and maximum of four values
/// \tparam T underlying type
/// \param a1 the first value
/// \param a2 the second value
/// \param a3 the third value
/// \param a4 the fourth value
/// \param minVal [out] the minimum of a1, a2, a3 and a4
/// \param maxVal [out] the maximum of a1, a2, a3 and a4
template <class T> void MinMax(const T a1, const T a2, const T a3, const T a4, T& minVal, T& maxVal);

/// Calculate the minimum and maximum of five values
/// \tparam T underlying type
/// \param a1 the first value
/// \param a2 the second value
/// \param a3 the third value
/// \param a4 the fourth value
/// \param a4 the fifth value
/// \param minVal [out] the minimum of a1, a2, a3, a4 and a5
/// \param maxVal [out] the maximum of a1, a2, a3, a4 and a5
template <class T> void MinMax(const T a1, const T a2, const T a3, const T a4, const T a5, T& minVal, T& maxVal);

/// Calculate the minimum and maximum of numValues values
/// \tparam T underlying type
/// \param values a series of numValues values
/// \param numValues the number of values in the series
/// \param minVal [out] the minimum value in values
/// \param minVal [out] the maximum value in values
template <class T> void MinMax(const T* values, int numValues, T& minVal, T& maxVal);
};
};

#include "SurgSim/Math/MinMax-inl.h"

#endif // SURGSIM_MATH_MINMAX_H
