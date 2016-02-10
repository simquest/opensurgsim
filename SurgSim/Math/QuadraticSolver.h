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

#ifndef SURGSIM_MATH_QUADRATICSOLVER_H
#define SURGSIM_MATH_QUADRATICSOLVER_H

namespace SurgSim
{
namespace Math
{

/// Find the roots of a quadratic equation
/// \tparam T The equation coefficient type
/// \param a, b, c The quadratic equation coefficient as \f$ax^2 + bx + c\f$
/// \param[out] roots The roots
/// \return The number of roots found and stored in 'roots'
template <class T>
int findRoots(const T& a, const T& b, const T& c, T* roots);

}; // Math
}; // SurgSim

#include "SurgSim/Math/QuadraticSolver-inl.h"

#endif // SURGSIM_MATH_QUADRATICSOLVER_H
