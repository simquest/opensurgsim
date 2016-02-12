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

#ifndef SURGSIM_MATH_CUBICSOLVER_H
#define SURGSIM_MATH_CUBICSOLVER_H

#include "SurgSim/Math/Polynomial.h"

namespace SurgSim
{
namespace Math
{

/// Find all roots in range \f$[0 \ldotp\ldotp 1]\f$ of a cubic equation
/// \tparam T The equation coefficient type
/// \param polynomial The cubic polynomial \f$ax^3 + bx^2 + cx + d\f$
/// \param[out] roots All roots ordered ascendingly in \f$[0 \ldotp\ldotp 1]\f$ if any
/// \return The number of roots found in \f$[0 \ldotp\ldotp 1]\f$ and saved in roots.
/// \f[
///  \begin{array}{lll}
///   P(x) &=& ax^3 + bx^2 + cx + d \\ \text{}
///   P'(x) &=& 3ax^2 + 2bx + c \Rightarrow \Delta = (2b)^2 - 4(3a)(c) = 4(b^2 - 3ac)
///  \end{array}
///  \\ \text{}
///  \left\{
///  \begin{array}{ll}
///   \Delta < 0 & \text{P is monotonic, P' is always the same sign, the sign of P'(0) = sign(c)} \\ \text{}
///   \Delta = 0 & \text{P is monotonic with an inflection point, P' is always the same sign, except at P'(root) = 0}
///   \\ \text{}
///   \Delta > 0 & \text{P is monotonic on 3 separate intervals}
///  \end{array}
///  \right.
/// \f]
template <class T>
int findRootsInRange01(const Polynomial<T, 3>& polynomial, T* roots);

}; // Math
}; // SurgSim

#include "SurgSim/Math/CubicSolver-inl.h"

#endif // SURGSIM_MATH_CUBICSOLVER_H
