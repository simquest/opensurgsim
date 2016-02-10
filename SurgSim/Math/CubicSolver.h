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

namespace SurgSim
{
namespace Math
{

/// Find the smallest root in range \f$[0 \ldotp\ldotp 1]\f$ of a cubic equation
/// \tparam T The equation coefficient type
/// \param a, b, c, d The cubic equation coefficient as \f$ax^3 + bx^2 + cx + d\f$
/// \param[out] root The smallest root in \f$[0 \ldotp\ldotp 1]\f$ if any
/// \return True if a root was found, False otherwise
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
bool findSmallestRootInRange01(const T& a, const T& b, const T& c, const T& d, T* root);

}; // Math
}; // SurgSim

#include "SurgSim/Math/CubicSolver-inl.h"

#endif // SURGSIM_MATH_CUBICSOLVER_H
