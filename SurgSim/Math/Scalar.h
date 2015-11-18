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
void clamp(T* value, T min, T max, T epsilon);

};
};

#include "SurgSim/Math/Scalar-inl.h"

#endif // SURGSIM_MATH_SCALAR_H
