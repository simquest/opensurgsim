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

#ifndef SURGSIM_MATH_QUADRATICSOLVER_INL_H
#define SURGSIM_MATH_QUADRATICSOLVER_INL_H

#include <limits>

namespace SurgSim
{
namespace Math
{

namespace
{
/// \tparam T The scalar type
/// \param a The scalar to test
/// \return True if the scalar a is 0 or close to 0 (within epsilon defined by the type T)
template <class T>
bool isZero(const T& a)
{
	return std::abs(a) <= std::numeric_limits<T>::epsilon();
}
}; // anonymous namespace

// warning C4723: potential divide by 0
#pragma warning(disable : 4723)

template <class T>
int findRoots(const T& a, const T& b, const T& c, T* roots)
{
	// Degenerate case? Not a quadratic equation, but linear
	if (isZero(a))
	{
		// Degenerate case? Not a linear equation
		if (isZero(b))
		{
			// 0x^2 + 0x + c = 0
			if (isZero(c))
			{
				// All scalars are solutions, let's peak 0
				roots[0] = 0.0;
				return 1;
			}

			// No solution (The equation is c = 0 with c != 0)
			return 0;
		}
		roots[0] = -c / b;
		return 1;
	}

	T delta = b * b - static_cast<T>(4.0) * a * c;
	if (isZero(delta))
	{
		roots[0] = -b / (static_cast<T>(2.0) * a);
		return 1;
	}
	else if (std::signbit(delta)) // Negative?
	{
		return 0;
	}

	T tmp = std::sqrt(delta);
	T scale = 1.0 / (static_cast<T>(2.0) * a);
	roots[0] = (-b - tmp) * scale;
	roots[1] = (-b + tmp) * scale;
	return 2;
}

}; // Math
}; // SurgSim

#endif // SURGSIM_MATH_QUADRATICSOLVER_INL_H
