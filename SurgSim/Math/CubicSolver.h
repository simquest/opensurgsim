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

#ifndef SURGSIM_MATH_CUBIC_SOLVER_H
#define SURGSIM_MATH_CUBIC_SOLVER_H

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
int findRoots(const T& a, const T& b, const T& c, T* roots)
{
	// Degenerate case? Not a quadratic equation, but linear
	if (std::abs(a) <= 1e-15)
	{
		// Degenerate case? Not a linear equation
		if (std::abs(b) <= 1e-15)
		{
			return 0;
		}
		roots[0] = -c / b;
		return 1;
	}

	T delta = b * b - static_cast<T>(4.0) * a * c;
	if (std::abs(delta) < 1e-15)
	{
		roots[0] = -b / (static_cast<T>(2.0) * a);
		return 1;
	}
	else if (std::signbit(delta)) // Negative?
	{
		return 0;
	}

	T tmp = std::sqrt(delta);
	roots[0] = (-b - tmp) / (static_cast<T>(2.0) * a);
	roots[1] = (-b + tmp) / (static_cast<T>(2.0) * a);
	return 2;
}

/// Evaluate a cubic equation
/// \tparam T The equation coefficient type
/// \param a, b, c, d The cubic equation coefficient as \f$ax^3 + bx^2 + cx + d\f$
/// \param x The value to evaluate the polynomial at
/// \return The polynomial evaluation at \f$x\f$, i.e. \f$ax^3 + bx^2 + cx + d\f$
template <class T>
T evaluatePolynomial(const T& a, const T& b, const T& c, const T& d, const T& x)
{
	return d + x * (c + x * (b + x * a));
}

/// Evaluate the derivative of a cubic equation
/// \tparam T The equation coefficient type
/// \param a, b, c, d The cubic equation coefficient as \f$ax^3 + bx^2 + cx + d\f$
/// \param x The value to evaluate the polynomial at
/// \return The polynomial derivative evaluation at \f$x\f$, i.e. \f$3ax^2 + 2bx + c\f$
template <class T>
T evaluatePolynomialDerivative(const T& a, const T& b, const T& c, const T& d, const T& x)
{
	return c + x * (static_cast<T>(2) * b + x * static_cast<T>(3) * a);
}

}; // Math
}; // SurgSim

//#include "SurgSim/Math/CubicSolver-inl.h"

#endif // SURGSIM_MATH_CUBIC_SOLVER_H
