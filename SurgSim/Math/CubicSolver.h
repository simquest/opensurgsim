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

#include <limits>

namespace SurgSim
{
namespace Math
{

template <class T>
T isZero(const T& a)
{
	return std::abs(a) <= std::numeric_limits<T>::epsilon();
}

/// Find the roots of a quadratic equation
/// \tparam T The equation coefficient type
/// \param a, b, c The quadratic equation coefficient as \f$ax^2 + bx + c\f$
/// \param[out] roots The roots
/// \return The number of roots found and stored in 'roots'
template <class T>
int findRoots(const T& a, const T& b, const T& c, T* roots)
{
	// Degenerate case? Not a quadratic equation, but linear
	if (isZero(a))
	{
		// Degenerate case? Not a linear equation
		if (isZero(b))
		{
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

/// Find the root of a cubic equation in a given range using a dichotomic search
/// \tparam T The equation coefficient type
/// \param a, b, c, d The cubic equation coefficient as \f$ax^3 + bx^2 + cx + d\f$
/// \param min, max The range to look into (\f$min <= max\f$)
/// \return The root
/// \note This function supposes that the polynomial is monotonic in the range \f$[min \ldotp\ldotp max]\f$
/// \note and has a solution.
template <class T>
T findRootInRange(const T& a, const T& b, const T& c, const T& d, T min, T max)
{
	T Pmin = evaluatePolynomial(a, b, c, d, min);
	if (isZero(Pmin))
	{
		return min;
	}

	T Pmax = evaluatePolynomial(a, b, c, d, max);
	if (isZero(Pmax))
	{
		return max;
	}

	while (max - min > std::numeric_limits<T>::epsilon())
	{
		T middle = (max + min) * 0.5;
		T Pmiddle = evaluatePolynomial(a, b, c, d, middle);
		if (isZero(Pmiddle))
		{
			return middle;
		}

		if (std::signbit(Pmin) == std::signbit(Pmiddle))
		{
			min = middle;
			Pmin = Pmiddle;
		}
		else
		{
			max = middle;
			Pmax = Pmiddle;
		}
	}

	return (max + min) * 0.5;
}

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
///   \Delta = 0 & \text{P is monotonic with an inflection point, P' is always the same sign, except at P'(root) = 0} \\ \text{}
///   \Delta > 0 & \text{P is monotonic on 3 separate intervals}
///  \end{array}
///  \right.
/// \f]
template <class T>
bool findSmallestRootInRange01(const T& a, const T& b, const T& c, const T& d, T* root)
{
	// Is 0 a root? P(0)=d=0?
	if (isZero(d))
	{
		root[0] = 0.0;
		return 1;
	}

	// Is degenerate?
	if (isZero(a))
	{
		T roots[2];
		int nRoots = findRoots(b, c, d, roots);

		root[0] = static_cast<T>(2);
		for (int i = 0; i < nRoots; ++i)
		{
			if (roots[i] >= 0.0 && roots[i] <= 1.0 && roots[i] < root[0])
			{
				root[0] = roots[i];
			}
		}
		return root[0] != static_cast<T>(2);
	}

	return false;
}

}; // Math
}; // SurgSim

//#include "SurgSim/Math/CubicSolver-inl.h"

#endif // SURGSIM_MATH_CUBIC_SOLVER_H
