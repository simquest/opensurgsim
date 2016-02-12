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

#ifndef SURGSIM_MATH_CUBICSOLVER_INL_H
#define SURGSIM_MATH_CUBICSOLVER_INL_H

#include <cmath>
#include <limits>
#include <vector>

#include "SurgSim/Math/IntervalArithmetic.h"
#include "SurgSim/Math/QuadraticSolver.h"

namespace SurgSim
{
namespace Math
{

namespace CubicSolver
{
/// \tparam T The scalar type
/// \param a The scalar to test
/// \return True if the scalar a is 0 or close to 0 (within epsilon defined by the type T)
template <class T>
bool isZero(const T& a)
{
	return std::abs(a) <= std::numeric_limits<T>::epsilon();
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
/// \note and has a solution (i.e. P(min) * P(max) < 0)
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
}; // namespace CubicSolver

template <class T>
int findRootsInRange01(const T& a, const T& b, const T& c, const T& d, T* roots)
{
	using CubicSolver::isZero;
	using CubicSolver::evaluatePolynomial;
	using CubicSolver::findRootInRange;

	int numberOfRoots = 0;

	// Is degenerate?
	if (isZero(a))
	{
		T rootsQuadratic[2];
		int nRoots = findRoots(b, c, d, rootsQuadratic);

		for (int i = 0; i < nRoots; ++i)
		{
			if (rootsQuadratic[i] >= 0.0 && rootsQuadratic[i] <= 1.0)
			{
				roots[numberOfRoots++] = rootsQuadratic[i];
			}
		}

		// Make sure they are ordered ascendingly
		if (numberOfRoots == 2 && roots[0] > roots[1])
		{
			std::swap(roots[0], roots[1]);
		}

		return numberOfRoots;
	}

	// General case, let's analyze the derivative...first we calculate the discriminant:
	T delta = static_cast<T>(4) * (b * b - static_cast<T>(3) * a * c);

	if (delta < 0.0 || isZero(delta))
	{
		// If delta = 0, P'(-b/3a) = 0, and P'(x) has the same sign anywhere else
		// Therefore P is monotonic (ascending or descending) and has 1 unique root over ]-Inf +Inf[
		//
		// If delta < 0, P' is never null and has always the same sign, the sign of P'(0) (i.e. sign(c))
		// Therefore P is monotonic (stricly ascending or descending) and has 1 unique root over ]-Inf +Inf[

		T P0 = d; // evaluatePolynomial(a, b, c, d, static_cast<T>(0));
		if (isZero(P0))
		{
			roots[0] = 0.0;
			return 1;
		}

		T P1 = evaluatePolynomial(a, b, c, d, static_cast<T>(1));
		if (isZero(P1))
		{
			roots[0] = static_cast<T>(1);
			return 1;
		}

		// P0 and P1 cannot be zero at this stage, so they both have a clear sign
		if (std::signbit(P0) != std::signbit(P1))
		{
			roots[0] = findRootInRange(a, b, c, d, static_cast<T>(0), static_cast<T>(1));
			return 1;
		}
	}
	else
	{
		// If the discriminant is positive, P'(x) has 2 roots {x0, x1}, which define 3 separate intervals to
		// study ]-Inf x0[, [x0 x1] and ]x1 +Inf[
		T tmp = std::sqrt(delta / 4.0);
		T scale = static_cast<T>(1) / (static_cast<T>(3) * a);
		T x0 = (-b - tmp) * scale;
		T x1 = (-b + tmp) * scale;
		if (x0 > x1)
		{
			std::swap(x0, x1);
		}
		// Note that x0 < x1 and x0 != x1 as delta != 0

		Interval<T> intervalx0x1(x0, x1);
		Interval<T> interval01(static_cast<T>(0), static_cast<T>(1));

		if (!intervalx0x1.overlapsWith(interval01))
		{
			// If there is no overlap, the interval [0..1] is isolated on one of the monotonic interval ]-Inf x0[
			// ]x1 +Inf[, therefore it contains at most 1 root

			T P0 = d; // evaluatePolynomial(a, b, c, d, static_cast<T>(0));
			if (isZero(P0))
			{
				roots[0] = 0.0;
				return 1;
			}

			T P1 = evaluatePolynomial(a, b, c, d, static_cast<T>(1));
			if (isZero(P1))
			{
				roots[0] = static_cast<T>(1);
				return 1;
			}

			// P0 and P1 cannot be zero at this stage, so they both have a clear sign
			if (std::signbit(P0) != std::signbit(P1))
			{
				roots[0] = findRootInRange(a, b, c, d, static_cast<T>(0), static_cast<T>(1));
				return 1;
			}
		}
		else
		{
			// Build the monotonic intervals within [0..1] to be analyzed one by one
			std::vector<Interval<T>> interval01;

			T lastValue = static_cast<T>(0);
			if (x0 > static_cast<T>(0) && x0 < static_cast<T>(1))
			{
				interval01.push_back(Interval<T>(lastValue, x0));
				lastValue = x0;
			}
			if (x1 > static_cast<T>(0) && x1 < static_cast<T>(1))
			{
				interval01.push_back(Interval<T>(lastValue, x1));
				lastValue = x1;
			}
			interval01.push_back(Interval<T>(lastValue, static_cast<T>(1)));

			for (auto interval : interval01)
			{
				// On each interval, only 1 root can be found
				T Pmin = evaluatePolynomial(a, b, c, d, interval.getMin());
				if (isZero(Pmin))
				{
					roots[numberOfRoots++] = interval.getMin();
				}
				else
				{
					T Pmax = evaluatePolynomial(a, b, c, d, interval.getMax());
					if (isZero(Pmax))
					{
						roots[numberOfRoots++] = interval.getMax();
					}
					else if (std::signbit(Pmin) != std::signbit(Pmax))
					{
						roots[numberOfRoots++] = findRootInRange(a, b, c, d, interval.getMin(), interval.getMax());
					}
				}
			}
		}
	}

	return numberOfRoots;
}

}; // Math
}; // SurgSim

#endif // SURGSIM_MATH_CUBICSOLVER_INL_H
