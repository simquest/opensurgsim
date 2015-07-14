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

#ifndef SURGSIM_MATH_POLYNOMIALROOTS_INL_H
#define SURGSIM_MATH_POLYNOMIALROOTS_INL_H

#include <iostream>
#include <algorithm>

#include "SurgSim/Math/Polynomial.h"

namespace SurgSim
{
namespace Math
{

// PolynomialRootsCommon
template <int N, typename T>
bool PolynomialRootsCommon<N, T>::isDegenerate() const
{
	return m_numData == DEGENERATE;
}

template <int N, typename T>
int PolynomialRootsCommon<N, T>::getNumRoots() const
{
	return m_numData;
}

template <int N, typename T>
T PolynomialRootsCommon<N, T>::operator[](const size_t i) const
{
	SURGSIM_ASSERT((m_numData >= 0) && (i < m_numData)) <<
			"Requesting a root beyond the number of roots available for this polynomial";
	return m_data[i];
}

// roots of an order-1 polynomial (linear)
template <typename T>
PolynomialRoots<1, T>::PolynomialRoots(const Polynomial<1, T>& p, const T& epsilon)
{
	solve(p.getCoefficient(1), p.getCoefficient(0), static_cast<T>(epsilon), &m_numData, &m_data);
}

// roots of an order-1 polynomial (linear)
template <typename T>
PolynomialRoots<2, T>::PolynomialRoots(const Polynomial<2, T>& p, const T& epsilon)
{
	solve(p.getCoefficient(2), p.getCoefficient(1), p.getCoefficient(0), static_cast<T>(epsilon), &m_numData, &m_data);
}
// Utilities: Solve for roots.
template <int N, typename T>
void solve(const T& a, const T& b, const T& epsilon, int* numRoots, std::array<T, N>* roots)
{
	if (isNearZero(a, epsilon))
	{
		// The "1-st degree polynomial" is really close to a constant.
		// If the constant is zero, there are infinitely many solutions; otherwise there are zero.
		if (isNearZero(b, epsilon))
		{
			*numRoots = PolynomialRootsCommon<N, T>::DEGENERATE;  // infinitely many solutions
		}
		else
		{
			*numRoots = 0;
		}
	}
	else
	{
		SURGSIM_ASSERT(N >= 1) << "Root array is not large enough to hold the roots of the polynomial";
		*numRoots = 1;
		(*roots)[0] = -b / a;
	}
}

template <int N, typename T>
void solve(const T& a, const T& b, const T& c, const T& epsilon, int* numRoots, std::array<T, N>* roots)
{
	if (isNearZero(a, epsilon))
	{
		// The "2nd degree polynomial" is really (close to) 1st degree or less.
		// We can delegate the solution in this case.
		solve(b, c, epsilon, numRoots, roots);
		return;
	}

	T discriminant = b * b - 4.*a * c;
	if (discriminant > epsilon)
	{
		SURGSIM_ASSERT(N >= 2) << "Root array is not large enough to hold the roots of the polynomial";
		*numRoots = 2;
		T sqrtDiscriminant = sqrt(discriminant);
		(*roots)[0] = (-b - sqrtDiscriminant) / (2 * a);
		(*roots)[1] = (-b + sqrtDiscriminant) / (2 * a);
	}
	else if (discriminant > -epsilon)
	{
		SURGSIM_ASSERT(N >= 1) << "Root array is not large enough to hold the roots of the polynomial";
		*numRoots = 1;
		(*roots)[0] = -b / (2 * a);
	}
	else
	{
		*numRoots = 0;
	}
}

}; // Math
}; // SurgSim

#endif // SURGSIM_MATH_POLYNOMIALROOTS_INL_H
