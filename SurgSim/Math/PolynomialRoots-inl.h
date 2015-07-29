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

namespace SurgSim
{
namespace Math
{

// PolynomialRootsCommon
template <typename T, int N>
bool PolynomialRootsCommon<T, N>::isDegenerate() const
{
	return m_numRoots == DEGENERATE;
}

template <typename T, int N>
int PolynomialRootsCommon<T, N>::getNumRoots() const
{
	return m_numRoots;
}

template <typename T, int N>
T PolynomialRootsCommon<T, N>::operator[](const int i) const
{
	SURGSIM_ASSERT((m_numRoots > i) && (i >= 0)) <<
			"Requesting a root beyond the number of roots available for this polynomial, " <<
			"or a root with a negative index.";
	return m_roots[i];
}

// roots of an degree-1 polynomial (linear)
template <typename T>
PolynomialRoots<T, 1>::PolynomialRoots(const Polynomial<T, 1>& p, const T& epsilon)
{
	solve<T, 1>(p.getCoefficient(1), p.getCoefficient(0), static_cast<T>(epsilon),
				&(this->m_numRoots), &(this->m_roots));
}

// roots of an degree-2 polynomial (quadratic)
template <typename T>
PolynomialRoots<T, 2>::PolynomialRoots(const Polynomial<T, 2>& p, const T& epsilon)
{
	solve<T, 2>(p.getCoefficient(2), p.getCoefficient(1), p.getCoefficient(0), static_cast<T>(epsilon),
				&(this->m_numRoots), &(this->m_roots));
}

// Utilities: Solve for roots of linear equation a * x + b = y
template <typename T, int N>
void solve(const T& a, const T& b, const T& epsilon, int* numRoots, std::array<T, N>* roots)
{
	static_assert(N >= 1, "Root array is not large enough to hold the roots of the polynomial");
	if (isNearZero(a, epsilon))
	{
		// The "1-st degree polynomial" is really close to a constant.
		// If the constant is zero, there are infinitely many solutions; otherwise there are zero.
		if (isNearZero(b, epsilon))
		{
			*numRoots = PolynomialRootsCommon<T, N>::DEGENERATE; // infinitely many solutions
		}
		else
		{
			*numRoots = 0;
		}
	}
	else
	{
		*numRoots = 1;
		(*roots)[0] = -b / a;
	}
}

// Utilities: Solve for roots of quadratic equation a * x^2 + b * x + c = y
template <typename T, int N>
void solve(const T& a, const T& b, const T& c, const T& epsilon, int* numRoots, std::array<T, N>* roots)
{
	static_assert(N >= 2, "Root array is not large enough to hold the roots of the polynomial");

	if (isNearZero(a, epsilon))
	{
		// The "2nd degree polynomial" is really (close to) 1st degree or less.
		// We can delegate the solution in this case.
		solve<T, N>(b, c, epsilon, numRoots, roots);
		return;
	}

	T discriminant = b * b - 4.0 * a * c;
	if (discriminant > epsilon)
	{
		*numRoots = 2;
		T sqrtDiscriminant = sqrt(discriminant);
		(*roots)[0] = (-b - sqrtDiscriminant) / (2 * a);
		(*roots)[1] = (-b + sqrtDiscriminant) / (2 * a);
	}
	else if (discriminant > -epsilon)
	{
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
