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

#ifndef SURGSIM_MATH_POLYNOMIALROOTS_H
#define SURGSIM_MATH_POLYNOMIALROOTS_H

#include <iostream>
#include <algorithm>

#include "SurgSim/Math/Polynomial.h"

namespace SurgSim
{
namespace Math
{

/// The (algebraic) roots of a Polynomial<N,T>.
/// If all coefficients of a polynomial are 0, the roots are degenerate: the polynomial equals 0 for any x.
/// Otherwise, there may be anywhere between 0 and N roots.
///
/// \tparam N degree of the polynomial for which roots are being calculated
/// \tparam T type of the coefficients and computations
template <int N, typename T = double> class PolynomialRoots;

/// The common base class for PolynomialRoots specializations for various N.
///
/// \tparam N degree of the polynomial for which roots are being calculated
/// \tparam T type of the coefficients and computations
template <int N, typename T = double>
class PolynomialRootsCommon
{
public:
	/// Indicator for a degenerate polynomial (infinite number of roots).
	static const int DEGENERATE = -1;

	/// \return true in the polynomial is degenerate
	bool isDegenerate() const;

	/// \return the number of available roots ot DEGENERATE if there are infinitely many
	int getNumRoots() const;

	/// Read only access to the roots of the polynomial
	/// \param i is the number of the root to return
	/// \return the value of the ith root
	/// \exception if there is no root of rank i
	T operator[](const int i) const;

private:
	/// @{
	/// Prohibit copying and assignment.
	PolynomialRootsCommon(const PolynomialRootsCommon&);
	PolynomialRootsCommon& operator=(const PolynomialRootsCommon&);
	/// @}

protected:
	/// Constructor. Since the constructor must define the roots, only allow construction from a derived class
	PolynomialRootsCommon() {}

	/// The number of roots available for the polynomial, or DEGENERATE if there are infinite roots
	int m_numData;

	/// An array of up to N roots for a degree N polynomial
	std::array<T, N> m_data;
};

// ======================================================================

/// Specialization for the roots of an order-1 polynomial (linear)
/// \tparam T type of the coefficients and computations
template <typename T>
class PolynomialRoots<1, T> : public PolynomialRootsCommon<1, T>
{
public:
	/// Constructor
	/// \param p the degree 1 polynomial for which the roots are to be calculated
	/// \param epsilon tolerance parameter for determining the number of valid, unique roots
	explicit PolynomialRoots(const Polynomial<1, T>& p, const T& epsilon = 1.0e-09);
};

// ======================================================================

/// Specialization for the roots of an order-2 polynomial (quadratic)
/// \tparam T type of the coefficients and computations
template <typename T>
class PolynomialRoots<2, T> : public PolynomialRootsCommon<2, T>
{
public:
	/// Constructor
	/// \param p the degree 2 polynomial for which the roots are to be calculated
	/// \param epsilon tolerance parameter for determining the number of valid, unique roots
	explicit PolynomialRoots(const Polynomial<2, T>& p, const T& epsilon = 1.0e-09);
};

/// Specialized solve routine for linear polynomials (2 coefficients)
/// \tparam N maximum number of roots that can be stored
/// \tparam T type of the coefficients and computations
/// \param a coefficient of the linear term
/// \param b coefficient of the constant term
/// \param epsilon tolerance parameter for determining the number of valid, unique roots
/// \param numRoots [out] number of roots calculated, or DEGENERATE if there are infinitely many
/// \param roots [out] array containing the calculated roots
/// \exception if there are more than N roots
template <int N, typename T>
void solve(const T& a, const T& b, const T& epsilon, int* numRoots, std::array<T, N>* roots);

/// Specialized solve routine for quadratic polynomials (3 coefficients)
/// \tparam N maximum number of roots that can be stored
/// \tparam T type of the coefficients and computations
/// \param a coefficient of the square term
/// \param b coefficient of the linear term
/// \param c coefficient of the constant term
/// \param epsilon tolerance parameter for determining the number of valid, unique roots
/// \param numRoots [out] number of roots calculated, or DEGENERATE if there are infinitely many
/// \param roots [out] array containing the calculated roots
/// \exception if there are more than N roots
template <int N, typename T>
void solve(const T& a, const T& b, const T& c, const T& epsilon, int* numRoots, std::array<T, N>* roots);

}; // Math
}; // SurgSim

#include "SurgSim/Math/PolynomialRoots-inl.h"

#endif // SURGSIM_MATH_POLYNOMIALROOTS_H
