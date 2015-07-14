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

#ifndef SURGSIM_MATH_POLYNOMIALVALUES_H
#define SURGSIM_MATH_POLYNOMIALVALUES_H

#include "SurgSim/Math/Polynomial.h"
#include "SurgSim/Math/PolynomialRoots.h"
#include "SurgSim/Math/IntervalArithmetic.h"
#include "SurgSim/Math/MinMax.h"

namespace SurgSim
{
namespace Math
{

/// Class to manage polynomial based calculations of interval boundaries.
///
/// \tparam N degree of the polynomial being managed
/// \tparam T type of the coefficients and computations
template <int N, class T> class PolynomialValues;

// ----------------------------------------------------------------------

/// Specialization for polynomials of degree 0 (constant)
/// \tparam T type of the coefficients and computations
template <class T>
class PolynomialValues<0, T>
{
public:
	/// Constructor. Initialize based on the polynomial p
	/// \param p polynomial on which the value calculations are based
	explicit PolynomialValues(const Polynomial<0, T>& p);

	/// \return the polynomial basis of this calculation
	const Polynomial<0, T>& getPolynomial() const;

	/// \param interval an interval on the independent variable over which the
	/// values are to be calculated
	/// \return the minimum and maximum polynomial values over interval
	Interval<T> valuesOverInterval(const Interval<T>& interval) const;

private:
	/// The polynomial under consideration
	Polynomial<0, T> m_polynomial;
};

// ----------------------------------------------------------------------

/// Specialization for polynomials of degree 1 (linear)
/// \tparam T type of the coefficients and computations
template <class T>
class PolynomialValues<1, T>
{
public:
	/// Constructor. Initialize based on the polynomial p
	/// \param p polynomial on which the value calculations are based
	explicit PolynomialValues(const Polynomial<1, T>& p);

	/// \return the polynomial basis of this calculation
	const Polynomial<1, T>& getPolynomial() const;

	/// \param interval an interval on the independent variable over which the
	/// values are to be calculated
	/// \return the minimum and maximum polynomial values over interval
	Interval<T> valuesOverInterval(const Interval<T>& interval) const;

private:
	/// The polynomial under consideration
	Polynomial<1, T> m_polynomial;
};

// ----------------------------------------------------------------------

/// Specialization for polynomials of degree 2 (quadratic)
/// \tparam T type of the coefficients and computations
template <class T>
class PolynomialValues<2, T>
{
public:
	/// Constructor. Initialize based on the polynomial p
	/// \param p polynomial on which the value calculations are based
	explicit PolynomialValues(const Polynomial<2, T>& p);

	/// \return the polynomial basis of this calculation
	const Polynomial<2, T>& getPolynomial() const;

	/// \return the derivative of the polynomial basis for this calculation
	const Polynomial<1, T>& getDerivative() const;

	/// \return the locations of the extrema for the polynomial
	const PolynomialRoots<1, T>& getLocationsOfExtrema() const;

	/// \param interval an interval on the independent variable over which the
	/// values are to be calculated
	/// \return the minimum and maximum polynomial values over interval
	Interval<T> valuesOverInterval(const Interval<T>& interval) const;

private:
	/// The polynomial under consideration
	Polynomial<2, T> m_polynomial;

	/// Cached version of the derivative of the polynomial
	Polynomial<1, T> m_derivative;

	/// Cached version of the locations of the extrema
	PolynomialRoots<1, T> m_locationOfExtremum;
};

// ----------------------------------------------------------------------

/// Calculate the minimum and maximum values of the dependent variable over a specified
/// range of the independent variable
/// \tparam N degree of the polynomial being managed
/// \tparam T type of the coefficients and computations
/// \param p polynomial on which the value calculations are based
/// \param interval an interval on the independent variable over which the
/// values are to be calculated
/// \return the minimum and maximum polynomial values over interval
template <int N, class T>
Interval<T> valuesOverInterval(const Polynomial<N, T>& p, const Interval<T>& interval);

}; // Math
}; // SurgSim

#include "SurgSim/Math/PolynomialValues-inl.h"

#endif // SURGSIM_MATH_POLYNOMIALVALUES_H
