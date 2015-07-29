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
/// \tparam T type of the coefficients and computations
/// \tparam N degree of the polynomial being managed
template <typename T, int N> class PolynomialValues;

/// PolynomialValues<T, 0> specializes the PolynomialValues class for degree 0 (constant polynomials)
/// \sa PolynomialValues<T, N>
template <class T>
class PolynomialValues<T, 0>
{
public:
	/// Constructor. Initialize based on the polynomial p
	/// \param p polynomial on which the value calculations are based
	explicit PolynomialValues(const Polynomial<T, 0>& p);

	/// \return the polynomial basis of this calculation
	const Polynomial<T, 0>& getPolynomial() const;

	/// \param interval an interval on the independent variable over which the
	/// values are to be calculated
	/// \return the minimum and maximum polynomial values over interval
	Interval<T> valuesOverInterval(const Interval<T>& interval) const;

private:
	/// The polynomial under consideration
	Polynomial<T, 0> m_polynomial;
};

/// PolynomialValues<T, 1> specializes the PolynomialValues class for degree 1 (linear polynomials)
/// \sa PolynomialValues<T, N>
template <class T>
class PolynomialValues<T, 1>
{
public:
	/// Constructor. Initialize based on the polynomial p
	/// \param p polynomial on which the value calculations are based
	explicit PolynomialValues(const Polynomial<T, 1>& p);

	/// \return the polynomial basis of this calculation
	const Polynomial<T, 1>& getPolynomial() const;

	/// \param interval an interval on the independent variable over which the
	/// values are to be calculated
	/// \return the minimum and maximum polynomial values over interval
	Interval<T> valuesOverInterval(const Interval<T>& interval) const;

private:
	/// The polynomial under consideration
	Polynomial<T, 1> m_polynomial;
};

/// PolynomialValues<T, 2> specializes the PolynomialValues class for degree 2 (quadratic polynomials)
/// \sa PolynomialValues<T, N>
template <class T>
class PolynomialValues<T, 2>
{
public:
	/// Constructor. Initialize based on the polynomial p
	/// \param p polynomial on which the value calculations are based
	explicit PolynomialValues(const Polynomial<T, 2>& p);

	/// \return the polynomial basis of this calculation
	const Polynomial<T, 2>& getPolynomial() const;

	/// \return the derivative of the polynomial basis for this calculation
	const Polynomial<T, 1>& getDerivative() const;

	/// \return the locations of the extrema for the polynomial
	const PolynomialRoots<T, 1>& getLocationsOfExtrema() const;

	/// \param interval an interval on the independent variable over which the
	/// values are to be calculated
	/// \return the minimum and maximum polynomial values over interval
	Interval<T> valuesOverInterval(const Interval<T>& interval) const;

private:
	/// The polynomial under consideration
	Polynomial<T, 2> m_polynomial;

	/// Cached version of the derivative of the polynomial
	Polynomial<T, 1> m_derivative;

	/// Cached version of the locations of the extrema
	PolynomialRoots<T, 1> m_locationOfExtremum;
};

/// Calculate the minimum and maximum values of the dependent variable over a specified
/// range of the independent variable
/// \tparam N degree of the polynomial being managed
/// \tparam T type of the coefficients and computations
/// \param p polynomial on which the value calculations are based
/// \param interval an interval on the independent variable over which the
/// values are to be calculated
/// \return the minimum and maximum polynomial values over interval
template <int N, class T>
Interval<T> valuesOverInterval(const Polynomial<T, N>& p, const Interval<T>& interval);

}; // Math
}; // SurgSim

#include "SurgSim/Math/PolynomialValues-inl.h"

#endif // SURGSIM_MATH_POLYNOMIALVALUES_H
