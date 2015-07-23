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

#ifndef SURGSIM_MATH_POLYNOMIAL_H
#define SURGSIM_MATH_POLYNOMIAL_H

#include <ostream>
#include <iostream>

namespace SurgSim
{
namespace Math
{

template <typename T> class Interval;

/// Define an utility function for comparing individual coefficients to 0.
///
/// \tparam T underlying data type used as either the scalar or as the data type
/// for the interval
/// \param value the value to compare
/// \param epsilon the tolerance
/// \return true if the value is within epsilon of 0
template <typename T>
bool isNearZero(const T& value, const T& epsilon = static_cast<T>(1.0e-09));

// ======================================================================

/// Polynomial<N, T> defines the concept of an N degree polynomial with type T
/// coefficients and provides operations on them including arithmetic operations,
/// construction, and IO.
///
/// \tparam N order of the Polynomial
/// \tparam T underlying data type over which the interval is defined.
///
/// \sa Polynomial<0, T>, Polynomial<1, T>, Polynomial<2, T> and Polynomial<3, T>
template <int N, typename T = double> class Polynomial;

/// Polynomial<0, T> defines the concept of a 0 degree polynomial with type T
/// coefficients and provides operations on them including arithmetic operations,
/// construction, and IO.
///
/// \tparam T underlying data type over which the interval is defined.
///
/// \sa Polynomial<1, T>, Polynomial<2, T>, Polynomial<3, T> and Polynomial<N, T>
template <typename T>
class Polynomial<0, T>
{
public:
	/// Constructor
	Polynomial();

	/// Constructor
	/// \param a0 coefficient of the 0 order term
	explicit Polynomial(const T& a0);

	/// Evaluate the polynomial at a point
	/// \param x point at which to evaluate the polynomial
	/// \return the value of the polynomial at x
	T evaluate(const T& x) const;

	/// @{
	/// Standard arithmetic operators extended to intervals
	Polynomial<0, T> operator- () const;
	Polynomial<0, T> operator+ (const Polynomial<0, T>& rhs) const;
	Polynomial<0, T>& operator+= (const Polynomial<0, T>& rhs);
	Polynomial<0, T> operator- (const Polynomial<0, T>& rhs) const;
	Polynomial<0, T>& operator-= (const Polynomial<0, T>& rhs);
	/// @}

	/// \return the derivative of the polynomial
	Polynomial<0, T> derivative() const;

	/// \return true if all coefficients of the polynomial are with epsilon of 0
	bool isNearZero(const T& epsilon = static_cast<T>(1.0e-09)) const;

	/// \param i index of the desired coefficient
	/// \return the value of coefficient i
	T getCoefficient(const size_t i) const;

	/// Set a specified coefficient to a desired value
	/// \param i index of the desired coefficient
	/// \param value the value to be set in the coefficient
	/// \exception if i is greater than the polynomial order
	void setCoefficient(const size_t i, const T& value);

private:
	/// @{
	/// Coefficient of the polynomial
	T m_a0;
	/// @}
};

// ======================================================================

/// Polynomial<1, T> defines the concept of a 1 degree polynomial with type T
/// coefficients and provides operations on them including arithmetic operations,
/// construction, and IO.
///
/// \tparam T underlying data type over which the interval is defined.
///
/// \sa Polynomial<0, T>, Polynomial<2, T>, Polynomial<3, T> and Polynomial<N, T>
template <typename T>
class Polynomial<1, T>
{
public:
	/// Constructor
	Polynomial();

	/// Constructor
	/// \param a0 coefficient of the 0 order term
	/// \param a1 coefficient of the 1 order term
	Polynomial(const T& a0, const T& a1);

	/// Evaluate the polynomial at a point
	/// \param x point at which to evaluate the polynomial
	/// \return the value of the polynomial at x
	T evaluate(const T& x) const;

	/// @{
	/// Standard arithmetic operators extended to intervals
	Polynomial<1, T> operator- () const;
	Polynomial<1, T> operator+ (const Polynomial<1, T>& rhs) const;
	Polynomial<1, T>& operator+= (const Polynomial<1, T>& rhs);
	Polynomial<1, T> operator- (const Polynomial<1, T>& rhs) const;
	Polynomial<1, T>& operator-= (const Polynomial<1, T>& rhs);
	/// @}

	/// \return the derivative of the polynomial
	Polynomial<0, T> derivative() const;

	/// \return true if all coefficients of the polynomial are with epsilon of 0
	bool isNearZero(const T& epsilon = static_cast<T>(1.0e-09)) const;

	/// \param i index of the desired coefficient
	/// \return the value of coefficient i
	T getCoefficient(const size_t i) const;

	/// Set a specified coefficient to a desired value
	/// \param i index of the desired coefficient
	/// \param value the value to be set in the coefficient
	/// \exception if i is greater than the polynomial order
	void setCoefficient(const size_t i, const T& value);

private:
	/// @{
	/// Coefficient of the polynomial
	T m_a0;
	T m_a1;
	/// @}
};

// ======================================================================

/// Polynomial<2, T> defines the concept of a 2 degree polynomial with type T
/// coefficients and provides operations on them including arithmetic operations,
/// construction, and IO.
///
/// \tparam T underlying data type over which the interval is defined.
///
/// \sa Polynomial<0, T>, Polynomial<1, T>, Polynomial<3, T> and Polynomial<N, T>
template <typename T>
class Polynomial<2, T>
{
public:
	/// Constructor
	Polynomial();

	/// Constructor
	/// \param a0 coefficient of the 0 order term
	/// \param a1 coefficient of the 1 order term
	/// \param a2 coefficient of the 2 order term
	Polynomial(const T& a0, const T& a1, const T& a2);

	/// Evaluate the polynomial at a point
	/// \param x point at which to evaluate the polynomial
	/// \return the value of the polynomial at x
	T evaluate(const T& x) const;

	/// @{
	/// Standard arithmetic operators extended to intervals
	Polynomial<2, T> operator- () const;
	Polynomial<2, T> operator+ (const Polynomial<2, T>& rhs) const;
	Polynomial<2, T>& operator+= (const Polynomial<2, T>& rhs);
	Polynomial<2, T> operator- (const Polynomial<2, T>& rhs) const;
	Polynomial<2, T>& operator-= (const Polynomial<2, T>& rhs);
	/// @}

	/// \return the derivative of the polynomial
	Polynomial<1, T> derivative() const;

	/// \return true if all coefficients of the polynomial are with epsilon of 0
	bool isNearZero(const T& epsilon = static_cast<T>(1.0e-09)) const;

	/// \param i index of the desired coefficient
	/// \return the value of coeficient i
	T getCoefficient(const size_t i) const;

	/// Set a specified coefficient to a desired value
	/// \param i index of the desired coefficient
	/// \param value the value to be set in the coefficient
	/// \exception if i is greater than the polynomial order
	void setCoefficient(const size_t i, const T& value);

private:
	/// @{
	/// Coefficient of the polynomial
	T m_a0;
	T m_a1;
	T m_a2;
	/// @}
};

// ======================================================================

/// Polynomial<3, T> defines the concept of a 3 degree polynomial with type T
/// coefficients and provides operations on them including arithmetic operations,
/// construction, and IO.
///
/// \tparam T underlying data type over which the interval is defined.
///
/// \sa Polynomial<0, T>, Polynomial<1, T>, Polynomial<2, T> and Polynomial<N, T>
template <typename T>
class Polynomial<3, T>
{
public:
	/// Constructor
	Polynomial();

	/// Constructor
	/// \param a0 coefficient of the 0 order term
	/// \param a1 coefficient of the 1 order term
	/// \param a2 coefficient of the 2 order term
	/// \param a3 coefficient of the 3 order term
	Polynomial(const T& a0, const T& a1, const T& a2, const T& a3);

	/// Evaluate the polynomial at a point
	/// \param x point at which to evaluate the polynomial
	/// \return the value of the polynomial at x
	T evaluate(const T& x) const;

	/// @{
	/// Standard arithmetic operators extended to intervals
	Polynomial<3, T> operator- () const;
	Polynomial<3, T> operator+ (const Polynomial<3, T>& rhs) const;
	Polynomial<3, T>& operator+= (const Polynomial<3, T>& rhs);
	Polynomial<3, T> operator- (const Polynomial<3, T>& rhs) const;
	Polynomial<3, T>& operator-= (const Polynomial<3, T>& rhs);
	/// @}

	/// \return the derivative of the polynomial
	Polynomial<2, T> derivative() const;

	/// \return true if all coefficients of the polynomial are with epsilon of 0
	bool isNearZero(const T& epsilon = static_cast<T>(1.0e-09)) const;

	/// \param i index of the desired coefficient
	/// \return the value of coefficient i
	T getCoefficient(const size_t i) const;

	/// Set a specified coefficient to a desired value
	/// \param i index of the desired coefficient
	/// \param value the value to be set in the coefficient
	/// \exception if i is greater than the polynomial order
	void setCoefficient(const size_t i, const T& value);

private:
	/// @{
	/// Coefficient of the polynomial
	T m_a0;
	T m_a1;
	T m_a2;
	T m_a3;
	/// @}
};

// Operators

/// Multiply two polynomials of arbitrary degree. This current implementation is limited to
/// a resulting polynomial of no more than degree 3 (i.e. N + M <= 3) because of limits in the
/// underlying polynomial representations.
/// \tparam N degree of the first polynomial
/// \tparam M degree of the second polynomial
/// \tparam T underlying data type over which the interval is defined.
/// \param p first polynomial of degree N
/// \param q second polynomial of degree M
/// \return p * q as a polynomial of degree N + M
/// \exception if N + M > 3
template <int N, int M, typename T>
Polynomial < N + M, T > operator*(const Polynomial<N, T>& p, const Polynomial<M, T>& q);

/// Multiply two polynomials of degree 1.
/// \tparam T underlying data type over which the interval is defined.
/// \param p first polynomial of degree 1
/// \param q second polynomial of degree 1
/// \return p * q as a polynomial of degree 2
template <typename T>
Polynomial<2, T> operator*(const Polynomial<1, T>& p, const Polynomial<1, T>& q);

/// Multiply two polynomials of degree 2 and 1 respectively.
/// \tparam T underlying data type over which the interval is defined.
/// \param p first polynomial of degree 2
/// \param q second polynomial of degree 1
/// \return p * q as a polynomial of degree 3
template <typename T>
Polynomial<3, T> operator*(const Polynomial<2, T>& p, const Polynomial<1, T>& q);

/// Multiply two polynomials of degree 1 and 2 respectively.
/// \tparam T underlying data type over which the interval is defined.
/// \param p first polynomial of degree 1
/// \param q second polynomial of degree 2
/// \return p * q as a polynomial of degree 3
template <typename T>
Polynomial<3, T> operator*(const Polynomial<1, T>& p, const Polynomial<2, T>& q);

// ======================================================================

/// Square a degree 0 polynomial
/// \tparam T underlying data type over which the interval is defined.
/// \param p polynomial of degree 0
/// \return p^2 as a polynomial of degree 0
template <typename T>
Polynomial<0, T> square(const Polynomial<0, T>& p);

/// Square a degree 1 polynomial
/// \tparam T underlying data type over which the interval is defined.
/// \param p polynomial of degree 1
/// \return p^2 as a polynomial of degree 2
template <typename T>
Polynomial<2, T> square(const Polynomial<1, T>& p);

// ======================================================================

/// Write a textual version of a Polynomial to an output stream
/// \tparam N degree of the polynomial
/// \tparam T underlying type of the polynomial coefficients
/// \param stream the ostream to be written to
/// \param p the polynomial to write
/// \return the active ostream
template <int N, typename T>
std::ostream& operator<<(std::ostream& stream, const Polynomial<N, T>& p);

}; // Math
}; // SurgSim

#include "SurgSim/Math/Polynomial-inl.h"

#endif // SURGSIM_MATH_POLYNOMIAL_H
