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

#include <iostream>

namespace SurgSim
{
namespace Math
{

namespace
{
double polynomial_epsilon = 1.0e-09;
}

/// Define an utility function for comparing individual coefficients to 0.
///
/// \tparam T underlying data type used as either the scalar or as the data type
/// for the interval
/// \param value the value to compare
/// \param epsilon the tolerance
/// \return true if the value is within epsilon of 0
template <typename T>
bool isNearZero(const T& value, const T& epsilon = static_cast<T>(polynomial_epsilon));

/// Polynomial<T, N> defines the concept of an N degree polynomial with type T
/// coefficients and provides operations on them including arithmetic operations,
/// construction, and IO.
///
/// \tparam T underlying data type over which the interval is defined.
/// \tparam N degree of the Polynomial
template <typename T, int N> class Polynomial
{
	static_assert(N >= 0, "Polynomials must have degree >= 0.");
	static_assert(N <= 3, "Polynomials of degree > 3 are not yet supported.");
};

/// Polynomial<T, 0> specializes the Polynomial class for degree 0 (constant polynomials)
/// \sa Polynomial<T, N>
template <typename T>
class Polynomial<T, 0>
{
public:
	/// Constructor
	Polynomial();

	/// Constructor
	/// \param a0 coefficient of the 0 degree term
	explicit Polynomial(const T& a0);

	/// Evaluate the polynomial at a point
	/// \param x point at which to evaluate the polynomial
	/// \return the value of the polynomial at x
	T evaluate(const T& x) const;

	/// @{
	/// Standard arithmetic operators extended to intervals
	T& operator[](const size_t i);
	const T& operator[](const size_t i) const;
	Polynomial<T, 0> operator- () const;
	Polynomial<T, 0> operator+ (const Polynomial<T, 0>& rhs) const;
	Polynomial<T, 0>& operator+= (const Polynomial<T, 0>& rhs);
	Polynomial<T, 0> operator- (const Polynomial<T, 0>& rhs) const;
	Polynomial<T, 0>& operator-= (const Polynomial<T, 0>& rhs);
	/// @}

	/// \param epsilon the closeness parameter
	/// \return true if all coefficients of the polynomial are within an epsilon of 0
	bool isNearZero(const T& epsilon = static_cast<T>(polynomial_epsilon)) const;

	/// \param p the test polynomial
	/// \param epsilon the closeness parameter
	/// \return true if all the coefficients of the current polynomial are within an epsilon of
	/// the corresponding coefficient in p
	bool isApprox(const Polynomial<T, 0>& p, const T& epsilon) const;

	/// \param i index of the desired coefficient
	/// \return the value of coefficient i
	T getCoefficient(size_t i) const;

	/// Set a specified coefficient to a desired value
	/// \param i index of the desired coefficient
	/// \param value the value to be set in the coefficient
	/// \exception if i is greater than the polynomial degree
	void setCoefficient(size_t i, const T& value);

private:
	/// @{
	/// Coefficient of the polynomial
	T m_a0;
	/// @}
};

/// Polynomial<T, 1> specializes the Polynomial class for degree 1 (linear polynomials)
/// \sa Polynomial<T, N>
template <typename T>
class Polynomial<T, 1>
{
public:
	/// Constructor
	Polynomial();

	/// Constructor
	/// \param a0 coefficient of the 0 degree term
	/// \param a1 coefficient of the 1 degree term
	Polynomial(const T& a0, const T& a1);

	/// Evaluate the polynomial at a point
	/// \param x point at which to evaluate the polynomial
	/// \return the value of the polynomial at x
	T evaluate(const T& x) const;

	/// @{
	/// Standard arithmetic operators extended to intervals
	T& operator[](const size_t i);
	const T& operator[](const size_t i) const;
	Polynomial<T, 1> operator- () const;
	Polynomial<T, 1> operator+ (const Polynomial<T, 1>& rhs) const;
	Polynomial<T, 1>& operator+= (const Polynomial<T, 1>& rhs);
	Polynomial<T, 1> operator- (const Polynomial<T, 1>& rhs) const;
	Polynomial<T, 1>& operator-= (const Polynomial<T, 1>& rhs);
	/// @}

	/// \return the derivative of the polynomial
	Polynomial<T, 0> derivative() const;

	/// \param epsilon the closeness parameter
	/// \return true if all coefficients of the polynomial are within an epsilon of 0
	bool isNearZero(const T& epsilon = static_cast<T>(polynomial_epsilon)) const;

	/// \param p the test polynomial
	/// \param epsilon the closeness parameter
	/// \return true if all the coefficients of the current polynomial are within an epsilon of
	/// the corresponding coefficient in p
	bool isApprox(const Polynomial<T, 1>& p, const T& epsilon) const;

	/// \param i index of the desired coefficient
	/// \return the value of coefficient i
	T getCoefficient(size_t i) const;

	/// Set a specified coefficient to a desired value
	/// \param i index of the desired coefficient
	/// \param value the value to be set in the coefficient
	/// \exception if i is greater than the polynomial degree
	void setCoefficient(size_t i, const T& value);

private:
	/// @{
	/// Coefficient of the polynomial
	T m_a0;
	T m_a1;
	/// @}
};

/// Polynomial<T, 2> specializes the Polynomial class for degree 2 (quadratic polynomials)
/// \sa Polynomial<T, N>
template <typename T>
class Polynomial<T, 2>
{
public:
	/// Constructor
	Polynomial();

	/// Constructor
	/// \param a0 coefficient of the 0 degree term
	/// \param a1 coefficient of the 1 degree term
	/// \param a2 coefficient of the 2 degree term
	Polynomial(const T& a0, const T& a1, const T& a2);

	/// Evaluate the discriminant of a quadratic polynomial
	/// \return the discriminant (b^2 - a4c)
	T discriminant() const;

	/// Evaluate the polynomial at a point
	/// \param x point at which to evaluate the polynomial
	/// \return the value of the polynomial at x
	T evaluate(const T& x) const;

	/// @{
	/// Standard arithmetic operators extended to intervals
	T& operator[](const size_t i);
	const T& operator[](const size_t i) const;
	Polynomial<T, 2> operator- () const;
	Polynomial<T, 2> operator+ (const Polynomial<T, 2>& rhs) const;
	Polynomial<T, 2>& operator+= (const Polynomial<T, 2>& rhs);
	Polynomial<T, 2> operator- (const Polynomial<T, 2>& rhs) const;
	Polynomial<T, 2>& operator-= (const Polynomial<T, 2>& rhs);
	/// @}

	/// \return the derivative of the polynomial
	Polynomial<T, 1> derivative() const;

	/// \param epsilon the closeness parameter
	/// \return true if all coefficients of the polynomial are within an epsilon of 0
	bool isNearZero(const T& epsilon = static_cast<T>(polynomial_epsilon)) const;

	/// \param p the test polynomial
	/// \param epsilon the closeness parameter
	/// \return true if all the coefficients of the current polynomial are within an epsilon of
	/// the corresponding coefficient in p
	bool isApprox(const Polynomial<T, 2>& p, const T& epsilon) const;

	/// \param i index of the desired coefficient
	/// \return the value of coefficient i
	T getCoefficient(size_t i) const;

	/// Set a specified coefficient to a desired value
	/// \param i index of the desired coefficient
	/// \param value the value to be set in the coefficient
	/// \exception if i is greater than the polynomial degree
	void setCoefficient(size_t i, const T& value);

private:
	/// @{
	/// Coefficient of the polynomial
	T m_a0;
	T m_a1;
	T m_a2;
	/// @}
};

/// Polynomial<T, 3> specializes the Polynomial class for degree 3 (cubic polynomials)
/// \sa Polynomial<T, N>
template <typename T>
class Polynomial<T, 3>
{
public:
	/// Constructor
	Polynomial();

	/// Constructor
	/// \param a0 coefficient of the 0 degree term
	/// \param a1 coefficient of the 1 degree term
	/// \param a2 coefficient of the 2 degree term
	/// \param a3 coefficient of the 3 degree term
	Polynomial(const T& a0, const T& a1, const T& a2, const T& a3);

	/// Evaluate the polynomial at a point
	/// \param x point at which to evaluate the polynomial
	/// \return the value of the polynomial at x
	T evaluate(const T& x) const;

	/// @{
	/// Standard arithmetic operators extended to intervals
	T& operator[](const size_t i);
	const T& operator[](const size_t i) const;
	Polynomial<T, 3> operator- () const;
	Polynomial<T, 3> operator+ (const Polynomial<T, 3>& rhs) const;
	Polynomial<T, 3>& operator+= (const Polynomial<T, 3>& rhs);
	Polynomial<T, 3> operator- (const Polynomial<T, 3>& rhs) const;
	Polynomial<T, 3>& operator-= (const Polynomial<T, 3>& rhs);
	/// @}

	/// \return the derivative of the polynomial
	Polynomial<T, 2> derivative() const;

	/// \param epsilon the closeness parameter
	/// \return true if all coefficients of the polynomial are within an epsilon of 0
	bool isNearZero(const T& epsilon = static_cast<T>(polynomial_epsilon)) const;

	/// \param p the test polynomial
	/// \param epsilon the closeness parameter
	/// \return true if all the coefficients of the current polynomial are within an epsilon of
	/// the corresponding coefficient in p
	bool isApprox(const Polynomial<T, 3>& p, const T& epsilon) const;

	/// \param i index of the desired coefficient
	/// \return the value of coefficient i
	T getCoefficient(size_t i) const;

	/// Set a specified coefficient to a desired value
	/// \param i index of the desired coefficient
	/// \param value the value to be set in the coefficient
	/// \exception if i is greater than the polynomial degree
	void setCoefficient(size_t i, const T& value);

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
template <typename T, int N, int M>
Polynomial < T, N + M > operator*(const Polynomial<T, N>& p, const Polynomial<T, M>& q);

/// Multiply two polynomials of degree 1.
/// \tparam T underlying data type over which the interval is defined.
/// \param p first polynomial of degree 1
/// \param q second polynomial of degree 1
/// \return p * q as a polynomial of degree 2
template <typename T>
Polynomial<T, 2> operator*(const Polynomial<T, 1>& p, const Polynomial<T, 1>& q);

/// Multiply two polynomials of degree 2 and 1 respectively.
/// \tparam T underlying data type over which the interval is defined.
/// \param p first polynomial of degree 2
/// \param q second polynomial of degree 1
/// \return p * q as a polynomial of degree 3
template <typename T>
Polynomial<T, 3> operator*(const Polynomial<T, 2>& p, const Polynomial<T, 1>& q);

/// Multiply two polynomials of degree 1 and 2 respectively.
/// \tparam T underlying data type over which the interval is defined.
/// \param p first polynomial of degree 1
/// \param q second polynomial of degree 2
/// \return p * q as a polynomial of degree 3
template <typename T>
Polynomial<T, 3> operator*(const Polynomial<T, 1>& p, const Polynomial<T, 2>& q);

/// Square a degree 0 polynomial
/// \tparam T underlying data type over which the interval is defined.
/// \param p polynomial of degree 0
/// \return p^2 as a polynomial of degree 0
template <typename T>
Polynomial<T, 0> square(const Polynomial<T, 0>& p);

/// Square a degree 1 polynomial
/// \tparam T underlying data type over which the interval is defined.
/// \param p polynomial of degree 1
/// \return p^2 as a polynomial of degree 2
template <typename T>
Polynomial<T, 2> square(const Polynomial<T, 1>& p);

/// Write a textual version of a Polynomial to an output stream
/// \tparam T underlying type of the polynomial coefficients
/// \tparam N degree of the polynomial
/// \param stream the ostream to be written to
/// \param p the polynomial to write
/// \return the active stream
template <typename T, int N>
std::ostream& operator<<(std::ostream& stream, const Polynomial<T, N>& p);

}; // Math
}; // SurgSim

#include "SurgSim/Math/Polynomial-inl.h"

#endif // SURGSIM_MATH_POLYNOMIAL_H
