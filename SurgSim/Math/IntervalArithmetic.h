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

#ifndef SURGSIM_MATH_INTERVALARITHMETIC_H
#define SURGSIM_MATH_INTERVALARITHMETIC_H

#include <array>
#include <ostream>

namespace SurgSim
{
namespace Math
{

/// Interval defines the concept of a mathematical interval and provides operations on it
/// including arithmetic operations, construction, and IO.
///
/// \tparam T underlying data type over which the interval is defined.
///
/// \sa Interval_nD<T, N> and Interval_nD<T, 3>
template <class T>
class Interval
{
	template <class P>
	friend void IntervalArithmetic_add(const Interval<P>& a, const Interval<P>& b, Interval<P>* res); // +
	template <class P>
	friend void IntervalArithmetic_addadd(const Interval<P>& a, const Interval<P>& b, Interval<P>* res); // +=( + )
	template <class P>
	friend void IntervalArithmetic_sub(const Interval<P>& a, const Interval<P>& b, Interval<P>* res); // -
	template <class P>
	friend void IntervalArithmetic_addsub(const Interval<P>& a, const Interval<P>& b, Interval<P>* res); // +=( - )
	template <class P>
	friend void IntervalArithmetic_mul(const Interval<P>& a, const Interval<P>& b, Interval<P>* res); // *
	template <class P>
	friend void IntervalArithmetic_addmul(const Interval<P>& a, const Interval<P>& b, Interval<P>* res); // += ( * )
	template <class P>
	friend void IntervalArithmetic_submul(const Interval<P>& a, const Interval<P>& b, Interval<P>* res); // -= ( * )

public:
	/// Constructor
	Interval();

	/// Constructor
	/// \param min Lower bound of the constructed interval
	/// \param max Upper bound of the constructed interval
	/// \exception if max is less than min
	Interval<T>(T min, T max);

	/// Copy Constructor
	/// \param i Interval to be copied
	Interval<T>(const Interval<T>& i);

	/// Move constructor
	/// \param i Interval to be copied
	Interval<T>(Interval<T>&& i);

	/// Assignment operator
	/// \param i Interval to be copied
	Interval<T>& operator =(const Interval<T>& i);

	/// Move assignment operator
	/// \param i Interval to be moved
	Interval<T>& operator=(Interval<T>&& i);

	/// Generate an interval from min to max based on the inputs
	/// \param a1 first input value
	/// \param a2 second input value
	/// \return an interval spanning the minimum input to the maximum input.
	static Interval<T> minToMax(const T a1, const T a2);

	/// Generate an interval from min to max based on the inputs
	/// \param a1 first input value
	/// \param a2 second input value
	/// \param a3 third input value
	/// \return an interval spanning the minimum input to the maximum input.
	static Interval<T> minToMax(const T a1, const T a2, const T a3);

	/// Generate an interval from min to max based on the inputs
	/// \param a1 first input value
	/// \param a2 second input value
	/// \param a3 third input value
	/// \param a4 fourth input value
	/// \return an interval spanning the minimum input to the maximum input.
	static Interval<T> minToMax(const T a1, const T a2, const T a3, const T a4);

	/// \param i the interval the current interval will be tested against
	/// \return true if the input interval overlaps the current interval
	bool overlapsWith(const Interval<T>& i) const;

	/// \param val the value to test for inclusion in the interval
	/// \return true if the current interval contains val
	bool contains(T val) const;

	/// \return true if the current interval contains 0
	bool containsZero() const;

	/// \param i the interval to be tested
	/// \param epsilon the nearness parameter
	/// \return true if the current interval is within epsilon of the input interval
	bool isApprox(const Interval<T>& i, const T& epsilon) const;

	/// \param i the interval to be tested
	/// \return true if the current interval is identical to the input interval
	bool operator ==(const Interval<T>& i) const;

	/// \param i the interval to be tested
	/// \return true if the current interval is not identical to the input interval
	bool operator !=(const Interval<T>& i) const;

	/// Widens the current interval by thickness on both sides
	/// \param thickness the amount to widen the current interval on both sides
	/// \return the current interval after modification
	Interval<T>& addThickness(const T thickness);

	/// Widens the current interval on one end to include x
	/// \param x the value to be included in the interval
	/// \return the current interval extended to include x
	Interval<T>& extendToInclude(T x);

	/// Widens the current interval on both ends to include i
	/// \param i the interval to be wholly contained in the current interval
	/// \return the current interval extended to include the entirety of i
	Interval<T>& extendToInclude(const Interval<T>& i);

	/// @{
	/// Standard arithmetic operators extended to intervals
	Interval<T> operator +(const Interval<T>& i) const;
	Interval<T> operator +(T v) const;
	Interval<T>& operator +=(const Interval<T>& i);
	Interval<T>& operator +=(T v);
	Interval<T> operator -() const;
	Interval<T> operator -(const Interval<T>& i) const;
	Interval<T> operator -(T v) const;
	Interval<T>& operator -=(const Interval<T>& i);
	Interval<T>& operator -=(T v);
	Interval<T> operator *(const Interval<T>& i) const;
	Interval<T> operator *(T v) const;
	Interval<T>& operator *=(const Interval<T>& i);
	Interval<T>& operator *=(T v);
	/// @}

	/// \return the inverse of the current interval
	/// \exception if the interval includes 0
	Interval<T> inverse() const;

	/// \param i the interval to be divided by
	/// \return the current interval multiplied by the inverse of i
	/// \exception if i includes 0
	Interval<T> operator /(const Interval<T>& i) const;

	/// \param i the interval to be divided by
	/// \return the current interval multiplied by the inverse of i
	/// \exception if i includes 0
	/// \note the current interval is modified by this operation
	Interval<T>& operator /=(const Interval<T>& i);

	/// \return the square of the current interval
	/// \note if the original interval contains 0, then the result will have the minimum identically set to 0
	Interval<T> square() const;

	/// \return the lower limit of the interval
	T getMin() const;

	/// \return the upper limit of the interval
	T getMax() const;

	/// \return the interval from the lower limit to the midpoint
	Interval<T> lowerHalf() const;

	/// \return the interval from the midpoint to the upper limit
	Interval<T> upperHalf() const;

private:
	/// The lower (m_min) and upper (m_max) limits of the interval
	T m_min, m_max;
};


/// Interval_nD defines the concept of a group of mathematical intervals and provides operations on them
/// including arithmetic operations, construction, and IO.
///
/// \tparam T underlying data type over which the interval is defined.
/// \tparam N number of intervals in the group.
///
/// \sa Interval<T> and Interval_nD<T, 3>
template <class T, int N>
class Interval_nD
{
public:
	static_assert(N >= 1, "Interval_nD<T, N> cannot be instantiated with N<=0.");

	/// Constructor
	Interval_nD();

	/// Constructor
	/// \param x array of N intervals to be copied into the group
	explicit Interval_nD<T, N>(const std::array<Interval<T>, N>& x);

	/// Copy Constructor
	/// \param interval interval group to copied
	Interval_nD<T, N>(const Interval_nD<T, N>& interval);

	/// Move constructor
	/// \param i Interval to be copied
	Interval_nD<T, N>(Interval_nD<T, N>&& i);

	/// Constructor
	/// \param a array of N values to be used as the respective minimums for the interval entries.
	/// \param b array of N values to be used as the respective maximums for the interval entries.
	Interval_nD<T, N>(const std::array<T, N>& a, const std::array<T, N>& b);

	/// Assignment operator
	/// \param interval Interval group to be copied
	Interval_nD<T, N>& operator =(const Interval_nD<T, N>& interval);

	/// Move assignment operator
	/// \param i Interval to be moved
	Interval_nD<T, N>& operator=(Interval_nD<T, N>&& i);

	/// \param interval the interval group the current group will be tested against
	/// \return true if the input group interval overlaps the current group
	bool overlapsWith(const Interval_nD<T, N>& interval) const;

	/// \param interval the interval group to be tested
	/// \param epsilon the nearness parameter
	/// \return true if each interval in the input group is approximately equal to its correspondent
	/// element in interval.
	bool isApprox(const Interval_nD<T, N>& interval, const T& epsilon) const;

	/// \param interval the interval group to be tested
	/// \return true if the current interval group is identical to the input group
	bool operator ==(const Interval_nD<T, N>& interval) const;

	/// \param interval the interval group to be tested
	/// \return true if the current interval group is not identical to the input group
	bool operator !=(const Interval_nD<T, N>& interval) const;

	/// Widens every interval in the current interval group by thickness on both sides
	/// \param thickness the amount to widen on both sides
	/// \return the current interval group after modification
	Interval_nD<T, N>& addThickness(const double thickness);

	/// @{
	/// Standard arithmetic operators extended to interval groups
	Interval_nD<T, N> operator +(const Interval_nD<T, N>& interval) const;
	Interval_nD<T, N>& operator +=(const Interval_nD<T, N>& interval);
	Interval_nD<T, N> operator -(const Interval_nD<T, N>& interval) const;
	Interval_nD<T, N>& operator -=(const Interval_nD<T, N>& interval);
	Interval_nD<T, N> operator *(const Interval_nD<T, N>& interval) const;
	Interval_nD<T, N>& operator *=(const Interval_nD<T, N>& interval);
	/// @}

	/// \return the inverse of each interval in the interval group
	/// \exception if any interval includes 0
	Interval_nD<T, N> inverse(void) const;

	/// \param interval the interval to be divided by
	/// \return the product of each interval in the group multiplied by the inverse of
	/// its correspondent in interval
	/// \exception if any component of interval includes 0
	Interval_nD<T, N> operator /(const Interval_nD<T, N>& interval) const;

	/// \param interval the interval to be divided by
	/// \return the product of each interval in the group multiplied by the inverse of
	/// its correspondent in interval
	/// \note the current interval is modified by this operation
	Interval_nD<T, N>& operator /=(const Interval_nD<T, N>& interval);

	/// \param interval the input interval group
	/// \return the interval dot product of the current group and interval
	Interval<T> dotProduct(const Interval_nD<T, N>& interval) const;

	/// \return the square of the interval magnitude for the current group
	Interval<T> magnitudeSquared() const;

	/// \return the interval magnitude for the current group
	Interval<T> magnitude() const;

	/// \param i the selector for the interval to be returned
	/// \return the ith interval in the current group
	const Interval<T>& getAxis(size_t i) const;

private:
	/// The N dimensional group of intervals
	std::array<Interval<T>, N> m_interval;
};

/// Interval_nD<T,3> defines the concept of a group of mathematical intervals specialized to 3 intervals and provides
/// operations on them including arithmetic operations, construction, and IO.
///
/// \tparam T underlying data type over which the interval is defined.
///
/// \sa Interval<T> and Interval_nD<T, N>
template <class T>
class Interval_nD<T, 3>
{
	template <class P>
	friend void IntervalArithmetic_add(const Interval_nD<P, 3>& a, const Interval_nD<P, 3>& b, Interval_nD<P, 3>* res);
	template <class P>
	friend void IntervalArithmetic_sub(const Interval_nD<P, 3>& a, const Interval_nD<P, 3>& b, Interval_nD<P, 3>* res);
	template <class P>
	friend void IntervalArithmetic_crossProduct(const Interval_nD<P, 3>& a, const Interval_nD<P, 3>& b,
			Interval_nD<P, 3>* res);
	template <class P>
	friend void IntervalArithmetic_dotProduct(const Interval_nD<P, 3>& a, const Interval_nD<P, 3>& b,
			Interval<P>* res);

public:
	/// Constructor
	Interval_nD();

	/// Constructor
	/// \param x array of 3 intervals to be copied into the group
	explicit Interval_nD<T, 3>(const std::array<Interval<T>, 3>& x);

	/// Constructor
	/// \param x first interval to be added to the 3 group
	/// \param y second interval to be added to the 3 group
	/// \param z third interval to be added to the 3 group
	Interval_nD<T, 3>(Interval<T> x, Interval<T> y, Interval<T> z);

	/// Copy Constructor
	/// \param i interval 3 group to copied
	Interval_nD<T, 3>(const Interval_nD<T, 3>& i);

	/// Move constructor
	/// \param i Interval to be copied
	Interval_nD<T, 3>(Interval_nD<T, 3>&& i);

	/// Constructor
	/// \param a array of 3 values to be used as the respective minimums for the interval entries.
	/// \param b array of 3 values to be used as the respective maximums for the interval entries.
	Interval_nD<T, 3>(const std::array<T, 3>& a, const std::array<T, 3>& b);

	/// Assignment operator
	/// \param i Interval 3 group to be copied
	Interval_nD<T, 3>& operator =(const Interval_nD<T, 3>& i);

	/// Move assignment operator
	/// \param i Interval to be moved
	Interval_nD<T, 3>& operator=(Interval_nD<T, 3>&& i);

	/// \param interval the interval group the current group will be tested against
	/// \return true if the input 3 group interval overlaps the current 3 group
	bool overlapsWith(const Interval_nD<T, 3>& interval) const;

	/// \param interval the interval group to be tested
	/// \param epsilon the nearness parameter
	/// \return true if each interval in the input group is approximately equal to its correspondent
	/// element in interval.
	bool isApprox(const Interval_nD<T, 3>& interval, const T& epsilon) const;

	/// \param i the interval group to be tested
	/// \return true if the current interval 3 group is identical to the input 3 group i
	bool operator ==(const Interval_nD<T, 3>& i) const;

	/// \param i the interval group to be tested
	/// \return true if the current interval 3 group is not identical to the input 3 group i
	bool operator !=(const Interval_nD<T, 3>& i) const;

	/// Widens every interval in the current interval group by thickness on both sides
	/// \param thickness the amount to widen on both sides
	/// \return the current interval group after modification
	Interval_nD<T, 3>& addThickness(const double thickness);

	/// @{
	/// Standard arithmetic operators extended to 3 interval groups
	Interval_nD<T, 3> operator +(const Interval_nD<T, 3>& i) const;
	Interval_nD<T, 3>& operator +=(const Interval_nD<T, 3>& i);
	Interval_nD<T, 3> operator -(const Interval_nD<T, 3>& i) const;
	Interval_nD<T, 3>& operator -=(const Interval_nD<T, 3>& i);
	Interval_nD<T, 3> operator *(const Interval_nD<T, 3>& i) const;
	Interval_nD<T, 3>& operator *=(const Interval_nD<T, 3>& i);
	/// @}

	/// \return the inverse of each interval in the 3 interval group
	/// \exception if any interval includes 0
	Interval_nD<T, 3> inverse(void) const;

	/// \param i the interval to be divided by
	/// \return the product of each interval in the 3 group multiplied by the inverse of
	/// its correspondent in i
	/// \exception if any component of interval includes 0
	Interval_nD<T, 3> operator /(const Interval_nD<T, 3>& i) const;

	/// \param i the interval to be divided by
	/// \return the product of each interval in the 3 group multiplied by the inverse of
	/// its correspondent in i
	/// \note the current interval is modified by this operation
	Interval_nD<T, 3>& operator /=(const Interval_nD<T, 3>& i);

	/// \param i the input interval group
	/// \return the interval dot product of the current 3 group and interval
	Interval<T> dotProduct(const Interval_nD<T, 3>& i) const;

	/// \param i the input interval group
	/// \return the interval cross product of the current 3 group and interval
	Interval_nD<T, 3> crossProduct(const Interval_nD<T, 3>& i) const;

	/// \return the square of the interval magnitude for the current 3 group
	Interval<T> magnitudeSquared() const;

	/// \return the interval magnitude for the current 3 group
	Interval<T> magnitude() const;

	/// \param i the selector for the interval to be returned
	/// \return the ith interval in the current 3 group
	const Interval<T>& getAxis(size_t i) const;

private:
	/// The 3 dimensional group of intervals
	std::array<Interval<T>, 3> m_interval;
};

// Interval utilities

/// \tparam T underlying type of the interval
/// \param v the scalar
/// \param i the interval
/// \return the sum of the scalar v and the interval i
template <typename T>
Interval<T> operator+ (T v, const Interval<T>& i);

/// \tparam T underlying type of the interval
/// \param v the scalar
/// \param i the interval
/// \return the product of the scalar v and the interval i
template <typename T>
Interval<T> operator* (T v, const Interval<T>& i);

/// Write a textual version of the interval to an output stream
/// \tparam T underlying type of the interval
/// \param o the ostream to be written to
/// \param interval the interval to write
/// \return the active ostream
template <typename T>
std::ostream& operator<< (std::ostream& o, const Interval<T>& interval);

/// Calculate the sum of two intervals
/// \tparam P underlying type of the interval
/// \param a the first interval
/// \param b the second interval
/// \param res [out] the result of a + b
template <class P>
void IntervalArithmetic_add(const Interval<P>& a, const Interval<P>& b,
							Interval<P>* res); // +

/// Calculate the sum of three intervals res + a + b
/// \tparam P underlying type of the interval
/// \param a the first interval
/// \param b the second interval
/// \param res [in/out] the result of res + a + b
template <class P>
void IntervalArithmetic_addadd(const Interval<P>& a, const Interval<P>& b,
							   Interval<P>* res); // +=( + )

/// Calculate the difference of two intervals
/// \tparam P underlying type of the interval
/// \param a the first interval
/// \param b the second interval
/// \param res [out] the result of a - b
template <class P>
void IntervalArithmetic_sub(const Interval<P>& a, const Interval<P>& b,
							Interval<P>* res); // -

/// Add the difference of two intervals to an existing value
/// \tparam P underlying type of the interval
/// \param a the first interval
/// \param b the second interval
/// \param res [in/out] the result of res + (a - b)
template <class P>
void IntervalArithmetic_addsub(const Interval<P>& a, const Interval<P>& b,
							   Interval<P>* res); // +=( - )

/// Calculate the product of two intervals
/// \tparam P underlying type of the interval
/// \param a the first interval
/// \param b the second interval
/// \param res [out] the result of a * b
template <class P>
void IntervalArithmetic_mul(const Interval<P>& a, const Interval<P>& b,
							Interval<P>* res); // *

/// Add the product of two intervals to an existing value
/// \tparam P underlying type of the interval
/// \param a the first interval
/// \param b the second interval
/// \param res [in/out] the result of res + (a * b)
template <class P>
void IntervalArithmetic_addmul(const Interval<P>& a, const Interval<P>& b,
							   Interval<P>* res); // += ( * )

/// Subtract the product of two intervals from an existing value
/// \tparam P underlying type of the interval
/// \param a the first interval
/// \param b the second interval
/// \param res [in/out] the result of res - (a * b)
template <class P>
void IntervalArithmetic_submul(const Interval<P>& a, const Interval<P>& b,
							   Interval<P>* res); // -= ( * )

// Interval nD utilities

/// Write a textual version of an interval group to an output stream
/// \tparam T underlying type of the interval
/// \tparam N number of intervals in the group
/// \param o the ostream to be written to
/// \param interval the interval group to write
/// \return the active ostream
template <typename T, int N>
std::ostream& operator<< (std::ostream& o, const Interval_nD<T, N>& interval);

// Interval 3D utilities

/// Calculate the sum of two interval groups
/// \tparam P underlying type of the interval
/// \param a the first interval group
/// \param b the second interval group
/// \param res [out] the result of a + b
template <class P>
void IntervalArithmetic_add(const Interval_nD<P, 3>& a, const Interval_nD<P, 3>& b,
							Interval_nD<P, 3>* res);

/// Calculate the difference of two interval groups
/// \tparam P underlying type of the interval
/// \param a the first interval group
/// \param b the second interval group
/// \param res [out] the result of a - b
template <class P>
void IntervalArithmetic_sub(const Interval_nD<P, 3>& a, const Interval_nD<P, 3>& b,
							Interval_nD<P, 3>* res);

/// Calculate the dot product of two interval groups
/// \tparam P underlying type of the interval
/// \param a the first interval group
/// \param b the second interval group
/// \param res [out] the dot product of a and b
template <class P>
void IntervalArithmetic_dotProduct(const Interval_nD<P, 3>& a, const Interval_nD<P, 3>& b,
								   Interval<P>* res);

/// Calculate the cross product of two interval groups
/// \tparam P underlying type of the interval
/// \param a the first interval group
/// \param b the second interval group
/// \param res [out] the cross product of a and b
template <class P>
void IntervalArithmetic_crossProduct(const Interval_nD<P, 3>& a, const Interval_nD<P, 3>& b,
									 Interval_nD<P, 3>* res);

}; // Math
}; // SurgSim

#include "SurgSim/Math/IntervalArithmetic-inl.h"

#endif // SURGSIM_MATH_INTERVALARITHMETIC_H