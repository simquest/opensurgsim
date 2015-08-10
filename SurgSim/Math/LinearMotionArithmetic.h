// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest Solutions Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SURGSIM_MATH_LINEARMOTIONARITHMETIC_H
#define SURGSIM_MATH_LINEARMOTIONARITHMETIC_H

#include <array>
#include <iostream>

#include <Eigen/Core>

#include "SurgSim/Math/IntervalArithmetic.h"
#include "SurgSim/Math/PolynomialValues.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{
namespace Math
{

/// LinearMotion is (intentionally) a lot like Interval, but it deals with linear motion where
/// all operands start and end their motion simultaneously.
///
/// LinearMotion results in much tighter bounds compared to Interval, since Interval must consider
/// *any* value of each operand, and LinearMotion only considers values that are synchronous with
/// one another.
///
/// The bounds of a LinearMotion are a start value and an end value; there's no requirement that
/// start <= end (or vice versa).
///
/// Many operations on LinearMotion arguments (*, /) return results that are not linear in time,
/// so those operations will return Interval instead.
///
/// \tparam T underlying data type over which the linear motion is defined.
///
/// \sa Interval<T>, and IntervalND<T, N>
template <typename T>
class LinearMotion
{
public:
	/// Constructor
	LinearMotion();

	/// Constructor
	/// \param start initial value of the constructed linear motion
	/// \param end final value of the constructed linear motion
	/// \exception if max is less than min
	LinearMotion(const T& start, const T& end);

	/// Copy constructor
	/// \param m Interval to be copied
	LinearMotion(const LinearMotion<T>& m);

	/// Move constructor
	/// \param m LinearMotion to be copied
	LinearMotion(LinearMotion<T>&& m);

	/// Assignment operator
	/// \param m Interval to be copied
	LinearMotion<T>& operator=(const LinearMotion<T>& m);

	/// Move assignment operator
	/// \param m Interval to be moved
	LinearMotion<T>& operator=(LinearMotion<T>&& m);

	/// Convert from LinearMotion to an Interval
	/// \return the conversion of the LinearMotion to an Interval
	Interval<T> toInterval() const;

	/// Returns a linear expression (degree-1 polynomial) whose value for t=0..1
	/// progresses from `start' to `end'.
	/// \return the conversion of the LinearMotion to a polynomial
	Polynomial<T, 1> toPolynomial() const;

	/// \return true if the linear motion crosses through 0.
	bool containsZero() const;

	/// \param i the linear motion to be tested
	/// \param epsilon the nearness parameter
	/// \return true if the current linear motion is within epsilon of the input linear motion
	bool isApprox(const LinearMotion<T>& i, const T& epsilon) const;

	/// \param m the linear motion to be tested
	/// \return true if the current linear motion is identical to the input linear motion
	bool operator==(const LinearMotion<T>& m) const;

	/// \param m the linear motion to be tested
	/// \return true if the current linear motion is not identical to the input linear motion
	bool operator!=(const LinearMotion<T>& m) const;

	/// @{
	/// Standard arithmetic operators extended to linear motions
	LinearMotion<T> operator+(const LinearMotion<T>& m) const;
	LinearMotion<T>& operator+=(const LinearMotion<T>& m);
	LinearMotion<T> operator-(const LinearMotion<T>& m) const;
	LinearMotion<T>& operator-=(const LinearMotion<T>& m);
	/// @}

	/// Standard arithmetic operators extended to interval groups
	/// \note Multiplication and division operators by their nature do not
	/// preserve time ordering and so the return value is an IntervalND instead
	/// of a LinearMotionND
	Interval<T> operator*(const LinearMotion<T>& m) const;

	/// Standard arithmetic operators extended to interval groups
	/// \note Multiplication and division operators by their nature do not
	/// preserve time ordering and so the return value is an IntervalND instead
	/// of a LinearMotionND
	/// \exception if any component of interval includes 0
	Interval<T> operator/(const LinearMotion<T>& m) const;

	/// \return the initial value of the linear motion
	T getStart() const;

	/// \return the end point of the linear motion
	T getEnd() const;

	/// \param t the parametric time at which to evaluate the linear motion
	/// \return the value of the linear motion at parametric time t
	T atTime(const T& t) const;

	/// \return the linear motion from the initial time to the midpoint
	LinearMotion<T> firstHalf() const;

	/// \return the linear motion from the midpoint to the endpoint
	LinearMotion<T> secondHalf() const;

private:
	/// The start point of the linear motion
	T m_start;

	/// The end point of the linear motion
	T m_end;
};

/// LinearMotionND<T, N> defines the concept of a group of linear motions and provides
/// operations on them including arithmetic operations, construction, and I/O.
///
/// \tparam T underlying data type over which the linear motion is defined.
/// \tparam N Dimensionality of the interval
///
/// \sa LinearMotionND<T> and IntervalArthmetic<T, N>
template <class T, int N>
class LinearMotionND
{
	static_assert(N > 0, "LinearMotion must have dimensionality > 0.");

public:
	/// Constructor
	LinearMotionND();

	/// Constructor
	/// \param x array of N motions to be copied into the group
	explicit LinearMotionND(const std::array<LinearMotion<T>, N>& x);

	/// Copy constructor
	/// \param motion motion group to copied
	LinearMotionND(const LinearMotionND<T, N>& motion);

	/// Move constructor
	/// \param motion motion to be copied
	LinearMotionND(LinearMotionND<T, N>&& motion);

	/// Constructor
	/// \param a array of N values to be used as the respective starts for the linear motion entries.
	/// \param b array of N values to be used as the respective ends for the linear motion entries.
	LinearMotionND(const std::array<T, N>& a, const std::array<T, N>& b);

	/// Assignment operator
	/// \param motion Linear motion group to be copied
	LinearMotionND<T, N>& operator=(const LinearMotionND<T, N>& motion);

	/// Move assignment operator
	/// \param motion Linear motion group to be moved
	LinearMotionND<T, N>& operator=(LinearMotionND<T, N>&& motion);

	/// Convert from LinearMotion to an Interval
	/// \return the conversion of the ND LinearMotion to an ND Interval
	IntervalND<T, N> toInterval() const;

	/// \param motion the linear motion group to be tested
	/// \param epsilon the nearness parameter
	/// \return true if each linear motion in the input group is approximately equal to its correspondent
	/// element in motion.
	bool isApprox(const LinearMotionND<T, N>& motion, const T& epsilon) const;

	/// \param motion the linear motion group to be tested
	/// \return true if the current linear motion group is identical to the input group
	bool operator==(const LinearMotionND<T, N>& motion) const;

	/// \param motion the linear motion group to be tested
	/// \return true if the current linear motion group is not identical to the input group
	bool operator!=(const LinearMotionND<T, N>& motion) const;

	/// @{
	/// Standard arithmetic operators extended to interval groups
	LinearMotionND<T, N> operator+(const LinearMotionND<T, N>& m) const;
	LinearMotionND<T, N>& operator+=(const LinearMotionND<T, N>& m);
	LinearMotionND<T, N> operator-(const LinearMotionND<T, N>& m) const;
	LinearMotionND<T, N>& operator-=(const LinearMotionND<T, N>& m);
	/// @}

	/// Standard arithmetic operators extended to interval groups
	/// \note Multiplication and division operators by their nature do not
	/// preserve time ordering and so the return value is an IntervalND instead
	/// of a LinearMotionND
	IntervalND<T, N> operator*(const LinearMotionND<T, N>& m) const;

	/// Standard arithmetic operators extended to interval groups
	/// \note Multiplication and division operators by their nature do not
	/// preserve time ordering and so the return value is an IntervalND instead
	/// of a LinearMotionND
	/// \exception if any component of interval includes 0
	IntervalND<T, N> operator/(const LinearMotionND<T, N>& m) const;

	/// \param motion the input linear motion group
	/// \return the interval dot product of the current group and interval
	Interval<T> dotProduct(const LinearMotionND<T, N>& motion) const;

	/// \param i the selector for the linear motion to be returned
	/// \return the ith interval in the current group
	/// \exception thrown if the requested axis is < 0 or greater than N - 1
	const LinearMotion<T>& getAxis(int i) const;

	/// \param start [out] the starting points of the linear motion group as an N dimension array.
	void getStart(std::array<T, N>* start) const;

	/// \param end [out] the ending points of the linear motion group as an N dimension array.
	void getEnd(std::array<T, N>* end) const;

	/// \return the linear motion from the starting point to the midpoint
	LinearMotionND<T, N> firstHalf() const;

	/// \return the linear motion from the midpoint to the ending point
	LinearMotionND<T, N> secondHalf() const;

private:
	/// The N dimensional group of linear motions
	std::array<LinearMotion<T>, N> m_motion;
};

/// LinearMotionND<T, 3> specializes the LinearMotionND<T, N> class for 3 dimensions
///
/// \sa LinearMotion<T>, LinearMotionND<T, N> and IntervalArthmetic<T, 3>
template <class T>
class LinearMotionND<T, 3>
{
public:
	/// Typedef for a vector 3 return
	typedef Eigen::Matrix<T, 3, 1> Vector3;

	/// Constructor
	LinearMotionND();

	/// Constructor
	/// \param x array of 3 linear motions to be copied into the group
	explicit LinearMotionND(const std::array<LinearMotion<T>, 3>& x);

	/// Constructor
	/// \param a first linear motion to be added to the 3 group
	/// \param b second linear motion to be added to the 3 group
	/// \param c third linear motion to be added to the 3 group
	LinearMotionND(const LinearMotion<T>& a, const LinearMotion<T>& b, const LinearMotion<T>& c);

	/// Copy constructor
	/// \param motion linear motion 3 group to be copied
	LinearMotionND(const LinearMotionND<T, 3>& motion);

	/// Move constructor
	/// \param motion Linear motion to be copied
	LinearMotionND(LinearMotionND<T, 3>&& motion);

	/// Constructor
	/// \param a array of 3 values to be used as the respective starts for the linear motion entries.
	/// \param b array of 3 values to be used as the respective ends for the linear motion entries.
	LinearMotionND(const std::array<T, 3>& a, const std::array<T, 3>& b);

	/// Assignment operator
	/// \param motion Linear motion 3 group to be copied
	LinearMotionND<T, 3>& operator=(const LinearMotionND<T, 3>& motion);

	/// Move assignment operator
	/// \param motion Linear motion 3 group to be moved
	LinearMotionND<T, 3>& operator=(LinearMotionND<T, 3>&& motion);

	/// Convert from LinearMotion to an Interval
	/// \return the conversion of the 3D LinearMotion to a 3D Interval
	IntervalND<T, 3> toInterval() const;

	/// \param motion the motion group to be tested
	/// \param epsilon the nearness parameter
	/// \return true if each linear motion in the input group is approximately equal to its correspondent
	/// element in motion.
	bool isApprox(const LinearMotionND<T, 3>& motion, const T& epsilon) const;

	/// \param motion the linear motion group to be tested
	/// \return true if the current linear motion 3 group is identical to the input 3 group motion
	bool operator==(const LinearMotionND<T, 3>& motion) const;

	/// \param motion the linear motion group to be tested
	/// \return true if the current linear motion 3 group is not identical to the input 3 group motion.
	bool operator!=(const LinearMotionND<T, 3>& motion) const;

	/// @{
	/// Standard arithmetic operators extended to 3 interval groups
	LinearMotionND<T, 3> operator+(const LinearMotionND<T, 3>& m) const;
	LinearMotionND<T, 3>& operator+=(const LinearMotionND<T, 3>& m);
	LinearMotionND<T, 3> operator-(const LinearMotionND<T, 3>& m) const;
	LinearMotionND<T, 3>& operator-=(const LinearMotionND<T, 3>& m);
	/// @}

	/// Standard arithmetic operators extended to interval groups
	/// \note Multiplication and division operators by their nature do not
	/// preserve time ordering and so the return value is an IntervalND instead
	/// of a LinearMotionND
	IntervalND<T, 3> operator*(const LinearMotionND<T, 3>& m) const;

	/// Standard arithmetic operators extended to interval groups
	/// \note Multiplication and division operators by their nature do not
	/// preserve time ordering and so the return value is an IntervalND instead
	/// of a LinearMotionND
	/// \exception if any component of interval includes 0
	IntervalND<T, 3> operator/(const LinearMotionND<T, 3>& m) const;

	/// \param motion the input linear motion 3 group
	/// \param range the range over which the dot product is to be evaluated.
	/// \return the interval dot product of the current 3 group and interval evaluated over the interval range.
	Interval<T> dotProduct(const LinearMotionND<T, 3>& motion, const Interval<T>& range) const;

	/// \param motion the input linear motion 3 group
	/// \param range the range over which the cross product is to be evaluated.
	/// \return the interval cross product of the current 3 group and interval evaluated over the interval range.
	IntervalND<T, 3> crossProduct(const LinearMotionND<T, 3>& motion, const Interval<T>& range) const;

	/// \return the square of the linear motion magnitude for the current 3 group
	Interval<T> magnitudeSquared(const Interval<T>& range) const;

	/// \return the linear motion magnitude for the current 3 group
	Interval<T> magnitude(const Interval<T>& range) const;

	/// \param i the selector for the linear motion to be returned
	/// \return the ith linear motion in the current 3 group
	/// \exception thrown if the requested axis is < 0 or greater than 2
	const LinearMotion<T>& getAxis(int i) const;

	/// \param start [out] the start of the linear motion 3D as a 3 value array
	void getStart(std::array<T, 3>* start) const;

	/// \param end [out] the end of the linear motion 3D as a 3 value array
	void getEnd(std::array<T, 3>* end) const;

	/// \return the start of the linear motion 3D as a 3 Vector
	Vector3 getStart() const;

	/// \return the end of the linear motion 3D as a 3 Vector
	Vector3 getEnd() const;

	/// \param t the parametric value at which to evaluate the linear motion
	/// \return the value of the linear motion 3D at time t as a 3 Vector
	Vector3 atTime(const T& t) const;

	/// \return the linear motion 3D from the start to the midpoint
	LinearMotionND<T, 3> firstHalf() const;

	/// \return the linear motion 3D from the midpoint to the start
	LinearMotionND<T, 3> secondHalf() const;

private:
	/// The 3 dimensional group of linear motions
	std::array<LinearMotion<T>, 3> m_motion;
};

// Linear motion utilities

/// Write a textual version of a linear motion to an output stream
/// \tparam T underlying type of the linear motion
/// \param o the ostream to be written to
/// \param motion the motion to write
/// \return the active ostream
template <typename T>
std::ostream& operator<<(std::ostream& o, const LinearMotion<T>& motion);

// Linear motion ND utilities

/// Write a textual version of a linear motion group to an output stream
/// \tparam T underlying type of the linear motion
/// \tparam N number of linear motions in the group
/// \param o the ostream to be written to
/// \param motion the motion group to write
/// \return the active ostream
template <typename T, int N>
std::ostream& operator<<(std::ostream& o, const LinearMotionND<T, N>& motion);

// Linear motion 3D utilities

/// Calculate an analytic dot product as a Polynomial
/// \tparam T underlying type of the linear motion
/// \param a the first linear motion 3 group
/// \param b the second linear motion 3 group
/// \return the dot product in a polynomial representation
template <class T>
Polynomial<T, 2> analyticDotProduct(const LinearMotionND<T, 3>& a, const LinearMotionND<T, 3>& b);

/// Calculate a single axis of an analytic cross product as a Polynomial
/// \tparam T underlying type of the linear motion
/// \tparam A axis to generate: 0 = X, 1=Y, 2=Z
/// \param a the first linear motion 3 group
/// \param b the second linear motion 3 group
/// \return the selected axis in a polynomial representation
template <class T, int A>
Polynomial<T, 2> analyticCrossProductAxis(const LinearMotionND<T, 3>& a, const LinearMotionND<T, 3>& b);

/// Calculate the X axis of an analytic cross product as a Polynomial
/// \tparam T underlying type of the linear motion
/// \param a the first linear motion 3 group
/// \param b the second linear motion 3 group
/// \return the X axis in a polynomial representation
template <class T>
Polynomial<T, 2> analyticCrossProductXAxis(const LinearMotionND<T, 3>& a, const LinearMotionND<T, 3>& b);

/// Calculate the Y axis of an analytic cross product as a Polynomial
/// \tparam T underlying type of the linear motion
/// \param a the first linear motion 3 group
/// \param b the second linear motion 3 group
/// \return the Y axis in a polynomial representation
template <class T>
Polynomial<T, 2> analyticCrossProductYAxis(const LinearMotionND<T, 3>& a, const LinearMotionND<T, 3>& b);

/// Calculate the Z axis of an analytic cross product as a Polynomial
/// \tparam T underlying type of the linear motion
/// \param a the first linear motion 3 group
/// \param b the second linear motion 3 group
/// \return the Z axis in a polynomial representation
template <class T>
Polynomial<T, 2> analyticCrossProductZAxis(const LinearMotionND<T, 3>& a, const LinearMotionND<T, 3>& b);

/// Calculate an analytic cross product as a Polynomial
/// \tparam T underlying type of the linear motion
/// \param a the first linear motion 3 group
/// \param b the second linear motion 3 group
/// \param [out] resultXAxis the X axis in a polynomial representation
/// \param [out] resultYAxis the Y axis in a polynomial representation
/// \param [out] resultZAxis the Z axis in a polynomial representation
template <class T>
void analyticCrossProduct(const LinearMotionND<T, 3>& a, const LinearMotionND<T, 3>& b,
						  Polynomial<T, 2>& resultXAxis, Polynomial<T, 2>& resultYAxis, Polynomial<T, 2>& resultZAxis);

/// Calculate an analytic cross product as a Polynomial, as a polynomial whose value for t=0..1 is
/// the value of the triple product.
/// \tparam T underlying type of the linear motion
/// \param a the first linear motion 3 group
/// \param b the second linear motion 3 group
/// \param c the third linear motion 3 group
/// \return a 3rd order polynomial representation of the triple product
template <class T>
Polynomial<T, 3> analyticTripleProduct(const LinearMotionND<T, 3>& a, const LinearMotionND<T, 3>& b,
									   const LinearMotionND<T, 3>& c);

/// Calculate the triple product, as an interval.
/// \tparam T underlying type of the linear motion
/// \param a the first linear motion 3 group
/// \param b the second linear motion 3 group
/// \param c the third linear motion 3 group
/// \param range the range over which the triple product is to be evaluated
/// \return an interval representation of the triple product
template <class T>
Interval<T> tripleProduct(const LinearMotionND<T, 3>& a, const LinearMotionND<T, 3>& b,
						  const LinearMotionND<T, 3>& c, const Interval<T>& range);

/// Calculate the magnitude squared of a linear motion 3 group as a polynomial
/// \tparam T underlying type of the linear motion
/// \param motion the linear motion 3 group
/// \return the magnitude squared of the linear motion as a polynomial
template <class T>
Polynomial<T, 2> analyticMagnitudeSquared(const LinearMotionND<T, 3>& motion);

}; // Math
}; // SurgSim

#include "SurgSim/Math/LinearMotionArithmetic-inl.h"

#endif // SURGSIM_MATH_LINEARMOTIONARITHMETIC_H
