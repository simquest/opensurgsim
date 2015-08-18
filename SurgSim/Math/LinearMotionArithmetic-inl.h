// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest Solutions Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SURGSIM_MATH_LINEARMOTIONARITHMETIC_INL_H
#define SURGSIM_MATH_LINEARMOTIONARITHMETIC_INL_H

namespace SurgSim
{
namespace Math
{

template <typename T>
LinearMotion<T>::LinearMotion() : m_start(static_cast<T>(0)), m_end(static_cast<T>(0)) {}

template <typename T>
LinearMotion<T>::LinearMotion(const T& start, const T& end) : m_start(start), m_end(end) {}

template <typename T>
LinearMotion<T>::LinearMotion(const LinearMotion<T>& m) : m_start(m.m_start), m_end(m.m_end) {}

template <typename T>
LinearMotion<T>::LinearMotion(LinearMotion<T>&& m)
{
	*this = std::move(m);
}

template <typename T>
LinearMotion<T>& LinearMotion<T>::operator=(const LinearMotion<T>& m)
{
	m_start = m.m_start;
	m_end = m.m_end;
	return *this;
}

template <typename T>
LinearMotion<T>& LinearMotion<T>::operator=(LinearMotion<T>&& m)
{
	m_start = std::move(m.m_start);
	m_end = std::move(m.m_end);
	return *this;
}

template <typename T>
Interval<T> LinearMotion<T>::toInterval() const
{
	return Interval<T>::minToMax(m_start, m_end);
}

template <typename T>
Polynomial<T, 1> LinearMotion<T>::toPolynomial() const
{
	return Polynomial<T, 1>(m_start, m_end - m_start);
}

template <typename T>
bool LinearMotion<T>::containsZero() const
{
	return toInterval().containsZero();
}

template <typename T>
bool LinearMotion<T>::isApprox(const LinearMotion<T>& m, const T& epsilon) const
{
	return (std::abs(m_start - m.m_start) <= epsilon) && (std::abs(m_end - m.m_end) <= epsilon);
}

template <typename T>
bool LinearMotion<T>::operator==(const LinearMotion<T>& m) const
{
	return ((m_start == m.m_start) && (m_end == m.m_end));
}

template <typename T>
bool LinearMotion<T>::operator!=(const LinearMotion<T>& m) const
{
	return !(*this == m);
}

template <typename T>
LinearMotion<T> LinearMotion<T>::operator+(const LinearMotion<T>& m) const
{
	return LinearMotion<T>(m_start + m.m_start, m_end + m.m_end);
}

template <typename T>
LinearMotion<T>& LinearMotion<T>::operator+=(const LinearMotion<T>& m)
{
	m_start += m.m_start;
	m_end += m.m_end;
	return *this;
}

template <typename T>
LinearMotion<T> LinearMotion<T>::operator-(const LinearMotion<T>& m) const
{
	return LinearMotion<T>(m_start - m.m_start, m_end - m.m_end);
}

template <typename T>
LinearMotion<T>& LinearMotion<T>::operator-=(const LinearMotion<T>& m)
{
	m_start -= m.m_start;
	m_end -= m.m_end;
	return *this;
}

template <typename T>
Interval<T> LinearMotion<T>::operator*(const LinearMotion<T>& m) const
{
	return this->toInterval() * m.toInterval();
}

template <typename T>
Interval<T> LinearMotion<T>::operator/(const LinearMotion<T>& m) const
{
	return this->toInterval() / m.toInterval();
}

template <typename T>
T LinearMotion<T>::getStart() const
{
	return m_start;
}

template <typename T>
T LinearMotion<T>::getEnd() const
{
	return m_end;
}

template <typename T>
T LinearMotion<T>::atTime(const T& t) const
{
	return ((static_cast<T>(1) - t) * m_start + t * m_end);
}

template <typename T>
LinearMotion<T> LinearMotion<T>::firstHalf() const
{
	return LinearMotion<T>(m_start, (m_start + m_end) * static_cast<T>(0.5));
}

template <typename T>
LinearMotion<T> LinearMotion<T>::secondHalf() const
{
	return LinearMotion<T>((m_start + m_end) * static_cast<T>(0.5), m_end);
}

// Class ND
template <typename T, int N>
LinearMotionND<T, N>::LinearMotionND()
{
}

template <typename T, int N>
LinearMotionND<T, N>::LinearMotionND(const std::array<LinearMotion<T>, N>& x)
{
	m_motion = x;
}

template <typename T, int N>
LinearMotionND<T, N>::LinearMotionND(const LinearMotionND<T, N>& motion)
{
	m_motion = motion.m_motion;
}

template <typename T, int N>
LinearMotionND<T, N>::LinearMotionND(LinearMotionND<T, N>&& motion)
{
	*this = std::move(motion);
}

template <typename T, int N>
LinearMotionND<T, N>::LinearMotionND(const std::array<T, N>& a, const std::array<T, N>& b)
{
	for (int i = 0; i < N; ++i)
	{
		m_motion[i] = LinearMotion<T>(a[i], b[i]);
	}
}

template <typename T, int N>
LinearMotionND<T, N>& LinearMotionND<T, N>::operator=(const LinearMotionND<T, N>& motion)
{
	m_motion = motion.m_motion;
	return *this;
}

template <typename T, int N>
LinearMotionND<T, N>& LinearMotionND<T, N>::operator=(LinearMotionND<T, N>&& motion)
{
	if (this != &motion)
	{
		m_motion = std::move(motion.m_motion);
	}

	return *this;
}

template <typename T, int N>
IntervalND<T, N> LinearMotionND<T, N>::toInterval() const
{
	std::array<Interval<T>, N> motions;
	for (int i = 0; i < N; ++i)
	{
		motions[i] = m_motion[i].toInterval();
	}

	return IntervalND<T, N>(motions);
}

template <typename T, int N>
bool LinearMotionND<T, N>::isApprox(const LinearMotionND<T, N>& motion, const T& epsilon) const
{
	for (int i = 0; i < N; i++)
	{
		if (!m_motion[i].isApprox(motion.m_interval[i], epsilon))
		{
			return false;
		}
	}
	return true;
}

template <typename T, int N>
bool LinearMotionND<T, N>::operator==(const LinearMotionND<T, N>& motion) const
{
	return (m_motion == motion.m_motion);
}

template <typename T, int N>
bool LinearMotionND<T, N>::operator!=(const LinearMotionND<T, N>& motion) const
{
	return !(this->operator==(motion));
}

template <typename T, int N>
LinearMotionND<T, N> LinearMotionND<T, N>::operator+(const LinearMotionND<T, N>& m) const
{
	LinearMotionND<T, N> ret(*this);
	ret += m;
	return ret;
}

template <typename T, int N>
LinearMotionND<T, N>& LinearMotionND<T, N>::operator+=(const LinearMotionND<T, N>& m)
{
	for (int i = 0; i < N; ++i)
	{
		m_motion[i] += m.m_motion[i];
	}
	return *this;
}

template <typename T, int N>
LinearMotionND<T, N> LinearMotionND<T, N>::operator-(const LinearMotionND<T, N>& m) const
{
	LinearMotionND<T, N> ret(*this);
	ret -= m;
	return ret;
}

template <typename T, int N>
LinearMotionND<T, N>& LinearMotionND<T, N>::operator-=(const LinearMotionND<T, N>& m)
{
	for (int i = 0; i < N; ++i)
	{
		m_motion[i] -= m.m_motion[i];
	}
	return *this;
}

template <typename T, int N>
IntervalND<T, N> LinearMotionND<T, N>::operator*(const LinearMotionND<T, N>& m) const
{
	return this->toInterval() * m.toInterval();
}

template <typename T, int N>
IntervalND<T, N> LinearMotionND<T, N>::operator/(const LinearMotionND<T, N>& m) const
{
	return this->toInterval() / m.toInterval();
}

template <typename T, int N>
Interval<T> LinearMotionND<T, N>::dotProduct(const LinearMotionND<T, N>& motion) const
{
	Interval<T> ret(static_cast<T>(0), static_cast<T>(0));
	for (int i = 0 ; i < N ; i++)
	{
		ret += m_motion[i] * motion.m_motion[i];
	}
	return ret;
}

template <typename T, int N>
const LinearMotion<T>& LinearMotionND<T, N>::getAxis(int i) const
{
	SURGSIM_ASSERT((i >= 0) && (i < N)) << "Asking for an axis greater than the dimensionality of the linear motion";
	return m_motion[i];
}

template <typename T, int N>
void LinearMotionND<T, N>::getStart(std::array<T, N>* start) const
{
	for (int i = 0; i < N; ++i)
	{
		(*start)[i] = m_motion[i].getStart();
	}
}

template <typename T, int N>
void LinearMotionND<T, N>::getEnd(std::array<T, N>* end) const
{
	for (int i = 0; i < N; ++i)
	{
		(*end)[i] = m_motion[i].getEnd();
	}
}

template <typename T, int N>
LinearMotionND<T, N> LinearMotionND<T, N>::firstHalf() const
{
	LinearMotionND<T, N> ret;
	for (int i = 0; i < N; ++i)
	{
		ret.m_motion[i] = m_motion[i].firstHalf();
	}
	return ret;
}

template <typename T, int N>
LinearMotionND<T, N> LinearMotionND<T, N>::secondHalf() const
{
	LinearMotionND<T, N> ret;
	for (int i = 0; i < N; ++i)
	{
		ret.m_motion[i] = m_motion[i].secondHalf();
	}
	return ret;
}

// Special case for dimension 3
template <typename T>
LinearMotionND<T, 3>::LinearMotionND()
{
}

template <typename T>
LinearMotionND<T, 3>::LinearMotionND(const std::array<LinearMotion<T>, 3>& x)
{
	m_motion[0] = x[0];
	m_motion[1] = x[1];
	m_motion[2] = x[2];
}

template <typename T>
LinearMotionND<T, 3>::LinearMotionND(const LinearMotion<T>& a, const LinearMotion<T>& b, const LinearMotion<T>& c)
{
	m_motion[0] = a;
	m_motion[1] = b;
	m_motion[2] = c;
}

template <typename T>
LinearMotionND<T, 3>::LinearMotionND(const LinearMotionND<T, 3>& motion)
{
	m_motion[0] = motion.m_motion[0];
	m_motion[1] = motion.m_motion[1];
	m_motion[2] = motion.m_motion[2];
}

template <typename T>
LinearMotionND<T, 3>::LinearMotionND(LinearMotionND<T, 3>&& motion)
{
	*this = std::move(motion);
}

template <typename T>
LinearMotionND<T, 3>::LinearMotionND(const std::array<T, 3>& a, const std::array<T, 3>& b)
{
	m_motion[0] = LinearMotion<T>(a[0], b[0]);
	m_motion[1] = LinearMotion<T>(a[1], b[1]);
	m_motion[2] = LinearMotion<T>(a[2], b[2]);
}

template <typename T>
LinearMotionND<T, 3>& LinearMotionND<T, 3>::operator=(const LinearMotionND<T, 3>& motion)
{
	m_motion[0] = motion.m_motion[0];
	m_motion[1] = motion.m_motion[1];
	m_motion[2] = motion.m_motion[2];
	return *this;
}

template <typename T>
LinearMotionND<T, 3>& LinearMotionND<T, 3>::operator=(LinearMotionND<T, 3>&& motion)
{
	m_motion[0] = std::move(motion.m_motion[0]);
	m_motion[1] = std::move(motion.m_motion[1]);
	m_motion[2] = std::move(motion.m_motion[2]);
	return *this;
}

template <typename T>
IntervalND<T, 3> LinearMotionND<T, 3>::toInterval() const
{
	std::array<Interval<T>, 3> intervals;
	intervals[0] = m_motion[0].toInterval();
	intervals[1] = m_motion[1].toInterval();
	intervals[2] = m_motion[2].toInterval();
	return IntervalND<T, 3>(intervals);
}

template <typename T>
bool LinearMotionND<T, 3>::isApprox(const LinearMotionND<T, 3>& motion, const T& epsilon) const
{
	return (m_motion[0].isApprox(motion.m_motion[0], epsilon) &&
			m_motion[1].isApprox(motion.m_motion[1], epsilon) &&
			m_motion[2].isApprox(motion.m_motion[2], epsilon));
}

template <typename T>
bool LinearMotionND<T, 3>::operator==(const LinearMotionND<T, 3>& motion) const
{
	return (m_motion[0] == motion.m_motion[0] &&
			m_motion[1] == motion.m_motion[1] &&
			m_motion[2] == motion.m_motion[2]);
}

template <typename T>
bool LinearMotionND<T, 3>::operator!=(const LinearMotionND<T, 3>& motion) const
{
	return !(this->operator==(motion));
}

template <typename T>
LinearMotionND<T, 3> LinearMotionND<T, 3>::operator+(const LinearMotionND<T, 3>& m) const
{
	LinearMotionND<T, 3> ret(*this);
	ret += m;
	return ret;
}

template <typename T>
LinearMotionND<T, 3>& LinearMotionND<T, 3>::operator+=(const LinearMotionND<T, 3>& m)
{
	m_motion[0] += m.m_motion[0];
	m_motion[1] += m.m_motion[1];
	m_motion[2] += m.m_motion[2];
	return *this;
}

template <typename T>
LinearMotionND<T, 3> LinearMotionND<T, 3>::operator-(const LinearMotionND<T, 3>& m) const
{
	LinearMotionND<T, 3> ret(*this);
	ret -= m;
	return ret;
}

template <typename T>
LinearMotionND<T, 3>& LinearMotionND<T, 3>::operator-=(const LinearMotionND<T, 3>& m)
{
	m_motion[0] -= m.m_motion[0];
	m_motion[1] -= m.m_motion[1];
	m_motion[2] -= m.m_motion[2];
	return *this;
}

template <typename T>
IntervalND<T, 3> LinearMotionND<T, 3>::operator*(const LinearMotionND<T, 3>& m) const
{
	return this->toInterval() * m.toInterval();
}

template <typename T>
IntervalND<T, 3> LinearMotionND<T, 3>::operator/(const LinearMotionND<T, 3>& m) const
{
	return this->toInterval() / m.toInterval();
}

template <typename T>
Interval<T> LinearMotionND<T, 3>::dotProduct(const LinearMotionND<T, 3>& motion, const Interval<T>& range) const
{
	return valuesOverInterval(analyticDotProduct(*this, motion), range);
}

template <typename T>
IntervalND<T, 3> LinearMotionND<T, 3>::crossProduct(const LinearMotionND<T, 3>& motion,
		const Interval<T>& range) const
{
	// toInterval().crossProduct(motion.toInterval())
	// results in intervals that are too broad.
	return IntervalND<T, 3>(valuesOverInterval(analyticCrossProductAxis<double, 0>(*this, motion), range),
							valuesOverInterval(analyticCrossProductAxis<double, 1>(*this, motion), range),
							valuesOverInterval(analyticCrossProductAxis<double, 2>(*this, motion), range));
}

template <typename T>
Interval<T> LinearMotionND<T, 3>::magnitudeSquared(const Interval<T>& range) const
{
	return valuesOverInterval(analyticMagnitudeSquared(*this), range);
}

template <typename T>
Interval<T> LinearMotionND<T, 3>::magnitude(const Interval<T>& range) const
{
	Interval<T> magnitudeSq = magnitudeSquared(range);
	// Both minimum and maximum are guaranteed to be non-negative.
	return Interval<T>(sqrt(magnitudeSq.getMin()), sqrt(magnitudeSq.getMax()));
}

template <typename T>
const LinearMotion<T>& LinearMotionND<T, 3>::getAxis(int i) const
{
	SURGSIM_ASSERT((i >= 0) && (i < 3)) << "Asking for an axis greater than the dimensionality of the linear motion";
	return m_motion[i];
}

template <typename T>
void LinearMotionND<T, 3>::getStart(std::array<T, 3>* start) const
{
	(*start)[0] = m_motion[0].getStart();
	(*start)[1] = m_motion[1].getStart();
	(*start)[2] = m_motion[2].getStart();
}

template <typename T>
void LinearMotionND<T, 3>::getEnd(std::array<T, 3>* end) const
{
	(*end)[0] = m_motion[0].getEnd();
	(*end)[1] = m_motion[1].getEnd();
	(*end)[2] = m_motion[2].getEnd();
}

template <typename T>
typename LinearMotionND<T, 3>::Vector3 LinearMotionND<T, 3>::getStart() const
{
	return LinearMotionND<T, 3>::Vector3(m_motion[0].getStart(), m_motion[1].getStart(), m_motion[2].getStart());
}

template <typename T>
typename LinearMotionND<T, 3>::Vector3 LinearMotionND<T, 3>::getEnd() const
{
	return Vector3(m_motion[0].getEnd(), m_motion[1].getEnd(), m_motion[2].getEnd());
}

template <typename T>
typename LinearMotionND<T, 3>::Vector3 LinearMotionND<T, 3>::atTime(const T& t) const
{
	return Vector3(m_motion[0].atTime(t), m_motion[1].atTime(t), m_motion[2].atTime(t));
}

template <typename T>
LinearMotionND<T, 3> LinearMotionND<T, 3>::firstHalf() const
{
	LinearMotionND<T, 3> ret;
	ret.m_motion[0] = m_motion[0].firstHalf();
	ret.m_motion[1] = m_motion[1].firstHalf();
	ret.m_motion[2] = m_motion[2].firstHalf();
	return ret;
}

template <typename T>
LinearMotionND<T, 3> LinearMotionND<T, 3>::secondHalf() const
{
	LinearMotionND<T, 3> ret;
	ret.m_motion[0] = m_motion[0].secondHalf();
	ret.m_motion[1] = m_motion[1].secondHalf();
	ret.m_motion[2] = m_motion[2].secondHalf();
	return ret;
}

// Utility functions not part of any class

// Interval functions
template <typename T>
std::ostream& operator<<(std::ostream& o, const LinearMotion<T>& motion)
{
	o << "(" << motion.getStart() << " -> " << motion.getEnd() << ")";
	return o;
}

// Interval ND functions
template <typename T, int N>
std::ostream& operator<<(std::ostream& o, const LinearMotionND<T, N>& motion)
{
	o << "([" << motion.getAxis(0).getStart();
	for (int i = 1; i < N; ++i)
	{
		o << "," << motion.getAxis(i).getStart();
	}
	o << "] -> [" << motion.getAxis(0).getEnd();
	for (int i = 1; i < N; ++i)
	{
		o << "," << motion.getAxis(i).getEnd();
	}
	o << "])";
	return o;
}

// Interval 3D functions
template <typename T>
Polynomial<T, 2> analyticDotProduct(const LinearMotionND<T, 3>& a, const LinearMotionND<T, 3>& b)
{
	return a.getAxis(0).toPolynomial() * b.getAxis(0).toPolynomial() +
		   a.getAxis(1).toPolynomial() * b.getAxis(1).toPolynomial() +
		   a.getAxis(2).toPolynomial() * b.getAxis(2).toPolynomial();
}

template <typename T, int A>
Polynomial<T, 2> analyticCrossProductAxis(const LinearMotionND<T, 3>& a, const LinearMotionND<T, 3>& b)
{
	// The labels here are probably a bit confusing for anyone else, but at least this makes sense.
	// For A == 0, the "Y" and "Z" mean what they say, and the output is the X component.
	// For A == 1, they get rotated "down" by one (so Y -> Z, Z -> X), and the output is the Y component.
	// For A == 2, they get rotated "down" by two (so Y -> X, Z -> Y), and the output is the Z component.
	const LinearMotion<T>& aY = a.getAxis((A + 1) % 3);
	const LinearMotion<T>& aZ = a.getAxis((A + 2) % 3);
	const LinearMotion<T>& bY = b.getAxis((A + 1) % 3);
	const LinearMotion<T>& bZ = b.getAxis((A + 2) % 3);
	return aY.toPolynomial() * bZ.toPolynomial() - aZ.toPolynomial() * bY.toPolynomial();
}

template <typename T>
Polynomial<T, 2> analyticCrossProductXAxis(const LinearMotionND<T, 3>& a, const LinearMotionND<T, 3>& b)
{
	return analyticCrossProductAxis<double, 0>(a, b);
}

template <typename T>
Polynomial<T, 2> analyticCrossProductYAxis(const LinearMotionND<T, 3>& a, const LinearMotionND<T, 3>& b)
{
	return analyticCrossProductAxis<double, 1>(a, b);
}

template <typename T>
Polynomial<T, 2> analyticCrossProductZAxis(const LinearMotionND<T, 3>& a, const LinearMotionND<T, 3>& b)
{
	return analyticCrossProductAxis<double, 2>(a, b);
}

template <typename T>
void analyticCrossProduct(const LinearMotionND<T, 3>& a, const LinearMotionND<T, 3>& b,
						  Polynomial<T, 2>* resultXAxis, Polynomial<T, 2>* resultYAxis, Polynomial<T, 2>* resultZAxis)
{
	(*resultXAxis) = analyticCrossProductXAxis(a, b);
	(*resultYAxis) = analyticCrossProductYAxis(a, b);
	(*resultZAxis) = analyticCrossProductZAxis(a, b);
}

template <typename T>
Polynomial <T, 3> analyticTripleProduct(const LinearMotionND<T, 3>& a, const LinearMotionND<T, 3>& b,
										const LinearMotionND<T, 3>& c)
{
	const Polynomial<T, 1> aX = a.getAxis(0).toPolynomial();
	const Polynomial<T, 1> aY = a.getAxis(1).toPolynomial();
	const Polynomial<T, 1> aZ = a.getAxis(2).toPolynomial();
	const Polynomial<T, 1> bX = b.getAxis(0).toPolynomial();
	const Polynomial<T, 1> bY = b.getAxis(1).toPolynomial();
	const Polynomial<T, 1> bZ = b.getAxis(2).toPolynomial();
	const Polynomial<T, 1> cX = c.getAxis(0).toPolynomial();
	const Polynomial<T, 1> cY = c.getAxis(1).toPolynomial();
	const Polynomial<T, 1> cZ = c.getAxis(2).toPolynomial();
	return ((bY * cZ - bZ * cY) * aX + (bZ * cX - bX * cZ) * aY + (bX * cY - bY * cX) * aZ);
}

template <typename T>
static Interval<T> tripleProduct(const LinearMotionND<T, 3>& a, const LinearMotionND<T, 3>& b,
								 const LinearMotionND<T, 3>& c, const Interval<T>& range)
{
	return valuesOverInterval(analyticTripleProduct(a, b, c), range);
}

template <typename T>
Polynomial<T, 2> analyticMagnitudeSquared(const LinearMotionND<T, 3>& motion)
{
	return analyticDotProduct(motion, motion);
}

}; // Math
}; // SurgSim

#endif // SURGSIM_MATH_LINEARMOTIONARITHMETIC_INL_H
