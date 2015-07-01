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

#ifndef SURGSIM_MATH_INTERVALARITHMETIC_INL_H
#define SURGSIM_MATH_INTERVALARITHMETIC_INL_H

#include <ostream>

#include "SurgSim/Math/MinMax.h"

namespace SurgSim
{
namespace Math
{

template <class T>
Interval<T>::Interval(): m_min(static_cast<T>(0)), m_max(static_cast<T>(0)) {}

template <class T>
Interval<T>::Interval(T min, T max): m_min(min), m_max(max)
{
	SURGSIM_ASSERT(min <= max) << "Incorrect order of interval bounds";
}

template <class T>
Interval<T>::Interval(const Interval<T>& i): m_min(i.m_min), m_max(i.m_max) {}

template <class T>
Interval<T>::Interval(Interval<T>&& i): m_min(static_cast<T>(0)), m_max(static_cast<T>(0))
{
	m_min = i.m_min;
	m_max = i.m_max;
}

template <class T>
Interval<T>& Interval<T>::operator=(const Interval<T>& i)
{
	m_min = i.m_min;
	m_max = i.m_max;
	return *this;
}

template <class T>
Interval<T>& Interval<T>::operator=(Interval<T>&& i)
{
	if (this != &i)
	{
		m_min = i.m_min;
		m_max = i.m_max;
	}

	return *this;
}

template <class T>
Interval<T> Interval<T>::minToMax(const T a1, const T a2)
{
	T min, max;
	minMax(a1, a2, &min, &max);
	return Interval<T>(min, max);
}

template <class T>
Interval<T> Interval<T>::minToMax(const T a1, const T a2, const T a3)
{
	T min, max;
	minMax(a1, a2, a3, &min, &max);
	return Interval<T>(min, max);
}

template <class T>
Interval<T> Interval<T>::minToMax(const T a1, const T a2, const T a3, const T a4)
{
	T min, max;
	minMax(a1, a2, a3, a4, &min, &max);
	return Interval<T>(min, max);
}

template <class T>
bool Interval<T>::overlapsWith(const Interval<T>& i) const
{
	return (m_min <= i.m_max && i.m_min <= m_max);
}

template <class T>
bool Interval<T>::contains(T val) const
{
	return (m_min <= val && m_max >= val);
}

template <class T>
bool Interval<T>::containsZero() const
{
	return contains(static_cast<T>(0));
}

template <class T>
bool Interval<T>::isApprox(const Interval<T>& i, const T& epsilon) const
{
	return (std::abs(m_min - i.m_min) <= epsilon) && (std::abs(m_max - i.m_max) <= epsilon);
}

template <class T>
bool Interval<T>::operator ==(const Interval<T>& i) const
{
	return (m_min == i.m_min && m_max == i.m_max);
}

template <class T>
bool Interval<T>::operator !=(const Interval<T>& i) const
{
	return !(this->operator==(i));
}

template <class T>
Interval<T>& Interval<T>::addThickness(const T thickness)
{
	m_min -= thickness;
	m_max += thickness;
	return *this;
}

template <class T>
Interval<T>& Interval<T>::extendToInclude(T x)
{
	if (x < m_min)
	{
		m_min = x;
	}
	else if (x > m_max)
	{
		m_max = x;
	}
	return *this;
}

template <class T>
Interval<T>& Interval<T>::extendToInclude(const Interval<T>& i)
{
	if (i.m_min < m_min)
	{
		m_min = i.m_min;
	}
	if (i.m_max > m_max)
	{
		m_max = i.m_max;
	}
	return *this;
}

template <class T>
Interval<T> Interval<T>::operator +(const Interval<T>& i) const
{
	return Interval<T>(m_min + i.m_min, m_max + i.m_max);
}

template <class T>
Interval<T> Interval<T>::operator +(T v) const
{
	return Interval<T>(m_min + v, m_max + v);
}

template <class T>
Interval<T>& Interval<T>::operator +=(const Interval<T>& i)
{
	m_min += i.m_min;
	m_max += i.m_max;
	return *this;
}

template <class T>
Interval<T>& Interval<T>::operator +=(T v)
{
	m_min += v;
	m_max += v;
	return *this;
}

template <class T>
Interval<T> Interval<T>::operator -() const
{
	return Interval<T>(-m_max, -m_min);
}

template <class T>
Interval<T> Interval<T>::operator -(const Interval<T>& i) const
{
	return Interval<T>(m_min - i.m_max, m_max - i.m_min);
}

template <class T>
Interval<T> Interval<T>::operator -(T v) const
{
	return Interval<T>(m_min - v, m_max - v);
}

template <class T>
Interval<T>& Interval<T>::operator -=(const Interval<T>& i)
{
	m_min -= i.m_max;
	m_max -= i.m_min;
	return *this;
}

template <class T>
Interval<T>& Interval<T>::operator -=(T v)
{
	m_min -= v;
	m_max -= v;
	return *this;
}

template <class T>
Interval<T> Interval<T>::operator *(const Interval<T>& i) const
{
	return minToMax(m_min * i.m_min, m_min * i.m_max, m_max * i.m_min, m_max * i.m_max);
}

template <class T>
Interval<T> Interval<T>::operator *(T v) const
{
	if (v >= 0)
	{
		return Interval<T>(m_min * v, m_max * v);
	}
	else
	{
		return Interval<T>(m_max * v, m_min * v);
	}
}

template <class T>
Interval<T>& Interval<T>::operator *=(const Interval<T>& i)
{
	*this = minToMax(m_min * i.m_min, m_min * i.m_max, m_max * i.m_min, m_max * i.m_max);
	return *this;
}

template <class T>
Interval<T>& Interval<T>::operator *=(T v)
{
	*this = minToMax(v * m_min, v * m_max);
	return *this;
}

template <class T>
Interval<T> Interval<T>::inverse() const
{
	SURGSIM_ASSERT(!containsZero()) << "Cannot invert or divide by an interval containing 0. Interval: [" <<
									getMin() << ", " << getMax() << "]" << std::endl;
	return Interval<T>(static_cast<T>(1) / m_max, static_cast<T>(1) / m_min);
}

template <class T>
Interval<T> Interval<T>::operator /(const Interval<T>& i) const
{
	return (*this) * i.inverse();
}

template <class T>
Interval<T>& Interval<T>::operator /=(const Interval<T>& i)
{
	return (*this) = (*this) * i.inverse();
}

template <class T>
Interval<T> Interval<T>::square() const
{
	T lowerBoundSquared = m_min * m_min;
	T upperBoundSquared = m_max * m_max;
	T minSquare, maxSquare;
	minMax(lowerBoundSquared, upperBoundSquared, &minSquare, &maxSquare);
	return Interval<T>((m_min < 0 && m_max > 0) ? 0 : minSquare, maxSquare);
}

template <class T>
T Interval<T>::getMin() const
{
	return m_min;
}

template <class T>
T Interval<T>::getMax() const
{
	return m_max;
}

template <class T>
Interval<T> Interval<T>::lowerHalf() const
{
	return Interval<T>(m_min, (m_min + m_max) * static_cast<T>(0.5));
}

template <class T>
Interval<T> Interval<T>::upperHalf() const
{
	return Interval<T>((m_min + m_max) * static_cast<T>(0.5), m_max);
}

// Class nD
template <class T, int N>
Interval_nD<T, N>::Interval_nD()
{
	for (int i = 0; i < N; i++)
	{
		m_interval[i] = Interval<T>();
	}
}

template <class T, int N>
Interval_nD<T, N>::Interval_nD(const std::array<Interval<T>, N>& x)
{
	for (int i = 0; i < N; i++)
	{
		m_interval[i] = x[i];
	}
}

template <class T, int N>
Interval_nD<T, N>::Interval_nD(const Interval_nD<T, N>& interval)
{
	for (int i = 0; i < N; i++)
	{
		m_interval[i] = interval.m_interval[i];
	}
}

template <class T, int N>
Interval_nD<T, N>::Interval_nD(Interval_nD<T, N>&& i)
{
	for (int j = 0; j < N; j++)
	{
		m_interval[j] = i.m_interval[j];
	}
}

template <class T, int N>
Interval_nD<T, N>::Interval_nD(const std::array<T, N>& a, const std::array<T, N>& b)
{
	for (int i = 0; i < N; ++i)
	{
		m_interval[i] = Interval<T>::minToMax(a[i], b[i]);
	}
}

template <class T, int N>
Interval_nD<T, N>& Interval_nD<T, N>::operator =(const Interval_nD<T, N>& interval)
{
	for (int i = 0; i < N; i++)
	{
		m_interval[i] = interval.m_interval[i];
	}
	return *this;
}

template <class T, int N>
Interval_nD<T, N>& Interval_nD<T, N>::operator=(Interval_nD<T, N>&& i)
{
	if (this != &i)
	{
		for (int j = 0; j < N; j++)
		{
			m_interval[j] = i.m_interval[j];
		}
	}

	return *this;
}

template <class T, int N>
bool Interval_nD<T, N>::overlapsWith(const Interval_nD<T, N>& interval) const
{
	// For the rectangular [hyper]prisms to overlap, they must overlap in all axes.
	for (int i = 0; i < N; ++i)
	{
		if (!m_interval[i].overlapsWith(interval.m_interval[i]))
		{
			return false;
		}
	}
	return true;
}

template <class T, int N>
bool Interval_nD<T, N>::isApprox(const Interval_nD<T, N>& interval, const T& epsilon) const
{
	for (int i = 0; i < N; i++)
	{
		if (!m_interval[i].isApprox(interval.m_interval[i], epsilon))
		{
			return false;
		}
	}
	return true;
}

template <class T, int N>
bool Interval_nD<T, N>::operator ==(const Interval_nD<T, N>& interval) const
{
	for (int i = 0; i < N; i++)
	{
		if (m_interval[i] != interval.m_interval[i])
		{
			return false;
		}
	}
	return true;
}

template <class T, int N>
bool Interval_nD<T, N>::operator !=(const Interval_nD<T, N>& interval) const
{
	return !(this->operator==(interval));
}

template <class T, int N>
Interval_nD<T, N>& Interval_nD<T, N>::addThickness(const double thickness)
{
	for (int i = 0; i < N; i++)
	{
		m_interval[i].addThickness(thickness);
	}
	return *this;
}

template <class T, int N>
Interval_nD<T, N> Interval_nD<T, N>::operator +(const Interval_nD<T, N>& interval) const
{
	Interval_nD<T, N> ret;
	for (int i = 0; i < N; i++)
	{
		ret.m_interval[i] = m_interval[i] + interval.m_interval[i];
	}
	return ret;
}

template <class T, int N>
Interval_nD<T, N>& Interval_nD<T, N>::operator +=(const Interval_nD<T, N>& interval)
{
	for (int i = 0; i < N; i++)
	{
		m_interval[i] += interval.m_interval[i];
	}
	return *this;
}

template <class T, int N>
Interval_nD<T, N> Interval_nD<T, N>::operator -(const Interval_nD<T, N>& interval) const
{
	Interval_nD<T, N> ret;
	for (int i = 0; i < N; i++)
	{
		ret.m_interval[i] = m_interval[i] - interval.m_interval[i];
	}
	return ret;
}

template <class T, int N>
Interval_nD<T, N>& Interval_nD<T, N>::operator -=(const Interval_nD<T, N>& interval)
{
	for (int i = 0; i < N; i++)
	{
		m_interval[i] -= interval.m_interval[i];
	}
	return *this;
}

template <class T, int N>
Interval_nD<T, N> Interval_nD<T, N>::operator *(const Interval_nD<T, N>& interval) const
{
	Interval_nD<T, N> ret;
	for (int i = 0; i < N; i++)
	{
		ret.m_interval[i] = m_interval[i] * interval.m_interval[i];
	}
	return ret;
}

template <class T, int N>
Interval_nD<T, N>& Interval_nD<T, N>::operator *=(const Interval_nD<T, N>& interval)
{
	for (int i = 0; i < N; i++)
	{
		m_interval[i] *= interval.m_interval[i];
	}
	return *this;
}

template <class T, int N>
Interval_nD<T, N> Interval_nD<T, N>::inverse(void) const
{
	Interval_nD<T, N> ret;
	for (int i = 0; i < N; i++)
	{
		ret.m_interval[i] = m_interval[i].inverse();
	}
	return ret;
}

template <class T, int N>
Interval_nD<T, N> Interval_nD<T, N>::operator /(const Interval_nD<T, N>& interval) const
{
	Interval_nD<T, N> ret;
	for (int i = 0; i < N; i++)
	{
		ret.m_interval[i] = m_interval[i] / interval.m_interval[i];
	}
	return ret;
}

template <class T, int N>
Interval_nD<T, N>& Interval_nD<T, N>::operator /=(const Interval_nD<T, N>& interval)
{
	for (int i = 0; i < N; i++)
	{
		m_interval[i] /= interval.m_interval[i];
	}
	return *this;
}

template <class T, int N>
Interval<T> Interval_nD<T, N>::dotProduct(const Interval_nD<T, N>& interval) const
{
	Interval<T> ret(static_cast<T>(0), static_cast<T>(0));
	for (int i = 0; i < N; i++)
	{
		ret += m_interval[i] * interval.m_interval[i];
	}
	return ret;
}

template <class T, int N>
Interval<T> Interval_nD<T, N>::magnitudeSquared() const
{
	Interval<T> result = m_interval[0].square();
	for (int i = 1; i < N; ++i)
	{
		result += m_interval[i].square();
	}
	return result;
}

template <class T, int N>
Interval<T> Interval_nD<T, N>::magnitude() const
{
	Interval<T> magnitudeSq = magnitudeSquared();
	// Both minimum and maximum are guaranteed to be non-negative.
	return Interval<T>(sqrt(magnitudeSq.getMin()), sqrt(magnitudeSq.getMax()));
}

template <class T, int N>
const Interval<T>& Interval_nD<T, N>::getAxis(size_t i) const
{
	return m_interval[i];
}

template <class T>
Interval_nD<T, 3>::Interval_nD()
{
	m_interval[0] = Interval<T>();
	m_interval[1] = Interval<T>();
	m_interval[2] = Interval<T>();
}

template <class T>
Interval_nD<T, 3>::Interval_nD(const std::array<Interval<T>, 3>& x)
{
	m_interval[0] = x[0];
	m_interval[1] = x[1];
	m_interval[2] = x[2];
}

template <class T>
Interval_nD<T, 3>::Interval_nD(Interval<T> x, Interval<T> y, Interval<T> z)
{
	m_interval[0] = x;
	m_interval[1] = y;
	m_interval[2] = z;
}

template <class T>
Interval_nD<T, 3>::Interval_nD(const Interval_nD<T, 3>& i)
{
	m_interval[0] = i.m_interval[0];
	m_interval[1] = i.m_interval[1];
	m_interval[2] = i.m_interval[2];
}

template <class T>
Interval_nD<T, 3>::Interval_nD(Interval_nD<T, 3>&& i)
{
	m_interval[0] = i.m_interval[0];
	m_interval[1] = i.m_interval[1];
	m_interval[2] = i.m_interval[2];
}

template <class T>
Interval_nD<T, 3>::Interval_nD(const std::array<T, 3>& a, const std::array<T, 3>& b)
{
	m_interval[0] = Interval<T>::minToMax(a[0], b[0]);
	m_interval[1] = Interval<T>::minToMax(a[1], b[1]);
	m_interval[2] = Interval<T>::minToMax(a[2], b[2]);
}

template <class T>
Interval_nD<T, 3>& Interval_nD<T, 3>::operator =(const Interval_nD<T, 3>& i)
{
	m_interval[0] = i.m_interval[0];
	m_interval[1] = i.m_interval[1];
	m_interval[2] = i.m_interval[2];
	return *this;
}

template <class T>
Interval_nD<T, 3>& Interval_nD<T, 3>::operator=(Interval_nD<T, 3>&& i)
{
	if (this != &i)
	{
		m_interval[0] = i.m_interval[0];
		m_interval[1] = i.m_interval[1];
		m_interval[2] = i.m_interval[2];
	}

	return *this;
}

template <class T>
bool Interval_nD<T, 3>::overlapsWith(const Interval_nD<T, 3>& interval) const
{
	// For the rectangular prisms to overlap, they must overlap in all axes.
	return (m_interval[0].overlapsWith(interval.m_interval[0]) && m_interval[1].overlapsWith(interval.m_interval[1])
			&& m_interval[2].overlapsWith(interval.m_interval[2]));
}

template <class T>
bool Interval_nD<T, 3>::isApprox(const Interval_nD<T, 3>& i, const T& epsilon) const
{
	return (m_interval[0].isApprox(i.m_interval[0], epsilon) && m_interval[1].isApprox(i.m_interval[1], epsilon) &&
			m_interval[2].isApprox(i.m_interval[2], epsilon));
}

template <class T>
bool Interval_nD<T, 3>::operator ==(const Interval_nD<T, 3>& i) const
{
	return (m_interval[0] == i.m_interval[0] && m_interval[1] == i.m_interval[1] && m_interval[2] == i.m_interval[2]);
}

template <class T>
bool Interval_nD<T, 3>::operator !=(const Interval_nD<T, 3>& i) const
{
	return !(this->operator==(i));
}

template <class T>
Interval_nD<T, 3>& Interval_nD<T, 3>::addThickness(const double thickness)
{
	m_interval[0].addThickness(thickness);
	m_interval[1].addThickness(thickness);
	m_interval[2].addThickness(thickness);
	return *this;
}

template <class T>
Interval_nD<T, 3> Interval_nD<T, 3>::operator +(const Interval_nD<T, 3>& i) const
{
	return Interval_nD<T, 3>(m_interval[0] + i.m_interval[0], m_interval[1] + i.m_interval[1],
							 m_interval[2] + i.m_interval[2]);
}

template <class T>
Interval_nD<T, 3>& Interval_nD<T, 3>::operator +=(const Interval_nD<T, 3>& i)
{
	m_interval[0] += i.m_interval[0];
	m_interval[1] += i.m_interval[1];
	m_interval[2] += i.m_interval[2];
	return *this;
}

template <class T>
Interval_nD<T, 3> Interval_nD<T, 3>::operator -(const Interval_nD<T, 3>& i) const
{
	return Interval_nD<T, 3>(m_interval[0] - i.m_interval[0], m_interval[1] - i.m_interval[1],
							 m_interval[2] - i.m_interval[2]);
}

template <class T>
Interval_nD<T, 3>& Interval_nD<T, 3>::operator -=(const Interval_nD<T, 3>& i)
{
	m_interval[0] -= i.m_interval[0];
	m_interval[1] -= i.m_interval[1];
	m_interval[2] -= i.m_interval[2];
	return *this;
}

template <class T>
Interval_nD<T, 3> Interval_nD<T, 3>::operator *(const Interval_nD<T, 3>& i) const
{
	return Interval_nD<T, 3>(m_interval[0] * i.m_interval[0], m_interval[1] * i.m_interval[1],
							 m_interval[2] * i.m_interval[2]);
}

template <class T>
Interval_nD<T, 3>& Interval_nD<T, 3>::operator *=(const Interval_nD<T, 3>& i)
{
	m_interval[0] *= i.m_interval[0];
	m_interval[1] *= i.m_interval[1];
	m_interval[2] *= i.m_interval[2];
	return *this;
}

template <class T>
Interval_nD<T, 3> Interval_nD<T, 3>::inverse(void) const
{
	return Interval_nD<T, 3>(m_interval[0].inverse(), m_interval[1].inverse(), m_interval[2].inverse());
}

template <class T>
Interval_nD<T, 3> Interval_nD<T, 3>::operator /(const Interval_nD<T, 3>& i) const
{
	return Interval_nD<T, 3>(m_interval[0] / i.m_interval[0], m_interval[1] / i.m_interval[1],
							 m_interval[2] / i.m_interval[2]);
}

template <class T>
Interval_nD<T, 3>& Interval_nD<T, 3>::operator /=(const Interval_nD<T, 3>& i)
{
	m_interval[0] /= i.m_interval[0];
	m_interval[1] /= i.m_interval[1];
	m_interval[2] /= i.m_interval[2];
	return *this;
}

template <class T>
Interval<T> Interval_nD<T, 3>::dotProduct(const Interval_nD<T, 3>& i) const
{
	return (m_interval[0] * i.m_interval[0] + m_interval[1] * i.m_interval[1] + m_interval[2] * i.m_interval[2]);
}

template <class T>
Interval_nD<T, 3> Interval_nD<T, 3>::crossProduct(const Interval_nD<T, 3>& i) const
{
	return Interval_nD<T, 3>(m_interval[1] * i.m_interval[2] - m_interval[2] * i.m_interval[1],
							 m_interval[2] * i.m_interval[0] - m_interval[0] * i.m_interval[2],
							 m_interval[0] * i.m_interval[1] - m_interval[1] * i.m_interval[0]);
}

template <class T>
Interval<T> Interval_nD<T, 3>::magnitudeSquared() const
{
	return m_interval[0].square() + m_interval[1].square() + m_interval[2].square();
}

template <class T>
Interval<T> Interval_nD<T, 3>::magnitude() const
{
	Interval<T> magnitudeSq = magnitudeSquared();
	// Both minimum and maximum are guaranteed to be non-negative.
	return Interval<T>(sqrt(magnitudeSq.getMin()), sqrt(magnitudeSq.getMax()));
}

template <class T>
const Interval<T>& Interval_nD<T, 3>::getAxis(size_t i) const
{
	return m_interval[i];
}

// Utility functions not part of any class
template <typename T>
Interval<T> operator+ (T v, const Interval<T>& i)
{
	return i + v;
}

template <typename T>
Interval<T> operator* (T v, const Interval<T>& i)
{
	return i * v;
}

template <class T>
void IntervalArithmetic_add(const Interval<T>& a, const Interval<T>& b, Interval<T>* res)
{
	res->m_min = a.m_min + b.m_min;
	res->m_max = a.m_max + b.m_max;
}

template <class T>
void IntervalArithmetic_addadd(const Interval<T>& a, const Interval<T>& b, Interval<T>* res)
{
	res->m_min += a.m_min + b.m_min;
	res->m_max += a.m_max + b.m_max;
}

template <class T>
void IntervalArithmetic_sub(const Interval<T>& a, const Interval<T>& b, Interval<T>* res)
{
	res->m_min = a.m_min - b.m_max;
	res->m_max = a.m_max - b.m_min;
}

template <class T>
void IntervalArithmetic_addsub(const Interval<T>& a, const Interval<T>& b, Interval<T>* res)
{
	res->m_min += a.m_min - b.m_max;
	res->m_max += a.m_max - b.m_min;
}

template <class T>
void IntervalArithmetic_mul(const Interval<T>& a, const Interval<T>& b, Interval<T>* res)
{
	T min, max;
	minMax(a.m_min * b.m_min, a.m_min * b.m_max, a.m_max * b.m_min, a.m_max * b.m_max, &min, &max);
	res->m_min = min;
	res->m_max = max;
}

template <class T>
void IntervalArithmetic_addmul(const Interval<T>& a, const Interval<T>& b, Interval<T>* res)
{
	T min, max;
	minMax(a.m_min * b.m_min, a.m_min * b.m_max, a.m_max * b.m_min, a.m_max * b.m_max, &min, &max);
	res->m_min += min;
	res->m_max += max;
}

template <class T>
void IntervalArithmetic_submul(const Interval<T>& a, const Interval<T>& b, Interval<T>* res)
{
	T min, max;
	minMax(a.m_min * b.m_min, a.m_min * b.m_max, a.m_max * b.m_min, a.m_max * b.m_max, &min, &max);
	res->m_min -= max;
	res->m_max -= min;
}

// Interval nD functions
template <typename T>
std::ostream& operator<< (std::ostream& o, const Interval<T>& interval)
{
	o << "[" << interval.getMin() << "," << interval.getMax() << "]";
	return o;
}

// Interval nD functions
template <typename T, int N>
std::ostream& operator<< (std::ostream& o, const Interval_nD<T, N>& interval)
{
	o << "(" << interval.getAxis(0);
	for (int i = 1; i < N; ++i)
	{
		o << ";" << interval.getAxis(i);
	}
	o << ")";
	return o;
}

// Interval 3D functions
template <class T>
void IntervalArithmetic_add(const Interval_nD<T, 3>& a, const Interval_nD<T, 3>& b, Interval_nD<T, 3>* res)
{
	IntervalArithmetic_add(a.m_interval[0], b.m_interval[0], &(res->m_interval[0]));
	IntervalArithmetic_add(a.m_interval[1], b.m_interval[1], &(res->m_interval[1]));
	IntervalArithmetic_add(a.m_interval[2], b.m_interval[2], &(res->m_interval[2]));
}

template <class T>
void IntervalArithmetic_sub(const Interval_nD<T, 3>& a, const Interval_nD<T, 3>& b, Interval_nD<T, 3>* res)
{
	IntervalArithmetic_sub(a.m_interval[0], b.m_interval[0], &(res->m_interval[0]));
	IntervalArithmetic_sub(a.m_interval[1], b.m_interval[1], &(res->m_interval[1]));
	IntervalArithmetic_sub(a.m_interval[2], b.m_interval[2], &(res->m_interval[2]));
}

template <class T>
void IntervalArithmetic_crossProduct(const Interval_nD<T, 3>& a, const Interval_nD<T, 3>& b, Interval_nD<T, 3>* res)
{
	IntervalArithmetic_mul(a.m_interval[1], b.m_interval[2], &(res->m_interval[0]));
	IntervalArithmetic_submul(a.m_interval[2], b.m_interval[1], &(res->m_interval[0]));

	IntervalArithmetic_mul(a.m_interval[2], b.m_interval[0], &(res->m_interval[1]));
	IntervalArithmetic_submul(a.m_interval[0], b.m_interval[2], &(res->m_interval[1]));

	IntervalArithmetic_mul(a.m_interval[0], b.m_interval[1], &(res->m_interval[2]));
	IntervalArithmetic_submul(a.m_interval[1], b.m_interval[0], &(res->m_interval[2]));
}

template <class T>
void IntervalArithmetic_dotProduct(const Interval_nD<T, 3>& a, const Interval_nD<T, 3>& b, Interval<T>* res)
{
	IntervalArithmetic_mul(a.m_interval[0], b.m_interval[0], res);
	IntervalArithmetic_addmul(a.m_interval[1], b.m_interval[1], res);
	IntervalArithmetic_addmul(a.m_interval[2], b.m_interval[2], res);
}

}; // Math
}; // SurgSim

#endif // SURGSIM_MATH_INTERVALARITHMETIC_INL_H