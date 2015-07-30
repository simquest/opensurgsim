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

#ifndef SURGSIM_MATH_POLYNOMIAL_INL_H
#define SURGSIM_MATH_POLYNOMIAL_INL_H

#include <stdlib.h>
#include <algorithm>

#include "SurgSim/Math/IntervalArithmetic.h"

namespace SurgSim
{
namespace Math
{

template <typename T>
bool isNearZero(const T& value, const T& epsilon)
{
	return (value + epsilon >= 0 && value - epsilon <= 0);
}

// Polynomial of degree 0

template <class T>
Polynomial<T, 0>::Polynomial() : m_a0(static_cast<T>(0))
{
}

template <class T>
Polynomial<T, 0>::Polynomial(const T& a0) : m_a0(a0)
{
}

template <class T>
T Polynomial<T, 0>::evaluate(const T& x) const
{
	return m_a0;
}

template <class T>
T& Polynomial<T, 0>::operator[](const size_t i)
{
	return const_cast<T&>(const_cast<const Polynomial<T, 0>*>(this)->operator[](i));
}

template <class T>
const T& Polynomial<T, 0>::operator[](const size_t i) const
{
	SURGSIM_ASSERT(i <= 0) << "Attempting to set a coefficient greater than the polynomial degree";
	return m_a0;
}

template <class T>
Polynomial<T, 0> Polynomial<T, 0>::operator- () const
{
	return Polynomial(-m_a0);
}

template <class T>
Polynomial<T, 0> Polynomial<T, 0>::operator+ (const Polynomial<T, 0>& rhs) const
{
	return Polynomial(m_a0 + rhs.m_a0);
}

template <class T>
Polynomial<T, 0>& Polynomial<T, 0>::operator+= (const Polynomial<T, 0>& rhs)
{
	m_a0 += rhs.m_a0;
	return *this;
}

template <class T>
Polynomial<T, 0> Polynomial<T, 0>::operator- (const Polynomial<T, 0>& rhs) const
{
	return Polynomial(m_a0 - rhs.m_a0);
}

template <class T>
Polynomial<T, 0>& Polynomial<T, 0>::operator-= (const Polynomial<T, 0>& rhs)
{
	m_a0 -= rhs.m_a0;
	return *this;
}

template <class T>
bool Polynomial<T, 0>::isNearZero(const T& epsilon) const
{
	return SurgSim::Math::isNearZero(m_a0, epsilon);
}

template <class T>
bool Polynomial<T, 0>::isApprox(const Polynomial<T, 0>& p, const T& epsilon) const
{
	return ((*this) - p).isNearZero(epsilon);
}

template <class T>
T Polynomial<T, 0>::getCoefficient(size_t i) const
{
	switch (i)
	{
		case 0:
			return m_a0;
		default:
			return 0;
	}
}

template <class T>
void Polynomial<T, 0>::setCoefficient(size_t i, const T& value)
{
	SURGSIM_ASSERT(i <= 0) << "Attempting to set a coefficient greater than the polynomial degree";
	m_a0 = value;
}

// Polynomial of degree 1

template <class T>
Polynomial<T, 1>::Polynomial() : m_a0(static_cast<T>(0)), m_a1(static_cast<T>(0))
{
}

template <class T>
Polynomial<T, 1>::Polynomial(const T& a0, const T& a1) : m_a0(a0), m_a1(a1)
{
}

template <class T>
T Polynomial<T, 1>::evaluate(const T& x) const
{
	return m_a1 * x + m_a0;
}

template <class T>
T& Polynomial<T, 1>::operator[](const size_t i)
{
	return const_cast<T&>(const_cast<const Polynomial<T, 1>*>(this)->operator[](i));
}

template <class T>
const T& Polynomial<T, 1>::operator[](const size_t i) const
{
	SURGSIM_ASSERT(i <= 1) << "Attempting to set a coefficient greater than the polynomial degree";
	switch (i)
	{
		case 0:
		{
			return m_a0;
		}
		default:
		{
			return m_a1;
		}
	}
}

template <class T>
Polynomial<T, 1> Polynomial<T, 1>::operator- () const
{
	return Polynomial(-m_a0, -m_a1);
}

template <class T>
Polynomial<T, 1> Polynomial<T, 1>::operator+ (const Polynomial<T, 1>& rhs) const
{
	return Polynomial(m_a0 + rhs.m_a0, m_a1 + rhs.m_a1);
}

template <class T>
Polynomial<T, 1>& Polynomial<T, 1>::operator+= (const Polynomial<T, 1>& rhs)
{
	m_a0 += rhs.m_a0;
	m_a1 += rhs.m_a1;
	return *this;
}

template <class T>
Polynomial<T, 1> Polynomial<T, 1>::operator- (const Polynomial<T, 1>& rhs) const
{
	return Polynomial(m_a0 - rhs.m_a0, m_a1 - rhs.m_a1);
}

template <class T>
Polynomial<T, 1>& Polynomial<T, 1>::operator-= (const Polynomial<T, 1>& rhs)
{
	m_a0 -= rhs.m_a0;
	m_a1 -= rhs.m_a1;
	return *this;
}

template <class T>
Polynomial<T, 0> Polynomial<T, 1>::derivative() const
{
	return Polynomial<T, 0>(m_a1);
}

template <class T>
bool Polynomial<T, 1>::isNearZero(const T& epsilon) const
{
	return SurgSim::Math::isNearZero(m_a0, epsilon) && SurgSim::Math::isNearZero(m_a1, epsilon);
}

template <class T>
bool Polynomial<T, 1>::isApprox(const Polynomial<T, 1>& p, const T& epsilon) const
{
	return ((*this) - p).isNearZero(epsilon);
}

template <class T>
T Polynomial<T, 1>::getCoefficient(size_t i) const
{
	switch (i)
	{
		case 0:
		{
			return m_a0;
		}
		case 1:
		{
			return m_a1;
		}
		default:
		{
			return 0;
		}
	}
}

template <class T>
void Polynomial<T, 1>::setCoefficient(size_t i, const T& value)
{
	SURGSIM_ASSERT(i <= 1) << "Attempting to set a coefficient greater than the polynomial degree";
	switch (i)
	{
		case 0:
		{
			m_a0 = value;
			break;
		}
		case 1:
		{
			m_a1 = value;
			break;
		}
	}
}

// Polynomial of degree 2

template <class T>
Polynomial<T, 2>::Polynomial() : m_a0(static_cast<T>(0)), m_a1(static_cast<T>(0)), m_a2(static_cast<T>(0))
{
}

template <class T>
Polynomial<T, 2>::Polynomial(const T& a0, const T& a1, const T& a2) : m_a0(a0), m_a1(a1), m_a2(a2)
{
}

template <class T>
T Polynomial<T, 2>::evaluate(const T& x) const
{
	return (m_a2 * x + m_a1) * x + m_a0;
}

template <class T>
T& Polynomial<T, 2>::operator[](const size_t i)
{
	return const_cast<T&>(const_cast<const Polynomial<T, 2>*>(this)->operator[](i));
}

template <class T>
const T& Polynomial<T, 2>::operator[](const size_t i) const
{
	SURGSIM_ASSERT(i <= 2) << "Attempting to set a coefficient greater than the polynomial degree";
	switch (i)
	{
		case 0:
		{
			return m_a0;
		}
		case 1:
		{
			return m_a1;
		}
		default:
		{
			return m_a2;
		}
	}
}

template <class T>
Polynomial<T, 2> Polynomial<T, 2>::operator- () const
{
	return Polynomial(-m_a0, -m_a1, -m_a2);
}

template <class T>
Polynomial<T, 2> Polynomial<T, 2>::operator+ (const Polynomial<T, 2>& rhs) const
{
	return Polynomial(m_a0 + rhs.m_a0, m_a1 + rhs.m_a1, m_a2 + rhs.m_a2);
}

template <class T>
Polynomial<T, 2>& Polynomial<T, 2>::operator+= (const Polynomial<T, 2>& rhs)
{
	m_a0 += rhs.m_a0;
	m_a1 += rhs.m_a1;
	m_a2 += rhs.m_a2;
	return *this;
}

template <class T>
Polynomial<T, 2> Polynomial<T, 2>::operator- (const Polynomial<T, 2>& rhs) const
{
	return Polynomial(m_a0 - rhs.m_a0, m_a1 - rhs.m_a1, m_a2 - rhs.m_a2);
}

template <class T>
Polynomial<T, 2>& Polynomial<T, 2>::operator-= (const Polynomial<T, 2>& rhs)
{
	m_a0 -= rhs.m_a0;
	m_a1 -= rhs.m_a1;
	m_a2 -= rhs.m_a2;
	return *this;
}

template <class T>
Polynomial<T, 1> Polynomial<T, 2>::derivative() const
{
	return Polynomial<T, 1>(m_a1, 2 * m_a2);
}

template <class T>
bool Polynomial<T, 2>::isNearZero(const T& epsilon) const
{
	return SurgSim::Math::isNearZero(m_a0, epsilon) &&
		   SurgSim::Math::isNearZero(m_a1, epsilon) &&
		   SurgSim::Math::isNearZero(m_a2, epsilon);
}

template <class T>
bool Polynomial<T, 2>::isApprox(const Polynomial<T, 2>& p, const T& epsilon) const
{
	return ((*this) - p).isNearZero(epsilon);
}

template <class T>
T Polynomial<T, 2>::getCoefficient(size_t i) const
{
	switch (i)
	{
		case 0:
		{
			return m_a0;
		}
		case 1:
		{
			return m_a1;
		}
		case 2:
		{
			return m_a2;
		}
		default:
		{
			return 0;
		}
	}
}

template <class T>
void Polynomial<T, 2>::setCoefficient(size_t i, const T& value)
{
	SURGSIM_ASSERT(i <= 2) << "Attempting to set a coefficient greater than the polynomial degree";
	switch (i)
	{
		case 0:
		{
			m_a0 = value;
			break;
		}
		case 1:
		{
			m_a1 = value;
			break;
		}
		case 2:
		{
			m_a2 = value;
			break;
		}
	}
}

// Polynomial of degree 3

template <class T>
Polynomial<T, 3>::Polynomial() :
	m_a0(static_cast<T>(0)),
	m_a1(static_cast<T>(0)),
	m_a2(static_cast<T>(0)),
	m_a3(static_cast<T>(0))
{
}

template <class T>
Polynomial<T, 3>::Polynomial(const T& a0, const T& a1, const T& a2, const T& a3) :
	m_a0(a0),
	m_a1(a1),
	m_a2(a2),
	m_a3(a3)
{
}

template <class T>
T Polynomial<T, 3>::evaluate(const T& x) const
{
	return ((m_a3 * x + m_a2) * x + m_a1) * x + m_a0;
}

template <class T>
T& Polynomial<T, 3>::operator[](const size_t i)
{
	return const_cast<T&>(const_cast<const Polynomial<T, 3>*>(this)->operator[](i));
}

template <class T>
const T& Polynomial<T, 3>::operator[](const size_t i) const
{
	SURGSIM_ASSERT(i <= 3) << "Attempting to set or access a coefficient greater than the polynomial degree";
	switch (i)
	{
		case 0:
		{
			return m_a0;
		}
		case 1:
		{
			return m_a1;
		}
		case 2:
		{
			return m_a2;
		}
		default:
		{
			return m_a3;
		}
	}
}

template <class T>
Polynomial<T, 3> Polynomial<T, 3>::operator- () const
{
	return Polynomial(-m_a0, -m_a1, -m_a2, -m_a3);
}

template <class T>
Polynomial<T, 3> Polynomial<T, 3>::operator+ (const Polynomial<T, 3>& rhs) const
{
	return Polynomial(m_a0 + rhs.m_a0, m_a1 + rhs.m_a1, m_a2 + rhs.m_a2, m_a3 + rhs.m_a3);
}

template <class T>
Polynomial<T, 3>& Polynomial<T, 3>::operator+= (const Polynomial<T, 3>& rhs)
{
	m_a0 += rhs.m_a0;
	m_a1 += rhs.m_a1;
	m_a2 += rhs.m_a2;
	m_a3 += rhs.m_a3;
	return *this;
}

template <class T>
Polynomial<T, 3> Polynomial<T, 3>::operator- (const Polynomial<T, 3>& rhs) const
{
	return Polynomial(m_a0 - rhs.m_a0, m_a1 - rhs.m_a1, m_a2 - rhs.m_a2, m_a3 - rhs.m_a3);
}

template <class T>
Polynomial<T, 3>& Polynomial<T, 3>::operator-= (const Polynomial<T, 3>& rhs)
{
	m_a0 -= rhs.m_a0;
	m_a1 -= rhs.m_a1;
	m_a2 -= rhs.m_a2;
	m_a3 -= rhs.m_a3;
	return *this;
}

template <class T>
Polynomial<T, 2> Polynomial<T, 3>::derivative() const
{
	return Polynomial<T, 2>(m_a1, 2 * m_a2, 3 * m_a3);
}

template <class T>
bool Polynomial<T, 3>::isNearZero(const T& epsilon) const
{
	return SurgSim::Math::isNearZero(m_a0, epsilon) &&
		   SurgSim::Math::isNearZero(m_a1, epsilon) &&
		   SurgSim::Math::isNearZero(m_a2, epsilon) &&
		   SurgSim::Math::isNearZero(m_a3, epsilon);
}

template <class T>
bool Polynomial<T, 3>::isApprox(const Polynomial<T, 3>& p, const T& epsilon) const
{
	return ((*this) - p).isNearZero(epsilon);
}

template <class T>
T Polynomial<T, 3>::getCoefficient(size_t i) const
{
	switch (i)
	{
		case 0:
		{
			return m_a0;
		}
		case 1:
		{
			return m_a1;
		}
		case 2:
		{
			return m_a2;
		}
		case 3:
		{
			return m_a3;
		}
		default:
		{
			return 0;
		}
	}
}

template <class T>
void Polynomial<T, 3>::setCoefficient(size_t i, const T& value)
{
	SURGSIM_ASSERT(i <= 3) << "Attempting to set a coefficient greater than the polynomial degree";
	switch (i)
	{
		case 0:
		{
			m_a0 = value;
			break;
		}
		case 1:
		{
			m_a1 = value;
			break;
		}
		case 2:
		{
			m_a2 = value;
			break;
		}
		case 3:
		{
			m_a3 = value;
			break;
		}
	}
}

// Operators

template <typename T, int N, int M>
Polynomial < T, N + M > operator*(const Polynomial<T, N>& p, const Polynomial<T, M>& q)
{
	Polynomial < T, N + M > result;
	for (int i = 0;  i <= N + M;  ++i)
	{
		T coeff = 0;
		int jMin = std::max(0, i - M);
		int jMax = std::min(i, N);
		for (int j = jMin;  j <= jMax; ++j)
		{
			coeff += p.getCoefficient(j) * q.getCoefficient(i - j);
		}
		result.setCoefficient(i, coeff);
	}
	return result;
}

template <typename T>
Polynomial<T, 2> operator*(const Polynomial<T, 1>& p, const Polynomial<T, 1>& q)
{
	const T p0 = p.getCoefficient(0);
	const T p1 = p.getCoefficient(1);
	const T q0 = q.getCoefficient(0);
	const T q1 = q.getCoefficient(1);
	return Polynomial<T, 2>(p0 * q0, p0 * q1 + p1 * q0, p1 * q1);
}

template <typename T>
Polynomial<T, 3> operator*(const Polynomial<T, 2>& p, const Polynomial<T, 1>& q)
{
	const T p0 = p.getCoefficient(0);
	const T p1 = p.getCoefficient(1);
	const T p2 = p.getCoefficient(2);
	const T q0 = q.getCoefficient(0);
	const T q1 = q.getCoefficient(1);
	return Polynomial<T, 3>(p0 * q0, p0 * q1 + p1 * q0, p1 * q1 + p2 * q0, p2 * q1);
}

template <typename T>
Polynomial<T, 3> operator*(const Polynomial<T, 1>& p, const Polynomial<T, 2>& q)
{
	const T p0 = p.getCoefficient(0);
	const T p1 = p.getCoefficient(1);
	const T q0 = q.getCoefficient(0);
	const T q1 = q.getCoefficient(1);
	const T q2 = q.getCoefficient(2);
	return Polynomial<T, 3>(p0 * q0, p0 * q1 + p1 * q0, p0 * q2 + p1 * q1, p1 * q2);
}

template <typename T>
Polynomial<T, 0> square(const Polynomial<T, 0>& p)
{
	const T c = p.getCoefficient(0);
	return Polynomial<T, 0>(c * c);
}

template <typename T>
Polynomial<T, 2> square(const Polynomial<T, 1>& p)
{
	const T p0 = p.getCoefficient(0);
	const T p1 = p.getCoefficient(1);
	return Polynomial<T, 2>(p0 * p0, 2 * p1 * p0, p1 * p1);
}

template <typename T, int N>
inline std::ostream& operator<<(std::ostream& stream, const Polynomial<T, N>& p)
{
	stream << "(";
	for (int i = N; i > 1; --i)
	{
		stream << p.getCoefficient(i) << "*x^" << i << " + ";
	}
	if (N >= 1)
	{
		stream << p.getCoefficient(1) << "*x + ";
	}
	stream << p.getCoefficient(0) << ")";
	return stream;
}

}; // Math
}; // SurgSim

#endif // SURGSIM_MATH_POLYNOMIAL_INL_H
