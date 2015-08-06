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

#ifndef SURGSIM_MATH_POLYNOMIALVALUES_INL_H
#define SURGSIM_MATH_POLYNOMIALVALUES_INL_H

namespace SurgSim
{
namespace Math
{

template <class T>
PolynomialValues<T, 0>::PolynomialValues(const Polynomial<T, 0>& p) : m_polynomial(p)
{
}

template <class T>
const Polynomial<T, 0>& PolynomialValues<T, 0>::getPolynomial() const
{
	return m_polynomial;
}

template <class T>
Interval<T> PolynomialValues<T, 0>::valuesOverInterval(const Interval<T>&) const
{
	return Interval<T>(m_polynomial.evaluate(0), m_polynomial.evaluate(0));
}

template <class T>
PolynomialValues<T, 1>::PolynomialValues(const Polynomial<T, 1>& p) : m_polynomial(p) {}

template <class T>
const Polynomial<T, 1>& PolynomialValues<T, 1>::getPolynomial() const
{
	return m_polynomial;
}

template <class T>
Interval<T> PolynomialValues<T, 1>::valuesOverInterval(const Interval<T>& interval) const
{
	return Interval<T>::minToMax(m_polynomial.evaluate(interval.getMin()),
								 m_polynomial.evaluate(interval.getMax()));
}

template <class T>
PolynomialValues<T, 2>::PolynomialValues(const Polynomial<T, 2>& p) : m_polynomial(p),
	m_derivative(m_polynomial.derivative()),
	m_locationOfExtremum(m_derivative)
{
}

template <class T>
const Polynomial<T, 2>& PolynomialValues<T, 2>::getPolynomial() const
{
	return m_polynomial;
}

template <class T>
const Polynomial<T, 1>& PolynomialValues<T, 2>::getDerivative() const
{
	return m_derivative;
}

template <class T>
const PolynomialRoots<T, 1>& PolynomialValues<T, 2>::getLocationsOfExtrema() const
{
	return m_locationOfExtremum;
}

template <class T>
Interval<T> PolynomialValues<T, 2>::valuesOverInterval(const Interval<T>& interval) const
{
	// Always consider the endpoints.
	Interval<T> result = Interval<T>::minToMax(m_polynomial.evaluate(interval.getMin()),
						 m_polynomial.evaluate(interval.getMax()));

	if (m_locationOfExtremum.getNumRoots() > 0)
	{
		// There is an extremum (min or max)...
		if (interval.contains(m_locationOfExtremum[0]))
		{
			//...and it occurs somewhere in the middle of the interval.
			// The value at the extremum needs to be made a part of the result interval.
			result.extendToInclude(m_polynomial.evaluate(m_locationOfExtremum[0]));
		}
	}
	return result;
}

template <class T, int N>
Interval<T> valuesOverInterval(const Polynomial<T, N>& p, const Interval<T>& interval)
{
	return PolynomialValues<T, N>(p).valuesOverInterval(interval);
}

}; // Math
}; // SurgSim

#endif // SURGSIM_MATH_POLYNOMIALVALUES_INL_H
