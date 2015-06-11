#ifndef POLYNOMIAL_VALUES__H
#define POLYNOMIAL_VALUES__H

#include "Math/Algebra/Polynomial.h"
using SQ::Polynomial;
#include "Math/Algebra/PolynomialRoots.h"
using SQ::PolynomialRoots;

#include "IntervalArithmetic.h"

#include "Math/SqMinMax.h"
using SqMath::MinMax;

#define FORCE_INLINE __forceinline

namespace SQ
{
	template <int N, class T> class PolynomialValues;

	// ----------------------------------------------------------------------

	template <class T>
	class PolynomialValues<0,T>
	{
	public:
		PolynomialValues(const Polynomial<0,T>& p) : m_polynomial(p) {}

		const Polynomial<0,T>& getPolynomial() const { return m_polynomial; }

		template <class T>	
		FORCE_INLINE Interval<T> valuesOverInterval(const Interval<T>& interval) const
		{
			// There can be no extremum, it's a constant. =)
			return Interval<T>::minToMax(m_polynomial.evaluate(0));
		}

	private:
		Polynomial<0,T> m_polynomial;
	};

	// ----------------------------------------------------------------------

	template <class T>
	class PolynomialValues<1,T>
	{
	public:
		PolynomialValues(const Polynomial<1,T>& p) : m_polynomial(p) {}

		const Polynomial<1,T>& getPolynomial() const { return m_polynomial; }

		template <class T>	
		FORCE_INLINE Interval<T> valuesOverInterval(const Interval<T>& interval) const
		{
			// There can be no extremum, it's a linear expression. =)
			return Interval<T>::minToMax(m_polynomial.evaluate(interval.getMin()), m_polynomial.evaluate(interval.getMax()));
		}

	private:
		Polynomial<1,T> m_polynomial;
	};

	// ----------------------------------------------------------------------

	template <class T>
	class PolynomialValues<2,T>
	{
	public:
		PolynomialValues(const Polynomial<2,T>& p)
			: m_polynomial(p), m_derivative(m_polynomial.derivative()), m_locationOfExtremum(m_derivative)
		{}

		const Polynomial<2,T>&      getPolynomial() const         { return m_polynomial; }
		const Polynomial<1,T>&      getDerivative() const         { return m_derivative; }
		const PolynomialRoots<1,T>& getLocationsOfExtrema() const { return m_locationOfExtremum; }

		template <class T>	
		FORCE_INLINE Interval<T> valuesOverInterval(const Interval<T>& interval) const
		{
			// Always consider the endpoints.
			Interval<T> result = Interval<T>::minToMax(m_polynomial.evaluate(interval.getMin()), m_polynomial.evaluate(interval.getMax()));

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

	private:
		Polynomial<2,T>      m_polynomial;
		Polynomial<1,T>      m_derivative;
		PolynomialRoots<1,T> m_locationOfExtremum;
	};

	// ----------------------------------------------------------------------

	template <int N, class T>
	class PolynomialValues<N,T>
	{
	public:
		PolynomialValues(const Polynomial<N,T>& p)
			: m_polynomial(p), m_derivative(m_polynomial.derivative()), m_locationsOfExtrema(m_derivative)
		{}

		const Polynomial<N,T>&        getPolynomial() const         { return m_polynomial; }
		const Polynomial<N-1,T>&      getDerivative() const         { return m_derivative; }
		const PolynomialRoots<N-1,T>& getLocationsOfExtrema() const { return m_locationsOfExtrema; }

		template <class T>	
		FORCE_INLINE Interval<T> valuesOverInterval(const Interval<T>& interval) const
		{
			// Always consider the endpoints.
			Interval<T> result = Interval<T>::minToMax(m_polynomial.evaluate(interval.getMin()), m_polynomial.evaluate(interval.getMax()));

			for (int i = 0;  i < m_locationsOfExtrema.getNumRoots();  ++i)
			{
				// Only consider the extrema that lie within the interval!
				if (interval.contains(m_locationsOfExtrema[i]))
				{
					// The value at this extremum needs to be made a part of the result interval.
					result.extendToInclude(m_polynomial.evaluate(m_locationsOfExtrema[i]));
				}
			}
			return result;
		}

	private:
		Polynomial<N,T>        m_polynomial;
		Polynomial<N-1,T>      m_derivative;
		PolynomialRoots<N-1,T> m_locationsOfExtrema;
	};

	// ----------------------------------------------------------------------


	template <int N, class T>	
	FORCE_INLINE Interval<T> valuesOverInterval(const Polynomial<N,T>& p, const Interval<T>& interval) 
	{
		return PolynomialValues<N,T>(p).valuesOverInterval(interval);
	}
}


#undef FORCE_INLINE
#endif // POLYNOMIAL_VALUES__H
