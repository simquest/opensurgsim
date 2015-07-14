#ifndef SQ_POLYNOMIAL__H
#define SQ_POLYNOMIAL__H

#include <stdlib.h>
#include <ostream>

#include <iostream>

#include <algorithm>

template <typename T> class Interval;  // used in the PolynomialEvaluation arg/result types; MUST match the declaration in IntervalArithmetic.h.

namespace SQ {
	using std::min;
	using std::max;

	template <int N, typename T = double> class Polynomial;

	namespace {

		template <typename T>
		inline bool isNearZero(T value, T fuzz = 1e-9)
		{
			return (value + fuzz >= 0 && value - fuzz <= 0);
		}

	}

	// ======================================================================

	template <typename T>
	class PolynomialEvaluation
	{
	public:
		typedef T           scalarType;
		typedef T           scalarEvaluationResult;
	};

	template <typename T>
	class PolynomialEvaluation<Interval<T> >
	{
	public:
		typedef T           scalarType;
		typedef Interval<T> scalarEvaluationResult;
	};

	// ======================================================================

	// order-0 polynomial, also known as "a constant" =)
	template <typename T>
	class Polynomial<0, T> : public PolynomialEvaluation<T>
	{
	public:
		Polynomial() : m_a0(0) {};
		explicit Polynomial(T a0) : m_a0(a0) {};

		scalarEvaluationResult   evaluate(scalarType x) const   { return m_a0; }

		Polynomial operator- () const { return Polynomial(-m_a0); }

		// Note that this is, for now, just addition and subtraction for polynomials of the *same* degree.
		Polynomial operator+ (const Polynomial& rhs) const { return Polynomial(m_a0 + rhs.m_a0); }
		Polynomial operator- (const Polynomial& rhs) const { return Polynomial(m_a0 - rhs.m_a0); }
		Polynomial& operator+= (const Polynomial& rhs)     { m_a0 += rhs.m_a0; return *this; }
		Polynomial& operator-= (const Polynomial& rhs)     { m_a0 -= rhs.m_a0; return *this; }

		Polynomial<0,T> derivative() const { return Polynomial<0,T>(0); }

		bool isNearZero(T fuzz = 1e-9) const { return isNearZero(m_a0, fuzz); }

		T getCoefficient(int i) const
		{
			switch (i)
			{
			case 0:  return m_a0;
			default: return 0;
			}
		}

		void setCoefficient(int i, T value)
		{
			switch (i)
			{
			case 0:  m_a0 = value;  break;
			}
		}

	private:
		T m_a0;
	};

	// ======================================================================

	// order-1 polynomial (linear)
	template <typename T>
	class Polynomial<1, T> : public PolynomialEvaluation<T>
	{
	public:
		Polynomial() : m_a0(0), m_a1(0) {};
		Polynomial(T a0, T a1) : m_a0(a0), m_a1(a1) {};

		scalarEvaluationResult   evaluate(scalarType x) const   { return m_a1*x + m_a0; }

		Polynomial operator- () const { return Polynomial(-m_a0, -m_a1); }

		// Note that this is, for now, just addition and subtraction for polynomials of the *same* degree.
		Polynomial operator+ (const Polynomial& rhs) const { return Polynomial(m_a0 + rhs.m_a0, m_a1 + rhs.m_a1); }
		Polynomial operator- (const Polynomial& rhs) const { return Polynomial(m_a0 - rhs.m_a0, m_a1 - rhs.m_a1); }
		Polynomial& operator+= (const Polynomial& rhs)     { m_a0 += rhs.m_a0;  m_a1 += rhs.m_a1;  return *this; }
		Polynomial& operator-= (const Polynomial& rhs)     { m_a0 -= rhs.m_a0;  m_a1 -= rhs.m_a1;  return *this; }

		Polynomial<0,T> derivative() const { return Polynomial<0,T>(m_a1); }

		bool isNearZero(T fuzz = 1e-9) const { return isNearZero(m_a0, fuzz) && isNearZero(m_a1, fuzz); }

		T getCoefficient(int i) const
		{
			switch (i)
			{
			case 0:  return m_a0;
			case 1:  return m_a1;
			default: return 0;
			}
		}

		void setCoefficient(int i, T value)
		{
			switch (i)
			{
			case 0:  m_a0 = value;  break;
			case 1:  m_a1 = value;  break;
			}
		}

	private:
		T m_a0;
		T m_a1;
	};

	// ======================================================================

	// order-2 polynomial (quadratic)
	template <typename T>
	class Polynomial<2, T> : public PolynomialEvaluation<T>
	{
	public:
		Polynomial() : m_a0(0), m_a1(0), m_a2(0) {};
		Polynomial(T a0, T a1, T a2) : m_a0(a0), m_a1(a1), m_a2(a2) {};

		scalarEvaluationResult   evaluate(scalarType x) const   { return (m_a2*x + m_a1)*x + m_a0; }

		Polynomial operator- () const { return Polynomial(-m_a0, -m_a1, -m_a2); }

		// Note that this is, for now, just addition and subtraction for polynomials of the *same* degree.
		Polynomial operator+ (const Polynomial& rhs) const { return Polynomial(m_a0 + rhs.m_a0, m_a1 + rhs.m_a1, m_a2 + rhs.m_a2); }
		Polynomial operator- (const Polynomial& rhs) const { return Polynomial(m_a0 - rhs.m_a0, m_a1 - rhs.m_a1, m_a2 - rhs.m_a2); }
		Polynomial& operator+= (const Polynomial& rhs)     { m_a0 += rhs.m_a0;  m_a1 += rhs.m_a1;  m_a2 += rhs.m_a2;  return *this; }
		Polynomial& operator-= (const Polynomial& rhs)     { m_a0 -= rhs.m_a0;  m_a1 -= rhs.m_a1;  m_a2 -= rhs.m_a2;  return *this; }

		Polynomial<1,T> derivative() const { return Polynomial<1,T>(m_a1, 2*m_a2); }

		bool isNearZero(T fuzz = 1e-9) const { return isNearZero(m_a0, fuzz) && isNearZero(m_a1, fuzz) && isNearZero(m_a2, fuzz); }

		T getCoefficient(int i) const
		{
			switch (i)
			{
			case 0:  return m_a0;
			case 1:  return m_a1;
			case 2:  return m_a2;
			default: return 0;
			}
		}

		void setCoefficient(int i, T value)
		{
			switch (i)
			{
			case 0:  m_a0 = value;  break;
			case 1:  m_a1 = value;  break;
			case 2:  m_a2 = value;  break;
			}
		}

	private:
		T m_a0;
		T m_a1;
		T m_a2;
	};

	// ======================================================================

	// order-3 polynomial (cubic)
	template <typename T>
	class Polynomial<3, T> : public PolynomialEvaluation<T>
	{
	public:
		Polynomial() : m_a0(0), m_a1(0), m_a2(0), m_a3(0) {};
		Polynomial(T a0, T a1, T a2, T a3) : m_a0(a0), m_a1(a1), m_a2(a2), m_a3(a3) {};

		scalarEvaluationResult   evaluate(scalarType x) const   { return ((m_a3*x + m_a2)*x + m_a1)*x + m_a0; }

		Polynomial operator- () const { return Polynomial(-m_a0, -m_a1, -m_a2, -m_a3); }

		// Note that this is, for now, just addition and subtraction for polynomials of the *same* degree.
		Polynomial operator+ (const Polynomial& rhs) const { return Polynomial(m_a0 + rhs.m_a0, m_a1 + rhs.m_a1, m_a2 + rhs.m_a2, m_a3 + rhs.m_a3); }
		Polynomial operator- (const Polynomial& rhs) const { return Polynomial(m_a0 - rhs.m_a0, m_a1 - rhs.m_a1, m_a2 - rhs.m_a2, m_a3 - rhs.m_a3); }
		Polynomial& operator+= (const Polynomial& rhs)     { m_a0 += rhs.m_a0;  m_a1 += rhs.m_a1;  m_a2 += rhs.m_a2;  m_a3 += rhs.m_a3;  return *this; }
		Polynomial& operator-= (const Polynomial& rhs)     { m_a0 -= rhs.m_a0;  m_a1 -= rhs.m_a1;  m_a2 -= rhs.m_a2;  m_a3 -= rhs.m_a3;  return *this; }

		Polynomial<2,T> derivative() const { return Polynomial<2,T>(m_a1, 2*m_a2, 3*m_a3); }

		bool isNearZero(T fuzz = 1e-9) const { return isNearZero(m_a0, fuzz) && isNearZero(m_a1, fuzz) && isNearZero(m_a2, fuzz) && isNearZero(m_a3, fuzz); }

		T getCoefficient(int i) const
		{
			switch (i)
			{
			case 0:  return m_a0;
			case 1:  return m_a1;
			case 2:  return m_a2;
			case 3:  return m_a3;
			default: return 0;
			}
		}

		void setCoefficient(int i, T value)
		{
			switch (i)
			{
			case 0:  m_a0 = value;  break;
			case 1:  m_a1 = value;  break;
			case 2:  m_a2 = value;  break;
			case 3:  m_a3 = value;  break;
			}
		}

	private:
		T m_a0;
		T m_a1;
		T m_a2;
		T m_a3;
	};

	// ======================================================================

	// generic code for order-N polynomial that hasn't been specialized above
	// Not implemented because we haven't needed it yet...  --bert

	// ======================================================================

	// Operators

	template <int N, int M, typename T>
	Polynomial<N+M,T> operator*(const Polynomial<N,T>& p, const Polynomial<M,T>& q)
	{
//		Polynomial<N+M,T> result2;
// 		for (int i = 0;  i <= N;  ++i)
//		{
//			for (int j = 0;  j <= M;  ++j)
//			{
//				result2.setCoefficient(i+j, result2.getCoefficient(i+j) + p.getCoefficient(i)*q.getCoefficient(j));
//			}
//		}
		// This generic implementation is sloooooow under Visual Studio 2005.
		Polynomial<N+M,T> result;
		for (int i = 0;  i <= N+M;  ++i)
		{
			T coeff = 0;
			int jMin = max(0, i-M);
			int jMax = min(i, N);
			for (int j = jMin;  j <= jMax;  ++j)
			{
				//int k = i-j;
				//SQ_DEBUG_ASSERT((i-j) >= 0 && (i-j) <= M, "bad k index");
				coeff += p.getCoefficient(j)*q.getCoefficient(i-j);
			}
			result.setCoefficient(i, coeff);
		}
		return result;
	}

	template <typename T>
	Polynomial<2,T> operator*(const Polynomial<1,T>& p, const Polynomial<1,T>& q)
	{
		const T pb = p.getCoefficient(0);
		const T pa = p.getCoefficient(1);
		const T qb = q.getCoefficient(0);
		const T qa = q.getCoefficient(1);
		return Polynomial<2,T>(pb*qb, pb*qa+pa*qb, pa*qa);
	}

	template <typename T>
	Polynomial<3,T> operator*(const Polynomial<2,T>& p, const Polynomial<1,T>& q)
	{
		const T p0 = p.getCoefficient(0);
		const T p1 = p.getCoefficient(1);
		const T p2 = p.getCoefficient(2);
		const T q0 = q.getCoefficient(0);
		const T q1 = q.getCoefficient(1);
		return Polynomial<3,T>(p0*q0, p0*q1+p1*q0, p1*q1+p2*q0, p2*q1);
	}

	template <typename T>
	Polynomial<3,T> operator*(const Polynomial<1,T>& p, const Polynomial<2,T>& q)
	{
		const T p0 = p.getCoefficient(0);
		const T p1 = p.getCoefficient(1);
		const T q0 = q.getCoefficient(0);
		const T q1 = q.getCoefficient(1);
		const T q2 = q.getCoefficient(2);
		return Polynomial<3,T>(p0*q0, p0*q1+p1*q0, p0*q2+p1*q1, p1*q2);
	}

	template <typename T>
	Polynomial<4,T> operator*(const Polynomial<2,T>& p, const Polynomial<2,T>& q)
	{
		const T p0 = p.getCoefficient(0);
		const T p1 = p.getCoefficient(1);
		const T p2 = p.getCoefficient(2);
		const T q0 = q.getCoefficient(0);
		const T q1 = q.getCoefficient(1);
		const T q2 = q.getCoefficient(2);
		return Polynomial<4,T>(p0*q0, p0*q1+p1*q0, p0*q2+p1*q1+p2*q0, p1*q2+p2*q1, p2*q2);
	}

	// ======================================================================

	template <typename T>
	Polynomial<0,T> square(const Polynomial<0,T>& p)
	{
		const T c = p.getCoefficient(0);
		return Polynomial<0,T>(c*c);
	}

	template <typename T>
	Polynomial<2,T> square(const Polynomial<1,T>& p)
	{
		const T pb = p.getCoefficient(0);
		const T pa = p.getCoefficient(1);
		return Polynomial<2,T>(pb*pb, 2*pa*pb, pa*pa);
	}

	template <typename T>
	Polynomial<4,T> square(const Polynomial<2,T>& p)
	{
		const T p0 = p.getCoefficient(0);
		const T p1 = p.getCoefficient(1);
		const T p2 = p.getCoefficient(2);
		return Polynomial<4,T>(p0*p0, 2*p0*p1, 2*p0*p2+p1*p1, 2*p1*p2, p2*p2);
	}

	// ======================================================================

	template <int N, typename T>
	inline std::ostream& operator<<(std::ostream& stream, const Polynomial<N,T>& p)
	{
		stream << "(";
		for (int i = N;  i > 1;  --i)
			stream << p.getCoefficient(i) << "*x^" << i << " + ";
		if (N >= 1)
			stream << p.getCoefficient(1) << "*x + ";
		stream << p.getCoefficient(0) << "*1)";
		return stream;
	}
}

#endif // SQ_POLYNOMIAL__H
