#ifndef SQ_POLYNOMIAL_ROOTS__H
#define SQ_POLYNOMIAL_ROOTS__H

#include "Math/Algebra/Polynomial.h"
#include <SqAssert.h>

#include <iostream>

#include <algorithm>

namespace SQ {
	/// The (algebraic) roots of a Polynomial<N,T>.
	//  If all coefficients of a polynomial are 0, the roots are degenerate: the polynomial equals 0 for any x.
	//  Otherwise, there may be anywhere between 0 and N roots.
	template <int N, typename T = double> class PolynomialRoots;

	/// The common base class for PolynomialRoots specializations for various N.
	//  It allows us to write a bunch of this code once, rather than copying it into each and every specialized class.
	template <int N, typename T = double>
	class PolynomialRootsCommon
	{
	public:
		// The constructor doesn't really initialize anything... the derived class will need to do that.
		PolynomialRootsCommon() {};

	private:
		// Prohibit copying and assignment.
		PolynomialRootsCommon(const PolynomialRootsCommon&);
		PolynomialRootsCommon& operator=(const PolynomialRootsCommon&);

	public:

		bool isDegenerate() const { return m_numData == DEGENERATE; }

		//int getNumRoots() const  { return((m_numData < 0) ? 0 : m_numData); }
		int getNumRoots() const  { return m_numData; }

		// Only allow read access
		T operator[](int i) const { SQ_DEBUG_ASSERT(i >= 0 && i < m_numData, "index");  return m_data[i]; }

	protected:
		/// If m_numData is set to this value, the solution is degenerate.
		//  The value is < 0, so it can safely be returned from getNumRoots() and used for loops etc.
		static const int DEGENERATE = -1;

	protected:
		int m_numData;
		T   m_data[N];
	};

	// ======================================================================

	// roots of an order-1 polynomial (linear)
	template <typename T>
	class PolynomialRoots<1, T> : public PolynomialRootsCommon<1, T>
	{
	public:
		explicit PolynomialRoots(const Polynomial<1,T>& p)
		{
			solve(p.getCoefficient(1), p.getCoefficient(0), 1e-9,
				m_numData, m_data);
		}

		static void solve(T a, T b, T fuzz, int& numRoots, T* roots)
		{
			if (isNearZero(a, fuzz))
			{
				// Oops, our "1-st degree polynomial" is really close to a constant.
				// If the constant is zero, there are infinitely many solutions; otherwise there are zero.
				// NB: should we normalize the coefficients before the checks on a and b?
				if (isNearZero(b, fuzz))
				{
					numRoots = DEGENERATE;  // infinitely many solutions
				}
				else
				{
					numRoots = 0;
				}
			}
			else
			{
				numRoots = 1;
				roots[0] = -b/a;
			}
		}

	};

	// ======================================================================

	// roots of an order-2 polynomial (quadratic)
	template <typename T>
	class PolynomialRoots<2, T> : public PolynomialRootsCommon<2, T>
	{
	public:
		explicit PolynomialRoots(const Polynomial<2,T>& p)
		{
			solve(p.getCoefficient(2), p.getCoefficient(1), p.getCoefficient(0), 1e-9,
				m_numData, m_data);
		}

		static void solve(T a, T b, T c, T fuzz, int& numRoots, T* roots)
		{
			if (isNearZero(a, fuzz))
			{
				// Oops, our "2nd degree polynomial" is really (close to) 1st degree or less.
				// We can delegate the actual solving in this case.
				// NB: should we normalize the coefficients before the check on a?
				PolynomialRoots<1,T>::solve(b, c, fuzz, numRoots, roots);
				return;
			}

			T discriminant = b*b - 4.*a*c;
			if (discriminant > 0.)
			{
				numRoots = 2;
				T sqrtDiscriminant = sqrt(discriminant);
				roots[0] = (-b - sqrtDiscriminant) / (2*a);
				roots[1] = (-b + sqrtDiscriminant) / (2*a);
			}
			else if (discriminant > -fuzz)
			{
				// exactly equal to 0 is statistically unlikely unless b and c are identically zero, but we may as well be anal-retentive =)
				// NB: should this check be normalized in some way?
				numRoots = 1;
				roots[0] = -b / (2*a);
			}
			else
			{
				numRoots = 0;
			}
		}
	};

	// ======================================================================

	// roots of an order-3 polynomial (cubic)
	// Not implemented yet...  --bert

	// ======================================================================

}

#endif // SQ_POLYNOMIAL_ROOTS__H
