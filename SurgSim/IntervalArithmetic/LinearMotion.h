#ifndef LINEAR_MOTION__H
#define LINEAR_MOTION__H

#include "Math/Algebra/Polynomial.h"
using SQ::Polynomial;
#include "Math/Algebra/PolynomialRoots.h"
using SQ::PolynomialRoots;
#include "Math/Algebra/SqVec3.h"

#include "IntervalArithmetic.h"
#include "PolynomialValues.h"

#include <ostream>
using std::ostream;

#define FORCE_INLINE __forceinline


/*! LinearMotion is (intentionally) a lot like Interval, but it deals with linear motion where all operands start and end their motion simultaneously.
 *
 * LinearMotion results in much tighter bounds compared to Interval, since Interval must consider *any* value of each operand,
 * and LinearMotion only considers values that are synchronous with one another.
 *
 * The bounds of a LinearMotion are a start value and an end value; there's no requirement that start <= end (or vice versa).
 *
 * Many operations on LinearMotion arguments (*, /) return results that are not linear in time, so those operations will return Interval instead.
 */
template <class T>
class LinearMotion
{
private:
  T m_start;
  T m_end;

public:
  LinearMotion<T>() : m_start((T)0), m_end((T)0) {};
  LinearMotion<T>(T start, T end) : m_start(start), m_end(end) {};
  LinearMotion<T>(const LinearMotion<T> &m) : m_start(m.m_start), m_end(m.m_end) {};

  LinearMotion<T>& operator= (const LinearMotion<T> &m) { m_start = m.m_start;  m_end = m.m_end;  return *this; };

  Interval<T> toInterval() const
  {
	  return Interval<T>::minToMax(m_start, m_end);
  }

  // Returns a linear expression (degree-1 polynomial) whose value for t=0..1 progress from `start' to `end'.
  Polynomial<1,T> toPolynomial() const
  {
	  // The 1st argument is the constant term; the 2nd argument is the linear term.
	  return Polynomial<1,T>(m_start, m_end-m_start);
  }

  // Returns true if the linear motion crosses through 0.
  bool containsZero() const { return !((m_start<(T)0 && m_end<(T)0) || (m_start>(T)0 && m_end>(T)0)); };

  // You probably should not use the exact comparison operators. =)
  bool operator== (const LinearMotion<T>& m) const { return ((m_start == m.m_start) && (m_end == m.m_end)); };
  bool operator!= (const LinearMotion<T>& m) const { return !(*this == m); };

  LinearMotion<T>  operator+  (const LinearMotion<T>& m) const { return LinearMotion<T>(m_start + m.m_start, m_end + m.m_end); };
  LinearMotion<T>& operator+= (const LinearMotion<T>& m) { m_start += m.m_start;  m_end += m.m_end;  return *this; };

  LinearMotion<T>  operator-  (const LinearMotion<T>& m) const { return LinearMotion<T>(m_start - m.m_start, m_end - m.m_end); };
  LinearMotion<T>& operator-= (const LinearMotion<T>& m) { m_start -= m.m_start;  m_end -= m.m_end;  return *this; };

private:
  // Deprecated and removed from public API; should go away in the future.
  Interval<T> operator* (const LinearMotion<T>& m) const { return this->toInterval() * m.toInterval(); };
  Interval<T> operator/ (const LinearMotion<T>& m) const { return this->toInterval() / m.toInterval(); };

public:

  T getStart() const { return m_start; }
  T getEnd() const   { return m_end; }
  T atTime(T t) const { return ((1-t)*m_start + t*m_end); }

  LinearMotion<T> firstHalf() const  { return LinearMotion<T>(m_start, (m_start+m_end)*(T)0.5); }
  LinearMotion<T> secondHalf() const { return LinearMotion<T>((m_start+m_end)*(T)0.5, m_end); }
};


template <class T, int N>
class LinearMotion_nD
{
private:
  Interval<T> m_x[N];

public:
  LinearMotion_nD<T,N>() {};
  explicit LinearMotion_nD<T,N>(const LinearMotion<T>* x) { for (int i = 0;  i < N;  ++i) { m_x[i] = x[i]; } };
  LinearMotion_nD<T,N>(const LinearMotion_nD<T,N>& motion) { for (int i = 0;  i < N;  ++i) { m_x[i] = motion.x[i]; } };
  LinearMotion_nD<T,N>(const T* a, const T* b) { for (int i = 0;  i < N;  ++i)  m_x[i] = LinearMotion<T>(a[i], b[i]); };

  LinearMotion_nD<T,N>& operator= (const LinearMotion_nD<T,N>& motion) { for (int i = 0;  i < N;  ++i) { m_x[i] = motion.m_x[i]; }  return *this; };

  Interval_nD<T,N> toInterval() const
  {
	  Interval<T> motions[N];
	  for (int i = 0;  i < N;  ++i)
		  motions[i] = m_x[i].toInterval();
	  return Interval_nD<T,N>(motions);
  }

  // You probably should not use the exact comparison operators. =)
  bool operator== (const LinearMotion_nD<T,N>& motion) const { for (int i = 0;  i < N;  ++i) { if (m_x[i] != motion.m_x[i]) return false; } return true; };
  bool operator!= (const LinearMotion_nD<T,N>& motion) const { return !(*this == motion); };

  LinearMotion_nD<T,N>& operator+= (const LinearMotion_nD<T,N> &i) { for (int i = 0;  i < N;  ++i) { m_x[i] += i.m_x[i]; }  return *this; };
  LinearMotion_nD<T,N>  operator+  (const LinearMotion_nD<T,N> &i) const { Interval_nD<T,N> ret(*this);  ret += i;  return ret; }

  LinearMotion_nD<T,N>& operator-= (const LinearMotion_nD<T,N> &i) { for (int i = 0;  i < N;  ++i) { m_x[i] -= i.m_x[i]; }  return *this; };
  LinearMotion_nD<T,N>  operator-  (const LinearMotion_nD<T,N> &i) const { Interval_nD<T,N> ret(*this);  ret -= i;  return ret; }

  Interval_nD<T,N> operator* (const LinearMotion_nD<T,N> &i) const { return this->toInterval() * i.toInterval(); };
  Interval_nD<T,N> operator/ (const LinearMotion_nD<T,N> &i) const { return this->toInterval() / i.toInterval(); };

  FORCE_INLINE Interval<T> dotProduct(const Interval_nD<T,N> &motion) const { Interval<T> ret((T)0,(T)0); for(int i=0 ; i<N ; i++) ret+=m_x[i]*motion.x[i]; return ret; };

  const LinearMotion<T>& getAxis(size_t i) const { return m_x[i]; }

  void getStart(T start[N]) const { for (int i = 0;  i < N;  ++i) start[i] = m_x[i].getStart(); }
  void getEnd(T end[N]) const     { for (int i = 0;  i < N;  ++i) end[i]   = m_x[i].getEnd(); }

  LinearMotion_nD<T,N> firstHalf() const
  {
	  LinearMotion_nD<T,N> ret;
	  for (int i = 0;  i < N;  ++i)
		  ret[i] = m_x[i].firstHalf();
	  return ret;
  }
  LinearMotion_nD<T,N> secondHalf() const
  {
	  LinearMotion_nD<T,N> ret;
	  for (int i = 0;  i < N;  ++i)
		  ret[i] = m_x[i].secondHalf();
	  return ret;
  }
};

// Special case for dimension 3
template <class T>
class LinearMotion_nD<T,3>
{
private:
	LinearMotion<T> m_x[3];

public:
	LinearMotion_nD<T,3>() {};
	explicit LinearMotion_nD<T,3>(const LinearMotion<T>* x) { for (int i = 0;  i < 3;  ++i) { m_x[i] = x[i]; } };
	LinearMotion_nD<T,3>(const LinearMotion<T>& a, const LinearMotion<T>& b, const LinearMotion<T>& c) { m_x[0] = a;  m_x[1] = b;  m_x[2] = c; };
	LinearMotion_nD<T,3>(const LinearMotion_nD<T,3>& motion) { for (int i = 0;  i < 3;  ++i) { m_x[i] = motion.m_x[i]; } };
	LinearMotion_nD<T,3>(const T* a, const T* b) { m_x[0] = LinearMotion<T>(a[0], b[0]); m_x[1] = LinearMotion<T>(a[1], b[1]); m_x[2] = LinearMotion<T>(a[2], b[2]); };

	LinearMotion_nD<T,3>& operator= (const LinearMotion_nD<T,3>& motion) { for (int i = 0;  i < 3;  ++i) { m_x[i] = motion.m_x[i]; }  return *this; };

	Interval_nD<T,3> toInterval() const
	{
		Interval<T> motions[3];
		for (int i = 0;  i < 3;  ++i)
			motions[i] = m_x[i].toInterval();
		return Interval_nD<T,3>(motions);
	}

	// You probably should not use the exact comparison operators. =)
	bool operator== (const LinearMotion_nD<T,3>& motion) const { for (int i = 0;  i < 3;  ++i) { if (m_x[i] != motion.m_x[i]) return false; } return true; };
	bool operator!= (const LinearMotion_nD<T,3>& motion) const { return !(*this == motion); };

	LinearMotion_nD<T,3>& operator+= (const LinearMotion_nD<T,3>& m) { for (int i = 0;  i < 3;  ++i) { m_x[i] += m.m_x[i]; }  return *this; };
	LinearMotion_nD<T,3>  operator+  (const LinearMotion_nD<T,3>& m) const { LinearMotion_nD<T,3> ret(*this);  ret += m;  return ret; }

	LinearMotion_nD<T,3>& operator-= (const LinearMotion_nD<T,3>& m) { for (int i = 0;  i < 3;  ++i) { m_x[i] -= m.m_x[i]; }  return *this; };
	LinearMotion_nD<T,3>  operator-  (const LinearMotion_nD<T,3>& m) const { LinearMotion_nD<T,3> ret(*this);  ret -= m;  return ret; }

private:
	// Deprecated and removed from public API; should go away in the future.
	Interval_nD<T,3> operator* (const LinearMotion_nD<T,3>& m) const { return this->toInterval() * m.toInterval(); };
	Interval_nD<T,3> operator/ (const LinearMotion_nD<T,3>& m) const { return this->toInterval() / m.toInterval(); };

public:
	// TODO: we should implement this for generic <T,N> case, too!!!
	FORCE_INLINE static Polynomial<2,T> analyticDotProduct(const LinearMotion_nD<T,3>& a, const LinearMotion_nD<T,3>& b)
	{
		return a.getAxis(0).toPolynomial() * b.getAxis(0).toPolynomial()
			+  a.getAxis(1).toPolynomial() * b.getAxis(1).toPolynomial()
			+  a.getAxis(2).toPolynomial() * b.getAxis(2).toPolynomial();
	}

	FORCE_INLINE Interval<T> dotProduct(const LinearMotion_nD<T,3>& motion, const Interval<T>& range) const
	{
		return valuesOverInterval(analyticDotProduct(*this, motion), range);
	}

private:
	// TODO, maybe: should this be moved out of the "core" LinearMotion class?
	template <int A>
	FORCE_INLINE static Polynomial<2,T> analyticCrossProductAxis(const LinearMotion_nD<T,3>& a, const LinearMotion_nD<T,3>& b)
	{
		// The labels here are probably a bit confusing for anyone else, but at least this makes sense.
		// For A == 0, the "Y" and "Z" mean what they say, and the output is the X component.
		// For A == 1, they get rotated "down" by one (so Y -> Z, Z -> X), and the output is the Y component.
		// For A == 2, they get rotated "down" by two (i.e. "up" by one, so Y -> X, Z -> Y), and the output is the Z component.
		const LinearMotion<T>& aY = a.getAxis((A+1) % 3);
		const LinearMotion<T>& aZ = a.getAxis((A+2) % 3);
		const LinearMotion<T>& bY = b.getAxis((A+1) % 3);
		const LinearMotion<T>& bZ = b.getAxis((A+2) % 3);
		return aY.toPolynomial() * bZ.toPolynomial() - aZ.toPolynomial() * bY.toPolynomial();
	}

public:
	// To do, maybe: should this be moved out of the "core" LinearMotion class?
	FORCE_INLINE static Polynomial<2,T> analyticCrossProductXAxis(const LinearMotion_nD<T,3>& a, const LinearMotion_nD<T,3>& b)
	{
		return analyticCrossProductAxis<0>(a, b);
	}
	FORCE_INLINE static Polynomial<2,T> analyticCrossProductYAxis(const LinearMotion_nD<T,3>& a, const LinearMotion_nD<T,3>& b)
	{
		return analyticCrossProductAxis<1>(a, b);
	}
	FORCE_INLINE static Polynomial<2,T> analyticCrossProductZAxis(const LinearMotion_nD<T,3>& a, const LinearMotion_nD<T,3>& b)
	{
		return analyticCrossProductAxis<2>(a, b);
	}

	// To do, maybe: should this be moved out of the "core" LinearMotion class?
	FORCE_INLINE static void analyticCrossProduct(const LinearMotion_nD<T,3>& a, const LinearMotion_nD<T,3>& b,
		Polynomial<2,T>& resultXAxis, Polynomial<2,T>& resultYAxis, Polynomial<2,T>& resultZAxis)
	{
		resultXAxis = analyticCrossProductXAxis(a, b);
		resultYAxis = analyticCrossProductYAxis(a, b);
		resultZAxis = analyticCrossProductZAxis(a, b);
	}

	// To do, maybe: should this be moved out of the "core" LinearMotion class?
	FORCE_INLINE Interval_nD<T,3> crossProduct(const LinearMotion_nD<T,3>& motion, const Interval<T>& range) const
	{
		// The naive approach, i.e.
		//   toInterval().crossProduct(motion.toInterval())
		// results in intervals that are way too broad.
		return Interval_nD<T,3>(
			valuesOverInterval(analyticCrossProductAxis<0>(*this, motion), range),
			valuesOverInterval(analyticCrossProductAxis<1>(*this, motion), range),
			valuesOverInterval(analyticCrossProductAxis<2>(*this, motion), range) );
	}

	// Get the triple product, as a polynomial whose value for t=0..1 is the value of the triple product.
	// To do, maybe: should this be moved out of the "core" LinearMotion class?
	static FORCE_INLINE Polynomial<3,T> analyticTripleProduct(const LinearMotion_nD<T,3>& a, const LinearMotion_nD<T,3>& b, const LinearMotion_nD<T,3>& c)
	{
		const Polynomial<1,T> aX = a.getAxis(0).toPolynomial();
		const Polynomial<1,T> aY = a.getAxis(1).toPolynomial();
		const Polynomial<1,T> aZ = a.getAxis(2).toPolynomial();
		const Polynomial<1,T> bX = b.getAxis(0).toPolynomial();
		const Polynomial<1,T> bY = b.getAxis(1).toPolynomial();
		const Polynomial<1,T> bZ = b.getAxis(2).toPolynomial();
		const Polynomial<1,T> cX = c.getAxis(0).toPolynomial();
		const Polynomial<1,T> cY = c.getAxis(1).toPolynomial();
		const Polynomial<1,T> cZ = c.getAxis(2).toPolynomial();
		return ( (bY * cZ - bZ * cY) * aX + (bZ * cX - bX * cZ) * aY + (bX * cY - bY * cX) * aZ );
	}

	// Get the triple product values, as an interval.
	// To do, maybe: should this be moved out of the "core" LinearMotion class?
	static FORCE_INLINE Interval<T> tripleProduct(const LinearMotion_nD<T,3>& a, const LinearMotion_nD<T,3>& b, const LinearMotion_nD<T,3>& c,
		const Interval<T>& range)
	{
		// The naive approach, i.e.
		//   toInterval().dotProduct(motion2.crossProduct(motion3))
		// results in intervals that are way too broad.
		return valuesOverInterval(analyticTripleProduct(a, b, c), range);
	}

	// To do, maybe: should this be moved out of the "core" LinearMotion class?
	Polynomial<2,T> analyticMagnitudeSquared() const
	{
		//return square(getAxis(0).toPolynomial()) + square(getAxis(1).toPolynomial()) + square(getAxis(2).toPolynomial());
		return analyticDotProduct(*this, *this);
	}

	// To do, maybe: should this be moved out of the "core" LinearMotion class?
	FORCE_INLINE Interval<T> magnitudeSquared(const Interval<T>& range) const
	{
		return valuesOverInterval(analyticMagnitudeSquared(), range);
	}

	// To do, maybe: should this be moved out of the "core" LinearMotion class?
	Interval<T> magnitude(const Interval<T>& range) const
	{
		Interval<T> magnitudeSq = magnitudeSquared(range);
		// Both minimum and maximum are guaranteed to be non-negative.
		return Interval<T>(sqrt(magnitudeSq.getMin()), sqrt(magnitudeSq.getMax()));
	}

	const LinearMotion<T>& getAxis(size_t i) const { return m_x[i]; }

	void getStart(T start[3]) const { for (int i = 0;  i < 3;  ++i) start[i] = m_x[i].getStart(); }
	void getEnd(T end[3]) const     { for (int i = 0;  i < 3;  ++i) end[i]   = m_x[i].getEnd(); }
	SqVec3<T> getStart() const { return SqVec3<T>(m_x[0].getStart(), m_x[1].getStart(), m_x[2].getStart()); }
	SqVec3<T> getEnd() const   { return SqVec3<T>(m_x[0].getEnd(),   m_x[1].getEnd(),   m_x[2].getEnd()); }
	SqVec3<T> atTime(T t) const { return SqVec3<T>(m_x[0].atTime(t),   m_x[1].atTime(t),   m_x[2].atTime(t)); }

	LinearMotion_nD<T,3> firstHalf() const
	{
		LinearMotion_nD<T,3> ret;
		for (int i = 0;  i < 3;  ++i)
			ret[i] = m_x[i].firstHalf();
		return ret;
	}
	LinearMotion_nD<T,3> secondHalf() const
	{
		LinearMotion_nD<T,3> ret;
		for (int i = 0;  i < 3;  ++i)
			ret[i] = m_x[i].secondHalf();
		return ret;
	}
};


template <typename T>
inline ostream& operator<< (ostream& o, const LinearMotion<T>& motion)
{
	o << "(" << motion.getStart() << " -> " << motion.getEnd() << ")";
	return o;
}

template <typename T, int N>
inline ostream& operator<< (ostream& o, const LinearMotion_nD<T,N>& motion)
{
	o << "([" << motion.getAxis(0).getStart();
	for (int i = 1;  i < N;  ++i)
		o << "," << motion.getAxis(i).getStart();
	o << "] -> [" << motion.getAxis(0).getEnd();
	for (int i = 1;  i < N;  ++i)
		o << "," << motion.getAxis(i).getEnd();
	o << "])";
	return o;
}

#undef FORCE_INLINE
#endif // LINEAR_MOTION__H
