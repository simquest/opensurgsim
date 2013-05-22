#ifndef SQ_MATH_UTILS_H
#define SQ_MATH_UTILS_H

#undef  _USE_MATH_DEFINES  // stop the compiler from whining
#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <limits>

#ifdef _MSC_VER  // Microsoft Visual Studio C/C++ compiler
#pragma warning(push)
#pragma warning(disable: 4251)  // suppress class dll-interface warnings
#endif

// Define float versions of the math constants
#define M_Ef (float(M_E))
#define M_LOG2Ef (float(M_LOG2E))
#define M_LOG10Ef (float(M_LOG10E))
#define M_LN2f (float(M_LN2))
#define M_LN10f (float(M_LN10))
#define M_PIf (float(M_PI))
#define M_PI_2f (float(M_PI_2))
#define M_PI_4f (float(M_PI_4))
#define M_1_PIf (float(M_1_PI))
#define M_2_PIf (float(M_2_PI))
#define M_2_SQRTPIf (float(M_2_SQRTPI))
#define M_SQRT2f (float(M_SQRT2))
#define M_SQRT1_2f (float(M_SQRT1_2))


// Type shorthand
typedef unsigned char uchar;
typedef unsigned short ushort;
typedef unsigned int uint;
typedef unsigned long ulong;
#if defined(_MSC_VER)
typedef __int64 llong;
typedef unsigned __int64 ullong;
#elif defined(__SIZEOF_LONG_LONG__) && (__SIZEOF_LONG_LONG__+0 == 8)
typedef long long llong;
typedef unsigned long long ullong;
#else
// Well, I don't know how to properly define llong and ullong.
// (Should we issue a warning?)
#endif

namespace SqMath 
{ 

// round to the nearest n (n should be positive non-zero)
inline float Round(float x, float n) 
{ 
	return ((int)((x + (x >= 0 ? n/2.0f : -n/2.0f)) / n)) * n;
}
inline double Round(double x, double n)
{ 
	return ((int)((x + (x >= 0 ? n/2.0 : -n/2.0)) / n)) * n;
}

// Microsoft does not know how to take powers of small negative numbers, so we have to tell them how to do it
inline float Cbrt(float x) { return x < 0.0 ? -pow(-x,1.0f/3.0f) : pow(x,1.0f/3.0f); }
inline double Cbrt(double x) { return x < 0.0 ? -pow(-x,1.0/3.0) : pow(x,1.0/3.0); }

/// utility template to get a random number
template <class T> inline
T RandI(T vmax) { return T(double(vmax) * (double(rand()) / double(RAND_MAX + 1))); }
template <class T> inline
T RandI(T vmin, T vmax) { return vmin + T(double(vmax - vmin) * (double(rand()) / double(RAND_MAX + 1))); }
template <class T> inline
T Rand(T vmax = T(1)) { return vmax * ((T)rand() / (T)RAND_MAX); }
template <class T> inline
T Rand(T vmin, T vmax) { return vmin + (vmax - vmin) * ((T)rand() / (T)RAND_MAX); }
/// seed the pseudorandom number generator
inline void	SRand() { srand((unsigned)time(NULL)); }

// useful macros
template <class T>
inline T Rad2Deg(T x) { return T(180.0/M_PI)*x; }
template <class T>
inline T Deg2Rad(T x) { return T(M_PI/180.0)*x; }
template <class T>
inline T Rot2Deg(T x) { return T(360.0)*x; }
template <class T>
inline T Deg2Rot(T x) { return T(1.0/360.0)*x; }
template <class T>
inline T Sign(T x) { return x>T(0) ? T(1) : (x<T(0) ? T(-1) : T(0)); }
template <class T>
inline T Sqr(T x) { return x*x; }
template <class T>
inline T ClampLo(T x, T lo) { return x < lo ? lo : x; }
template <class T>
inline T ClampHi(T x, T hi) { return x > hi ? hi : x; }
template <class T>
inline T Clamp(T x, T lo, T hi) { return x < lo ? lo : x > hi ? hi : x; }
template <class T, class U>
inline T Mix(T x, T y, U a) { return x * (U(1) - a) + y * a; }

/// return true or false, depending on whether a given number
/// is bounded by the other two numbers
/// this checks x in [y,z], not (y,z)!
template <class T, class U, class V> inline
bool InBetween (T x, U y, V z)
{
	return (y < z) ?
				(x >= y && x <= z) : 
				(x <= y && x >= z);
}

// XXX TODO: move the following block into SqMinMax.h, and include that file from here

// minimum of 2 values
template <class T> inline
T Min(T x, T y)
{
	return x <= y ? x : y;
}

// minimum index of 2 values
template <class T> inline
size_t MinIndex(T x, T y)
{
	return x <= y ? 0 : 1;
}

// maximum of 2 values
template <class T> inline
T Max(T x, T y)
{
	return x >= y ? x : y;
}

// maximum index of 2 values
template <class T> inline
size_t MaxIndex(T x, T y)
{
	return x >= y ? 0 : 1;
}

// minimum of 3 values
template <class T> inline
T Min(T x, T y, T z)
{
	return x <= y ?(x <= z ? x : z) :(y <= z ? y : z);
}

// minimum index of 3 values
template <class T> inline
size_t MinIndex(T x, T y, T z)
{
	return x <= y ?(x <= z ? 0 : 2) :(y <= z ? 1 : 2);
}

// maximum of 3 values
template <class T> inline
T Max(T x, T y, T z)
{
	return x >= y ?(x >= z ? x : z) :(y >= z ? y : z);
}

// maximum index of 3 values
template <class T> inline
size_t MaxIndex(T x, T y, T z)
{
	return x >= y ?(x >= z ? 0 : 2) :(y >= z ? 1 : 2);
}

// minimum of 4 values
template <class T> inline
T Min(T x, T y, T z, T w)
{
	return x <= y ?(x <= z ?(x <= w ? x : w) :(z <= w ? z : w)) : (y <= z ?(y <= w ? y : w) :(z <= w ? z : w));
}

// minimum index of 4 values
template <class T> inline
size_t MinIndex(T x, T y, T z, T w)
{
	return x <= y ?(x <= z ?(x <= w ? 0 : 3) :(z <= w ? 2 : 3)) : (y <= z ?(y <= w ? 1 : 3)  :(z <= w ? 2 : 3));
}

// maximum of 4 values
template <class T> inline
T Max(T x, T y, T z, T w)
{
	return x >= y ?(x >= z ?(x >= w ? x : w) :(z >= w ? z : w)) : (y >= z ?(y >= w ? y : w) :(z >= w ? z : w));
}

// maximum index of 4 values
template <class T> inline
size_t MaxIndex(T x, T y, T z, T w)
{
	return x >= y ?(x >= z ?(x >= w ? 0 : 3) :(z >= w ? 2 : 3)) : (y >= z ?(y >= w ? 1 : 3) :(z >= w ? 2 : 3));
}

// minimum of 5 values
template <class T> inline
T Min(T x, T y, T z, T w, T v)
{
	return x <= y ?(x <= z ?(x <= w ? ((x <= v) ? x : v) : (w <= v ? w : v )) :(z <= w ? (z <= v ? z : v ) : (w <= v ? w : v ))) : 
		(y <= z ?(y <= w ? (y <= v ? y : v ) : (w <= v ? w : v )) :(z <= w ? (z <= v ? z : v ) : (w <= v ? w : v )));
}

// minimum index of 5 values
template <class T> inline
size_t MinIndex(T x, T y, T z, T w, T v)
{
	return x <= y ?(x <= z ?(x <= w ? ((x <= v) ? 0 : 4) : (w <= v ? 3 : 4 )) :(z <= w ? (z <= v ? 2 : 4 ) : (w <= v ? 3 : 4 ))) : 
		(y <= z ?(y <= w ? (y <= v ? 1 : 4 ) : (w <= v ? 3 : 4 )) :(z <= w ? (z <= v ? 2 : 4 ) : (w <= v ? 3 : 4 )));
}

// maximum of 5 values
template <class T> inline
T Max(T x, T y, T z, T w, T v)
{
	return x >= y ?(x >= z ?(x >= w ? ((x >= v) ? x : v) : (w >= v ? w : v )) :(z >= w ? (z >= v ? z : v ) : (w >= v ? w : v ))) : 
		(y >= z ?(y >= w ? (y >= v ? y : v ) : (w >= v ? w : v )) :(z >= w ? (z >= v ? z : v ) : (w >= v ? w : v )));
}

// maximum index of 5 values
template <class T> inline
size_t MaxIndex(T x, T y, T z, T w, T v)
{
	return x >= y ?(x >= z ?(x >= w ? ((x >= v) ? 0 : 4) : (w >= v ? 3 : 4 )) :(z >= w ? (z >= v ? 2 : 4 ) : (w >= v ? 3 : 4 ))) : 
		(y >= z ?(y >= w ? (y >= v ? 1 : 4 ) : (w >= v ? 3 : 4 )) :(z >= w ? (z >= v ? 2 : 4 ) : (w >= v ? 3 : 4 )));
}

// XXX TODO: move the preceding block into SqMinMax.h, and include that file from here

// a bit of IEEE 754 stuff
struct Float
{
	Float(float v0) : v(v0) {}
	Float(unsigned int i0) : i(i0) {}
	union
	{
		unsigned int i;
		struct
		{
			// This bit field definition assumes little-endian bit ordering and is evil.
			// We should use explicit masks and shifts, which is what the compiler will do anyway.  --bert
			unsigned int frac : 23;
			unsigned int exp : 8;
			unsigned int sign : 1;
		} m;
		float v;
	};
};

struct Double
{
	Double(double v0) : v(v0) {}
	Double(ullong i0) : i(i0) {}
	union
	{
		ullong i;
		struct
		{
			// This bit field definition assumes little-endian bit ordering and is evil.
			// We should use explicit masks and shifts, which is what the compiler will do anyway.  --bert
			ullong frac : 52;
			ullong exp : 11;
			ullong sign : 1;
		} m;
		double v;
	};
};

/// Returns true if f1 is within M_TOL of 0.0.
template <class T> inline
bool IsZero(T val)
{ 
	return (val > -std::numeric_limits<T>::epsilon() && val < std::numeric_limits<T>::epsilon());
}

// Check for exact positive/negative zero value.
inline bool IsExactlyZero(float v)
{
	Float f(v);
	return (0 == f.m.exp && 0 == f.m.frac);
}

inline bool IsExactlyZero(double v)
{
	Double d(v);
	return (0 == d.m.exp && 0 == d.m.frac);
}

// checks for NaN
inline bool IsNaN(float v)
{
	Float f(v);
	return (255 == f.m.exp && 0 != f.m.frac);
}

inline bool IsNaN(double v)
{
	Double d(v);
	return (2047 == d.m.exp && 0 != d.m.frac);
}

// checks for Inf
inline bool IsInf(float v)
{
	Float f(v);
	return (255 == f.m.exp && 0 == f.m.frac);
}

inline bool IsInf(double v)
{
	Double d(v);
	return (2047 == d.m.exp && 0 == d.m.frac);
}

inline bool IsPosInf(float v)
{
	Float f(v);
	return (255 == f.m.exp && 0 == f.m.frac && 0 == f.m.sign);
}

inline bool IsPosInf(double v)
{
	Double d(v);
	return (2047 == d.m.exp && 0 == d.m.frac && 0 == d.m.sign);
}

inline bool IsNegInf(float v)
{
	Float f(v);
	return (255 == f.m.exp && 0 == f.m.frac && 1 == f.m.sign);
}

inline bool IsNegInf(double v)
{
	Double d(v);
	return (2047 == d.m.exp && 0 == d.m.frac && 1 == d.m.sign);
}

// Check for denormalized values (too small to represent with full precision, but non-zero).
// Note that arithmetic operations with denormalized values can be very slow!
inline bool IsDenormalized(float v)
{
	Float f(v);
	return (0 == f.m.exp && 0 != f.m.frac);
}

inline bool IsDenormalized(double v)
{
	Double d(v);
	return (0 == d.m.exp && 0 != d.m.frac);
}

// Check if a number is valid (i.e. not infinite or NaN).  Denormalized values are considered valid!
inline bool IsValid(float v)
{
	Float f(v);
	return 255 != f.m.exp;
}

inline bool IsValid(double v)
{
	Double d(v);
	return 2047 != d.m.exp;
}

// bit operators
// many of these are taken from "http://www.sjbaker.org/steve/software/cute_code.html"
// is 'n' a power of 2 integer?
// - zero and one are considered to be powers of two
// - a power-of-two int has only one '1' bit, so zero'ing it should give zero back
inline bool IsPow2(unsigned int n)
{ 
	return (n & (n-1)) == 0;
}

inline unsigned int NextPow2(unsigned int n)
{
	if(IsPow2(n)) return n;
	unsigned int i = 1;
	while(i < n) i <<= 1;
	return i;
}

// how many 1's are in 'n'?
inline unsigned int
GetNumOnes(unsigned int n)
{
	n = (n & 0x55555555) + ((n & 0xaaaaaaaa) >> 1);
	n = (n & 0x33333333) + ((n & 0xcccccccc) >> 2);
	n = (n & 0x0f0f0f0f) + ((n & 0xf0f0f0f0) >> 4);
	n = (n & 0x00ff00ff) + ((n & 0xff00ff00) >> 8);
	n = (n & 0x0000ffff) + ((n & 0xffff0000) >> 16);
	return n;
}

/// return the factorial of an integer
inline int Factorial(int i)
{
	int res = 1;
	for(int j = i; j > 1; --j)
	{
		res *= j;
	}
	return res;
}

/// return the evaluation of a binomial expression
inline int Binomial(int n, int i)
{
	return Factorial(n) / (Factorial(i) * Factorial(n-i));
}

// Integer power
inline int Pow(int base, int exp)
{
	int res = 1;
	for(int i = 0; i < exp; ++i)
	{
		res *= base;
	}
	return res;
}

// Interleave two 16 bit unsigned integers bitwise
// The bits in i go to the even bits, the bits in j go to the odd bits
inline uint InterleaveBits(uint i, uint j)
{
	uint res = 0;
	for(uint k = 0; k < 16; ++k)
	{
		res >>= 1;
		res |= (i & 1) << 31;
		i >>= 1;
		res >>= 1;
		res |= (j & 1) << 31;
		j >>= 1;
	}
	return res;
}

}

#ifdef _MSC_VER  // Microsoft Visual Studio C/C++ compiler
#pragma warning(pop)
#endif

#endif 
