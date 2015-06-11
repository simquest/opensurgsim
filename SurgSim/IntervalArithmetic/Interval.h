#ifndef __INTERVALARITHMETIC_H__
#define __INTERVALARITHMETIC_H__

#include <ostream>
using std::ostream;

#include "Math/SqMinMax.h"
#include <SqAssert.h>

#define FORCE_INLINE __forceinline

class IntervalArithmeticException
{
  const char* what() const throw()
  {
    return "Can't invert or divide by an interval containing 0!";
  }
};


template <class T> class Interval
{
  template <class P> friend void IntervalArithmetic_add(const Interval<P> &a,const Interval<P> &b,Interval<P> &res);     // +
  template <class P> friend void IntervalArithmetic_addadd(const Interval<P> &a,const Interval<P> &b,Interval<P> &res);  // +=( + )
  template <class P> friend void IntervalArithmetic_sub(const Interval<P> &a,const Interval<P> &b,Interval<P> &res);     // -
  template <class P> friend void IntervalArithmetic_addsub(const Interval<P> &a,const Interval<P> &b,Interval<P> &res);  // +=( - )
  template <class P> friend void IntervalArithmetic_mul(const Interval<P> &a,const Interval<P> &b,Interval<P> &res);     // *
  template <class P> friend void IntervalArithmetic_addmul(const Interval<P> &a,const Interval<P> &b,Interval<P> &res);  // += ( * )
  template <class P> friend void IntervalArithmetic_submul(const Interval<P> &a,const Interval<P> &b,Interval<P> &res);  // -= ( * )

private:
  T a,b;

public:
  Interval<T>():a((T)0),b((T)0){};
  Interval<T>(T min, T max):a(min),b(max)
  {
	  SQ_DEBUG_ASSERT(min <= max, "order of interval bounds");
  }
  Interval<T>(const Interval<T> &i):a(i.a),b(i.b){};

  Interval<T> &operator =(const Interval<T> &i){ a=i.a; b=i.b; return *this; };

  static Interval<T> minToMax(T a1, T a2)              { T min, max;   SqMath::MinMax(a1, a2, min, max);           return Interval<T>(min, max); }
  static Interval<T> minToMax(T a1, T a2, T a3)        { T min, max;   SqMath::MinMax(a1, a2, a3, min, max);       return Interval<T>(min, max); }
  static Interval<T> minToMax(T a1, T a2, T a3, T a4)  { T min, max;   SqMath::MinMax(a1, a2, a3, a4, min, max);   return Interval<T>(min, max); }

  bool overlapsWith(const Interval<T>& i) const { return (a <= i.b && i.a <= b); };
  bool contains(T val) const { return (a <= val && b >= val); };
  bool containsZero() const { return (a <= (T)0 && b >= (T)0); };

  // You probably should not use the exact comparison operators. =)
  bool operator ==(const Interval<T>& i) const { if(a==i.a && b==i.b) return true; return false; };
  bool operator !=(const Interval<T>& i) const { if(a!=i.a || b!=i.b) return true; return false; };

  //bool operator >(const Interval<T>& i)  const { if(a>i.a && b==i.b) return true; return false; };
  //bool operator >=(const Interval<T>& i) const { if(a==i.a && b==i.b) return true; return false; };

  //bool operator <(const Interval<T>& i)  const { for(int i=0 ; i<n ; i++) if(x[i]>=interval.x[i]) return false; return true; };
  //bool operator <=(const Interval<T>& i) const { for(int i=0 ; i<n ; i++) if(x[i]>interval.x[i]) return false; return true; };

  Interval<T>& addThickness(const T thickness) { a-=thickness; b+=thickness; return *this; };
  Interval<T>& extendToInclude(T x) { if (x < a) a = x;  else if (x > b) b = x;  return *this; };
  Interval<T>& extendToInclude(const Interval<T>& i) { if (i.a < a) a = i.a;  if (i.b > b) b = i.b;  return *this; };

  Interval<T>  operator +(const Interval<T>& i) const { return Interval<T>(a+i.a , b+i.b); };
  Interval<T>  operator +(T v) const                  { return Interval<T>(a+v, b+v); };
  Interval<T>& operator +=(const Interval<T>& i)      { a+=i.a; b+=i.b; return *this; };
  Interval<T>& operator +=(T v)                       { a+=v; b+=v; return *this; };

  Interval<T>  operator -() const                     { return Interval<T>(-b, -a); };
  Interval<T>  operator -(const Interval<T>& i) const { return Interval<T>(a-i.b , b-i.a); };
  Interval<T>  operator -(T v) const                  { return Interval<T>(a-v, b-v); };
  Interval<T>& operator -=(const Interval<T>& i)      { a-=i.b; b-=i.a; return *this; };
  Interval<T>& operator -=(T v)                       { a-=v; b-=v; return *this; };

  Interval<T>  operator *(const Interval<T>& i) const { return minToMax(a*i.a,a*i.b,b*i.a,b*i.b); };
  Interval<T>  operator *(T v) const                  { if (v >= 0) { return Interval<T>(a*v, b*v); } else { return Interval<T>(b*v, a*v); } };
  Interval<T>& operator *=(const Interval<T>& i)      { *this = minToMax(a*i.a,a*i.b,b*i.a,b*i.b);  return *this; };
  Interval<T>& operator *=(T v)                       { if (v >= 0) { a*=v; b*=v; } else { T tmp=a; a=b*v; b=tmp*v; } };

  // Inverse or division with interval containing 0 is not define
  // 1/x is strictly monotonic decreasing on ]-inf,0[ and ]0,+inf[
  Interval<T> inverse() const { if (! containsZero()) return Interval<T>( ((T)1)/b , ((T)1)/a ); else throw IntervalArithmeticException(); };

  Interval<T> operator /(const Interval<T> &i) const { return (*this)*i.inverse(); };
  Interval<T> &operator /=(const Interval<T> &i) { Interval<T> tmp=(*this)*i.inverse(); a=tmp.a; b=tmp.b; return *this; };

  Interval<T> square() const
  {
	  T lowerBoundSquared = a*a;
	  T upperBoundSquared = b*b;
	  T minSquare, maxSquare;
	  MinMax(lowerBoundSquared, upperBoundSquared, minSquare, maxSquare);
	  return Interval<T>((a < 0 && b > 0) ? 0 : minSquare, maxSquare);
  }

  T getMin() const   { return a; }
  T getMax() const   { return b; }

  Interval<T> lowerHalf() const  { return Interval<T>(a, (a+b)*(T)0.5); }
  Interval<T> upperHalf() const  { return Interval<T>((a+b)*(T)0.5, b); }
};

template <typename T>
Interval<T> operator+ (T v, const Interval<T>& i) { return i+v; };

template <typename T>
Interval<T> operator* (T v, const Interval<T>& i) { return i*v; };


// TO DO: the stand-alone methods duplicate operator functionality and should be *removed*.
// = ( + )
template <class T> void IntervalArithmetic_add(const Interval<T> &a,const Interval<T> &b,Interval<T> &res)
{
  res.a = a.a+b.a;
  res.b = a.b+b.b;
}
// += ( + )
template <class T> void IntervalArithmetic_addadd(const Interval<T> &a,const Interval<T> &b,Interval<T> &res)
{
  res.a += a.a+b.a;
  res.b += a.b+b.b;
}

// = ( - )
template <class T> void IntervalArithmetic_sub(const Interval<T> &a,const Interval<T> &b,Interval<T> &res)
{
  res.a = a.a-b.b;
  res.b = a.b-b.a;
}
// += ( - )
template <class T> void IntervalArithmetic_addsub(const Interval<T> &a,const Interval<T> &b,Interval<T> &res)
{
  res.a += a.a-b.b;
  res.b += a.b-b.a;
}

// = ( * )
template <class T> void IntervalArithmetic_mul(const Interval<T> &a,const Interval<T> &b,Interval<T> &res)
{
  T min,max;
  SqMath::MinMax(a.a*b.a,a.a*b.b,a.b*b.a,a.b*b.b,min,max);
  res.a = min;
  res.b = max;
}
// += ( * )
template <class T> void IntervalArithmetic_addmul(const Interval<T> &a,const Interval<T> &b,Interval<T> &res)
{
  T min,max;
  SqMath::MinMax(a.a*b.a,a.a*b.b,a.b*b.a,a.b*b.b,min,max);
  res.a += min;
  res.b += max;
}
// -= ( * )
template <class T> void IntervalArithmetic_submul(const Interval<T> &a,const Interval<T> &b,Interval<T> &res)
{
  T min,max;
  SqMath::MinMax(a.a*b.a,a.a*b.b,a.b*b.a,a.b*b.b,min,max);
  res.a -= max;
  res.b -= min;
}








template <class T, int n> class Interval_nD
{
private:
  Interval<T> x[n];

public:
  Interval_nD<T,n>(){ for(int i=0 ; i<n ; i++) x[i]=(T)0.0; };
  explicit Interval_nD<T,n>(const Interval<T>* _x){ for(int i=0 ; i<n ; i++) x[i]=_x[i]; };
  Interval_nD<T,n>(const Interval_nD<T,n>& interval){ for(int i=0 ; i<n ; i++) x[i]=interval.x[i]; };
  Interval_nD<T,n>(const T* a, const T* b) { for (int i = 0;  i < n;  ++i)  x[i]=Interval<T>::minToMax(a[i], b[i]); };

  Interval_nD<T,n> &operator =(const Interval_nD<T,n>& interval){ for(int i=0 ; i<n ; i++) x[i]=interval.x[i]; return *this; };

  bool overlapsWith(const Interval_nD<T,n>& interval) const
  {
	  // For the rectangular [hyper]prisms to overlap, they must overlap in all axes.
	  for (int i = 0;  i < n;  ++i) {
		  if (! x[i].overlapsWith(interval.x[i]))
			  return false;
	  }
	  return true;
  }

  // You probably should not use the exact comparison operators. =)
  bool operator ==(const Interval_nD<T,n>& interval) const { for(int i=0 ; i<n ; i++) if(x[i]!=interval.x[i]) return false; return true; };
  bool operator !=(const Interval_nD<T,n>& interval) const { for(int i=0 ; i<n ; i++) if(x[i]!=interval.x[i]) return true; return false; };

  //bool operator >(const Interval_nD<T,n>& interval)  const { for(int i=0 ; i<n ; i++) if(x[i]<=interval.x[i]) return false; return true; };
  //bool operator >=(const Interval_nD<T,n>& interval) const { for(int i=0 ; i<n ; i++) if(x[i]<interval.x[i]) return false; return true; };

  //bool operator <(const Interval_nD<T,n>& interval)  const { for(int i=0 ; i<n ; i++) if(x[i]>=interval.x[i]) return false; return true; };
  //bool operator <=(const Interval_nD<T,n>& interval) const { for(int i=0 ; i<n ; i++) if(x[i]>interval.x[i]) return false; return true; };

  Interval_nD<T,n>& addThickness(const double thickness) { for(int i=0 ; i<n ; i++) x[i].addThickness(thickness); return *this; };

  Interval_nD<T,n> operator +(const Interval_nD<T,n> &interval) const { Interval_nD<T,n> ret; for(int i=0 ; i<n ; i++) ret.x[i]=x[i]+interval.x[i]; return ret; };
  Interval_nD<T,n> &operator +=(const Interval_nD<T,n> &interval) { for(int i=0 ; i<n ; i++) x[i]+=interval.x[i]; return *this; };

  Interval_nD<T,n> operator -(const Interval_nD<T,n> &interval) const { Interval_nD<T,n> ret; for(int i=0 ; i<n ; i++) ret.x[i]=x[i]-interval.x[i]; return ret; };
  Interval_nD<T,n> &operator -=(const Interval_nD<T,n> &interval) { for(int i=0 ; i<n ; i++) x[i]-=interval.x[i]; return *this; };

  Interval_nD<T,n> operator *(const Interval_nD<T,n> &interval) const { Interval_nD<T,n> ret; for(int i=0 ; i<n ; i++) ret.x[i]=x[i]*interval.x[i]; return ret; };
  Interval_nD<T,n> &operator *=(const Interval_nD<T,n> &interval) { for(int i=0 ; i<n ; i++) x[i]*=interval.x[i]; return *this; };

  Interval_nD<T,n> inverse(void) const { Interval_nD<T,n> ret; for(int i=0 ; i<n ; i++) ret.x[i]=x[i].inverse(); return ret; };

  Interval_nD<T,n> operator /(const Interval_nD<T,n> &interval) const { Interval_nD<T,n> ret; for(int i=0 ; i<n ; i++) ret.x[i]=x[i]/interval.x[i]; return ret; };
  Interval_nD<T,n> &operator /=(const Interval_nD<T,n> &interval) { for(int i=0 ; i<n ; i++) x[i]/=interval.x[i]; return *this; };

  FORCE_INLINE Interval<T> dotProduct(const Interval_nD<T,n> &interval) const { Interval<T> ret((T)0,(T)0); for(int i=0 ; i<n ; i++) ret+=x[i]*interval.x[i]; return ret; };

  Interval<T> magnitudeSquared() const
  {
	  Interval<T> result = x[0].square();
	  for (int i = 1;  i < n;  ++i)
	  {
		  result += x[i].square();
	  }
	  return result;
  }

  Interval<T> magnitude() const
  {
	  Interval<T> magnitudeSq = magnitudeSquared();
	  // Both minimum and maximum are guaranteed to be non-negative.
	  return Interval<T>(sqrt(magnitudeSq.getMin()), sqrt(magnitudeSq.getMax()));
  }
  const Interval<T>& getAxis(size_t i) const { return x[i]; }
};

// Special case for dimension 3
template <class T> class Interval_nD<T,3>
{
  template <class P> friend void IntervalArithmetic_crossProduct(const Interval_nD<P,3> &a,const Interval_nD<P,3> &b,Interval_nD<P,3> &res);
  template <class P> friend void IntervalArithmetic_add(const Interval_nD<P,3> &a,const Interval_nD<P,3> &b,Interval_nD<P,3> &res);
  template <class P> friend void IntervalArithmetic_sub(const Interval_nD<P,3> &a,const Interval_nD<P,3> &b,Interval_nD<P,3> &res);
  template <class P> friend void IntervalArithmetic_dotProduct(const Interval_nD<P,3> &a,const Interval_nD<P,3> &b, Interval<P> &res);

private:
  Interval<T> x[3];

public:
  Interval_nD<T,3>(){ x[0]=Interval<T>(); x[1]=Interval<T>(); x[2]=Interval<T>(); };
  explicit Interval_nD<T,3>(const Interval<T>* _x){ x[0]=_x[0]; x[1]=_x[1]; x[2]=_x[2]; };
  Interval_nD<T,3>(Interval<T> _x,Interval<T> _y,Interval<T> _z){ x[0]=_x; x[1]=_y; x[2]=_z; };
  Interval_nD<T,3>(const Interval_nD<T,3>& i){ x[0]=i.x[0]; x[1]=i.x[1]; x[2]=i.x[2]; };
  Interval_nD<T,3>(const T* a, const T* b) { x[0]=Interval<T>::minToMax(a[0], b[0]); x[1]=Interval<T>::minToMax(a[1], b[1]); x[2]=Interval<T>::minToMax(a[2], b[2]); };

  Interval_nD<T,3> &operator =(const Interval_nD<T,3>& i){ x[0]=i.x[0]; x[1]=i.x[1]; x[2]=i.x[2]; return *this; };

  bool overlapsWith(const Interval_nD<T,3>& interval) const
  {
	  // For the rectangular prisms to overlap, they must overlap in all axes.
	  return (x[0].overlapsWith(interval.x[0]) && x[1].overlapsWith(interval.x[1]) && x[2].overlapsWith(interval.x[2]));
  }

  // You probably should not use the exact comparison operators. =)
  bool operator ==(const Interval_nD<T,3>& i) const { return (x[0]==i.x[0] && x[1]==i.x[1] && x[2]==i.x[2] ); };
  bool operator !=(const Interval_nD<T,3>& i) const { return (x[0]!=i.x[0] || x[1]!=i.x[1] || x[2]!=i.x[2] ); };

  //bool operator >(const Interval_nD<T,3>& i)  const { return (x[0]> i.x[0] && x[1]> i.x[1] && x[2]> i.x[2] ); };
  //bool operator >=(const Interval_nD<T,3>& i) const { return (x[0]>=i.x[0] && x[1]>=i.x[1] && x[2]>=i.x[2] ); };

  //bool operator <(const Interval_nD<T,3>& i)  const { return (x[0]< i.x[0] && x[1]< i.x[1] && x[2]< i.x[2] ); };
  //bool operator <=(const Interval_nD<T,3>& i) const { return (x[0]<=i.x[0] && x[1]<=i.x[1] && x[2]<=i.x[2] ); };

  Interval_nD<T,3>& addThickness(const double thickness) { x[0].addThickness(thickness); x[1].addThickness(thickness); x[2].addThickness(thickness); return *this; };

  Interval_nD<T,3> operator +(const Interval_nD<T,3> &i) const { return Interval_nD<T,3>(x[0]+i.x[0] , x[1]+i.x[1] , x[2]+i.x[2]); };
  Interval_nD<T,3> &operator +=(const Interval_nD<T,3> &i) { x[0]+=i.x[0]; x[1]+=i.x[1]; x[2]+=i.x[2]; return *this; };

  Interval_nD<T,3> operator -(const Interval_nD<T,3> &i) const { return Interval_nD<T,3>(x[0]-i.x[0] , x[1]-i.x[1] , x[2]-i.x[2]); };
  Interval_nD<T,3> &operator -=(const Interval_nD<T,3> &i) { x[0]-=i.x[0]; x[1]-=i.x[1]; x[2]-=i.x[2]; return *this; };

  Interval_nD<T,3> operator *(const Interval_nD<T,3> &i) const { return Interval_nD<T,3>(x[0]*i.x[0] , x[1]*i.x[1] , x[2]*i.x[2]); };
  Interval_nD<T,3> &operator *=(const Interval_nD<T,3> &i) { x[0]*=i.x[0]; x[1]*=i.x[1]; x[2]*=i.x[2]; return *this; };

  Interval_nD<T,3> inverse(void) const { return Interval_nD<T,3>( x[0].inverse() , x[1].inverse() , x[2].inverse() ); };

  Interval_nD<T,3> operator /(const Interval_nD<T,3> &i) const { return Interval_nD<T,3>( x[0]/i.x[0] , x[1]/i.x[1] , x[2]/i.x[2]); };
  Interval_nD<T,3> &operator /=(const Interval_nD<T,3> &i) { x[0]/=i.x[0]; x[1]/=i.x[1]; x[2]/=i.x[2]; return *this; };

  // The output interval covers *any* possible combination of values from the inputs, which may be far broader than what you may obtain under clever assumptions.
  FORCE_INLINE Interval<T> dotProduct(const Interval_nD<T,3> &i) const { return (x[0]*i.x[0] + x[1]*i.x[1] + x[2]*i.x[2]); };
  // The output interval covers *any* possible combination of values from the inputs, which may be far broader than what you may obtain under clever assumptions.
  FORCE_INLINE Interval_nD<T,3> crossProduct(const Interval_nD<T,3> &i) const { return Interval_nD<T,3>( x[1]*i.x[2] - x[2]*i.x[1] , x[2]*i.x[0] - x[0]*i.x[2] , x[0]*i.x[1] - x[1]*i.x[0] ); };

  Interval<T> magnitudeSquared() const
  {
	  return x[0].square() + x[1].square() + x[2].square();
  }

  Interval<T> magnitude() const
  {
	  Interval<T> magnitudeSq = magnitudeSquared();
	  // Both minimum and maximum are guaranteed to be non-negative.
	  return Interval<T>(sqrt(magnitudeSq.getMin()), sqrt(magnitudeSq.getMax()));
  }

  const Interval<T>& getAxis(size_t i) const { return x[i]; }
};

template <class T> void IntervalArithmetic_add(const Interval_nD<T,3> &a,const Interval_nD<T,3> &b,Interval_nD<T,3> &res)
{
  IntervalArithmetic_add( a.x[0] , b.x[0] , res.x[0] );
  IntervalArithmetic_add( a.x[1] , b.x[1] , res.x[1] );
  IntervalArithmetic_add( a.x[2] , b.x[2] , res.x[2] );
}

template <class T> void IntervalArithmetic_sub(const Interval_nD<T,3> &a,const Interval_nD<T,3> &b,Interval_nD<T,3> &res)
{
  IntervalArithmetic_sub( a.x[0] , b.x[0] , res.x[0] );
  IntervalArithmetic_sub( a.x[1] , b.x[1] , res.x[1] );
  IntervalArithmetic_sub( a.x[2] , b.x[2] , res.x[2] );
}

template <class T> void IntervalArithmetic_dotProduct(const Interval_nD<T,3> &a,const Interval_nD<T,3> &b, Interval<T> &res)
{
  IntervalArithmetic_mul( a.x[0] , b.x[0] , res );
  IntervalArithmetic_addmul( a.x[1] , b.x[1] , res );
  IntervalArithmetic_addmul( a.x[2] , b.x[2] , res );
}

template <class T> void IntervalArithmetic_crossProduct(const Interval_nD<T,3> &a,const Interval_nD<T,3> &b,Interval_nD<T,3> &res)
{
  IntervalArithmetic_mul( a.x[1] , b.x[2] , res.x[0] );
  IntervalArithmetic_submul( a.x[2] , b.x[1] , res.x[0] );

  IntervalArithmetic_mul( a.x[2] , b.x[0] , res.x[1] );
  IntervalArithmetic_submul( a.x[0] , b.x[2] , res.x[1] );

  IntervalArithmetic_mul( a.x[0] , b.x[1] , res.x[2] );
  IntervalArithmetic_submul( a.x[1] , b.x[0] , res.x[2] );
}

template <typename T>
inline ostream& operator<< (ostream& o, const Interval<T>& interval)
{
	o << "[" << interval.getMin() << "," << interval.getMax() << "]";
	return o;
}

template <typename T, int n>
inline ostream& operator<< (ostream& o, const Interval_nD<T,n>& interval)
{
	o << "(" << interval.getAxis(0);
	for (int i = 1;  i < n;  ++i)
		o << ";" << interval.getAxis(i);
	o << ")";
	return o;
}

#undef FORCE_INLINE
#endif
