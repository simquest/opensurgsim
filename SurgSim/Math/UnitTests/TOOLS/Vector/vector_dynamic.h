#ifndef __vector_dynamic_h__
#define __vector_dynamic_h__

#include <stdio.h>              // va_list, va_arg, va_start
#include <stdarg.h>             // va_list, va_arg, va_start
#include "TOOLS/allocator.h"    // Allocation policy for dynamic vectors
#include "TOOLS/debug.h"        // assert, assert_printf...
#include "vector_generic.h"
#include <iostream>             // ostream
#include <string>

using namespace std;
#include "TOOLS/reference_countable.h"

#define EPSILON 0.00001

//#define Dynamic_Vector_Debug_Constructor_Empty
//#define Dynamic_Vector_Debug_Constructor_Set
//#define Dynamic_Vector_Debug_Constructor_Copy
//#define Dynamic_Vector_Debug_Destrcutor
//#define Dynamic_Vector_Debug_Bracket
//#define Dynamic_Vector_Debug_Bracket_Const
//#define Dynamic_Vector_Debug_GetSize
//#define Dynamic_Vector_Debug_GetPointer

//////////////////////////////////////////////////
// Dynamic_Vector class
template<class T, class Allocator = Allocator_malloc<T> > class Dynamic_Vector
{
  /////////////////////////////////////
  // Set
  template <class T0,class V1>                                          friend void  vec_set(V1 &v1, T0 value);

  // Null
  template <class V1>                                                   friend void  vec_null(V1 &v1);
  template <class T0,class V1>                                          friend void  vec_null(V1 &v1);

  // Copy
  template <class V1,class V2>                                          friend void  vec_copy(V1 &v1,const V2 &v2);
  template <class T1,class T2,class V1,class V2>                        friend void  vec_copy(V1 &v1,const V2 &v2);

  // Addition
  template <class V1,class V2>                                          friend void  vec_add(V1 &v1, const V2 &v2);
  template <class T1,class T2,class V1,class V2>                        friend void  vec_add(V1 &v1, const V2 &v2);
  template <class V1,class V2,class Vres>                               friend void  vec_add(const V1 &v1, const V2 &v2, Vres &res);
  template <class T1,class T2,class Tres,class V1,class V2,class Vres>  friend void  vec_add(const V1 &v1, const V2 &v2, Vres &res);

  // Triadic operation res = v1 + v2*scale (or v1 += v2*scale)
  template <class T0,class V1,class V2>                                 friend void  vec_triadic(V1 &v1,const V2 &v2,T0 scale);
  template <class T0,class T1,class T2,class V1,class V2>               friend void  vec_triadic(V1 &v1,const V2 &v2,T0 scale);
  template <class T0,class V1,class V2,class Vres>                      friend void  vec_triadic(const V1 &v1,const V2 &v2,Vres &res,T0 scale);
  template <class T0,class T1,class T2,class Tres,class V1,class V2,class Vres>  friend void  vec_triadic(const V1 &v1,const V2 &v2,Vres &res,T0 scale);

  // Substraction
  template <class V1,class V2>                                          friend void  vec_sub(V1 &v1, const V2 &v2);
  template <class T1,class T2,class V1,class V2>                        friend void  vec_sub(V1 &v1, const V2 &v2);
  template <class V1,class V2,class Vres>                               friend void  vec_sub(const V1 &v1, const V2 &v2, Vres &res);
  template <class T1,class T2,class Tres,class V1,class V2,class Vres>  friend void  vec_sub(const V1 &v1, const V2 &v2, Vres &res);

  // Opposite
  template <class V1>                                                   friend void  vec_opposite(V1 &v1);
  template <class T0,class V1>                                          friend void  vec_opposite(V1 &v1);
  template <class V1,class V2>                                          friend void  vec_opposite(const V1 &v1,V2 &v2);
  template <class T1,class T2,class V1,class V2>                        friend void  vec_opposite(const V1 &v1,V2 &v2);

  // Scale
  template <class T0,class V1>                                          friend void  vec_scale(V1 &v1,T0 factor);
  template <class T1,class T0,class V1>                                 friend void  vec_scale(V1 &v1,T0 factor);
  template <class T0,class V1,class V2>                                 friend void  vec_scale(const V1 &v1,V2 &v2,T0 factor);
  template <class T1,class T2,class T0,class V1,class V2>               friend void  vec_scale(const V1 &v1,V2 &v2,T0 factor);

  // invScale
  template <class T0,class V1>                                          friend void  vec_invScale(V1 &v1,T0 factor);
  template <class T1,class T0,class V1>                                 friend void  vec_invScale(V1 &v1,T0 factor);
  template <class T0,class V1,class V2>                                 friend void  vec_invScale(const V1 &v1,V2 &v2,T0 factor);
  template <class T1,class T2,class T0,class V1,class V2>               friend void  vec_invScale(const V1 &v1,V2 &v2,T0 factor);

  // Multiplication element per element
  template <class V1,class V2>                                          friend void  vec_mul(V1 &v1, const V2 &v2);
  template <class T1,class T2,class V1,class V2>                        friend void  vec_mul(V1 &v1, const V2 &v2);
  template <class V1,class V2,class Vres>                               friend void  vec_mul(const V1 &v1, const V2 &v2, Vres &res);
  template <class T1,class T2,class Tres,class V1,class V2,class Vres>  friend void  vec_mul(const V1 &v1, const V2 &v2, Vres &res);

  // Dot Product
  template <class T0,class V1,class V2>                                 friend T0     vec_dotProduct(const V1 &v1, const V2 &v2);
  template <class T0,class T1,class T2,class V1,class V2>               friend T0     vec_dotProduct(const V1 &v1, const V2 &v2);

  // Norm, NormSQ, Normalization
  template <class T0,class V1>                                          friend T0     vec_norm(const V1 &v1);
  template <class T0,class T1,class V1>                                 friend T0     vec_norm(const V1 &v1);
  template <class T0,class V1>                                          friend T0     vec_normSQ(const V1 &v1);
  template <class T0,class T1,class V1>                                 friend T0     vec_normSQ(const V1 &v1);
  template <class T0,class V1>                                          friend T0     vec_normalize(V1 &v1);
  template <class T0,class T1,class V1>                                 friend T0     vec_normalize(V1 &v1);

  // Cross Product
  template <class T0,class V1,class V2>                                 friend void  vec_crossProduct(V1 &v1, const V2 &v2);
  template <class T1,class T2,class V1,class V2>                        friend void  vec_crossProduct(V1 &v1, const V2 &v2);
  template <class V1,class V2,class Vres>                               friend void  vec_crossProduct(const V1 &v1, const V2 &v2, Vres &res);
  template <class T1,class T2,class Tres,class V1,class V2,class Vres>  friend void  vec_crossProduct(const V1 &v1, const V2 &v2, Vres &res);

  // Cross Product 3D
  template <class T0,class V1,class V2>                                 friend void  vec_crossProduct3D(V1 &v1, const V2 &v2);
  template <class T1,class T2,class V1,class V2>                        friend void  vec_crossProduct3D(V1 &v1, const V2 &v2);
  template <class V1,class V2,class Vres>                               friend void  vec_crossProduct3D(const V1 &v1, const V2 &v2, Vres &res);
  template <class T1,class T2,class Tres,class V1,class V2,class Vres>  friend void  vec_crossProduct3D(const V1 &v1, const V2 &v2, Vres &res);

  // Clamp
  template <class T0,class V1>                                          friend void  vec_clamp(V1 &v1,T0 min, T0 max);
  template <class T0,class T1,class V1>                                 friend void  vec_clamp(V1 &v1,T0 min, T0 max);
  template <class T0,class V1,class Vres>                               friend void  vec_clamp(const V1 &v1, Vres &res,T0 min, T0 max);
  template <class T0,class T1,class Tres,class V1,class Vres>           friend void  vec_clamp(const V1 &v1, Vres &res,T0 min, T0 max);

  // Test orthogonality and colinearity
  template <class T0,class V1,class V2>                                 friend bool  vec_areOrthogonal(const V1 &v1,const V2 &v2);
  template <class T0,class T1,class T2,class V1,class V2>               friend bool  vec_areOrthogonal(const V1 &v1,const V2 &v2);
  //template <class T0,class V1,class V2,class V3> bool  vec_areColinear(const V1 &v1,const V2 &v2,const V3 &v3);

  // Get Sub Vector
  template <class V1,class V2>                                          friend void  vec_getSubVector(const V1 &v1,V2 &v2,unsigned int size,unsigned int startV1,unsigned int startV2);
  template <class T1,class T2,class V1,class V2>                        friend void  vec_getSubVector(const V1 &v1,V2 &v2,unsigned int size,unsigned int startV1,unsigned int startV2);
  /////////////////////////////////////

protected:
  unsigned int  size;
  unsigned int  allocatedSize;
  T *           data;

  //String      name;

  //void copy(T* data, unsigned int size);
public:  
  Dynamic_Vector(void);
  Dynamic_Vector(unsigned int n);
  Dynamic_Vector(unsigned int n,T a,...);
  Dynamic_Vector(const Dynamic_Vector<T,Allocator> &v);
  ~Dynamic_Vector();

  Dynamic_Vector& operator=(const Dynamic_Vector& v);

  //void setName(String n){ name=n; };

  /// Grow the internally allocated memory to the specified number of elements, unless there are already more allocated.
  /// Does not change the size of the vector.
  inline void   reserve(unsigned int storageSize);

  /// Resize the vector.  Grow or shrink the allocated memory to exactly match the vector size.
  /// Existing data will be preserved, but if the size grows any new elements may be garbage.
  inline void   resizeSetMemory(unsigned int newSize);
  /// Resize the vector.  Grow the allocated memory if needed for the vector to fit.
  /// Existing data will be preserved, but if the size grows any new elements may be garbage.
  inline void   resizeGrowMemory(unsigned int newSize);

  /// Increase the vector's size if it's smaller than the specified size.
  /// Does not change the size if the vector is already larger than requested!
  /// Existing data will be preserved, but if the size grows any new elements may be garbage.
  inline void   resizeToAtLeast(unsigned int minSize);

  /// Resize the vector.  (Currently the semantics are the same as resizeSetMemory(), but this will probably change!)
  //  XXX TODO: change the semantics to resizeGrowMemory().
  void          resize(unsigned int newSize) { resizeSetMemory(newSize); }

  /// Resize the vector if it's smaller than the specified size.  An alias for Dynamic_Vector::resizeToAtLeast().
  /// Note that the semantics of this method are quite different than the semantics of Dynamic_Matrix::resizeUpIfNecessary(), which is confusing!
  //  XXX TODO: get rid of this, and replace uses with one of the other applicable methods.
  void          resizeUpIfNecessary(unsigned int minSize) { resizeToAtLeast(minSize); }

  void          add(T &o);

  T &           operator [](unsigned int i);
  T             operator [](unsigned int i) const;

  Dynamic_Vector<T> &  operator +=(Dynamic_Vector<T> &a)
  {
    assert_printf(size==a.size,"Different size !");
    vec_generic_add<T,T>(data,a.data,size);
    return *this;
  };

  Dynamic_Vector<T> &  operator -=(Dynamic_Vector<T> &a)
  {
    assert_printf(size==a.size,"Different size !");
    vec_generic_sub<T,T>(data,a.data,size);
    return *this;
  };

  Dynamic_Vector<T> &  operator *=(T value)
  { vec_generic_scale<T,T>(data,size,value); return *this; };

  Dynamic_Vector<T> &  operator /=(T value)
  { vec_generic_invScale<T,T>(data,size,value); return *this; };

  T operator *(Dynamic_Vector<T> &a)
  { return vec_generic_dotProduct<T,T,T>(data,a.data,size); };

  Dynamic_Vector<T>& operator ^=(Dynamic_Vector<T> &a)
  { vec_generic_crossProduct<T,T>(data,a.data,size); return *this;};

  unsigned int  getSize(void) const;

  T *           getPointer(void);
  const T *     getPointer(void) const;
};
//////////////////////////////////////////////////

/////////////////////////////////////
// Output
template <class T,class Allocator> ostream & operator <<(ostream &o, const Dynamic_Vector<T,Allocator> &v);
/////////////////////////////////////

#include "vector_dynamic.hpp"

#endif
