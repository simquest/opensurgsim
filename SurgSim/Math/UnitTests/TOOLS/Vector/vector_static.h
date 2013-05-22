#ifndef __vector_static_h__
#define __vector_static_h__

#include <stdio.h>              // va_list, va_arg, va_start
#include <stdarg.h>             // va_list, va_arg, va_start
#include "TOOLS/debug.h"        // assert, assert_printf...
#include "TOOLS/Vector/vector_generic.h"  // vec_generic_add
#include <iostream>             // ostream
using namespace std;

//#define Static_Vector_Debug_Constructor_Set
//#define Static_Vector_Debug_Constructor_Copy
//#define Static_Vector_Debug_Bracket
//#define Static_Vector_Debug_Bracket_Const
//#define Static_Vector_Debug_GetSize
//#define Static_Vector_Debug_GetPointer

//////////////////////////////////////////////////
// Static_Vector class
template <class T,unsigned int n> class Static_Vector
{
  /////////////////////////////////////
  // Set
  template <class T2,class V1>                                          friend void  vec_set(V1 &v1, T2 value);

  // Null
  template <class V1>                                                   friend void  vec_null(V1 &v1);
  template <class T2,class V1>                                          friend void  vec_null(V1 &v1);

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
  template <class T0,class V1,class V2>                                  friend bool  vec_areOrthogonal(const V1 &v1,const V2 &v2);
  template <class T0,class T1,class T2,class V1,class V2>                friend bool  vec_areOrthogonal(const V1 &v1,const V2 &v2);
  //template <class T0,class V1,class V2,class V3> bool  vec_areColinear(const V1 &v1,const V2 &v2,const V3 &v3);

  // Get Sub Vector
  template <class V1,class V2>                                          friend void  vec_getSubVector(const V1 &v1,V2 &v2,unsigned int size,unsigned int startV1,unsigned int startV2);
  template <class T1,class T2,class V1,class V2>                        friend void  vec_getSubVector(const V1 &v1,V2 &v2,unsigned int size,unsigned int startV1,unsigned int startV2);
  /////////////////////////////////////

protected:
  T             data[n];

public:
  Static_Vector(void){};
  Static_Vector(T a,...);
  Static_Vector(const T* ptr);
  Static_Vector(const Static_Vector<T,n> &v);

  T &           operator [](unsigned int i);
  T             operator [](unsigned int i) const;

  Static_Vector<T,n> &  operator +=(Static_Vector<T,n> &a)
  {
    vec_generic_add<T,T>(data,a.data,n);
    return *this;
  };

  unsigned int  getSize(void) const;
  T *           getPointer(void);
};
//////////////////////////////////////////////////

/////////////////////////////////////
// Output
template <class T,unsigned int n>  ostream & operator <<(ostream &o, const Static_Vector<T,n> &v);
/////////////////////////////////////

#include "vector_static.hpp"

#endif
