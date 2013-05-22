#ifndef __vector_generic__h__
#define __vector_generic__h__

#include <math.h>               // sqrt...
#include <limits>
#include "TOOLS/debug.h"        // assert, assert_printf...
//#include <SqAssert.h>

/////////////////////////////////////
// Pseudo-constants

template <typename T> inline T Vector_Generic_Normalization_Epsilon();
template <typename T> inline T Vector_Generic_Orthogonality_Epsilon();

/////////////////////////////////////
// Generic functions on vectors

// Set
template <class T,class V1>                                           void  vec_generic_set(V1 &v1, unsigned int size,T value);
template <class T>                                                    void  vec_generic_set(T *, unsigned int size,T value);

// Null
template <class V1>                                                   void  vec_generic_null(V1 &v1,unsigned int size);
template <class T>                                                    void  vec_generic_null(T *,unsigned int size);

// Copy
template <class V1,class V2>                                          void  vec_generic_copy(V1 &v1,const V2 &v2,unsigned int size);
template <class T1,class T2>                                          void  vec_generic_copy(T1 *,const T2 *,unsigned int size);

// Addition
template <class V1,class V2>                                          void  vec_generic_add(V1 &v1,const V2 &v2,unsigned int size);
template <class T1,class T2>                                          void  vec_generic_add(T1 *,const T2 *,unsigned int size);
template <class V1,class V2,class Vres>                               void  vec_generic_add(const V1 &v1, const V2 &v2, Vres &res,unsigned int size);
template <class T1,class T2,class Tres>                               void  vec_generic_add(const T1 *v1, const T2 *v2, Tres *res,unsigned int size);

// Triadic operation res = v1 + v2*scale (or v1 += v2*scale)
template <class T,class V1,class V2>                                  void  vec_generic_triadic(V1 &v1,const V2 &v2,T scale,unsigned int size);
template <class T,class T1,class T2>                                  void  vec_generic_triadic(T1 *v1,const T2 *v2,T scale,unsigned int size);
template <class T,class V1,class V2,class Vres>                       void  vec_generic_triadic(const V1 &v1,const V2 &v2,Vres &res,T scale,unsigned int size);
template <class T,class T1,class T2,class Tres>                       void  vec_generic_triadic(const T1 *v1,const T2 *v2,Tres *res,T scale,unsigned int size);

// Substraction
template <class V1,class V2>                                          void  vec_generic_sub(V1 &v1,const V2 &v2,unsigned int size);
template <class T1,class T2>                                          void  vec_generic_sub(T1 *,const T2 *,unsigned int size);
template <class V1,class V2,class Vres>                               void  vec_generic_sub(const V1 &v1, const V2 &v2, Vres &res,unsigned int size);
template <class T1,class T2,class Tres>                               void  vec_generic_sub(const T1 *v1, const T2 *v2, Tres *res,unsigned int size);

// Opposite
template <class V1>                                                   void  vec_generic_opposite(V1 &v1,unsigned int size);
template <class T1>                                                   void  vec_generic_opposite(T1 *,unsigned int size);
template <class V1,class V2>                                          void  vec_generic_opposite(const V1 &v1,V2 &v2,unsigned int size);
template <class T1,class T2>                                          void  vec_generic_opposite(const T1 *v1,T2 *v2,unsigned int size);

// Scale
template <class T,class V1>                                           void  vec_generic_scale(V1 &v1,unsigned int size,T factor);
template <class T,class T1>                                           void  vec_generic_scale(T1 *v1,unsigned int size,T factor);
template <class T,class V1,class V2>                                  void  vec_generic_scale(const V1 &v1,V2 &v2,unsigned int size,T factor);
template <class T,class T1,class T2>                                  void  vec_generic_scale(const T1 *v1,T2 *v2,unsigned int size,T factor);

// invScale
template <class T,class V1>                                           void  vec_generic_invScale(V1 &v1,unsigned int size,T factor);
template <class T,class T1>                                           void  vec_generic_invScale(T1 *v1,unsigned int size,T factor);
template <class T,class V1,class V2>                                  void  vec_generic_invScale(const V1 &v1,V2 &v2,unsigned int size,T factor);
template <class T,class T1,class T2>                                  void  vec_generic_invScale(const T1 *v1,T2 *v2,unsigned int size,T factor);

// Multiplication element per element
template <class V1,class V2>                                          void  vec_generic_mul(V1 &v1, const V2 &v2,unsigned int size);
template <class T1,class T2>                                          void  vec_generic_mul(T1 *v1, const T2 *v2,unsigned int size);
template <class V1,class V2,class Vres>                               void  vec_generic_mul(const V1 &v1, const V2 &v2, Vres &res,unsigned int size);
template <class T1,class T2,class Tres>                               void  vec_generic_mul(const T1 *v1, const T2 *v2, Tres *res,unsigned int size);

// Dot Product
template <class T,class V1,class V2>                                  T     vec_generic_dotProduct(const V1 &v1, const V2 &v2,unsigned int size);
template <class T,class T1,class T2>                                  T     vec_generic_dotProduct(const T1 *v1, const T2 *v2,unsigned int size);

// Norm, NormSQ, Normalization
template <class T,class V1>                                           T     vec_generic_norm(const V1 &v1,unsigned int size);
template <class T,class T1>                                           T     vec_generic_norm(const T1 *v1,unsigned int size);

template <class T,class V1>                                           T     vec_generic_normSQ(const V1 &v1,unsigned int size);
template <class T,class T1>                                           T     vec_generic_normSQ(const T1 *v1,unsigned int size);

template <class T,class V1>                                           T     vec_generic_normalize(V1 &v1,unsigned int size);
template <class T,class T1>                                           T     vec_generic_normalize(T1 *v1,unsigned int size);

// Cross Product
template <class T,class V1,class V2>                                  void  vec_generic_crossProduct(V1 &v1, const V2 &v2,unsigned int size);
template <class T1,class T2>                                          void  vec_generic_crossProduct(T1 *v1, const T2 *v2,unsigned int size);
template <class V1,class V2,class Vres>                               void  vec_generic_crossProduct(const V1 &v1, const V2 &v2, Vres &res,unsigned int size);
template <class T1,class T2,class Tres>                               void  vec_generic_crossProduct(const T1 *v1, const T2 *v2, Tres *res,unsigned int size);

// Cross Product 3D
template <class T,class V1,class V2>                                  void  vec_generic_crossProduct3D(V1 &v1, const V2 &v2);
template <class T1,class T2>                                          void  vec_generic_crossProduct3D(T1 *v1, const T2 *v2);
template <class V1,class V2,class Vres>                               void  vec_generic_crossProduct3D(const V1 &v1, const V2 &v2, Vres &res);
template <class T1,class T2,class Tres>                               void  vec_generic_crossProduct3D(const T1 *v1, const T2 *v2, Tres *res);

// Clamp
template <class T,class V1>                                           void  vec_generic_clamp(V1 &v1,T min, T max,unsigned int size);
template <class T,class T1>                                           void  vec_generic_clamp(T1 *v1,T min, T max,unsigned int size);
template <class T,class V1,class Vres>                                void  vec_generic_clamp(const V1 &v1, Vres &res,T min, T max,unsigned int size);
template <class T,class T1,class Tres>                                void  vec_generic_clamp(const T1 *v1, Tres *res,T min, T max,unsigned int size);

// Test orthogonality and colinearity
template <class T,class V1,class V2>                                  bool  vec_generic_areOrthogonal(const V1 &v1,const V2 &v2,unsigned int size);
template <class T,class T1,class T2>                                  bool  vec_generic_areOrthogonal(const T1 *v1,const T2 *v2,unsigned int size);
//template <class T,class V1,class V2,class V3>                         bool  vec_generic_areColinear(const V1 &v1,const V2 &v2,const V3 &v3,unsigned int size);
//template <class T1,class T2,class T3>                                 bool  vec_generic_areColinear(const T1 *v1,const T2 *v2,const T3 *v3,unsigned int size);

// Get Sub Vector
template <class V1,class V2>                                          void  vec_generic_getSubVector(const V1 &v1,V2 &v2,unsigned int size,unsigned int startV1=0,unsigned int startV2=0);
template <class T1,class T2>                                          void  vec_generic_getSubVector(const T1 *v1,T2 *v2,unsigned int size,unsigned int startV1=0,unsigned int startV2=0);

/////////////////////////////////////

#include "vector_generic.hpp"

#endif
