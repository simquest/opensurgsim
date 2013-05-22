#ifndef __vector_specialized_static_dynamic_h__
#define __vector_specialized_static_dynamic_h__

// All those functions are based on the generic one (kind of shortcut inlined thanks to the template mechanism)
#include "./vector_generic.h"
#include "TOOLS/debug.h"        // assert, assert_printf...

/////////////////////////////////////
// Specialized functions on Static_Vector and Dynamic_Vector

// Set
template <class T,class V1>                                           void  vec_set(V1 &v1, T value);

// Null
template <class V1>                                                   void  vec_null(V1 &v1);
template <class T,class V1>                                           void  vec_null(V1 &v1);

// Copy
template <class V1,class V2>                                          void  vec_copy(V1 &v1,const V2 &v2);
template <class T1,class T2,class V1,class V2>                        void  vec_copy(V1 &v1,const V2 &v2);

// Addition
template <class V1,class V2>                                          void  vec_add(V1 &v1, const V2 &v2);
template <class T1,class T2,class V1,class V2>                        void  vec_add(V1 &v1, const V2 &v2);
template <class V1,class V2,class Vres>                               void  vec_add(const V1 &v1, const V2 &v2, Vres &res);
template <class T1,class T2,class Tres,class V1,class V2,class Vres>  void  vec_add(const V1 &v1, const V2 &v2, Vres &res);

// Triadic operation res = v1 + v2*scale (or v1 += v2*scale)
template <class T,class V1,class V2>                                  void  vec_triadic(V1 &v1,const V2 &v2,T scale);
template <class T,class T1,class T2,class V1,class V2>                void  vec_triadic(V1 &v1,const V2 &v2,T scale);
template <class T,class V1,class V2,class Vres>                       void  vec_triadic(const V1 &v1,const V2 &v2,Vres &res,T scale);
template <class T,class T1,class T2,class Tres,class V1,class V2,class Vres>  void  vec_triadic(const V1 &v1,const V2 &v2,Vres &res,T scale);

// Substraction
template <class V1,class V2>                                          void  vec_sub(V1 &v1, const V2 &v2);
template <class T1,class T2,class V1,class V2>                        void  vec_sub(V1 &v1, const V2 &v2);
template <class V1,class V2,class Vres>                               void  vec_sub(const V1 &v1, const V2 &v2, Vres &res);
template <class T1,class T2,class Tres,class V1,class V2,class Vres>  void  vec_sub(const V1 &v1, const V2 &v2, Vres &res);

// Opposite
template <class V1>                                                   void  vec_opposite(V1 &v1);
template <class T,class V1>                                           void  vec_opposite(V1 &v1);
template <class V1,class V2>                                          void  vec_opposite(const V1 &v1,V2 &v2);
template <class T1,class T2,class V1,class V2>                        void  vec_opposite(const V1 &v1,V2 &v2);

// Scale
template <class T,class V1>                                           void  vec_scale(V1 &v1,T factor);
template <class T1,class T,class V1>                                  void  vec_scale(V1 &v1,T factor);
template <class T,class V1,class V2>                                  void  vec_scale(const V1 &v1,V2 &v2,T factor);
template <class T1,class T2,class T,class V1,class V2>                void  vec_scale(const V1 &v1,V2 &v2,T factor);

// invScale
template <class T,class V1>                                           void  vec_invScale(V1 &v1,T factor);
template <class T1,class T,class V1>                                  void  vec_invScale(V1 &v1,T factor);
template <class T,class V1,class V2>                                  void  vec_invScale(const V1 &v1,V2 &v2,T factor);
template <class T1,class T2,class T,class V1,class V2>                void  vec_invScale(const V1 &v1,V2 &v2,T factor);

// Multiplication element per element
template <class V1,class V2>                                          void  vec_mul(V1 &v1, const V2 &v2);
template <class T1,class T2,class V1,class V2>                        void  vec_mul(V1 &v1, const V2 &v2);
template <class V1,class V2,class Vres>                               void  vec_mul(const V1 &v1, const V2 &v2, Vres &res);
template <class T1,class T2,class Tres,class V1,class V2,class Vres>  void  vec_mul(const V1 &v1, const V2 &v2, Vres &res);

// Dot Product
template <class T,class V1,class V2>                                  T     vec_dotProduct(const V1 &v1, const V2 &v2);
template <class T,class T1,class T2,class V1,class V2>                T     vec_dotProduct(const V1 &v1, const V2 &v2);

// Norm, NormSQ, Normalization
template <class T,class V1>                                           T     vec_norm(const V1 &v1);
template <class T,class T1,class V1>                                  T     vec_norm(const V1 &v1);
template <class T,class V1>                                           T     vec_normSQ(const V1 &v1);
template <class T,class T1,class V1>                                  T     vec_normSQ(const V1 &v1);
template <class T,class V1>                                           T     vec_normalize(V1 &v1);
template <class T,class T1,class V1>                                  T     vec_normalize(V1 &v1);

// Cross Product
template <class T,class V1,class V2>                                  void  vec_crossProduct(V1 &v1, const V2 &v2);
template <class T1,class T2,class V1,class V2>                        void  vec_crossProduct(V1 &v1, const V2 &v2);
template <class V1,class V2,class Vres>                               void  vec_crossProduct(const V1 &v1, const V2 &v2, Vres &res);
template <class T1,class T2,class Tres,class V1,class V2,class Vres>  void  vec_crossProduct(const V1 &v1, const V2 &v2, Vres &res);

// Cross Product 3D
template <class T,class V1,class V2>                                  void  vec_crossProduct3D(V1 &v1, const V2 &v2);
template <class T1,class T2,class V1,class V2>                        void  vec_crossProduct3D(V1 &v1, const V2 &v2);
template <class V1,class V2,class Vres>                               void  vec_crossProduct3D(const V1 &v1, const V2 &v2, Vres &res);
template <class T1,class T2,class Tres,class V1,class V2,class Vres>  void  vec_crossProduct3D(const V1 &v1, const V2 &v2, Vres &res);

// Clamp
template <class T,class V1>                                           void  vec_clamp(V1 &v1,T min, T max);
template <class T,class T1,class V1>                                  void  vec_clamp(V1 &v1,T min, T max);
template <class T,class V1,class Vres>                                void  vec_clamp(const V1 &v1, Vres &res,T min, T max);
template <class T,class T1,class Tres,class V1,class Vres>            void  vec_clamp(const V1 &v1, Vres &res,T min, T max);

// Test orthogonality and colinearity
template <class T,class V1,class V2>                                  bool  vec_areOrthogonal(const V1 &v1,const V2 &v2);
template <class T,class T1,class T2,class V1,class V2>                bool  vec_areOrthogonal(const V1 &v1,const V2 &v2);
//template <class T,class V1,class V2,class V3> bool  vec_areColinear(const V1 &v1,const V2 &v2,const V3 &v3);

// Get Sub Vector
template <class V1,class V2>                                          void  vec_getSubVector(const V1 &v1,V2 &v2,unsigned int size,unsigned int startV1=0,unsigned int startV2=0);
template <class T1,class T2,class V1,class V2>                        void  vec_getSubVector(const V1 &v1,V2 &v2,unsigned int size,unsigned int startV1=0,unsigned int startV2=0);
/////////////////////////////////////

#include "vector_specialized_static_dynamic.hpp"

#endif
