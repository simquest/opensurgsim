//////////////////////////////////////////////////
// Generic functions on vectors and specialized (for Static_Vector and Dynamic_Vector classes)

// Set
template <class T,class V1>                                           void  vec_set(V1 &v1, T value)
{ vec_generic_set<T>(v1.data,v1.getSize(),value); }

// Null
template <class V1>                                                   void  vec_null(V1 &v1)
{ vec_generic_null<V1>(v1,v1.getSize()); }

template <class T,class V1>                                           void  vec_null(V1 &v1)
{ vec_generic_null<T>(v1.data,v1.getSize()); }

// copy
template <class V1,class V2>                                          void  vec_copy(V1 &v1,const V2 &v2)
{
  assert_printf(v1.getSize()<=v2.getSize(),"v1.getSize()=%d > v2.getSize()=%d",v1.getSize(),v2.getSize());
  vec_generic_copy<V1,V2>(v1,v2,v2.getSize());
}

template <class T1,class T2,class V1,class V2>                        void  vec_copy(V1 &v1,const V2 &v2)
{
  assert_printf(v1.getSize()<=v2.getSize(),"v1.getSize()=%d > v2.getSize()=%d",v1.getSize(),v2.getSize());
  vec_generic_copy<T1,T2>(v1.data,v2.data,v2.getSize());
}

// Addition
template <class V1,class V2>                                          void  vec_add(V1 &v1,const V2 &v2)
{
  assert_printf(v1.getSize()<=v2.getSize(),"v1.getSize()=%d > v2.getSize()=%d",v1.getSize(),v2.getSize());
  vec_generic_add<V1,V2>(v1,v2,v2.getSize());
}

template <class T1,class T2,class V1,class V2>                        void  vec_add(V1 &v1,const V2 &v2)
{
  assert_printf(v1.getSize()<=v2.getSize(),"v1.getSize()=%d > v2.getSize()=%d",v1.getSize(),v2.getSize());
  vec_generic_add<T1,T2>(v1.data,v2.data,v2.getSize());
}

template <class V1,class V2,class Vres>                               void  vec_add(const V1 &v1, const V2 &v2, Vres &res)
{
  assert_printf(v1.getSize() ==v2.getSize(),"v1.getSize()=%d > v2.getSize()=%d",v1.getSize(),v2.getSize());
  assert_printf(res.getSize()>=v2.getSize(),"res.getSize()=%d < v2.getSize()=%d",res.getSize(),v2.getSize());
  assert_printf(res.getSize()>=v1.getSize(),"res.getSize()=%d < v1.getSize()=%d",res.getSize(),v1.getSize());
  vec_generic_add<V1,V2,Vres>(v1,v2,res,res.getSize());
}

template <class T1,class T2,class Tres,class V1,class V2,class Vres>  void  vec_add(const V1 &v1, const V2 &v2, Vres &res)
{
  assert_printf(v1.getSize() ==v2.getSize(),"v1.getSize()=%d > v2.getSize()=%d",v1.getSize(),v2.getSize());
  assert_printf(res.getSize()>=v2.getSize(),"res.getSize()=%d < v2.getSize()=%d",res.getSize(),v2.getSize());
  assert_printf(res.getSize()>=v1.getSize(),"res.getSize()=%d < v1.getSize()=%d",res.getSize(),v1.getSize());
  vec_generic_add<T1,T2,Tres>(v1.data,v2.data,res.data,res.getSize());
}

// Triadic operation res = v1 + v2*scale (or v1 += v2*scale)
template <class T,class V1,class V2>                                  void  vec_triadic(V1 &v1,const V2 &v2,T scale)
{
  assert_printf(v1.getSize()>=v2.getSize(),"v1.getSize()=%d < v2.getSize()=%d",v1.getSize(),v2.getSize());
  vec_generic_triadic<T,V1,V2>(v1,v2,scale,v2.getSize());
}

template <class T,class T1,class T2,class V1,class V2>                void  vec_triadic(V1 &v1,const V2 &v2,T scale)
{
  assert_printf(v1.getSize()>=v2.getSize(),"v1.getSize()=%d < v2.getSize()=%d",v1.getSize(),v2.getSize());
  vec_generic_triadic<T,T1,T2>(v1.data,v2.data,scale,v2.getSize());
}

template <class T,class V1,class V2,class Vres>                       void  vec_triadic(const V1 &v1,const V2 &v2,Vres &res,T scale)
{
  assert_printf(res.getSize()>=v1.getSize(),"res.getSize()=%d < v1.getSize()=%d",res.getSize(),v1.getSize());
  assert_printf(v1.getSize()==v2.getSize(),"v1.getSize()=%d != v2.getSize()=%d",v1.getSize(),v2.getSize());
  vec_generic_triadic<T,V1,V2,Vres>(v1,v2,res,scale,v2.getSize());
}

template <class T,class T1,class T2,class Tres,class V1,class V2,class Vres>  void  vec_triadic(const V1 &v1,const V2 &v2,Vres &res,T scale)
{
  assert_printf(res.getSize()>=v1.getSize(),"res.getSize()=%d < v1.getSize()=%d",res.getSize(),v1.getSize());
  assert_printf(v1.getSize()==v2.getSize(),"v1.getSize()=%d != v2.getSize()=%d",v1.getSize(),v2.getSize());
  vec_generic_triadic<T,T1,T2,Tres>(v1.data,v2.data,res.data,scale,v2.getSize());
}

// Substraction
template <class V1,class V2>                                          void  vec_sub(V1 &v1, const V2 &v2)
{
  assert_printf(v1.getSize()<=v2.getSize(),"v1.getSize()=%d > v2.getSize()=%d",v1.getSize(),v2.getSize());
  vec_generic_sub<V1,V2>(v1,v2,v2.getSize());
}

template <class T1,class T2,class V1,class V2>                        void  vec_sub(V1 &v1, const V2 &v2)
{
  assert_printf(v1.getSize()<=v2.getSize(),"v1.getSize()=%d > v2.getSize()=%d",v1.getSize(),v2.getSize());
  vec_generic_sub<T1,T2,V1,V2>(v1.data,v2.data,v2.getSize());
}

template <class V1,class V2,class Vres>                               void  vec_sub(const V1 &v1, const V2 &v2, Vres &res)
{
  assert_printf(v1.getSize() ==v2.getSize(),"v1.getSize()=%d > v2.getSize()=%d",v1.getSize(),v2.getSize());
  assert_printf(res.getSize()>=v2.getSize(),"res.getSize()=%d < v2.getSize()=%d",res.getSize(),v2.getSize());
  assert_printf(res.getSize()>=v1.getSize(),"res.getSize()=%d < v1.getSize()=%d",res.getSize(),v1.getSize());
  vec_generic_sub<V1,V2,Vres>(v1,v2,res,res.getSize());
}

template <class T1,class T2,class Tres,class V1,class V2,class Vres>  void  vec_sub(const V1 &v1, const V2 &v2, Vres &res)
{
  assert_printf(v1.getSize() ==v2.getSize(),"v1.getSize()=%d > v2.getSize()=%d",v1.getSize(),v2.getSize());
  assert_printf(res.getSize()>=v2.getSize(),"res.getSize()=%d < v2.getSize()=%d",res.getSize(),v2.getSize());
  assert_printf(res.getSize()>=v1.getSize(),"res.getSize()=%d < v1.getSize()=%d",res.getSize(),v1.getSize());
  vec_generic_sub<T1,T2,Tres>(v1.data,v2.data,res.data,res.getSize());
}

// Opposite
template <class V1>                                                   void  vec_opposite(V1 &v1)
{ vec_generic_opposite<V1>(v1,v1.getSize()); }

template <class T,class V1>                                           void  vec_opposite(V1 &v1)
{ vec_generic_opposite<T,V1>(v1.data,v1.getSize()); }

template <class V1,class V2>                                          void  vec_opposite(const V1 &v1,V2 &v2)
{
  assert_printf(v1.getSize()<=v2.getSize(),"v1.getSize()=%d > v2.getSize()=%d",v1.getSize(),v2.getSize());
  vec_generic_opposite<V1,V2>(v1,v2,v1.getSize());
}

template <class T1,class T2,class V1,class V2>                        void  vec_opposite(const V1 &v1,V2 &v2)
{
  assert_printf(v1.getSize()<=v2.getSize(),"v1.getSize()=%d > v2.getSize()=%d",v1.getSize(),v2.getSize());
  vec_generic_opposite<T1,T2,V1,V2>(v1.data,v2.data,v1.getSize());
}

 // Scale
template <class T,class V1>                                           void  vec_scale(V1 &v1,T factor)
{ vec_generic_scale<T,V1>(v1,v1.getSize(),factor); }

template <class T1,class T,class V1>                                  void  vec_scale(V1 &v1,T factor)
{ vec_generic_scale<T,T1>(v1.data,v1.getSize(),factor); }

template <class T,class V1,class V2>                                  void  vec_scale(const V1 &v1,V2 &v2,T factor)
{
  assert_printf(v1.getSize()<=v2.getSize(),"v1.getSize()=%d > v2.getSize()=%d",v1.getSize(),v2.getSize());
  vec_generic_scale<T,V1,V2>(v1,v2,v2.getSize(),factor);
}

template <class T1,class T2,class T,class V1,class V2>                void  vec_scale(const V1 &v1,V2 &v2,T factor)
{
  assert_printf(v1.getSize()<=v2.getSize(),"v1.getSize()=%d > v2.getSize()=%d",v1.getSize(),v2.getSize());
  vec_generic_scale<T,T1,T2>(v1.data,v2.data,v2.getSize(),factor);
}

// invScale
template <class T,class V1>                                           void  vec_invScale(V1 &v1,T factor)
{ vec_generic_invScale<T,V1>(v1,v1.getSize(),factor); }

template <class T1,class T,class V1>                                  void  vec_invScale(V1 &v1,T factor)
{ vec_generic_invScale<T,T1>(v1.data,v1.getSize(),factor); }

template <class T,class V1,class V2>                                  void  vec_invScale(const V1 &v1,V2 &v2,T factor)
{
  assert_printf(v1.getSize()<=v2.getSize(),"v1.getSize()=%d > v2.getSize()=%d",v1.getSize(),v2.getSize());
  vec_generic_invScale<T,V1,V2>(v1,v2,v2.getSize(),factor);
}

template <class T1,class T2,class T,class V1,class V2>                void  vec_invScale(const V1 &v1,V2 &v2,T factor)
{
  assert_printf(v1.getSize()<=v2.getSize(),"v1.getSize()=%d > v2.getSize()=%d",v1.getSize(),v2.getSize());
  vec_generic_invScale<T,T1,T2>(v1.data,v2.data,v2.getSize(),factor);
}

// Multiplication element per element
template <class V1,class V2>                                          void  vec_mul(V1 &v1, const V2 &v2)
{
  assert_printf(v1.getSize()>=v2.getSize(),"v1.getSize()=%d < v2.getSize()=%d",v1.getSize(),v2.getSize());
  vec_generic_mul<V1,V2>(v1,v2,v2.getSize());
}

template <class T1,class T2,class V1,class V2>                        void  vec_mul(V1 &v1, const V2 &v2)
{
  assert_printf(v1.getSize()>=v2.getSize(),"v1.getSize()=%d < v2.getSize()=%d",v1.getSize(),v2.getSize());
  vec_generic_mul<T1,T2>(v1.data,v2.data,v2.getSize());
}

template <class V1,class V2,class Vres>                               void  vec_mul(const V1 &v1, const V2 &v2, Vres &res)
{
  assert_printf(v1.getSize() ==v2.getSize(),"v1.getSize()=%d > v2.getSize()=%d",v1.getSize(),v2.getSize());
  assert_printf(res.getSize()>=v2.getSize(),"res.getSize()=%d < v2.getSize()=%d",res.getSize(),v2.getSize());
  assert_printf(res.getSize()>=v1.getSize(),"res.getSize()=%d < v1.getSize()=%d",res.getSize(),v1.getSize());
  vec_generic_mul<V1,V2,Vres>(v1,v2,res,res.getSize());
}

template <class T1,class T2,class Tres,class V1,class V2,class Vres>  void  vec_mul(const V1 &v1, const V2 &v2, Vres &res)
{
  assert_printf(v1.getSize() ==v2.getSize(),"v1.getSize()=%d > v2.getSize()=%d",v1.getSize(),v2.getSize());
  assert_printf(res.getSize()>=v2.getSize(),"res.getSize()=%d < v2.getSize()=%d",res.getSize(),v2.getSize());
  assert_printf(res.getSize()>=v1.getSize(),"res.getSize()=%d < v1.getSize()=%d",res.getSize(),v1.getSize());
  vec_generic_mul<T1,T2,Vres>(v1.data,v2.data,res.data,res.getSize());
}

// Dot Product
template <class T,class V1,class V2>                                  T     vec_dotProduct(const V1 &v1, const V2 &v2)
{
  assert_printf(v1.getSize()==v2.getSize(),"v1.getSize()=%d != v2.getSize()=%d",v1.getSize(),v2.getSize());
  return vec_generic_dotProduct<T,V1,V2>(v1,v2,v2.getSize());
}

template <class T,class T1,class T2,class V1,class V2>                T     vec_dotProduct(const V1 &v1, const V2 &v2)
{
  assert_printf(v1.getSize()>=v2.getSize(),"v1.getSize()=%d < v2.getSize()=%d",v1.getSize(),v2.getSize());
  return vec_generic_dotProduct<T,T1,T2>(v1.data,v2.data,v2.getSize());
}

// Norm, NormSQ, Normalization
template <class T,class V1>                                           T     vec_norm(const V1 &v1)
{ return vec_generic_norm<T,V1>(v1,v1.getSize()); }

template <class T,class T1,class V1>                                  T     vec_norm(const V1 &v1)
{ return vec_generic_norm<T,T1>(v1.data,v1.getSize()); }

template <class T,class V1>                                           T     vec_normSQ(const V1 &v1)
{ return vec_generic_normSQ<T,V1>(v1,v1.getSize()); }

template <class T,class T1,class V1>                                  T     vec_normSQ(const V1 &v1)
{ return vec_generic_normSQ<T,T1>(v1.data,v1.getSize()); }

template <class T,class V1>                                           T     vec_normalize(V1 &v1)
{ return vec_generic_normalize<T,V1>(v1,v1.getSize()); }

template <class T,class T1,class V1>                                  T     vec_normalize(V1 &v1)
{ return vec_generic_normalize<T,T1>(v1.data,v1.getSize()); }

// Cross Product
template <class T,class V1,class V2>                                  void  vec_crossProduct(V1 &v1, const V2 &v2)
{ vec_generic_crossProduct<T,V1,V2>(v1,v2,v1.getSize()); }

template <class T1,class T2,class V1,class V2>                        void  vec_crossProduct(V1 &v1, const V2 &v2)
{ vec_generic_crossProduct<T1,T2>(v1.data,v2.data,v1.getSize()); }

template <class V1,class V2,class Vres>                               void  vec_crossProduct(const V1 &v1, const V2 &v2, Vres &res)
{ vec_generic_crossProduct<V1,V2,Vres>(v1,v2,res,res.getSize()); }

template <class T1,class T2,class Tres,class V1,class V2,class Vres>  void  vec_crossProduct(const V1 &v1, const V2 &v2, Vres &res)
{ vec_generic_crossProduct<T1,T2,Tres>(v1.data,v2.data,res.data,res.getSize()); }

// Cross Product 3D
template <class T,class V1,class V2>                                  void  vec_crossProduct3D(V1 &v1, const V2 &v2)
{ vec_generic_crossProduct3D<T,V1,V2>(v1,v2); }

template <class T1,class T2,class V1,class V2>                        void  vec_crossProduct3D(V1 &v1, const V2 &v2)
{ vec_generic_crossProduct3D<T1,T2>(v1.data,v2.data); }

template <class V1,class V2,class Vres>                               void  vec_crossProduct3D(const V1 &v1, const V2 &v2, Vres &res)
{ vec_generic_crossProduct3D<V1,V2,Vres>(v1,v2,res); }

template <class T1,class T2,class Tres,class V1,class V2,class Vres>  void  vec_crossProduct3D(const V1 &v1, const V2 &v2, Vres &res)
{ vec_generic_crossProduct3D<T1,T2,Tres>(v1.data,v2.data,res.data); }

// Clamp
template <class T,class V1>                                           void  vec_clamp(V1 &v1,T min, T max)
{ vec_generic_clamp<T,V1>(v1,min,max,v1.getSize()); }

template <class T,class T1,class V1>                                  void  vec_clamp(V1 &v1,T min, T max)
{ vec_generic_clamp<T,T1>(v1.data,min,max,v1.getSize()); }

template <class T,class V1,class Vres>                                void  vec_clamp(const V1 &v1, Vres &res,T min, T max)
{ vec_generic_clamp<T,V1,Vres>(v1,res,min,max,v1.getSize()); }

template <class T,class T1,class Tres,class V1,class Vres>            void  vec_clamp(const V1 &v1, Vres &res,T min, T max)
{ vec_generic_clamp<T,T1,Tres>(v1.data,res.data,min,max,v1.getSize()); }

// Test orthogonality and colinearity
template <class T,class V1,class V2>                                  bool  vec_areOrthogonal(const V1 &v1,const V2 &v2)
{
  assert_printf(v1.getSize()==v2.getSize(),"v1.getSize()=%d != v2.getSize()=%d",v1.getSize(),v2.getSize());
  return vec_generic_areOrthogonal<T,V1,V2>(v1,v2,v1.getSize());
}

template <class T,class T1,class T2,class V1,class V2>                bool  vec_areOrthogonal(const V1 &v1,const V2 &v2)
{
  assert_printf(v1.getSize()==v2.getSize(),"v1.getSize()=%d != v2.getSize()=%d",v1.getSize(),v2.getSize());
  return vec_generic_areOrthogonal<T,T1,T2,V1,V2>(v1.data,v2.data,v1.getSize());
}

//template <class T,class V1,class V2,class V3> bool  vec_areColinear(const V1 &v1,const V2 &v2,const V3 &v3);

// Get Sub Vector
template <class V1,class V2>                                          void  vec_getSubVector(const V1 &v1,V2 &v2,unsigned int size,unsigned int startV1,unsigned int startV2)
{
  assert_printf(v1.getSize() >= startV1+size,"v1.getSize()=%d < startV1+size (%d + %d)",v1.getSize(),startV1,size);
  assert_printf(v2.getSize() >= startV2+size,"v2.getSize()=%d < startV2+size (%d + %d)",v2.getSize(),startV2,size);
  vec_generic_getSubVector<V1,V2>(v1,v2,size,startV1,startV2);
}

template <class T1,class T2,class V1,class V2>                        void  vec_getSubVector(const V1 &v1,V2 &v2,unsigned int size,unsigned int startV1,unsigned int startV2)
{
  assert_printf(v1.getSize() >= startV1+size,"v1.getSize()=%d < startV1+size (%d + %d)",v1.getSize(),startV1,size);
  assert_printf(v2.getSize() >= startV2+size,"v2.getSize()=%d < startV2+size (%d + %d)",v2.getSize(),startV2,size);
  vec_generic_getSubVector<T1,T2>(v1.data,v2.data,size,startV1,startV2);
}

//////////////////////////////////////////////////
