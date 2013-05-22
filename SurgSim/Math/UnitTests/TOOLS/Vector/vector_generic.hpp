/** The smallest allowed magnitude for a vector being normalized.
 */
template <typename T>
inline T Vector_Generic_Normalization_Epsilon()
{
	// This is pretty conservative-- we CAN normalize much smaller vectors.
	// But below epsilon, we run the risk of nasty cancellation etc. numerical errors, so let's discourage that kind of thing.
	return std::numeric_limits<T>::epsilon();
}

/** The threshold used for considering orthogonality.
 */
template <typename T>
inline T Vector_Generic_Orthogonality_Epsilon()
{
	return (T) 1e-7;
}

template <class T> T my_sqrt(T value)
{ return (T)sqrt((double)value); }

// Set
template <class T,class V1>                                           void  vec_generic_set(V1 &v1, unsigned int size,T value)
{ for(unsigned int i=0; i<size; ++i) v1[i]=value; }

template <class T>                                                    void  vec_generic_set(T *v1, unsigned int size,T value)
{ for(unsigned int i=0; i<size; ++i) v1[i]=value; }

// Null
template <class V1>                                                   void  vec_generic_null(V1 &v1,unsigned int size)
{ for(unsigned int i=0; i<size; ++i) v1[i]=0; }

template <class T>                                                    void  vec_generic_null(T *v1, unsigned int size)
{ for(unsigned int i=0; i<size; ++i) v1[i]=0; }

// Copy
template <class V1,class V2>                                          void  vec_generic_copy(V1 &v1,const V2 &v2,unsigned int size)
{ for(unsigned int i=0; i<size; ++i) v1[i]=v2[i]; }

template <class T1,class T2>                                          void  vec_generic_copy(T1 *v1,const T2 *v2,unsigned int size)
{ for(unsigned int i=0; i<size; ++i) v1[i]=v2[i]; }

// Addition
template <class V1,class V2>                                          void  vec_generic_add(V1 &v1,const V2 &v2,unsigned int size)
{ for(unsigned int i=0; i<size; ++i) v1[i]+=v2[i]; }

template <class T1,class T2>                                          void  vec_generic_add(T1 *v1,const T2 *v2,unsigned int size)
{ for(unsigned int i=0; i<size; ++i) v1[i]+=v2[i]; }

template <class V1,class V2,class Vres>                               void  vec_generic_add(const V1 &v1, const V2 &v2, Vres &res,unsigned int size)
{ for(unsigned int i=0; i<size; ++i) res[i]=v1[i]+v2[i]; }

template <class T1,class T2,class Tres>                               void  vec_generic_add(const T1 *v1, const T2 *v2, Tres *res,unsigned int size)
{ for(unsigned int i=0; i<size; ++i) res[i]=v1[i]+v2[i]; }

// Triadic operation res = v1 + v2*scale (or v1 += v2*scale)
template <class T,class V1,class V2>                                  void  vec_generic_triadic(V1 &v1,const V2 &v2,T scale,unsigned int size)
{ for(unsigned int i=0; i<size; ++i) v1[i]+=v2[i]*scale; }

template <class T,class T1,class T2>                                  void  vec_generic_triadic(T1 *v1,const T2 *v2,T scale,unsigned int size)
{ for(unsigned int i=0; i<size; ++i) v1[i]+=v2[i]*scale; }

template <class T,class V1,class V2,class Vres>                       void  vec_generic_triadic(const V1 &v1,const V2 &v2,Vres &res,T scale,unsigned int size)
{ for(unsigned int i=0; i<size; ++i) res[i]=v1[i]+v2[i]*scale; }

template <class T,class T1,class T2,class Tres>                       void  vec_generic_triadic(const T1 *v1,const T2 *v2,Tres *res,T scale,unsigned int size)
{ for(unsigned int i=0; i<size; ++i) res[i]=v1[i]+v2[i]*scale; }

// Substraction
template <class V1,class V2>                                          void  vec_generic_sub(V1 &v1,const V2 &v2,unsigned int size)
{ for(unsigned int i=0; i<size; ++i) v1[i]-=v2[i]; }

template <class T1,class T2>                                          void  vec_generic_sub(T1 *v1,const T2 *v2,unsigned int size)
{ for(unsigned int i=0; i<size; ++i) v1[i]-=v2[i]; }

template <class V1,class V2,class Vres>                               void  vec_generic_sub(const V1 &v1, const V2 &v2, Vres &res,unsigned int size)
{ for(unsigned int i=0; i<size; ++i) res[i]=v1[i]-v2[i]; }

template <class T1,class T2,class Tres>                               void  vec_generic_sub(const T1 *v1, const T2 *v2, Tres *res,unsigned int size)
{ for(unsigned int i=0; i<size; ++i) res[i]=v1[i]-v2[i]; }

// Opposite
template <class V1>                                                   void  vec_generic_opposite(V1 &v1,unsigned int size)
{ for(unsigned int i=0; i<size; ++i) v1[i]=-v1[i]; }

template <class T1>                                                   void  vec_generic_opposite(T1 *v1,unsigned int size)
{ for(unsigned int i=0; i<size; ++i) v1[i]=-v1[i]; }

template <class V1,class V2>                                          void  vec_generic_opposite(const V1 &v1,V2 &v2,unsigned int size)
{ for(unsigned int i=0; i<size; ++i) v2[i] = -v1[i]; }

template <class T1,class T2>                                          void  vec_generic_opposite(const T1 *v1,T2 *v2,unsigned int size)
{ for(unsigned int i=0; i<size; ++i) v2[i] = -v1[i]; }

// Scale
template <class T,class V1>                                           void  vec_generic_scale(V1 &v1,unsigned int size,T factor)
{ for(unsigned int i=0; i<size; ++i) v1[i] *= factor; }

template <class T,class T1>                                           void  vec_generic_scale(T1 *v1,unsigned int size,T factor)
{ for(unsigned int i=0; i<size; ++i) v1[i] *= factor; }

template <class T,class V1,class V2>                                  void  vec_generic_scale(const V1 &v1,V2 &v2,unsigned int size,T factor)
{ for(unsigned int i=0; i<size; ++i) v2[i] = v1[i] * factor; }

template <class T,class T1,class T2>                                  void  vec_generic_scale(const T1 *v1,T2 *v2,unsigned int size,T factor)
{ for(unsigned int i=0; i<size; ++i) v2[i] = v1[i] * factor; }

// invScale
template <class T,class V1>                                           void  vec_generic_invScale(V1 &v1,unsigned int size,T factor)
{ for(unsigned int i=0; i<size; ++i) v1[i] /= factor; }

template <class T,class T1>                                           void  vec_generic_invScale(T1 *v1,unsigned int size,T factor)
{ for(unsigned int i=0; i<size; ++i) v1[i] /= factor; }

template <class T,class V1,class V2>                                  void  vec_generic_invScale(const V1 &v1,V2 &v2,unsigned int size,T factor)
{ for(unsigned int i=0; i<size; ++i) v2[i] = v1[i] / factor; }

template <class T,class T1,class T2>                                  void  vec_generic_invScale(const T1 *v1,T2 *v2,unsigned int size,T factor)
{ for(unsigned int i=0; i<size; ++i) v2[i] = v1[i] / factor; }

// Multiplication element per element
template <class V1,class V2>                                          void  vec_generic_mul(V1 &v1,const V2 &v2,unsigned int size)
{ for(unsigned int i=0; i<size; ++i) v1[i]*=v2[i]; }

template <class T1,class T2>                                          void  vec_generic_mul(T1 *v1,const T2 *v2,unsigned int size)
{ for(unsigned int i=0; i<size; ++i) v1[i]*=v2[i]; }

template <class V1,class V2,class Vres>                               void  vec_generic_mul(const V1 &v1, const V2 &v2, Vres &res,unsigned int size)
{ for(unsigned int i=0; i<size; ++i) res[i]=v1[i]*v2[i]; }

template <class T1,class T2,class Tres>                               void  vec_generic_mul(const T1 *v1, const T2 *v2, Tres *res,unsigned int size)
{ for(unsigned int i=0; i<size; ++i) res[i]=v1[i]*v2[i]; }

// Dot Product
template <class T,class V1,class V2>                                  T     vec_generic_dotProduct(const V1 &v1, const V2 &v2,unsigned int size)
{ T res=0; for(unsigned int i=0; i<size; ++i) res += v1[i]*v2[i]; return res; }
template <class T,class T1,class T2>                                  T     vec_generic_dotProduct(const T1 *v1, const T2 *v2,unsigned int size)
{ T res=0; for(unsigned int i=0; i<size; ++i) res += v1[i]*v2[i]; return res; }

// Norm, NormSQ, Normalization
template <class T,class V1>                                           T     vec_generic_norm(const V1 &v1,unsigned int size)
{ return my_sqrt( vec_generic_normSQ<T,V1>(v1,size)); }

template <class T,class T1>                                           T     vec_generic_norm(const T1 *v1,unsigned int size)
{ return my_sqrt( vec_generic_normSQ<T,T1>(v1,size) ); }

template <class T,class V1>                                           T     vec_generic_normSQ(const V1 &v1,unsigned int size)
{ T res=0; for(unsigned int i=0; i<size; ++i) res += v1[i]*v1[i]; return res; }

template <class T,class T1>                                           T     vec_generic_normSQ(const T1 *v1,unsigned int size)
{ T res=0; for(unsigned int i=0; i<size; ++i) res += v1[i]*v1[i]; return res; }

template <class T,class V1>                                           T     vec_generic_normalize(V1 &v1,unsigned int size)
{
	T n = vec_generic_norm<T,V1>(v1,size);
	if (n >= Vector_Generic_Normalization_Epsilon<T>())
	{
		for (unsigned int i=0; i<size; ++i)
			v1[i] /= n;
		return n;
	}
	else
	{
		// What do we want here?  Assertion failure?  Some arbitrary unit vector?  Unnormalized input?  Zero?
		// The problem is, it's hard to decide without knowing what the calling code wants to do...
		// JL 2011-10-25
		// NOT EASY TO SEE SOMETHING IN THE CONSOLE WHEN WE HAVE ZILIONS OF WARNING...SO, USE SQ_WARNING_ONCE !
		SQ_WARNING_ONCE("Tried to normalize a null vector");
		for (unsigned int i=0; i<size; ++i)
			v1[i] = 0;
		return 0;
	}
}

template <class T,class T1>                                           T     vec_generic_normalize(T1 *v1,unsigned int size)
{
	T n = vec_generic_norm<T,T1>(v1,size);
	if (n >= Vector_Generic_Normalization_Epsilon<T>())
	{
		for (unsigned int i=0; i<size; ++i)
			v1[i] /= n;
		return n;
	}
	else
	{
		// What do we want here?  Assertion failure?  Some arbitrary unit vector?  Unnormalized input?  Zero?
		// The problem is, it's hard to decide without knowing what the calling code wants to do...
		SQ_WARNING("Tried to normalize a null vector");
		for (unsigned int i=0; i<size; ++i)
			v1[i] = 0;
		return 0;
	}
}

// Cross Product
template <class T,class V1,class V2>                                  void  vec_generic_crossProduct(V1 &v1, const V2 &v2,unsigned int size)
{
  assert_printf(size>=3,"Cross product is relevant only for dimension >= 3");
  T tmp_n_1 = v1[0]*v2[1] - v1[1]*v2[0];
  T tmp_n_2 = v1[size-1]*v2[0] - v1[0]*v2[size-1];
  size-=2;
  for(unsigned int i=0;i<size;i++)
    v1[i] = v1[i+1]*v2[i+2] - v1[i+2]*v2[i+1];
  v1[size]   = tmp_n_2;
  v1[size+1] = tmp_n_1;
}

template <class T1,class T2>                                          void  vec_generic_crossProduct(T1 *v1, const T2 *v2,unsigned int size)
{
  assert_printf(size>=3,"Cross product is relevant only for dimension >= 3");
  T1 tmp_n_1 = v1[0]*v2[1] - v1[1]*v2[0];
  T1 tmp_n_2 = v1[size-1]*v2[0] - v1[0]*v2[size-1];
  size-=2;
  for(unsigned int i=0;i<size;i++)
    v1[i] = v1[i+1]*v2[i+2] - v1[i+2]*v2[i+1];
  v1[size]   = tmp_n_2;
  v1[size+1] = tmp_n_1;
}

template <class V1,class V2,class Vres>                               void  vec_generic_crossProduct(const V1 &v1, const V2 &v2, Vres &res,unsigned int size)
{
  assert_printf(size>=3,"Cross product is relevant only for dimension >= 3");
  res[size-1] = v1[0]*v2[1] - v1[1]*v2[0];
  res[size-2] = v1[size-1]*v2[0] - v1[0]*v2[size-1];
  size-=2;
  for(unsigned int i=0; i<size; ++i)
    res[i] = v1[size+1]*v2[size+2] - v1[size+2]*v2[size+1];
}

template <class T1,class T2,class Tres>                               void  vec_generic_crossProduct(const T1 *v1, const T2 *v2, Tres *res,unsigned int size)
{
  assert_printf(size>=3,"Cross product is relevant only for dimension >= 3");
  res[size-1] = v1[0]*v2[1] - v1[1]*v2[0];
  res[size-2] = v1[size-1]*v2[0] - v1[0]*v2[size-1];
  size-=2;
  for(unsigned int i=0; i<size; ++i)
    res[i] = v1[size+1]*v2[size+2] - v1[size+2]*v2[size+1];
}

// Cross Product 3D
template <class T,class V1,class V2>                                  void  vec_generic_crossProduct3D(V1 &v1, const V2 &v2)
{
  T tmp0=v1[1]*v2[2] - v1[2]*v2[1],tmp1=v1[2]*v2[0] - v1[0]*v2[2];
  v1[2] = v1[0]*v2[1] - v1[1]*v2[0];
  v1[0]=tmp0;
  v1[1]=tmp1;
}

template <class T1,class T2>                                          void  vec_generic_crossProduct3D(T1 *v1, const T2 *v2)
{
  T1 tmp0=v1[1]*v2[2] - v1[2]*v2[1],tmp1=v1[2]*v2[0] - v1[0]*v2[2];
  v1[2] = v1[0]*v2[1] - v1[1]*v2[0];
  v1[0]=tmp0;
  v1[1]=tmp1;
}

template <class V1,class V2,class Vres>                               void  vec_generic_crossProduct3D(const V1 &v1, const V2 &v2, Vres &res)
{
  res[0] = v1[1]*v2[2] - v1[2]*v2[1];
  res[1] = v1[2]*v2[0] - v1[0]*v2[2];
  res[2] = v1[0]*v2[1] - v1[1]*v2[0];
}

template <class T1,class T2,class Tres>                               void  vec_generic_crossProduct3D(const T1 *v1, const T2 *v2, Tres *res)
{
  res[0] = v1[1]*v2[2] - v1[2]*v2[1];
  res[1] = v1[2]*v2[0] - v1[0]*v2[2];
  res[2] = v1[0]*v2[1] - v1[1]*v2[0];
}

// Clamp
template <class T,class V1>                                           void  vec_generic_clamp(V1 &v1,T min, T max,unsigned int size)
{
  for(unsigned int i=0; i<size; ++i)
  {
    if(v1[i]<=min) v1[i]=min;
    else if(v1[i]>=max) v1[i]=max;
  }
}

template <class T,class T1>                                           void  vec_generic_clamp(T1 *v1,T min, T max,unsigned int size)
{
  for(unsigned int i=0; i<size; ++i)
  {
    if(v1[i]<=min) v1[i]=min;
    else if(v1[i]>=max) v1[i]=max;
  }
}

template <class T,class V1,class Vres>                                void  vec_generic_clamp(const V1 &v1, Vres &res,T min, T max,unsigned int size)
{
  for(unsigned int i=0; i<size; ++i)
  {
    if(v1[i]<=min) res[i]=min;
    else if(v1[i]>=max) res[i]=max;
    else res[i]=v1[i];
  }
}

template <class T,class T1,class Tres>                                void  vec_generic_clamp(const T1 *v1, Tres *res,T min, T max,unsigned int size)
{
  for(unsigned int i=0; i<size; ++i)
  {
    if(v1[i]<=min) res[i]=min;
    else if(v1[i]>=max) res[i]=max;
    else res[i]=v1[i];
  }
}

// Test orthogonality and colinearity
template <class T,class V1,class V2>                                  bool  vec_generic_areOrthogonal(const V1 &v1,const V2 &v2,unsigned int size)
{
  T temp=vec_generic_dotProduct<T>(v1,v2,size);
  if (temp >= -Vector_Generic_Orthogonality_Epsilon<T>() && temp <= Vector_Generic_Orthogonality_Epsilon<T>() )
    return true;
  return false;
}

template <class T,class T1,class T2>                                  bool  vec_generic_areOrthogonal(const T1 *v1,const T2 *v2,unsigned int size)
{
  T temp=vec_generic_dotProduct<T>(v1,v2,size);
  if (temp >= -Vector_Generic_Orthogonality_Epsilon<T>() && temp <= Vector_Generic_Orthogonality_Epsilon<T>() )
    return true;
  return false;
}
/*
template <class T,class V1,class V2,class V3>                         bool  vec_generic_areColinear(const V1 &v1,const V2 &v2,const V3 &v3,unsigned int size)
{
  Dynamic_Vector<T> v1v2(v1Size),v1v3(v1Size),crossProd(v1Size);
  vec_sub(v2,v1,v1v2);
  vec_sub(v3,v1,v1v3);
  crossProduct(v1v2,v1v3,crossProd);
  if( normSQ<T>(crossProd)<=EPSILON )
    return true;
  return false;
}

template <class T1,class T2,class T3>                                 bool  vec_generic_areColinear(const T1 *v1,const T2 *v2,const T3 *v3,unsigned int size)
{
  return false;
}
*/
// Get Sub Vector
template <class V1,class V2>                                          void  vec_generic_getSubVector(const V1 &v1,V2 &v2,unsigned int size,unsigned int startV1,unsigned int startV2)
{ for(unsigned int i=0; i<size; ++i) v2[startV2+i] = v1[startV1+i]; }

template <class T1,class T2>                                          void  vec_generic_getSubVector(const T1 *v1,T2 *v2,unsigned int size,unsigned int startV1,unsigned int startV2)
{ for(unsigned int i=0; i<size; ++i) v2[startV2+i] = v1[startV1+i]; }
