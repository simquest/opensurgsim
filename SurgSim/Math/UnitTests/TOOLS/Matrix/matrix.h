#ifndef __matrix_h__
#define __matrix_h__

#include <TOOLS/allocator.h>
#include <TOOLS/debug.h>        // assert, assert_printf...
#include <iostream> // ostream
using namespace std;

#define EPSILON 0.00001

//////////////////////////////////////////////////
// Dynamic_Matrix class
template <class T, class Allocator=Allocator_malloc<T> > class Dynamic_Matrix
{
protected:
  T *           data;
  unsigned int  nbRow,nbColumn,nbElem,nbAllocatedElem;

public:
  typedef T value_type;

  Dynamic_Matrix(void):data(0),nbRow(0),nbColumn(0),nbElem(0),nbAllocatedElem(0){};
  Dynamic_Matrix(unsigned int _nbRow,unsigned int _nbColumn);
  Dynamic_Matrix(unsigned int _nbRow,unsigned int _nbColumn,T a,...); // Defines the matrix element row by row
  Dynamic_Matrix(const Dynamic_Matrix& m);
  ~Dynamic_Matrix();

  Dynamic_Matrix& operator=(const Dynamic_Matrix& m);

  /// Grow the internally allocated memory to the specified number of elements, unless there are already more allocated.
  /// Does not change the size of the matrix, but existing data may be overwritten with garbage.
  inline void   reserve(unsigned int storageSize);
  /// Grow the internally allocated memory to the specified number of rows/columns, unless there are already more elements allocated.
  /// Does not change the size of the matrix, but existing data may be overwritten with garbage.
  inline void   reserve(unsigned int rows, unsigned int cols) { reserve(rows * cols); }

  /// Resize the matrix.  Grow or shrink the allocated memory to exactly match the matrix size.
  /// Existing data may be overwritten with garbage.
  inline void   resizeSetMemory(unsigned int newNumRows, unsigned int newNumColumns);
  /// Resize the matrix.  Grow the allocated memory if needed for the matrix to fit.
  /// Existing data may be overwritten with garbage.
  inline void   resizeGrowMemory(unsigned int newNumRows, unsigned int newNumColumns);

  /// Increase the matrix's row and/or column sizes if they are smaller than the specified sizes.
  /// Does not change the size in each dimension if that dimension is already larger than requested!
  /// Existing data may be overwritten with garbage.
  inline void   resizeToAtLeast(unsigned int minNumRows, unsigned int minNumColumns);

  /// Resize the matrix.  (Currently the semantics are the same as resizeSetMemory(), but this will probably change!)
  //  XXX TODO: change the semantics to resizeGrowMemory().
  void          resize(unsigned int newNumRows, unsigned int newNumColumns) { resizeSetMemory(newNumRows, newNumColumns); }

  /// Resize the vector if it's smaller than the specified size.  An alias for Dynamic_Matrix::resizeGrowMemory().
  /// Note that the semantics of this method are quite different than the semantics of Dynamic_Vector::resizeUpIfNecessary(), which is confusing!
  //  XXX TODO: get rid of this, and replace uses with one of the other applicable methods.
  void          resizeUpIfNecessary(unsigned int newNumRows, unsigned int newNumColumns) { resizeGrowMemory(newNumRows, newNumColumns); }

  T *           operator [](unsigned int numRow);
  const T *     operator [](unsigned int numRow) const;

  unsigned int  getNbRow(void) const;
  unsigned int  getNbColumn(void) const;
  unsigned int  getNbElem(void) const;

  T *           getPointer(void);   // Retrieve a pointer on all the elements...row by row
  const T *     getPointer(void) const;   // Retrieve a pointer on all the elements...row by row
};
//////////////////////////////////////////////////


//////////////////////////////////////////////////
// Static_Matrix class
template <class T,unsigned int nbRow,unsigned int nbColumn> class Static_Matrix
{
protected:
  T             data[nbRow][nbColumn];
  unsigned int  nbElem;

public:  
  typedef T value_type;

  Static_Matrix();
  Static_Matrix(T a,...); // Defines the matrix element row by row
  Static_Matrix(const Static_Matrix<T,nbRow,nbColumn> &m);

  T *           operator [](unsigned int numRow);
  T *           operator [](unsigned int numRow) const;

  unsigned int  getNbRow(void) const;
  unsigned int  getNbColumn(void) const;
  unsigned int  getNbElem(void) const;

  T *           getPointer(void);     // Retrieve a pointer on all the elements...row by row
  T *           getPointer(void) const;     // Retrieve a pointer on all the elements...row by row
};
//////////////////////////////////////////////////


//#define MAY_THROW throw(...)  // Hello, Microsoft!
#define MAY_THROW

/////////////////////////////////////
// Generic functions on matrices
  // Set
template <class M1,class Mres>                                        void  mat_copy(const M1 &m1, Mres &res);
template <class M1,class Mres>                                        void  mat_copy(const M1 &m1, Mres &res,unsigned int nbRow,unsigned int nbColumn);
template <class T1,class Tres,class M1,class Mres>                    void  mat_copy(const M1 &m1, Mres &res);
template <class T1,class Tres,class M1,class Mres>                    void  mat_copy(const M1 &m1, Mres &res,unsigned int nbRow,unsigned int nbColumn);
template <class T,class M1>                                           void  mat_null(M1 &m1);
template <class T,class M1>                                           void  mat_null(M1 &m1, unsigned int nbRow, unsigned int nbCol, unsigned int nbColAllocated);
template <class T,class M1>                                           void  mat_set(M1 &m1,T value);

template <class T,class M1>                                           void  mat_identity(M1 &m1);
template <class M1>													  void  mat_identity(M1 &m1, unsigned int nbRow, unsigned int nbCol);

template <class T,class M1>                                           void  mat_clamp(M1 &m1,T value,T min,T max);
  // Arithmetic operations
template <class T1,class T2,class M1,class M2>                        void  mat_add(M1 &m1, const M2 &m2); // Optimized
template <class M1,class M2>                                          void  mat_add(M1 &m1, const M2 &m2); // Generic but already very fast !
template <class M1,class M2,class Mres>                               void  mat_add(const M1 &m1, const M2 &m2, Mres &res);
template <class T1,class T2,class Tres,class M1,class M2,class Mres>  void  mat_add(const M1 &m1, const M2 &m2, Mres &res);
template <class M1,class M2>										  void  mat_add(M1 &m1, const M2 &m2, unsigned int nbRow, unsigned int nbCol);
template <class M1,class M2,class Mres>								  void  mat_add(const M1 &m1, const M2 &m2, Mres &res, unsigned int nbRow, unsigned int nbCol);


template <class T1,class T2,class M1,class M2>                        void  mat_sub(M1 &m1, const M2 &m2); // Optimized
template <class M1,class M2>                                          void  mat_sub(M1 &m1, const M2 &m2); // Generic but already very fast !
template <class M1,class M2,class Mres>                               void  mat_sub(const M1 &m1, const M2 &m2, Mres &res);
template <class T1,class T2,class Tres,class M1,class M2,class Mres>  void  mat_sub(const M1 &m1, const M2 &m2, Mres &res);
template <class M1,class M2>										  void  mat_sub(M1 &m1, const M2 &m2, unsigned int nbRow, unsigned int nbCol);
template <class M1,class M2,class Mres>								  void  mat_sub(const M1 &m1, const M2 &m2, Mres &res, unsigned int nbRow, unsigned int nbCol);

template <class T,class M1>                         void  mat_scale(M1 &m1, const T value);
template <class T,class M1,class Mres>              void  mat_scale(const M1 &m1, const T value, Mres &res) MAY_THROW;
template <class T,class M1>							void  mat_scale(M1 &m1, const T value, unsigned int nbRow, unsigned int nbCol);
template <class T,class M1,class Mres>				void  mat_scale(const M1 &m1, const T value, Mres &res, unsigned int nbRow, unsigned int nbCol) MAY_THROW;
template <class T,class M1>                         void  mat_invScale(M1 &m1, const T value) MAY_THROW;
template <class T,class M1,class Mres>              void  mat_invScale(const M1 &m1, const T value, Mres &res) MAY_THROW;
template <class T,class M1>							void  mat_invScale(M1 &m1, const T value, unsigned int nbRow, unsigned int nbCol);
template <class T,class M1,class Mres>				void  mat_invScale(const M1 &m1, const T value, Mres &res, unsigned int nbRow, unsigned int nbCol) MAY_THROW;
  // Multiplication
template <class M1,class M2,class Mres>             void  mat_mul(const M1 &m1,const M2 &m2, Mres &res) MAY_THROW;
template <class M1,class M2,class Mres>             void  mat_mul(const M1 &m1,const M2 &m2, Mres &res, unsigned int nbRow, unsigned int nbCol,unsigned int nbIntermediateCol) MAY_THROW;
template <class M1,class M2,class Mres>             void  mat_mul(const M1 &m1,const M2 &m2, Mres &res, unsigned int nbRow, unsigned int nbCol);
  // Inversion
template <class T,class M>                          void  mat_swapRows(M &m,unsigned int i,unsigned int j);
template <class T,class M>                          void  mat_swapRows(M &m,unsigned int i,unsigned int j,int dim);
//template <class T,class M>                          T     mat_coMatrix(const M &m);
//template <class T,class M>                          T     mat_determinant(const M &m);
#if 0  // ***DISABLED BAD CODE*** (see comments in .hpp)
// As an alternative, use the 4-arg (LU decomposition-based) version of mat_invert, which should work correctly.
template <class T,class M1,class Mres,class Mtmp>   void  mat_invert(const M1 &m1,Mres &res,Mtmp &tmp);
#endif // ***DISABLED BAD CODE***
template <class T,class M,class Minv,class MLU,class VLUperm>   void  mat_invert(const M &m,Minv &mInv,MLU &mLU,VLUperm &vLUPerm);
template <class T,class M,class Minv,class MLU,class VLUperm>   void  mat_invert(const M &m,Minv &mInv,MLU &mLU,VLUperm &vLUPerm,unsigned int dim);
template <class T,class M,class Minv,class MLU,class VLUperm>   void  mat_invert_band(const M &m,Minv &mInv,MLU &mLU,VLUperm &vLUPerm,unsigned int bandWidth);
template <class T,class M,class Minv,class MLU,class VLUperm>   void  mat_invert_band(const M &m,Minv &mInv,MLU &mLU,VLUperm &vLUPerm,unsigned int dim,unsigned int bandWidth);

template <class T,class M,class ML,class MU,class VPerm>   void  mat_LU_decomposition(const M &m1,ML &l,MU &u,VPerm &permutation);
template <class T,class M,class MLU,class Vperm>   void  mat_LU_decomposition(const M &a,MLU &LU,Vperm &permutation);
template <class T,class M,class MLU,class Vperm>   void  mat_LU_decomposition(const M &a,MLU &LU,Vperm &permutation,unsigned int dim);
template <class T,class M,class MLU,class Vperm>   void  mat_LU_decomposition_band(const M &a,MLU &LU,Vperm &permutation,unsigned int bandWidth);
template <class T,class M,class MLU,class Vperm>   void  mat_LU_decomposition_band(const M &a,MLU &LU,Vperm &permutation,unsigned int dim,unsigned int bandWidth);

template <class T,class MLU,class Vperm,class V1,class V2>   void  mat_solveWithLU(const MLU &LU,const Vperm &t,const V1 &b,V2 &x);
template <class T,class MLU,class Vperm>   void  mat_solveWithLU_strip(const MLU &LU,const Vperm &t,T *x,unsigned int strip);
template <class T,class MLU,class Vperm>   void  mat_solveWithLU(const MLU &LU,const Vperm &t,T *x);
template <class T,class MLU,class Vperm>   void  mat_solveWithLU(const MLU &LU,const Vperm &t,T *x,unsigned dim);
template <class T,class MLU,class Vperm>   void  mat_solveWithLU_band(const MLU &LU,const Vperm &t,T *x,unsigned int bandWidth);
template <class T,class MLU,class Vperm>   void  mat_solveWithLU_band(const MLU &LU,const Vperm &t,T *x,unsigned dim,unsigned int bandWidth);

  // Transposition
template <class M1,class Mres>                      void  mat_transpose(const M1 &m1, Mres &res) MAY_THROW;
template <class M1>                                 void  mat_transpose(M1 &m1);
template <class M1>                                 void  mat_transpose(M1 &m1,unsigned int dim);
template <class T,class M1>                         void  mat_transpose(M1 &a,unsigned int dim);
template <class M1,class MRes>              void  mat_transpose(const M1 &a,unsigned int nbRow,unsigned int nbCol,MRes &res);
  // Conditionment

/////////////////////////////////////


/////////////////////////////////////
// Generic functions 3D environnement (4D matrices)
  // 3D
template <class T,class Mres>             void  mat_rotationX3D(T angle, Mres &res) MAY_THROW;
template <class T,class Mres>             void  mat_rotationY3D(T angle, Mres &res) MAY_THROW;
template <class T,class Mres>             void  mat_rotationZ3D(T angle, Mres &res) MAY_THROW;
  // 4D
template <class T,class Mres>             void  mat_rotationX4D(T angle, Mres &res) MAY_THROW;
template <class T,class Mres>             void  mat_rotationY4D(T angle, Mres &res) MAY_THROW;
template <class T,class Mres>             void  mat_rotationZ4D(T angle, Mres &res) MAY_THROW;
template <class V1,class Mres>            void  mat_translation4D(const V1 &t, Mres &res) MAY_THROW;
template <class V1,class Mres>            void  mat_scale4D(const V1 &s, Mres &res) MAY_THROW;
/////////////////////////////////////


/////////////////////////////////////
// Output
template <class T, class Allocator> ostream & operator <<(ostream &o, const Dynamic_Matrix<T,Allocator> &m);
template <class T,unsigned int nbRow,unsigned int nbColumn> ostream & operator <<(ostream &o, const Static_Matrix<T,nbRow,nbColumn> &m);
/////////////////////////////////////

#include "matrix.hpp"

#endif
