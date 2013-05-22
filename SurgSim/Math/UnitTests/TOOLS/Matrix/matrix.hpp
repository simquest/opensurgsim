#include <math.h>   // sqrt...
#include <stdio.h>  // va_list, va_arg, va_start
#include <stdarg.h> // va_list, va_arg, va_start
#include <TOOLS/debug.h>
//XXX #include <SqAssert.h>
#include <SurgSim/Framework/Assert.h>

//////////////////////////////////////////////////
// Useful local functions for matrix
template <class T> static void _analyse_matrix_args(T *data,unsigned int nbElem,T first,va_list args)
{
  register unsigned int i=0;
  data[i]=first; i++;
  while(i<nbElem){ data[i]=(T)va_arg(args, int ); i++; }
}

template <> static void _analyse_matrix_args(double *data,unsigned int nbElem,double first,va_list args)
{
  register unsigned int i=0;
  data[i]=first; i++;
  while(i<nbElem){ data[i]=(double)va_arg(args, double ); i++; }
}

template <> static void _analyse_matrix_args(float *data,unsigned int nbElem,float first,va_list args)
{
  register unsigned int i=0;
  data[i]=first; i++;
  while(i<nbElem){ data[i]=(float)va_arg(args, double ); i++; }
}

template <> static void _analyse_matrix_args(long double *data,unsigned int nbElem,long double first,va_list args)
{
  register unsigned int i=0;
  data[i]=first; i++;
  while(i<nbElem){ data[i]=(long double)va_arg(args, double ); i++; }
}

template <> static void _analyse_matrix_args(long long *data,unsigned int nbElem,long long first,va_list args)
{
  register unsigned int i=0;
  data[i]=first; i++;
  while(i<nbElem){ data[i]=(long long)va_arg(args, long long ); i++; }
}

template <> static void _analyse_matrix_args(unsigned long long *data,unsigned int nbElem,unsigned long long first,va_list args)
{
  register unsigned int i=0;
  data[i]=first; i++;
  while(i<nbElem){ data[i]=(unsigned long long)va_arg(args, long long ); i++; }
}

template <> static void _analyse_matrix_args(bool *data,unsigned int nbElem,bool first,va_list args)
{
  register unsigned int i=0;
  data[i]=first; i++;
  while(i<nbElem){ data[i]=(va_arg(args,int)?true:false); i++; }
}

//////////////////////////////////////////////////


//////////////////////////////////////////////////
// Useful local functions to retrieve cosinus and sinus
template <class T> static void getCosSin(const T angle,T &cosA,T &sinA)
{
  cosA = (T)cos(angle);
  sinA = (T)sin(angle);
}

template <class T> static void getSqrt(const T angle,T &sqrtA)
{
  sqrtA = (T)sqrt(angle);
}

//////////////////////////////////////////////////


//////////////////////////////////////////////////
// Dynamic_Matrix class
template <class T, class Allocator>              Dynamic_Matrix<T,Allocator>::Dynamic_Matrix(unsigned int _nbRow,unsigned int _nbColumn)
:nbRow(_nbRow),nbColumn(_nbColumn),nbElem(_nbRow*_nbColumn),data(Allocator::alloc(_nbRow*_nbColumn)),nbAllocatedElem(_nbRow*_nbColumn)
{
//  assert_printf(data!=(T*)0,"Memory allocation failed (null pointer) !");
}

template <class T, class Allocator>              Dynamic_Matrix<T,Allocator>::Dynamic_Matrix(unsigned int _nbRow,unsigned int _nbColumn,T first,...)
:nbRow(_nbRow),nbColumn(_nbColumn),nbElem(_nbRow*_nbColumn),data(Allocator::alloc(_nbRow*_nbColumn)),nbAllocatedElem(_nbRow*_nbColumn)
{
  assert_printf(data!=(T*)0 ,"Memory allocation failed (null pointer) !");
  assert_printf(_nbRow!=0   ,"nbRow = 0, but parameters are set !");
  assert_printf(_nbColumn!=0,"nbColumn = 0, but parameters are set !");
  va_list args;
  va_start( args , first );
  _analyse_matrix_args( data , nbElem , first , args );
  va_end(args);
}

template <class T, class Allocator>              Dynamic_Matrix<T,Allocator>::Dynamic_Matrix(const Dynamic_Matrix &m)
:nbRow(m.nbRow),nbColumn(m.nbColumn),nbElem(m.nbElem),data(Allocator::alloc(m.nbAllocatedElem)),nbAllocatedElem(m.nbAllocatedElem)
{
  assert_printf(data!=(T*)0,"Memory allocation failed (null pointer) !");
  assert_printf(nbRow!=0   ,"nbRow = 0, but parameters are set !");
  assert_printf(nbColumn!=0,"nbColumn = 0, but parameters are set !");
  memcpy(data, m.data, nbElem * sizeof(T));
}

template <class T, class Allocator>              Dynamic_Matrix<T,Allocator>& Dynamic_Matrix<T,Allocator>::operator=(const Dynamic_Matrix& m)
{
	resizeGrowMemory(m.nbRow, m.nbColumn);
	memcpy(data, m.data, nbElem * sizeof(T));
	return *this;
}

template <class T, class Allocator>              Dynamic_Matrix<T,Allocator>::~Dynamic_Matrix()
{ Allocator::free(data, nbAllocatedElem); }

template <class T, class Allocator> void          Dynamic_Matrix<T,Allocator>::reserve(unsigned int storageSize)
{
	if (storageSize > nbAllocatedElem)
	{
		// Need to allocate more storage.
		nbAllocatedElem = storageSize;
		data = Allocator::realloc(data, nbAllocatedElem, false, nbElem); // the false means that we don't need to keep informations in the matrix!
		SURGSIM_ASSERT(data != 0 || nbAllocatedElem == 0) << "Memory allocation failed (null pointer) !";
	}
}

template <class T, class Allocator> void         Dynamic_Matrix<T,Allocator>::resizeSetMemory(unsigned int newNumRows, unsigned int newNumColumns)
{
	unsigned int newElem = newNumRows*newNumColumns;
	nbAllocatedElem      = newElem;
	data                 = Allocator::realloc(data, nbAllocatedElem, false, nbElem);
	SURGSIM_ASSERT(data != 0 || nbAllocatedElem == 0) << "Memory allocation failed (null pointer) !";
	nbRow                = newNumRows;
	nbColumn             = newNumColumns;
	nbElem               = newElem;
}

template <class T, class Allocator> void          Dynamic_Matrix<T,Allocator>::resizeGrowMemory(unsigned int newNumRows, unsigned int newNumColumns)
{
	unsigned int newElem = newNumRows*newNumColumns;
	reserve(newElem);
	nbRow                = newNumRows;
	nbColumn             = newNumColumns;
	nbElem               = newElem;
}

template <class T, class Allocator> void          Dynamic_Matrix<T,Allocator>::resizeToAtLeast(unsigned int minNumRows, unsigned int minNumColumns)
{
	if (minNumRows > nbRow)
	{
		if (minNumColumns > nbColumn)
		{
			// Resize in both directions.
			resizeGrowMemory(minNumRows, minNumColumns);
		}
		else
		{
			// Resize rows ONLY.
			resizeGrowMemory(minNumRows, nbColumn);
		}
	}
	else
	{
		if (minNumColumns > nbColumn)
		{
			// Resize columns ONLY.
			resizeGrowMemory(nbRow, minNumColumns);
		}
		else
		{
			// No resizing is necessary; both rows and columns are already meet the minimum.
		}
	}
}

template <class T, class Allocator> T*           Dynamic_Matrix<T,Allocator>::operator [](unsigned int numRow)
{
//  assert_printf(numRow<nbRow,"Access out of range, numRow=%d, nbRow=%d",numRow,nbRow);
  return &(data[numRow*nbColumn]);
}

template <class T, class Allocator> const T*     Dynamic_Matrix<T,Allocator>::operator [](unsigned int numRow) const
{
//  assert_printf(numRow<nbRow,"Access out of range, numRow=%d, nbRow=%d",numRow,nbRow);
  return &(data[numRow*nbColumn]);
}

template <class T, class Allocator> unsigned int Dynamic_Matrix<T,Allocator>::getNbRow(void) const
{ return nbRow; }

template <class T, class Allocator> unsigned int Dynamic_Matrix<T,Allocator>::getNbColumn(void) const
{ return nbColumn; }

template <class T, class Allocator> unsigned int Dynamic_Matrix<T,Allocator>::getNbElem(void) const
{ return nbElem; }

template <class T, class Allocator> T*           Dynamic_Matrix<T,Allocator>::getPointer(void)
{ return data; }

template <class T, class Allocator> const T*     Dynamic_Matrix<T,Allocator>::getPointer(void) const
{ return data; }
//////////////////////////////////////////////////


//////////////////////////////////////////////////
// Static_Matrix class
template <class T,unsigned int nbRow,unsigned int nbColumn>               Static_Matrix<T,nbRow,nbColumn>::Static_Matrix()
:nbElem(nbRow*nbColumn){}

template <class T,unsigned int nbRow,unsigned int nbColumn>               Static_Matrix<T,nbRow,nbColumn>::Static_Matrix(T first,...)
:nbElem(nbRow*nbColumn)
{
  assert_printf(nbRow!=0   ,"nbRow = 0, but parameters are set !");
  assert_printf(nbColumn!=0,"nbColumn = 0, but parameters are set !");
  va_list args;
  va_start( args , first );
  _analyse_matrix_args((T*)&(data[0][0]),nbElem,first,args);
  va_end(args);
}

template <class T,unsigned int nbRow,unsigned int nbColumn>               Static_Matrix<T,nbRow,nbColumn>::Static_Matrix(const Static_Matrix<T,nbRow,nbColumn> &m)
:nbElem(nbRow*nbColumn)
{
  mat_copy<T,T>(m, *this, nbRow, nbColumn);
}

template <class T,unsigned int nbRow,unsigned int nbColumn> T*            Static_Matrix<T,nbRow,nbColumn>::operator [](unsigned int numRow)
{
//  assert_printf(numRow<nbRow,"Access out of range, numRow=%d,nbRow=%d",numRow,nbRow);
  return (T*)(&data[numRow][0]);
}

template <class T,unsigned int nbRow,unsigned int nbColumn> T*            Static_Matrix<T,nbRow,nbColumn>::operator [](unsigned int numRow) const
{
//  assert_printf(numRow<nbRow,"Access out of range, numRow=%d,nbRow=%d",numRow,nbRow);
  return (T*)(&data[numRow][0]);
}

template <class T,unsigned int nbRow,unsigned int nbColumn> unsigned int  Static_Matrix<T,nbRow,nbColumn>::getNbRow(void) const
{ return nbRow; }

template <class T,unsigned int nbRow,unsigned int nbColumn> unsigned int  Static_Matrix<T,nbRow,nbColumn>::getNbColumn(void) const
{ return nbColumn; }

template <class T,unsigned int nbRow,unsigned int nbColumn> unsigned int  Static_Matrix<T,nbRow,nbColumn>::getNbElem(void) const
{ return nbElem; }

template <class T,unsigned int nbRow,unsigned int nbColumn> T *           Static_Matrix<T,nbRow,nbColumn>::getPointer(void)
{ return &(data[0][0]); }

template <class T,unsigned int nbRow,unsigned int nbColumn> T *           Static_Matrix<T,nbRow,nbColumn>::getPointer(void) const
{ return const_cast<T*>(&(data[0][0])); }
//////////////////////////////////////////////////


//////////////////////////////////////////////////
// Generic copy function helpers

namespace {

	template <typename T1, typename T2>                                 inline void data_copy(const T1* in, T2* out, size_t numElements)
	{
		T2* endAlignedOut = out + (numElements & ~3);  // one past the last element to be copied in the copy-4 loop
		while (out < endAlignedOut)
		{
			out[0] = in[0];  out[1] = in[1];  out[2] = in[2];  out[3] = in[3];
			out += 4; in += 4;
		}
		if (numElements & 2) { out[0] = in[0];  out[1]=in[1];  out += 2;  in += 2;}
		if (numElements & 1) { *out = *in; }
	}

	template <typename T>                                               inline void data_copy(const T* in, T* out, size_t numElements)
	{
		// If both data types are the same, we can just copy the memory.
		// This assumes we're not dealing with arrays of a struct/class with pointer members or virtual inheritance, but I think this is a fairly safe assumption. =)
		memcpy(out, in, numElements * sizeof(T));	// TODO: switch to the mythical sq_memcpy?
	}

};

//////////////////////////////////////////////////
// Generic functions on matrices
  // Set

// NB: this means you can call mat_copy(...) with no template arguments
template <class M1,class Mres>                                          void  mat_copy(const M1 &m1, Mres &res)
{
	assert_printf(res.getNbRow()>=m1.getNbRow()      ,"res.getNbRow()=%d < m1.getNbRow()=%d",res.getNbRow(),m1.getNbRow());
	assert_printf(res.getNbColumn()==m1.getNbColumn(),"res.getNbColumn()=%d != m1.getNbColumn()=%d",res.getNbColumn(),m1.getNbColumn());
	data_copy(m1.getPointer(), res.getPointer(), m1.getNbElem());
}

// NB: this means you can call mat_copy(...) with no template arguments
template <class M1,class Mres>                                          void  mat_copy(const M1 &m1, Mres &res,unsigned int nbRow,unsigned int nbColumn)
{
	assert_printf(res.getNbRow()>=nbRow,    "res.getNbRow()=%d < nbRow=%d", res.getNbRow(), nbRow);
	assert_printf(m1.getNbRow()>=nbRow,     "m1.getNbRow()=%d < nbRow=%d",  m1.getNbRow(),  nbRow);
	assert_printf(res.getNbColumn()>=nbColumn, "res.getNbColumn()=%d < nbColumn=%d", res.getNbColumn(), nbColumn);
	assert_printf(m1.getNbColumn()>=nbColumn,  "m1.getNbColumn()=%d < nbColumn=%d",  m1.getNbColumn(),  nbColumn);

	if (nbColumn == res.getNbColumn() && nbColumn == m1.getNbColumn())
	{
		// Data is contiguous-- do one bulk copy.
		data_copy(m1.getPointer(), res.getPointer(), nbRow*nbColumn);
	}
	else
	{
		// Data is not contiguous.  Warning: this means a performance hit!
		for (unsigned int row = 0;  row < nbRow;  ++row)
		{
			data_copy(m1[row], res[row], nbColumn);
		}
	}
}

// older interface-- must explicitly call mat_copy<T1,T2>(...)
template <class T1,class Tres,class M1,class Mres>                      void  mat_copy(const M1 &m1, Mres &res)
{
	assert_printf(res.getNbRow()>=m1.getNbRow()      ,"res.getNbRow()=%d < m1.getNbRow()=%d",res.getNbRow(),m1.getNbRow());
	assert_printf(res.getNbColumn()==m1.getNbColumn(),"res.getNbColumn()=%d != m1.getNbColumn()=%d",res.getNbColumn(),m1.getNbColumn());
	const T1* t1   = m1.getPointer();
	Tres*     tres = res.getPointer();
	data_copy(t1, tres, m1.getNbElem());
}

// older interface-- must explicitly call mat_copy<T1,T2>(...)
template <class T1,class Tres,class M1,class Mres>                      void  mat_copy(const M1 &m1, Mres &res,unsigned int nbRow,unsigned int nbColumn)
{
	assert_printf(res.getNbRow()>=nbRow      ,"res.getNbRow()=%d < nbRow=%d",res.getNbRow(),nbRow);
	if (nbColumn == res.getNbColumn() && nbColumn == m1.getNbColumn())
	{
		// Data is contiguous-- do one bulk copy.
		const T1* t1   = m1.getPointer();
		Tres*     tres = res.getPointer();
		data_copy(t1, tres, nbRow*nbColumn);
	}
	else
	{
		// Data is not contiguous.  Warning: this means a performance hit!
		for (unsigned int row = 0;  row < nbRow;  ++row)
		{
			const T1* t1   = m1[row];
			Tres*     tres = res[row];
			data_copy(t1, tres, nbColumn);
		}
	}
}

template <class T,class M1>                         void  mat_null(M1 &m1)
{
  T *data=m1.getPointer();
  unsigned int nbElem = m1.getNbElem() >> 2;
  while(nbElem--)
  { data[0]=(T)0; data[1]=(T)0; data[2]=(T)0; data[3]=(T)0; data+=4; }
  nbElem =  m1.getNbElem() & 3;
  while(nbElem--) *data++=(T)0;
}

template <class T,class M1>                         void  mat_null(M1 &m1, unsigned int nbRow, unsigned int nbCol, unsigned int nbColAllocated)
{
  T *data=m1.getPointer();
  for(unsigned int row=0 ; row<nbRow ; row++)
  {
    T *dataRow = &data[row*nbColAllocated];
    unsigned int nbElem = nbCol >> 2;
    while(nbElem--)
    { dataRow[0]=(T)0; dataRow[1]=(T)0; dataRow[2]=(T)0; dataRow[3]=(T)0; dataRow+=4; }
    nbElem =  nbCol & 3;
    while(nbElem--) *dataRow++=(T)0;
  }
}

template <class T,class M1>                         void  mat_set(M1 &m1,T value)
{
  register T *data=m1.getPointer();
  register unsigned int nbElem = m1.getNbElem();
  for(;nbElem>0;nbElem--)
  {
    *data=value;
    data++;
  }
}

template <class T,class M1>                         void  mat_identity(M1 &m1)
{
  assert_printf(m1.getNbRow()==m1.getNbColumn(),"Non square matrix m1.getNbRow()=%d m1.getNbColumn()=%d",m1.getNbRow(),m1.getNbColumn());
  T *data=m1.getPointer();
  unsigned int nbElem = m1.getNbElem() >> 2;
  while(nbElem--)
  { data[0]=0; data[1]=0; data[2]=0; data[3]=0; data+=4; }
  nbElem =  m1.getNbElem() & 3;
  if(nbElem & 2){ data[0]=0; data[1]=0; data+=2; }
  if(nbElem & 1) *data=0;

  nbElem = m1.getNbRow();
  data=m1.getPointer();
  unsigned int toAdd=nbElem+1;
  nbElem>>=2;
  while(nbElem--)
  { *data=1; data+=toAdd; *data=1; data+=toAdd; *data=1; data+=toAdd; *data=1; data+=toAdd; }
  nbElem = m1.getNbRow() & 3;
  if(nbElem & 2){ *data=1; data+=toAdd; *data=1; data+=toAdd; }
  if(nbElem & 1) *data=1;
}

template <class M1>                         void mat_identity(M1 &m1, unsigned int nbRow, unsigned int nbCol)
{
	for (unsigned int i = 0; i < nbRow; i++)
	{
		for (unsigned int j = 0; j < nbCol; j++)
		{
			if (i == j)
			{
				m1[i][j] = 1.0;
			}
			else
			{
				m1[i][j] = 0.0;
			}
		}
	}
}

template <class T,class M1>                         void  mat_clamp(M1 &m1,T value,T min,T max)
{
  register unsigned int nbElem=m1.getNbElem();
  register T* data=m1.getPointer();
  for(;nbElem>0;nbElem--)
  {
    if(*data>=min && *data<=max) *data=value;
    data++;
  }
}

  // Arithmetic operations
template <class M1,class M2>                        void  mat_add(M1 &m1, const M2 &m2)
{
  assert_printf(m1.getNbRow()   ==m2.getNbRow()   ,"m1.getNbRow() != m2.getNbRow()");
  assert_printf(m1.getNbColumn()==m2.getNbColumn(),"m1.getNbColumn() != m2.getNbColumn()");
  register unsigned int nbElem = m1.getNbElem() >> 2,i=0;
  while(nbElem--)
  {
    m1.getPointer()[i] += m2.getPointer()[i]; i++;
    m1.getPointer()[i] += m2.getPointer()[i]; i++;
    m1.getPointer()[i] += m2.getPointer()[i]; i++;
    m1.getPointer()[i] += m2.getPointer()[i]; i++;
  }
  nbElem = m1.getNbElem() & 3;
  while(nbElem--)
  { m1.getPointer()[i] += m2.getPointer()[i]; i++; }
}

template <class T1,class T2,class M1,class M2>      void  mat_add(M1 &m1, const M2 &m2)
{
  //assert_printf(m1.getNbRow()   ==m2.getNbRow()   ,"m1.getNbRow() != m2.getNbRow()");
  //assert_printf(m1.getNbColumn()==m2.getNbColumn(),"m1.getNbColumn() != m2.getNbColumn()");  
  T1 *d1=m1.getPointer();
  const T2 *d2=m2.getPointer();
  unsigned int nbElem = m1.getNbElem() >> 2;
  while(nbElem--)
  {
    d1[0] += d2[0]; d1[1] += d2[1]; d1[2] += d2[2]; d1[3] += d2[3];
    d1+=4; d2+=4;
  }
  nbElem = m1.getNbElem() & 3;
  while(nbElem--)
    *d1++ += *d2++;
}

template <class T1,class T2,class Tres,class M1,class M2,class Mres>             void  mat_add(const M1 &m1, const M2 &m2, Mres &res)
{
  assert_printf(m1.getNbRow()   ==m2.getNbRow()    ,"m1.getNbRow() != m2.getNbRow()");
  assert_printf(m1.getNbColumn()==m2.getNbColumn() ,"m1.getNbColumn() != m2.getNbColumn()");
  assert_printf(m1.getNbRow()   ==res.getNbRow()   ,"m1.getNbRow() != res.getNbRow()");
  assert_printf(m1.getNbColumn()==res.getNbColumn(),"m1.getNbColumn() != res.getNbColumn()");
  T1 *d1=m1.getPointer();
  T2 *d2=m2.getPointer();
  Tres *dres=res.getPointer();
  unsigned int nbElem = m1.getNbElem() >> 2;
  while(nbElem--)
  {
    dres[0]=d1[0]+d2[0]; dres[1]=d1[1]+d2[1]; dres[2]=d1[2]+d2[2]; dres[3]=d1[3]+d2[3];
    dres+=4; d1+=4; d2+=4;
  }
  nbElem = m1.getNbElem() & 3;
  while(nbElem--)
    *dres++ = *d1++ + *d2++;
}

template <class M1,class M2,class Mres>             void  mat_add(const M1 &m1, const M2 &m2, Mres &res)
{
  assert_printf(m1.getNbRow()   ==m2.getNbRow()    ,"m1.getNbRow() != m2.getNbRow()");
  assert_printf(m1.getNbColumn()==m2.getNbColumn() ,"m1.getNbColumn() != m2.getNbColumn()");
  assert_printf(m1.getNbRow()   ==res.getNbRow()   ,"m1.getNbRow() != res.getNbRow()");
  assert_printf(m1.getNbColumn()==res.getNbColumn(),"m1.getNbColumn() != res.getNbColumn()");
  unsigned int nbElem = m1.getNbElem() >> 2,i=0;
  while(nbElem--)
  {
    res.getPointer()[i] = m1.getPointer()[i] + m2.getPointer()[i]; i++;
    res.getPointer()[i] = m1.getPointer()[i] + m2.getPointer()[i]; i++;
    res.getPointer()[i] = m1.getPointer()[i] + m2.getPointer()[i]; i++;
    res.getPointer()[i] = m1.getPointer()[i] + m2.getPointer()[i]; i++;
  }
  nbElem = m1.getNbElem() & 3;
  while(nbElem--)
  { res.getPointer()[i] = m1.getPointer()[i] + m2.getPointer()[i]; i++; }
}

template <class M1,class M2>										  void  mat_add(M1 &m1, const M2 &m2, 
																					unsigned int nbRow, unsigned int nbCol)
{
	for (unsigned int i = 0; i < nbRow; i++)
	{
		for (unsigned int j = 0; j < nbCol; j++)
		{
			m1[i][j] += m2[i][j];
		}
	}
}
template <class M1,class M2,class Mres>								  void  mat_add(const M1 &m1, const M2 &m2, Mres &res, 
																					unsigned int nbRow, unsigned int nbCol)
{
	for (unsigned int i = 0; i < nbRow; i++)
	{
		for (unsigned int j = 0; j < nbCol; j++)
		{
			res[i][j] = m1[i][j] + m2[i][j];
		}
	}
}


template <class M1,class M2>                        void  mat_sub(M1 &m1, const M2 &m2)
{
  assert_printf(m1.getNbRow()   ==m2.getNbRow()   ,"m1.getNbRow() != m2.getNbRow()");
  assert_printf(m1.getNbColumn()==m2.getNbColumn(),"m1.getNbColumn() != m2.getNbColumn()");
  register unsigned int nbElem = m1.getNbElem() >> 2,i=0;
  while(nbElem--)
  {
    m1.getPointer()[i] -= m2.getPointer()[i]; i++;
    m1.getPointer()[i] -= m2.getPointer()[i]; i++;
    m1.getPointer()[i] -= m2.getPointer()[i]; i++;
    m1.getPointer()[i] -= m2.getPointer()[i]; i++;
  }
  nbElem = m1.getNbElem() & 3;
  while(nbElem--)
  { m1.getPointer()[i] -= m2.getPointer()[i]; i++; }
}

template <class T1,class T2,class M1,class M2>      void  mat_sub(M1 &m1, const M2 &m2)
{
  assert_printf(m1.getNbRow()   ==m2.getNbRow()   ,"m1.getNbRow() != m2.getNbRow()");
  assert_printf(m1.getNbColumn()==m2.getNbColumn(),"m1.getNbColumn() != m2.getNbColumn()");  
  T1 *d1=m1.getPointer();
  T2 *d2=m2.getPointer();
  unsigned int nbElem = m1.getNbElem() >> 2;
  while(nbElem--)
  {
    d1[0] -= d2[0]; d1[1] -= d2[1]; d1[2] -= d2[2]; d1[3] -= d2[3];
    d1+=4; d2+=4;
  }
  nbElem = m1.getNbElem() & 3;
  while(nbElem--)
    *d1++ -= *d2++;
}
template <class T1,class T2,class Tres,class M1,class M2,class Mres>             void  mat_sub(const M1 &m1, const M2 &m2, Mres &res)
{
  assert_printf(m1.getNbRow()   ==m2.getNbRow()    ,"m1.getNbRow() != m2.getNbRow()");
  assert_printf(m1.getNbColumn()==m2.getNbColumn() ,"m1.getNbColumn() != m2.getNbColumn()");
  assert_printf(m1.getNbRow()   ==res.getNbRow()   ,"m1.getNbRow() != res.getNbRow()");
  assert_printf(m1.getNbColumn()==res.getNbColumn(),"m1.getNbColumn() != res.getNbColumn()");
  T1 *d1=m1.getPointer();
  T2 *d2=m2.getPointer();
  Tres *dres=res.getPointer();
  unsigned int nbElem = m1.getNbElem() >> 2;
  while(nbElem--)
  {
    dres[0]=d1[0]-d2[0]; dres[1]=d1[1]-d2[1]; dres[2]=d1[2]-d2[2]; dres[3]=d1[3]-d2[3];
    dres+=4; d1+=4; d2+=4;
  }
  nbElem = m1.getNbElem() & 3;
  while(nbElem--)
    *dres++ = *d1++ - *d2++;
}

template <class M1,class M2,class Mres>             void  mat_sub(const M1 &m1, const M2 &m2, Mres &res)
{
  assert_printf(m1.getNbRow()   ==m2.getNbRow()    ,"m1.getNbRow() != m2.getNbRow()");
  assert_printf(m1.getNbColumn()==m2.getNbColumn() ,"m1.getNbColumn() != m2.getNbColumn()");
  assert_printf(m1.getNbRow()   ==res.getNbRow()   ,"m1.getNbRow() != res.getNbRow()");
  assert_printf(m1.getNbColumn()==res.getNbColumn(),"m1.getNbColumn() != res.getNbColumn()");
  unsigned int nbElem = m1.getNbElem() >> 2,i=0;
  while(nbElem--)
  {
    res.getPointer()[i] = m1.getPointer()[i] - m2.getPointer()[i]; i++;
    res.getPointer()[i] = m1.getPointer()[i] - m2.getPointer()[i]; i++;
    res.getPointer()[i] = m1.getPointer()[i] - m2.getPointer()[i]; i++;
    res.getPointer()[i] = m1.getPointer()[i] - m2.getPointer()[i]; i++;
  }
  nbElem = m1.getNbElem() & 3;
  while(nbElem--)
  { res.getPointer()[i] = m1.getPointer()[i] - m2.getPointer()[i]; i++; }
}

template <class M1,class M2>										  void  mat_sub(M1 &m1, const M2 &m2, 
																					unsigned int nbRow, unsigned int nbCol)
{
	for (unsigned int i = 0; i < nbRow; i++)
	{
		for (unsigned int j = 0; j < nbCol; j++)
		{
			m1[i][j] -= m2[i][j];
		}
	}
}
template <class M1,class M2,class Mres>								  void  mat_sub(const M1 &m1, const M2 &m2, Mres &res, 
																					unsigned int nbRow, unsigned int nbCol)
{
	for (unsigned int i = 0; i < nbRow; i++)
	{
		for (unsigned int j = 0; j < nbCol; j++)
		{
			res[i][j] = m1[i][j] - m2[i][j];
		}
	}
}


template <class T,class M1>                         void  mat_scale(M1 &m1, const T value)
{
  register unsigned int m1NbRow = m1.getNbRow();
  register unsigned int m1NbCol = m1.getNbColumn();
  for(register unsigned int i=0;i<m1NbRow;i++)
    for(register unsigned int j=0;j<m1NbCol;j++)
      m1[i][j] *= value;
}

template <class T,class M1,class Mres>              void  mat_scale(const M1 &m1, const T value, Mres &res) throw(...)
{
  register unsigned int m1NbRow = m1.getNbRow();
  register unsigned int m1NbCol = m1.getNbColumn();
#ifdef Matrix_Exceptions
  if(m1NbRow>res.getNbRow())      throw("Exception in \n template <class T,class M1,class Mres>    void  mat_scale(const M1 &m1, const T value, Mres &res) \n -> m1.getNbRow()    > res.getNbRow()");
  if(m1NbCol>res.getNbColumn())   throw("Exception in \n template <class T,class M1,class Mres>    void  mat_scale(const M1 &m1, const T value, Mres &res) \n -> m1.getNbColumn() > res.getNbColumn()");
#endif
  for(register unsigned int i=0;i<m1NbRow;i++)
    for(register unsigned int j=0;j<m1NbCol;j++)
      res[i][j] = m1[i][j] * value;
}

template <class T,class M1>							void  mat_scale(M1 &m1, const T value, unsigned int nbRow, unsigned int nbCol)
{
	for (unsigned int i = 0; i < nbRow; i++)
	{
		for (unsigned int j = 0; j < nbCol; j++)
		{
			m1[i][j] *= value;
		}
	}
}
template <class T,class M1,class Mres>				void  mat_scale(const M1 &m1, const T value, Mres &res, unsigned int nbRow, unsigned int nbCol) MAY_THROW
{
	for (unsigned int i = 0; i < nbRow; i++)
	{
		for (unsigned int j = 0; j < nbCol; j++)
		{
			res[i][j] = m1[i][j] * value;
		}
	}
}

template <class T,class M1>                         void  mat_invScale(M1 &m1, const T value) throw(...)
{
#ifdef Matrix_Exceptions
  if(value>(T)-EPSILON && value<(T)EPSILON) throw("Exception in \n template <class T,class M1>               void  mat_invScale(M1 &m1, const T value) \n -> value = 0 !");
#endif
  register unsigned int m1NbRow = m1.getNbRow();
  register unsigned int m1NbCol = m1.getNbColumn();
  for(register unsigned int i=0;i<m1NbRow;i++)
    for(register unsigned int j=0;j<m1NbCol;j++)
      m1[i][j] /= value;
}

template <class T,class M1,class Mres>              void  mat_invScale(const M1 &m1, const T value, Mres &res) throw(...)
{
  register unsigned int m1NbRow = m1.getNbRow();
  register unsigned int m1NbCol = m1.getNbColumn();
#ifdef Matrix_Exceptions
  if(value>(T)-EPSILON && value<(T)EPSILON) throw("Exception in \n template <class T,class M1,class Mres>    void  mat_invScale(const M1 &m1, const T value, Mres &res) \n -> value = 0 !");
  if(m1NbRow>res.getNbRow())      throw("Exception in \n template <class T,class M1,class Mres>    void  mat_invScale(const M1 &m1, const T value, Mres &res) \n -> m1.getNbRow()    > res.getNbRow()");
  if(m1NbCol>res.getNbColumn())   throw("Exception in \n template <class T,class M1,class Mres>    void  mat_invScale(const M1 &m1, const T value, Mres &res) \n -> m1.getNbColumn() > res.getNbColumn()");
#endif
  for(register unsigned int i=0;i<m1NbRow;i++)
    for(register unsigned int j=0;j<m1NbCol;j++)
      res[i][j] = m1[i][j] / value;
}

template <class T,class M1>							void  mat_invScale(M1 &m1, const T value, unsigned int nbRow, unsigned int nbCol)
{
	for (unsigned int i = 0; i < nbRow; i++)
	{
		for (unsigned int j = 0; j < nbCol; j++)
		{
			m1[i][j] /= value;
		}
	}
}
template <class T,class M1,class Mres>				void  mat_invScale(const M1 &m1, const T value, Mres &res, unsigned int nbRow, unsigned int nbCol) MAY_THROW
{
	for (unsigned int i = 0; i < nbRow; i++)
	{
		for (unsigned int j = 0; j < nbCol; j++)
		{
			res[i][j] = m1[i][j] / value;
		}
	}
}

  // Multiplication
template <class M1,class M2,class Mres>             void  mat_mul(const M1 &m1,const M2 &m2, Mres &res) throw(...)
{
  register unsigned int m1NbRow = m1.getNbRow();
  register unsigned int m1NbCol = m1.getNbColumn();
  register unsigned int m2NbCol = m2.getNbColumn();
#ifdef Matrix_Exceptions
  if(m1NbCol>m2.getNbRow())      throw("Exception in \n template <class M1,class M2,class Mres>   void  mat_mul(const M1 &m1,const M2 &m2, Mres &res) \n -> m1.getNbColumn() > m2.getNbRow()");
  if(m1NbRow>res.getNbRow())     throw("Exception in \n template <class M1,class M2,class Mres>   void  mat_mul(const M1 &m1,const M2 &m2, Mres &res) \n -> m1.getNbRow()    > res.getNbRow()");
  if(m2NbCol>res.getNbColumn())  throw("Exception in \n template <class M1,class M2,class Mres>   void  mat_mul(const M1 &m1,const M2 &m2, Mres &res) \n -> m2.getNbColumn() > res.getNbColumn()");
#endif
  register unsigned int i,j,k;
  for(i=0;i<m1NbRow;i++)
    for(j=0;j<m2NbCol;j++)
    {
      res[i][j]=0;
      for(k=0;k<m1NbCol;k++)
        res[i][j] += m1[i][k] * m2[k][j];
    }
}

template <class M1,class M2,class Mres>             void  mat_mul(const M1 &m1,const M2 &m2, Mres &res, unsigned int nbRow, unsigned int nbCol,unsigned int nbIntermediateCol) throw(...)
{
  register unsigned int m1NbRow = m1.getNbRow();
  register unsigned int m1NbCol = m1.getNbColumn();
  register unsigned int m2NbRow = m2.getNbRow();
  register unsigned int m2NbCol = m2.getNbColumn();
#ifdef Matrix_Exceptions
  if(m1NbCol<nbIntermediateCol)      throw("Exception in \n template <class M1,class M2,class Mres>   void  mat_mul(const M1 &m1,const M2 &m2, Mres &res) \n -> m1.getNbColumn() > m2.getNbRow()");
  if(m1NbRow<nbRow)     throw("Exception in \n template <class M1,class M2,class Mres>   void  mat_mul(const M1 &m1,const M2 &m2, Mres &res) \n -> m1.getNbRow()    > res.getNbRow()");
  if(m2NbCol<nbCol)  throw("Exception in \n template <class M1,class M2,class Mres>   void  mat_mul(const M1 &m1,const M2 &m2, Mres &res) \n -> m2.getNbColumn() > res.getNbColumn()");
  if(m2NbCol<nbIntermediateCol)  throw("Exception in \n template <class M1,class M2,class Mres>   void  mat_mul(const M1 &m1,const M2 &m2, Mres &res) \n -> m2.getNbColumn() > res.getNbColumn()");
#endif
  register unsigned int i,j,k;
  for(i=0;i<nbRow;i++)
    for(j=0;j<nbCol;j++)
    {
      res[i][j]=0;
      for(k=0;k<nbIntermediateCol;k++)
        res[i][j] += m1[i][k] * m2[k][j];
    }
}

template <class M1,class M2,class Mres>             void  mat_mul(const M1 &m1,const M2 &m2, Mres &res, unsigned int nbRow, unsigned int nbCol)
{
	register unsigned int i, j, k;
	for (i = 0; i < nbRow; i++)
	{
		for (j = 0; j < nbCol; j++)
		{
			res[i][j]=0;
			for(k = 0; k < nbCol; k++)
			{
				res[i][j] += m1[i][k] * m2[k][j];
			}
		}
	}
}


  // Inversion
/*
template <class T,class M>                          T     mat_determinant(const M &m)
{
  cout << "template <class M1,class Mres>            void  mat_invert(const M1 &m1,Mres &res) throw(...) \n not implemented yet !";
}

template <class T,class M>                          T     mat_coMatrix(const M &m)
{
  cout << "template <class M1,class Mres>            void  mat_invert(const M1 &m1,Mres &res) throw(...) \n not implemented yet !";
}

*/
template <class T,class M>                          void  mat_swapRows(M &m,unsigned int i,unsigned int j)
{
  register int dim=m.getNbColumn();
  while(dim--){ T tmp   = m[i][dim];  m[i][dim] = m[j][dim];  m[j][dim] = tmp; }
}

template <class T,class M>                          void  mat_swapRows(M &m,unsigned int i,unsigned int j,int dim)
{
  while(dim--){ T tmp   = m[i][dim];  m[i][dim] = m[j][dim];  m[j][dim] = tmp; }
}

#if 0  // ***DISABLED BAD CODE***
/* This function is BROKEN-- it does not currently give correct results!
 * Paul N noticed this, and also noticed a few suspicious-looking commented out lines in the code.
 * (Julien confirmed the problem, too.)  At this point, I don't trust this code enough to let it hang around without
 * very thorough testing, and it's not actually *used* anywhere, so it seems wisest to just disable it altogether for now.
 * As an alternative, use the 4-arg (LU decomposition-based) version of mat_invert, which should work correctly.
 * --bert 2012-01-05
 */
template <class T,class M1,class Mres,class Mtmp>   void  mat_invert(const M1 &m1,Mres &res,Mtmp &tmp)
{
  assert_printf(m1.getNbRow()==m1.getNbColumn(),"Non square matrix m1 (%dx%d)",m1.getNbRow(),m1.getNbColumn());
  assert_printf(m1.getNbRow()==res.getNbRow(),"m1.getNbRow()=%d != res.getNbRow()=%d",m1.getNbRow(),res.getNbRow());
  assert_printf(m1.getNbColumn()==res.getNbColumn(),"m1.getNbColumn()=%d != res.getNbColumn()=%d",m1.getNbColumn(),res.getNbColumn());
  assert_printf(m1.getNbRow()==tmp.getNbRow(),"m1.getNbRow()=%d != tmp.getNbRow()=%d",m1.getNbRow(),tmp.getNbRow());
  assert_printf(m1.getNbColumn()==tmp.getNbColumn(),"m1.getNbColumn()=%d != tmp.getNbColumn()=%d",m1.getNbColumn(),tmp.getNbColumn());
  unsigned int nbRow=m1.getNbRow();
  unsigned int nbCol=m1.getNbColumn();
  unsigned int i,j,k;
  unsigned int nbColBy4 = nbCol >> 2;
  unsigned int nbColLeft = nbCol & 3;
  
  // SHOULD BE UNCOMMENTED (or the caller needs to do it):  --bert
  //mat_identity<T>(res);
  mat_copy<T,T>(m1,tmp);
  if(tmp[0][0] == (T)0){
    for(i = 1; i <= nbRow; i++){
      assert_forced_printf(i!=nbRow,"Singular matrix");      
      if(tmp[i][0] != (T)0){
        mat_swapRows<T>(tmp,0,i);
        mat_swapRows<T>(res,0,i);
        break;
      }
    }
  }
  for(i = 0; i < nbRow; i++){
    //assert_forced_printf(tmp[i][i] != (T)0,"Singular matrix");
    if(tmp[i][i] != (T)1){
      T invTmpii=(T)1.0/tmp[i][i];
      for(j = 0; j < nbCol; j++)    res[i][j] *= invTmpii;
      for(j = i+1; j < nbCol; j++)  tmp[i][j] *= invTmpii;
      tmp[i][i] = T(1);
    }
    for(k = 0; k < i; k++){
      if(tmp[k][i] != T(0)){

        T *reskj = &res.getPointer()[k*nbCol];
        T *resij = &res.getPointer()[i*nbCol];
        T tmpki = tmp[k][i];
        for(j = nbColBy4; j>0; j--)
        { 
          reskj[0] -= resij[0] * tmpki;
          reskj[1] -= resij[1] * tmpki;
          reskj[2] -= resij[2] * tmpki;
          reskj[3] -= resij[3] * tmpki;
          reskj+=4; resij+=4;
        }
        for(j = nbColLeft; j>0; j--)
          *reskj++ -= *resij++ * tmpki;

        reskj = &tmp.getPointer()[k*nbCol+i+1];
        resij = &tmp.getPointer()[i*nbCol+i+1];
        for(j = i+1; j<nbCol; j++)
          *reskj++ -= *resij++ * tmpki;
        // POSSIBLY SHOULD BE UNCOMMENTED:  --bert
        //for(j = i+1; j < nbCol; j++)  tmp[k][j] -= tmp[i][j] * tmpki;

        tmp[k][i] = T(0);

      }
    }
    for(k = i+1; k < nbRow; k++){
      if(tmp[k][i] != T(0)){

        T *reskj = &res.getPointer()[k*nbCol];
        T *resij = &res.getPointer()[i*nbCol];
        T tmpki = tmp[k][i];
        for(j = nbColBy4; j>0; j--)
        { 
          reskj[0] -= resij[0] * tmpki;
          reskj[1] -= resij[1] * tmpki;
          reskj[2] -= resij[2] * tmpki;
          reskj[3] -= resij[3] * tmpki;
          reskj+=4; resij+=4;
        }
        for(j = nbColLeft; j>0; j--)
          *reskj++ -= *resij++ * tmpki;

        reskj = &tmp.getPointer()[k*nbCol+i+1];
        resij = &tmp.getPointer()[i*nbCol+i+1];
        for(j = i+1; j<nbCol; j++)
          *reskj++ -= *resij++ * tmpki;
        // POSSIBLY SHOULD BE UNCOMMENTED:  --bert
        //for(j = i+1; j < nbCol; j++)  tmp[k][j] -= tmp[i][j] * tmpki;

        tmp[k][i] = T(0);

      }
    }
  }
}
#endif // ***DISABLED BAD CODE***

template <class T,class M,class Minv,class MLU,class VLUperm>   void  mat_invert(const M &m,Minv &mInv,MLU &mLU,VLUperm &vLUPerm)
{
  assert_printf(m.getNbRow()==m.getNbColumn(),"Non square matrix");
  unsigned int n=m.getNbRow(),i=n;
  T *invColumn=mInv.getPointer();

  mat_LU_decomposition<T>(m,mLU,vLUPerm);
  mat_identity<T>(mInv);
  while(i--)
  {
    mat_solveWithLU<T>(mLU,vLUPerm,invColumn);
    invColumn+=n;
  }
  mat_transpose<T>(mInv);
}

template <class T,class M,class Minv,class MLU,class VLUperm>   void  mat_invert(const M &m,Minv &mInv,MLU &mLU,VLUperm &vLUPerm,unsigned int dim)
{
  assert_printf(m.getNbRow()==m.getNbColumn(),"Non square matrix");
  unsigned int n=m.getNbRow(),i=dim;
  T *invColumn=mInv.getPointer();

  mat_LU_decomposition<T>(m,mLU,vLUPerm,dim);
  mat_identity<T>(mInv);
  while(i--)
  {
    mat_solveWithLU<T>(mLU,vLUPerm,invColumn,dim);
    invColumn+=n;
  }
  mat_transpose<T>(mInv,dim);
}

template <class T,class M,class Minv,class MLU,class VLUperm>   void  mat_invert_band(const M &m,Minv &mInv,MLU &mLU,VLUperm &vLUPerm,unsigned int bandWidth)
{
  assert_printf(m.getNbRow()==m.getNbColumn(),"Non square matrix");
  unsigned int n=m.getNbRow(),i=n;
  T *invColumn=mInv.getPointer();

  mat_LU_decomposition_band<T>(m,mLU,vLUPerm,bandWidth);
  mat_identity<T>(mInv);
  while(i--)
  {
    mat_solveWithLU_band<T>(mLU,vLUPerm,invColumn,bandWidth);
    invColumn+=n;
  }
  mat_transpose_band<T>(mInv,bandWidth);
}

template <class T,class M,class Minv,class MLU,class VLUperm>   void  mat_invert_band(const M &m,Minv &mInv,MLU &mLU,VLUperm &vLUPerm,unsigned int dim,unsigned int bandWidth)
{
  assert_printf(m.getNbRow()==m.getNbColumn(),"Non square matrix");
  unsigned int n=m.getNbRow(),i=dim;
  T *invColumn=mInv.getPointer();

  mat_LU_decomposition_band<T>(m,mLU,vLUPerm,dim,bandWidth);
  mat_identity<T>(mInv);
  while(i--)
  {
    mat_solveWithLU_band<T>(mLU,vLUPerm,invColumn,dim,bandWidth);
    invColumn+=n;
  }
  mat_transpose_band<T>(mInv,dim,bandWidth);
}


template <class T,class M,class ML,class MU,class Vperm>   void  mat_LU_decomposition(const M &a,ML &L,MU &U,Vperm &t)
{
  assert_printf(a.getNbRow()==a.getNbColumn(),"Non square matrix a");
  unsigned int i,j,k;
  unsigned int n=a.getNbRow();
    
  mat_copy<T,T>(a,U);
  mat_identity<T>(L);
  for(i=0;i<n;i++) t[i]=i;

  for(k=0;k<n-1;k++)
  {
    //recherche un pivot non null
    unsigned int line=k;//matrix_pivot_non_nul_lower(n,U,k);
    while(U[line][k]==0)
    {
      line++;
      assert_forced_printf(line<n,"Pivot not found in LU decomposition !");
    }
    
    if (line != k)
    {
      mat_swapRows<T>(U,k,line);
      unsigned int tmp=t[k]; t[k]=t[line]; t[line]=tmp;
    }
    // LU
    for(i=k+1;i<n;i++) L[i][k]=U[i][k]/U[k][k];
    for(i=k+1;i<n;i++)
      for(j=k+1;j<n;j++) U[i][j]-=L[i][k]*U[k][j];
    for(j=0;j<=k;j++) U[k+1][j]=0;
  }
}

template <class T,class M,class MLU,class Vperm>   void  mat_LU_decomposition(const M &a,MLU &LU,Vperm &t)
{
  assert_printf(a.getNbRow()==a.getNbColumn(),"Non square matrix a");
  unsigned int i=a.getNbRow(),j,k=0;
  unsigned int n=i,nbIter;
    
  mat_copy<T,T>(a,LU);
  while(i--) t[i]=i;

  T *LUk=LU.getPointer();
  T *LUkk=LUk;
  for(;k<n-1;k++)
  {
		T* LUik=LUkk;
		// Get the bigger Gauss Pivot for the current entry k !
		{
			T big = (*LUkk<0?-*LUkk:*LUkk);
			unsigned int mu = k;
			for(i=k+1;i<n;i++)
			{
				LUik += n; const T trybig = (*LUik<0?-*LUik:*LUik);
				if(big < trybig) { big = trybig; mu = i; }
			}			
			if(mu!=k){ mat_swapRows<T>(LU,k,mu); t[k]=mu; }
		}

    LUik=LUkk+n;
    nbIter = n-k-1;
    // LU
    i=nbIter;
    while(i--)
    {
      T factor= (*LUik) / (*LUkk);
      *LUik = factor; // L part

      T *LUij=LUik+1;
      T *LUkj=LUk+k+1;
      
      // U part
      // 4 by 4
      j=(nbIter) >> 2;
      while(j--)
      {
        LUij[0] -= factor * LUkj[0];
        LUij[1] -= factor * LUkj[1];
        LUij[2] -= factor * LUkj[2];
        LUij[3] -= factor * LUkj[3];
        LUij+=4; LUkj+=4;
      }
      // 1 by 1 (for the left over)
      j=(nbIter) & 3;
      while(j--)
        *LUij++ -= factor * *LUkj++;

      LUik+=n;
    }
    LUk += n;     // Next line -> line++
    LUkk += n+1;  // Next diagonal element -> line++ column++
  }
}

template <class T,class M,class MLU,class Vperm>   void  mat_LU_decomposition(const M &a,MLU &LU,Vperm &t,unsigned int n)
{
//  assert_printf(a.getNbRow()==a.getNbColumn(),"Non square matrix a");
  unsigned int i=n,j,k=0;
  unsigned int offsetNextLine=a.getNbRow(),nbIter;
    
  mat_copy<T,T>(a,LU,n,n);
  while(i--) t[i]=i;

  T *LUk=LU.getPointer();
  T *LUkk=LUk;
  for(;k<n-1;k++)
  {
		T* LUik=LUkk;
		// Get the bigger Gauss Pivot for the current entry k !
		{
			T big = (*LUkk<0?-*LUkk:*LUkk);
			unsigned int mu = k;
			for(i=k+1;i<n;i++)
			{
				LUik += offsetNextLine; const T trybig = (*LUik<0?-*LUik:*LUik);
				if(big < trybig) { big = trybig; mu = i; }
			}			
			if(mu!=k){ mat_swapRows<T>(LU,k,mu,n); t[k]=mu; }
		}
    LUik=LUkk+offsetNextLine;
    nbIter = n-k-1;
    // LU
    i=nbIter;
    while(i--)
    {
      T factor= (*LUik) / (*LUkk);
      *LUik = factor; // L part

      T *LUij=LUik+1;
      T *LUkj=LUk+k+1;
      
      // U part
      // 4 by 4
      j=(nbIter) >> 2;
      while(j--)
      {
        LUij[0] -= factor * LUkj[0];
        LUij[1] -= factor * LUkj[1];
        LUij[2] -= factor * LUkj[2];
        LUij[3] -= factor * LUkj[3];
        LUij+=4; LUkj+=4;
      }
      // 1 by 1 (for the left over)
      j=(nbIter) & 3;
      while(j--)
        *LUij++ -= factor * *LUkj++;

      LUik+=offsetNextLine;
    }
    LUk += offsetNextLine;     // Next line -> line++
    LUkk += offsetNextLine+1;  // Next diagonal element -> line++ column++
  }
}

template <class T,class M,class MLU,class Vperm>   void  mat_LU_decomposition_band(const M &a,MLU &LU,Vperm &t,unsigned int bandWidth)
{
  assert_printf(a.getNbRow()==a.getNbColumn(),"Non square matrix a");
  unsigned int i=a.getNbRow(),j,k=0;
  unsigned int n=i,nbIter;
    
  mat_copy<T,T>(a,LU);
  while(i--) t[i]=i;

  T *LUk=LU.getPointer();
  T *LUkk=LUk;
  for(;k<n-1;k++)
  {
		T* LUik=LUkk;
		// Get the bigger Gauss Pivot for the current entry k !
		{
			T big = (*LUkk<0?-*LUkk:*LUkk);
			unsigned int mu = k , max = (k+bandWidth>=n ? n:k+bandWidth);
			for(i=k+1;i<max;i++)
			{
				LUik += n; const T trybig = (*LUik<0?-*LUik:*LUik);
				if(big < trybig) { big = trybig; mu = i; }
			}
			assert_printf(LU[mu][k]!=0.0,"Pivot not found => singular matrix !");
			if(mu!=k){ mat_swapRows<T>(LU,k,mu); t[k]=mu; }
		}

    LUik=LUkk+n;
    nbIter = n-k-1;
    // LU
    i=nbIter;
    while(i--)
    {
      T factor= (*LUik) / (*LUkk);
      *LUik = factor; // L part

      T *LUij=LUik+1;
      T *LUkj=LUk+k+1;
      
      // U part
      // 4 by 4
      j=(nbIter) >> 2;
      while(j--)
      {
        LUij[0] -= factor * LUkj[0];
        LUij[1] -= factor * LUkj[1];
        LUij[2] -= factor * LUkj[2];
        LUij[3] -= factor * LUkj[3];
        LUij+=4; LUkj+=4;
      }
      // 1 by 1 (for the left over)
      j=(nbIter) & 3;
      while(j--)
        *LUij++ -= factor * *LUkj++;

      LUik+=n;
    }
    LUk += n;     // Next line -> line++
    LUkk += n+1;  // Next diagonal element -> line++ column++
  }
}

template <class T,class M,class MLU,class Vperm>   void  mat_LU_decomposition_band(const M &a,MLU &LU,Vperm &t,unsigned int n,unsigned int bandWidth)
{
  assert_printf(a.getNbRow()==a.getNbColumn(),"Non square matrix a");
  unsigned int i=n,j,k=0;
  unsigned int offsetNextLine=a.getNbRow(),nbIter;
    
  mat_copy<T,T>(a,LU,n,n);
  while(i--) t[i]=i;

  T *LUk=LU.getPointer();
  T *LUkk=LUk;
  for(;k<n-1;k++)
  {
		T* LUik=LUkk;
		// Get the bigger Gauss Pivot for the current entry k !
		{
			T big = (*LUkk<0?-*LUkk:*LUkk);
			unsigned int mu = k , max = (k+bandWidth>=n ? n:k+bandWidth);
			for(i=k+1;i<max;i++)
			{
				LUik += offsetNextLine; const T trybig = (*LUik<0?-*LUik:*LUik);
				if(big < trybig) { big = trybig; mu = i; }
			}
			assert_printf(LU[mu][k]!=0.0,"Pivot not found => singular matrix !");
			if(mu!=k){ mat_swapRows<T>(LU,k,mu,n); t[k]=mu; }
		}
    LUik=LUkk+offsetNextLine;
    nbIter = n-k-1;
    // LU
    i=nbIter;
    while(i--)
    {
      T factor= (*LUik) / (*LUkk);
      *LUik = factor; // L part

      T *LUij=LUik+1;
      T *LUkj=LUk+k+1;
      
      // U part
      // 4 by 4
      j=(nbIter) >> 2;
      while(j--)
      {
        LUij[0] -= factor * LUkj[0];
        LUij[1] -= factor * LUkj[1];
        LUij[2] -= factor * LUkj[2];
        LUij[3] -= factor * LUkj[3];
        LUij+=4; LUkj+=4;
      }
      // 1 by 1 (for the left over)
      j=(nbIter) & 3;
      while(j--)
        *LUij++ -= factor * *LUkj++;

      LUik+=offsetNextLine;
    }
    LUk += offsetNextLine;     // Next line -> line++
    LUkk += offsetNextLine+1;  // Next diagonal element -> line++ column++
  }
}


template <class T,class MLU,class Vperm,class V1,class V2>   void  mat_solveWithLU(const MLU &LU,const Vperm &t,V1 &b,V2 &x)
{
  int i,j,n=x.getSize();

  // L resolution
  for(i=0;i<n;i++)
	  for(j=0; j<i; j++)
		  b[i]-=LU[i][j]*b[j];

  // U resolution
  for(i=n-1;i>=0;i--)
	{
	  x[i]=b[i];
	  for(j=i+1; j<n; j++)	
		  x[i]-=LU[i][j]*x[j];
	  x[i]/=LU[i][i];
	}
}

template <class T,class MLU,class Vperm>   void  mat_solveWithLU_strip(const MLU &LU,const Vperm &t,T *x,unsigned int strip)
{
  int i=1,n=(int)LU.getNbRow(),j=n;
  T *xj,*xi=x+strip,*LUi=LU.getPointer()+n,*LUij=LUi;

  // Swaping the vector accordingly to 't'
  while(j--)
  { T tmp=x[t[j]*strip]; x[t[j]*strip]=x[j*strip]; x[j*strip]=tmp; }

  // L resolution
  for(i=1;i<n;i++)
  {
    xj=x; LUij=LUi;

    register T value=*xi;
    j = i >> 2; // 4 by 4
	  while(j--)
    {
		  value-=LUij[0]*xj[0]; value-=LUij[1]*xj[strip]; value-=LUij[2]*xj[2*strip]; value-=LUij[3]*xj[3*strip];
      LUij+=4; xj+=4*strip;
    }    
    j = i & 3; // 1 by 1 (for the left over)
    while(j--){ value -= *LUij++ * *xj; xj+=strip; }
    *xi = value;

    xi+=strip; LUi+=n;
  }

  // U resolution
  xi-=strip; LUi-=n;
  for(i=n-1;i>=0;i--)
	{
    xj=xi+strip; LUij=LUi+i+1;

    register T value=*xi;    
    j=(n-i-1) >> 2; // 4 by 4
	  while(j--)
    {
		  value-=LUij[0]*xj[0]; value-=LUij[1]*xj[strip]; value-=LUij[2]*xj[2*strip]; value-=LUij[3]*xj[3*strip];
      LUij+=4; xj+=4*strip;
    }    
    j=(n-i-1) & 3; // 1 by 1 (for the left over)
    while(j--){ value -= *LUij++ * *xj; xj+=strip; }
    *xi = value/LUi[i];

    xi-=strip; LUi-=n;
	}
}

template <class T,class MLU,class Vperm>   void  mat_solveWithLU(const MLU &LU,const Vperm &t,T *x)
{
	unsigned int n=(int)LU.getNbRow() , i , j , firstNonNull=n;  // firstNonNull initialised : x might be all zeros

	// Apply the permutation on the x vector
	for(i=0;i<n;i++){ j=t[i]; register T temp = x[j]; x[j] = x[i]; x[i] = temp; }

	// L resolution accelerated by searching the first non zero value in 'x'
	for (i = 0; i < n; i++) if( x[i]!=0.0 ) { firstNonNull = i; break; }
	if (firstNonNull == n) return; // x is a null vector => solution = null vector !
	i = firstNonNull + 1;
	T *xi = x+firstNonNull , *LU_i_first=LU.getPointer() + firstNonNull + i * n;
	if(i<n) for (;;)
	{
		T sum[4]={0,0,0,0}; T* LUij = LU_i_first; T* xj = xi;
		j = (i - firstNonNull) >> 2;
		while (j--)
		{
			sum[0] += LUij[0] * xj[0];
			sum[1] += LUij[1] * xj[1];
			sum[2] += LUij[2] * xj[2];
			sum[3] += LUij[3] * xj[3];
			LUij+=4; xj+=4;
		}
		j = (i - firstNonNull) & 3;
		while (j--) sum[0] += *LUij++ * *xj++;
		x[i] -= (sum[0]+sum[1]+sum[2]+sum[3]);
		if (++i == n) break;
		LU_i_first += n;
	}

	// R resolution accelerated by searching the last non zero value in 'x'
	for (i = n-1;;i--) { if( x[i] ) { firstNonNull = i; break; } if(i==0) return;/*This case should never happens !*/ }
	LU_i_first = LU.getPointer() + (firstNonNull+1) * n;
	for (i = firstNonNull;;)
	{
		T* xj = x+i; LU_i_first -= n; T* LUij = LU_i_first+i;
		T sum[4] = {*xj,0,0,0}; T diag = *LUij;
		j = (firstNonNull - i) >> 2;
		while(j--)
		{
			sum[0] -= LUij[1] * xj[1];
			sum[1] -= LUij[2] * xj[2];
			sum[2] -= LUij[3] * xj[3];
			sum[3] -= LUij[4] * xj[4];
			LUij+=4; xj+=4;
		}
		j = (firstNonNull - i) & 3;
		while(j--)
			sum[0] -= *(++LUij) * *(++xj);
		x[i] = (sum[0]+sum[1]+sum[2]+sum[3]) / diag;
		if(i==0) break;
		i--;
	}
}

template <class T,class MLU,class Vperm>   void  mat_solveWithLU(const MLU &LU,const Vperm &t,T *x,unsigned int n)
{
	unsigned int nbCol=(int)LU.getNbColumn() , i , j , firstNonNull=n;  // firstNonNull initialised : x might be all zeros

	// Apply the permutation on the x vector
	for(i=0;i<n;i++){ j=t[i]; register T temp = x[j]; x[j] = x[i]; x[i] = temp; }

	// L resolution accelerated by searching the first non zero value in 'x'
	for (i = 0; i < n; i++) if( x[i]!=0.0 ) { firstNonNull = i; break; }
	if (firstNonNull == n) return; // x is a null vector => solution = null vector !
	i = firstNonNull + 1;
	const T* xi = x+firstNonNull;
	const T* LU_i_first = LU.getPointer() + firstNonNull + i * nbCol;
	if(i<n) for (;;)
	{
		T sum[4]={0,0,0,0};
		const T* LUij = LU_i_first;
		const T* xj = xi;
		j = (i - firstNonNull) >> 2;
		while (j--)
		{
			sum[0] += LUij[0] * xj[0];
			sum[1] += LUij[1] * xj[1];
			sum[2] += LUij[2] * xj[2];
			sum[3] += LUij[3] * xj[3];
			LUij+=4; xj+=4;
		}
		j = (i - firstNonNull) & 3;
		while (j--) sum[0] += *LUij++ * *xj++;
		x[i] -= (sum[0]+sum[1]+sum[2]+sum[3]);
		if (++i == n) break;
		LU_i_first += nbCol;
	}

	// R resolution accelerated by searching the last non zero value in 'x'
	for (i = n-1;;i--) { if( x[i] ) { firstNonNull = i; break; } if(i==0) return;/*This case should never happens !*/ }
	LU_i_first = LU.getPointer() + (firstNonNull+1) * nbCol;
	for (i = firstNonNull;;)
	{
		const T* xj = x+i;
		LU_i_first -= nbCol;
		const T* LUij = LU_i_first+i;
		T sum[4] = {*xj,0,0,0}; T diag = *LUij;
		j = (firstNonNull - i) >> 2;
		while(j--)
		{
			sum[0] -= LUij[1] * xj[1];
			sum[1] -= LUij[2] * xj[2];
			sum[2] -= LUij[3] * xj[3];
			sum[3] -= LUij[4] * xj[4];
			LUij+=4; xj+=4;
		}
		j = (firstNonNull - i) & 3;
		while(j--)
			sum[0] -= *(++LUij) * *(++xj);
		x[i] = (sum[0]+sum[1]+sum[2]+sum[3]) / diag;
		if(i==0) break;
		i--;
	}
}

template <class T,class MLU,class Vperm>   void  mat_solveWithLU_band(const MLU &LU,const Vperm &t,T *x,unsigned int bandWidth)
{
	unsigned int n=(int)LU.getNbRow() , i , j , firstNonNull=n;  // firstNonNull initialised : x might be all zeros
	unsigned int nbElemPerLine;

	// Apply the permutation on the x vector
	for(i=0;i<n;i++){ j=t[i]; register T temp = x[j]; x[j] = x[i]; x[i] = temp; }

	// L resolution accelerated by searching the first non zero value in 'x'
	for (i = 0; i < n; i++) if( x[i]!=0.0 ) { firstNonNull = i; break; }
	if (firstNonNull == n) return; // x is a null vector => solution = null vector !
	i = firstNonNull + 1;
	T *xi = x+firstNonNull , *LU_i_first=LU.getPointer() + firstNonNull + i * n;
	if(i<n) for (;;)
	{
		T sum[4]={0,0,0,0}; T* LUij = LU_i_first; T* xj = xi;
		nbElemPerLine = i - firstNonNull;
		j = nbElemPerLine >> 2;
		while (j--)
		{
			sum[0] += LUij[0] * xj[0];
			sum[1] += LUij[1] * xj[1];
			sum[2] += LUij[2] * xj[2];
			sum[3] += LUij[3] * xj[3];
			LUij+=4; xj+=4;
		}
		j = nbElemPerLine & 3;
		while (j--) sum[0] += *LUij++ * *xj++;
		x[i] -= (sum[0]+sum[1]+sum[2]+sum[3]);
		if (++i == n) break;
		LU_i_first += n;
	}

	// R resolution accelerated by searching the last non zero value in 'x'
	for (i = n-1;;i--) { if( x[i] ) { firstNonNull = i; break; } if(i==0) return;/*This case should never happens !*/ }
	LU_i_first = LU.getPointer() + (firstNonNull+1) * n;
	for (i = firstNonNull;;)
	{
		T* xj = x+i; LU_i_first -= n; T* LUij = LU_i_first+i;
		T sum[4] = {*xj,0,0,0}; T diag = *LUij;
		j = (firstNonNull - i) >> 2;
		while(j--)
		{
			sum[0] -= LUij[1] * xj[1];
			sum[1] -= LUij[2] * xj[2];
			sum[2] -= LUij[3] * xj[3];
			sum[3] -= LUij[4] * xj[4];
			LUij+=4; xj+=4;
		}
		j = (firstNonNull - i) & 3;
		while(j--)
			sum[0] -= *(++LUij) * *(++xj);
		x[i] = (sum[0]+sum[1]+sum[2]+sum[3]) / diag;
		if(i==0) break;
		i--;
	}
}

template <class T,class MLU,class Vperm>   void  mat_solveWithLU_band(const MLU &LU,const Vperm &t,T *x,unsigned int n,unsigned int bandWidth)
{
	unsigned int nbCol=(int)LU.getNbColumn() , i , j , firstNonNull=n;  // firstNonNull initialised : x might be all zeros
	unsigned int nbElemPerLine;

	// Apply the permutation on the x vector
	for(i=0;i<n;i++){ j=t[i]; register T temp = x[j]; x[j] = x[i]; x[i] = temp; }

	// L resolution accelerated by searching the first non zero value in 'x'
	for (i = 0; i < n; i++) if( x[i]!=0.0 ) { firstNonNull = i; break; }
	if (firstNonNull == n) return; // x is a null vector => solution = null vector !
	i = firstNonNull + 1;
	T *xi = x+firstNonNull , *LU_i_first=LU.getPointer() + firstNonNull + i * nbCol;
	if(i<n) for (;;)
	{
		T sum[4]={0,0,0,0}; T* LUij = LU_i_first; T* xj = xi;
		nbElemPerLine = i - firstNonNull;
		j = nbElemPerLine >> 2;
		while (j--)
		{
			sum[0] += LUij[0] * xj[0];
			sum[1] += LUij[1] * xj[1];
			sum[2] += LUij[2] * xj[2];
			sum[3] += LUij[3] * xj[3];
			LUij+=4; xj+=4;
		}
		j = nbElemPerLine & 3;
		while (j--) sum[0] += *LUij++ * *xj++;
		x[i] -= (sum[0]+sum[1]+sum[2]+sum[3]);
		if (++i == n) break;
		LU_i_first += nbCol;
	}

	// R resolution accelerated by searching the last non zero value in 'x'
	for (i = n-1;;i--) { if( x[i] ) { firstNonNull = i; break; } if(i==0) return; }
	LU_i_first = LU.getPointer() + (firstNonNull+1) * nbCol;
	for (i = firstNonNull;;)
	{
		T* xj = x+i; LU_i_first -= nbCol; T* LUij = LU_i_first+i;
		T sum[4] = {*xj,0,0,0}; T diag = *LUij;
		nbElemPerLine = firstNonNull - i;
		j = nbElemPerLine >> 2;
		while(j--)
		{
			sum[0] -= LUij[1] * xj[1];
			sum[1] -= LUij[2] * xj[2];
			sum[2] -= LUij[3] * xj[3];
			sum[3] -= LUij[4] * xj[4];
			LUij+=4; xj+=4;
		}
		j = nbElemPerLine & 3;
		while(j--)
			sum[0] -= *(++LUij) * *(++xj);
		x[i] = (sum[0]+sum[1]+sum[2]+sum[3]) / diag;
		if(i==0) break;
		i--;
	}
}

// Transposition
template <class M1,class Mres>                      void  mat_transpose(const M1 &m1, Mres &res) throw(...)
{
  register unsigned int m1NbRow = m1.getNbRow();
  register unsigned int m1NbCol = m1.getNbColumn();
#ifdef Matrix_Exceptions
  if(m1NbRow>res.getNbColumn()) throw("Exception in \n template <class M1,class Mres> void mat_transpose(const M1 &m1, Mres &res) \n -> m1.getNbRow() > res.getNbColumn()");
  if(m1NbCol>res.getNbRow())    throw("Exception in \n template <class M1,class Mres> void mat_transpose(const M1 &m1, Mres &res) \n -> m1.getNbColumn() > res.getNbRow()");
#endif
  for(register unsigned int i=0;i<m1NbRow;i++)
    for(register unsigned int j=0;j<m1NbCol;j++)
      res[j][i] = m1[i][j];
}

template <class T,class M1>                         void  mat_transpose(M1 &a)
{
  assert_printf(a.getNbRow()==a.getNbColumn(),"Cannot transpose a matrix on itself if it is not square !");
	T *akk=a.getPointer(),tmp[4];
	const unsigned int nbCol = a.getNbColumn();
	const unsigned int _2nbCol = nbCol << 1;
	const unsigned int _3nbCol = nbCol + _2nbCol;
  const unsigned int _4nbCol = nbCol << 2;
	unsigned int j,dim=nbCol;

	if(dim--==0) return;
  while(dim--)
  {
		T *aki=akk+1 , *aik=akk+nbCol;
    j = (dim+1) >> 2;
    while(j--)
    {
      tmp[0] = aki[0]; tmp[1] = aki[1]; tmp[2] = aki[2]; tmp[3] = aki[3];
      aki[0]=*aik; aki[1]=*(aik+nbCol); aki[2]=*(aik+_2nbCol); aki[3]=*(aik+_3nbCol);
      *aik=tmp[0]; *(aik+nbCol)=tmp[1]; *(aik+_2nbCol)=tmp[2]; *(aik+_3nbCol)=tmp[3];
      aki+=4; aik+=_4nbCol;
    }
    j = (dim+1) & 3;
		while(j--)
    { tmp[0] = *aki; *aki=*aik; *aik=tmp[0]; aik+=nbCol; aki++; }

    akk += nbCol+1;  // prepare next line (go to the next diagonal element)
  }
}

template <class T,class M1>                         void  mat_transpose(M1 &a,unsigned int dim)
{
  assert_printf(a.getNbRow()==a.getNbColumn(),"Cannot transpose a matrix on itself if it is not square !");
	
	T *akk=a.getPointer(),tmp[4];
	const unsigned int nbCol = a.getNbColumn();
	const unsigned int _2nbCol = nbCol << 1;
	const unsigned int _3nbCol = nbCol + _2nbCol;
  const unsigned int _4nbCol = nbCol << 2;
	unsigned int j;//, nbElemToTranspose;

	if(dim--==0) return;
  while(dim--)
  {
		T *aki=akk+1 , *aik=akk+nbCol;
    j = (dim+1) >> 2;
    while(j--)
    {
      tmp[0] = aki[0]; tmp[1] = aki[1]; tmp[2] = aki[2]; tmp[3] = aki[3];
      aki[0]=*aik; aki[1]=*(aik+nbCol); aki[2]=*(aik+_2nbCol); aki[3]=*(aik+_3nbCol);
      *aik=tmp[0]; *(aik+nbCol)=tmp[1]; *(aik+_2nbCol)=tmp[2]; *(aik+_3nbCol)=tmp[3];
      aki+=4; aik+=_4nbCol;
    }
    j = (dim+1) & 3;
		while(j--)
    { tmp[0] = *aki; *aki=*aik; *aik=tmp[0]; aik+=nbCol; aki++; }

    akk += nbCol+1;  // prepare next line (go to the next diagonal element)
  }
}

template <class T,class M1>                         void  mat_transpose(M1 &a,unsigned int nbRow,unsigned int nbCol)
{
	assert_printf(nbRow==nbCol,"Cannot transpose a matrix on itself if it is not square !");

	for (unsigned int i = 0; i < nbRow; i++)
	{
		for (unsigned int j = 0; j < nbCol; j++)
		{
			T temp = a[i][j];
			a[i][j] = a[j][i];
			a[j][i] = temp;
		}
	}
}

template <class M1,class MRes>              void  mat_transpose(const M1 &a,unsigned int nbRow,unsigned int nbCol,MRes &res)
{
	for (unsigned int i = 0; i < nbRow; i++)
	{
		for (unsigned int j = 0; j < nbCol; j++)
		{
			res[i][j] = a[j][i];
		}
	}
}

template <class T,class M1>                         void  mat_transpose_band(M1 &a,unsigned int bandWidth)
{
  assert_printf(a.getNbRow()==a.getNbColumn(),"Cannot transpose a matrix on itself if it is not square !");
	assert_printf(bandWidth%2==1,"The band width shoudd be an odd number (same number of element around the diagonal element)");
	T *akk=a.getPointer(),tmp[4];
	const unsigned int nbCol = a.getNbColumn();
	const unsigned int _2nbCol = nbCol << 1;
	const unsigned int _3nbCol = nbCol + _2nbCol;
  const unsigned int _4nbCol = nbCol << 2;
	unsigned int j, dim=nbCol, nbElemToTranspose;

	if(dim--==0) return;
  while(dim--)
  {
		nbElemToTranspose = ( (dim+1) > bandWidth>>1 ? bandWidth>>1 : dim+1 );
		T *aki=akk+1 , *aik=akk+nbCol;
    j = nbElemToTranspose >> 2;   
    while(j--)
    {
      tmp[0] = aki[0]; tmp[1] = aki[1]; tmp[2] = aki[2]; tmp[3] = aki[3];
      aki[0]=*aik; aki[1]=*(aik+nbCol); aki[2]=*(aik+_2nbCol); aki[3]=*(aik+_3nbCol);
      *aik=tmp[0]; *(aik+nbCol)=tmp[1]; *(aik+_2nbCol)=tmp[2]; *(aik+_3nbCol)=tmp[3];
      aki+=4; aik+=_4nbCol;
    }
    j = nbElemToTranspose & 3;
		while(j--)
    { tmp[0] = *aki; *aki=*aik; *aik=tmp[0]; aik+=nbCol; aki++; }

    akk += nbCol+1;  // prepare next line (go to the next diagonal element)
  }
}

template <class T,class M1>                         void  mat_transpose_band(M1 &a,unsigned int dim,unsigned  int bandWidth)
{
  assert_printf(a.getNbRow()==a.getNbColumn(),"Cannot transpose a matrix on itself if it is not square !");
	assert_printf(bandWidth%2==1,"The band width shoudd be an odd number (same number of element around the diagonal element)");
	
	T *akk=a.getPointer(),tmp[4];
	const unsigned int nbCol = a.getNbColumn();
	const unsigned int _2nbCol = nbCol << 1;
	const unsigned int _3nbCol = nbCol + _2nbCol;
  const unsigned int _4nbCol = nbCol << 2;
	unsigned int j,nbElemToTranspose;

	if(dim--==0) return;
  while(dim--)
  {
		nbElemToTranspose = ( (dim+1) > bandWidth>>1 ? bandWidth>>1 : dim+1 );
		T *aki=akk+1 , *aik=akk+nbCol;
    j = nbElemToTranspose >> 2;
    while(j--)
    {
      tmp[0] = aki[0]; tmp[1] = aki[1]; tmp[2] = aki[2]; tmp[3] = aki[3];
      aki[0]=*aik; aki[1]=*(aik+nbCol); aki[2]=*(aik+_2nbCol); aki[3]=*(aik+_3nbCol);
      *aik=tmp[0]; *(aik+nbCol)=tmp[1]; *(aik+_2nbCol)=tmp[2]; *(aik+_3nbCol)=tmp[3];
      aki+=4; aik+=_4nbCol;
    }
    j = nbElemToTranspose & 3;
		while(j--)
    { tmp[0] = *aki; *aki=*aik; *aik=tmp[0]; aik+=nbCol; aki++; }

    akk += nbCol+1;  // prepare next line (go to the next diagonal element)
  }
}
//////////////////////////////////////////////////


//////////////////////////////////////////////////
// Generic functions on matrices
  // 3D
template <class T,class Mres>                       void  mat_rotationX3D(T angle, Mres &res) throw(...)
{
#ifdef Matrix_Exceptions
  if(res.getSize()<3) throw("Exception in \n template <class T,class Mres>             void  mat_rotationX3D(T angle, Mres &res) \n -> res.getSize()<3 !");
#endif
  T cosA,sinA;
  getCosSin<T>(angle,cosA,sinA);
  res[0][0] = (T)1; res[0][1] = (T)0; res[0][2] = (T)0;
  res[1][0] = (T)0; res[1][1] = cosA; res[1][2] =-sinA;
  res[2][0] = (T)0; res[2][1] = sinA; res[2][2] = cosA;
}

template <class T,class Mres>                       void  mat_rotationY3D(T angle, Mres &res) throw(...)
{
#ifdef Matrix_Exceptions
  if(res.getSize()<3) throw("Exception in \n template <class T,class Mres>             void  mat_rotationY3D(T angle, Mres &res) \n -> res.getSize()<3 !");
#endif
  T cosA,sinA;
  getCosSin<T>(angle,cosA,sinA);
  res[0][0] = cosA; res[0][1] = (T)0; res[0][2] = sinA;
  res[1][0] = (T)0; res[1][1] = (T)1; res[1][2] = (T)0;
  res[2][0] =-sinA; res[2][1] = (T)0; res[2][2] = cosA;
}

template <class T,class Mres>                       void  mat_rotationZ3D(T angle, Mres &res) throw(...)
{
#ifdef Matrix_Exceptions
  if(res.getSize()<3) throw("Exception in \n template <class T,class Mres>             void  mat_rotationZ3D(T angle, Mres &res) \n -> res.getSize()<3 !");
#endif
  T cosA,sinA;
  getCosSin<T>(angle,cosA,sinA);
  res[0][0] = cosA; res[0][1] =-sinA; res[0][2] = (T)0;
  res[1][0] = sinA; res[1][1] = cosA; res[1][2] = (T)0;
  res[2][0] = (T)0; res[2][1] = (T)0; res[2][2] = (T)1;
}

  // 4D
template <class T,class Mres>                       void  mat_rotationX4D(T angle, Mres &res) throw(...)
{
#ifdef Matrix_Exceptions
  if(res.getSize()<4) throw("Exception in \n template <class T,class Mres>             void  mat_rotationX4D(T angle, Mres &res) \n -> res.getSize()<4 !");
#endif
  T cosA,sinA;
  getCosSin<T>(angle,cosA,sinA);
  res[0][0] = (T)1; res[0][1] = (T)0; res[0][2] = (T)0; res[0][3] = (T)0;
  res[1][0] = (T)0; res[1][1] = cosA; res[1][2] =-sinA; res[1][3] = (T)0;
  res[2][0] = (T)0; res[2][1] = sinA; res[2][2] = cosA; res[2][3] = (T)0;
  res[3][0] = (T)0; res[3][1] = (T)0; res[3][2] = (T)0; res[3][3] = (T)1;
}

template <class T,class Mres>                       void  mat_rotationY4D(T angle, Mres &res) throw(...)
{
#ifdef Matrix_Exceptions
  if(res.getSize()<4) throw("Exception in \n template <class T,class Mres>             void  mat_rotationY4D(T angle, Mres &res) \n -> res.getSize()<4 !");
#endif
  T cosA,sinA;
  getCosSin<T>(angle,cosA,sinA);
  res[0][0] = cosA; res[0][1] = (T)0; res[0][2] = sinA; res[0][3] = (T)0;
  res[1][0] = (T)0; res[1][1] = (T)1; res[1][2] = (T)0; res[1][3] = (T)0;
  res[2][0] =-sinA; res[2][1] = (T)0; res[2][2] = cosA; res[2][3] = (T)0;
  res[3][0] = (T)0; res[3][1] = (T)0; res[3][2] = (T)0; res[3][3] = (T)1;
}

template <class T,class Mres>                       void  mat_rotationZ4D(T angle, Mres &res) throw(...)
{
#ifdef Matrix_Exceptions
  if(res.getSize()<4) throw("Exception in \n template <class T,class Mres>             void  mat_rotationZ4D(T angle, Mres &res) \n -> res.getSize()<4 !");
#endif
  T cosA,sinA;
  getCosSin<T>(angle,cosA,sinA);
  res[0][0] = cosA; res[0][1] =-sinA; res[0][2] = (T)0; res[0][3] = (T)0;
  res[1][0] = sinA; res[1][1] = cosA; res[1][2] = (T)0; res[1][3] = (T)0;
  res[2][0] = (T)0; res[2][1] = (T)0; res[2][2] = (T)1; res[2][3] = (T)0;
  res[3][0] = (T)0; res[3][1] = (T)0; res[3][2] = (T)0; res[3][3] = (T)1;
}

template <class V1,class Mres>                      void  mat_translation4D(const V1 &t, Mres &res) throw(...)
{
#ifdef Matrix_Exceptions
  if(res.getSize()<4) throw("Exception in \n template <class T,class Mres>             void  mat_translation4D(T angle, Mres &res) \n -> res.getSize()<4 !");
#endif
  res[0][0] = (T)1; res[0][1] = (T)0; res[0][2] = (T)0; res[0][3] = t[0];
  res[1][0] = (T)0; res[1][1] = (T)1; res[1][2] = (T)0; res[1][3] = t[1];
  res[2][0] = (T)0; res[2][1] = (T)0; res[2][2] = (T)1; res[2][3] = t[2];
  res[3][0] = (T)0; res[3][1] = (T)0; res[3][2] = (T)0; res[3][3] = (T)1;
}

template <class V1,class Mres>                      void  mat_scale4D(const V1 &s, Mres &res) throw(...)
{
#ifdef Matrix_Exceptions
  if(res.getSize()<4) throw("Exception in \n template <class V1,class Mres>            void  mat_scale4D(const V1 &s, Mres &res) \n -> res.getSize()<4 !");
#endif
  res[0][0] = s[0]; res[0][1] = (T)0; res[0][2] = (T)0; res[0][3] = (T)0;
  res[1][0] = (T)0; res[1][1] = s[1]; res[1][2] = (T)0; res[1][3] = (T)0;
  res[2][0] = (T)0; res[2][1] = (T)0; res[2][2] = s[2]; res[2][3] = (T)0;
  res[3][0] = (T)0; res[3][1] = (T)0; res[3][2] = (T)0; res[3][3] = (T)1;
}
//////////////////////////////////////////////////


//////////////////////////////////////////////////
// Output
template <class T,class Allocator>                          ostream & operator <<(ostream &o, const Dynamic_Matrix<T,Allocator> &m)
{
  unsigned int nbRow=m.getNbRow(),i;
  unsigned int nbColumn=m.getNbColumn(),j;

  for(i=0;i<nbRow;i++)
  {
    //o << "(";
    for(j=0;j<nbColumn;j++)
      o << " " << m[i][j];
    //o << " )";
    if(i<nbRow-1) o << endl;
  }
  return o;
}
template <class T,unsigned int nbRow,unsigned int nbColumn> ostream & operator <<(ostream &o, const Static_Matrix<T,nbRow,nbColumn> &m)
{
  unsigned int i,j;

  for(i=0;i<nbRow;i++)
  {
    //o << "(";
    for(j=0;j<nbColumn;j++)
      o << " " << m[i][j];
    //o << " )";
    if(i<nbRow-1) o << endl;
  }
  return o;
}
//////////////////////////////////////////////////