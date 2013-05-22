#include "./vector_analyse.h"
//#include <SqAssert.h>

//////////////////////////////////////////////////
// Dynamic_Vector class
template <class T, class Allocator>               Dynamic_Vector<T,Allocator>::Dynamic_Vector() : size(0), allocatedSize(0), data(0) {};

template <class T, class Allocator>               Dynamic_Vector<T,Allocator>::Dynamic_Vector(unsigned int n)
  : size(n), allocatedSize(n), data(Allocator::alloc(allocatedSize))
{
#ifdef Dynamic_Vector_Constructor_Empty_Show
  cout << " **Debug** Dynamic_Vector<T>::Dynamic_Vector(" << n << ")" << endl;
#endif
  assert_printf(data!=(T*)0,"Memory allocation failed (null pointer) !");
}

template <class T, class Allocator>               Dynamic_Vector<T,Allocator>::Dynamic_Vector(unsigned int n,T first,...)
  : size(n), allocatedSize(n), data(Allocator::alloc(allocatedSize))
{
#ifdef Dynamic_Vector_Debug_Constructor_Set
  cout << " **Debug** Dynamic_Vector<T>::Dynamic_Vector(" << n << "," << first << ",...)" << endl;
#endif
  assert_printf(data!=(T*)0,"Memory allocation failed (null pointer) !");
  assert_printf(n!=0,"Dimension = 0, but parameters are set !");

  va_list args;
  va_start( args , first );
  _analyse_vector_args( data , size , first , args );
  va_end(args);
}

template <class T, class Allocator>               Dynamic_Vector<T,Allocator>::Dynamic_Vector(const Dynamic_Vector& v)
  : size(v.size), allocatedSize(v.size), data(Allocator::alloc(allocatedSize))
{
#ifdef Dynamic_Vector_Debug_Constructor_Copy
  cout << " **Debug** Dynamic_Vector<T>::Dynamic_Vector(const Dynamic_Vector &v)" << endl;
#endif
  SURGSIM_ASSERT(data != 0) << "Memory allocation failed (null pointer) !";

  memcpy(data, v.data, size * sizeof(T));
}

template <class T, class Allocator>               Dynamic_Vector<T,Allocator>& Dynamic_Vector<T,Allocator>::operator=(const Dynamic_Vector& v)
{
#ifdef Dynamic_Vector_Debug_Constructor_Copy
	cout << " **Debug** Dynamic_Vector<T>::Dynamic_Vector(const Dynamic_Vector &v)" << endl;
#endif
	resizeGrowMemory(v.size);
	memcpy(data, v.data, v.size * sizeof(T));
	return *this;
}


template <class T, class Allocator>               Dynamic_Vector<T,Allocator>::~Dynamic_Vector()
{
#ifdef Dynamic_Vector_Debug_Destrcutor
  cout << " **Debug** Dynamic_Vector<T>::~Dynamic_Vector()" << endl;
#endif
  Allocator::free(data, size);
}

template <class T, class Allocator> void          Dynamic_Vector<T,Allocator>::reserve(unsigned int storageSize)
{
	if (storageSize > allocatedSize)
	{
		// Need to allocate more storage.
		allocatedSize = storageSize;
		data = Allocator::realloc(data, allocatedSize, true, size); // the true means that we want to keep informations in the vector !
		SURGSIM_ASSERT(data != 0 || storageSize == 0) << "Memory allocation failed (null pointer) !";
	}
}
template <class T, class Allocator> void          Dynamic_Vector<T,Allocator>::resizeSetMemory(unsigned int newSize)
{
	allocatedSize = newSize;
	data = Allocator::realloc(data, allocatedSize, true, size); // the true means that we want to keep informations in the vector !
	SURGSIM_ASSERT(data != 0 || newSize == 0) << "Memory allocation failed (null pointer) !";
	size = newSize;
}
template <class T, class Allocator> void          Dynamic_Vector<T,Allocator>::resizeGrowMemory(unsigned int newSize)
{
	reserve(newSize);
	size = newSize;
}
template <class T, class Allocator> void          Dynamic_Vector<T,Allocator>::resizeToAtLeast(unsigned int minSize)
{
	// NB: if size is already greater than minSize, the size WILL NOT be changed!
	if (minSize > size)
	{
		resizeGrowMemory(minSize);
	}
}

template <class T, class Allocator> void          Dynamic_Vector<T,Allocator>::add(T &o)
{
  // XXX FIXME TODO: this really ought to use resizeGrowMemory(), but I'm too chicken to change it right now in case it introduces bugs.  --bert 2012-06-03
  resizeUpIfNecessary(size+1);
  data[size-1] = o;
}

template <class T, class Allocator> T &           Dynamic_Vector<T,Allocator>::operator [](unsigned int i)
{
#ifdef Dynamic_Vector_Debug_Bracket
  cout << " **Debug** T &Dynamic_Vector<T>::operator [](" << i << ")" << endl;
#endif
  assert_printf(i<size,"Access out of range i=%d, size=%d",i,size);
  return data[i];
}

template <class T, class Allocator> T             Dynamic_Vector<T,Allocator>::operator [](unsigned int i) const
{
#ifdef Dynamic_Vector_Debug_Bracket_Const
  cout << " **Debug** T Dynamic_Vector<T>::operator [](" << i << ") const" << endl;
#endif
  assert_printf(i<size,"Access out of range, i=%d, size=%d",i,size);
  return data[i];
}

template <class T, class Allocator> unsigned int  Dynamic_Vector<T,Allocator>::getSize(void) const
{
#ifdef Dynamic_Vector_Debug_GetSize
  cout << " **Debug** unsigned int Dynamic_Vector<T>::getSize(void) const -> " << size << endl;
#endif
  return size;
}

template <class T, class Allocator> T *           Dynamic_Vector<T,Allocator>::getPointer(void)
{
#ifdef Dynamic_Vector_Debug_GetPointer
  cout << " **Debug** T * Dynamic_Vector<T>::getPointer(void)" << endl;
#endif
  return data;
}

template <class T, class Allocator> const T *     Dynamic_Vector<T,Allocator>::getPointer(void) const
{
#ifdef Dynamic_Vector_Debug_GetPointer
  cout << " **Debug** T * Dynamic_Vector<T>::getPointer(void)" << endl;
#endif
  return data;
}
//////////////////////////////////////////////////


//////////////////////////////////////////////////
// Output
template <class T,class Allocator>  ostream & operator <<(ostream &o, const Dynamic_Vector<T,Allocator> &v)
{
  unsigned int size=v.getSize();
  o << "(";
  for(unsigned int i=0;i<size;i++)
    o << " " << v[i];
  o << " )";
  return o;
}
///////////////////////////////////////////////