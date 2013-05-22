#include "./vector_analyse.h"

//////////////////////////////////////////////////
// Static_Vector class
template <class T,unsigned int n>               Static_Vector<T,n>::Static_Vector(T first,...)
{
#ifdef Static_Vector_Debug_Constructor_Set
  cout << " **Debug** Static_Vector<T,n>::Static_Vector(" << first << ",...)" << endl;
#endif
  assert_printf(n!=0,"Dimension = 0, but parameters are set !");
  va_list args;
  va_start( args , first );
  _analyse_vector_args( data , n , first , args );
  va_end(args);
}

template <class T,unsigned int n>               Static_Vector<T,n>::Static_Vector(const T* ptr)
{
#ifdef Static_Vector_Debug_Constructor_Pointer
  cout << " **Debug** Static_Vector<T,n>::Static_Vector(const T* ptr)" << endl;
#endif

  for (unsigned int i = 0;  i < n;  ++i) data[i] = ptr[i];
}

template <class T,unsigned int n>               Static_Vector<T,n>::Static_Vector(const Static_Vector<T,n> &v)
{
#ifdef Static_Vector_Debug_Constructor_Copy
  cout << " **Debug** Static_Vector<T,n>::Static_Vector(const Static_Vector<T,n> &v)" << endl;
#endif

  for(register unsigned int i=0;i<n;i++) data[i] = v.data[i];
}

template <class T,unsigned int n> T &           Static_Vector<T,n>::operator [](unsigned int i)
{
#ifdef Static_Vector_Debug_Bracket
  cout << " **Debug** T &Static_Vector<T,n>::operator [](" << i << ")" << endl;
#endif
  assert_printf(i<n,"Access out of range, i=%d, size=%d",i,n);
  return data[i];
}

template <class T,unsigned int n> T             Static_Vector<T,n>::operator [](unsigned int i) const
{
#ifdef Static_Vector_Debug_Bracket_Const
  cout << " **Debug** T Static_Vector<T,n>::operator [](" << i << ") const" << endl;
#endif
  assert_printf(i<n,"Access out of range, i=%d, size=%d",i,n);
  return data[i];
}


template <class T,unsigned int n> unsigned int  Static_Vector<T,n>::getSize(void) const
{
#ifdef Static_Vector_Debug_GetSize
  cout << " **Debug** unsigned int Static_Vector<T,n>::getSize(void) const -> " << n << endl;
#endif  
  return n;
}

template <class T,unsigned int n> T *           Static_Vector<T,n>::getPointer(void)
{
#ifdef Static_Vector_Debug_GetPointer
  cout << " **Debug** T *Static_Vector<T,n>::getPointer(void)" << endl;
#endif  
  return data;
}
//////////////////////////////////////////////////


//////////////////////////////////////////////////
// Output
template <class T,unsigned int n>   ostream & operator <<(ostream &o, const Static_Vector<T,n> &v)
{
  o << "(";
  for(unsigned int i=0;i<n;i++)
    o << " " << v[i];
  o << " )";
  return o;
}
///////////////////////////////////////////////
