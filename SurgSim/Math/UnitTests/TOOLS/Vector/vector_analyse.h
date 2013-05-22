#ifndef __vector_analyse__
#define __vector_analyse__

//////////////////////////////////////////////////
// Useful functions to analyse arguments
template <class T> inline void _analyse_vector_args(T *data,unsigned int size,T first,va_list args)
{
  unsigned int index=1;
  data[0]=first;
  while(index<size) data[index++]=(T)va_arg(args, int );
}

template <> inline void _analyse_vector_args(double *data,unsigned int size,double first,va_list args)
{
  unsigned int index=1;
  data[0]=first;
  while(index<size) data[index++]=(double)va_arg(args, double );
}

template <> inline void _analyse_vector_args(float *data,unsigned int size,float first,va_list args)
{
  unsigned int index=1;
  data[0]=first;
  while(index<size) data[index++]=(float)va_arg(args, double );
}

template <> inline void _analyse_vector_args(long double *data,unsigned int size,long double first,va_list args)
{
  unsigned int index=1;
  data[0]=first;
  while(index<size) data[index++]=(long double)va_arg(args, double );
}

template <> inline void _analyse_vector_args(long long *data,unsigned int size,long long first,va_list args)
{
  unsigned int index=1;
  data[0]=first;
  while(index<size) data[index++]=(long long)va_arg(args, long long );
}

template <> inline void _analyse_vector_args(unsigned long long *data,unsigned int size,unsigned long long first,va_list args)
{
  unsigned int index=1;
  data[0]=first;
  while(index<size) data[index++]=(unsigned long long)va_arg(args, long long );
}

template <> inline void _analyse_vector_args(bool *data,unsigned int size,bool first,va_list args)
{
  unsigned int index=1;
  data[0]=first;
  while(index<size) data[index++]= (va_arg(args,int)?true:false);
}
//////////////////////////////////////////////////

#endif
