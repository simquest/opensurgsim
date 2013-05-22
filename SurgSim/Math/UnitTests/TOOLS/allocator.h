#ifndef __allocator_h__
#define __allocator_h__

#include <malloc.h>
#include <stdexcept>

template <typename T> class Allocator_malloc
{
public:
	static inline T *alloc(int nbElem)
	{
		return checkPointer(::malloc( nbElem*sizeof(T) ), nbElem);
	}
	static inline void free(T* data, int nbElem)
	{
		if (data) ::free(data);
	};
	static inline T *realloc(T *data,int nbElem,bool keepOldInfo=false,int oldNbElem=0)
	{
		if (data) return checkPointer(::realloc(static_cast<void*>(data), nbElem*sizeof(T)), nbElem);
		else      return checkPointer(::malloc(nbElem*sizeof(T)), nbElem);
	};
protected:
	static inline T* checkPointer(void* buffer, int nbElem)
	{
		if (nbElem > 0 && ! buffer)
		{
			static const std::bad_alloc nomem;
			throw nomem;
		}
		return static_cast<T*>(buffer);
	}
};

template <typename T> class Allocator_new
{
public:
  static inline T *alloc(int nbElem)
  { return new T[nbElem]; };
  static inline void free(void *data, int nbElem)
  { if(data) delete [] data; };
  static inline T *realloc(T *data,int nbElem,bool keepOldInfo=false,int oldNbElem=0)
  {
    T *newData=new T[nbElem];
    if(data && keepOldInfo){
      register unsigned int i=0,maxElem = (nbElem>oldNbElem ? oldNbElem:nbElem);
      for(i=0;i<maxElem;i++)
        newData[i]=data[i];
    }
    if(data) delete [] data;
    return newData;
  };
};


#endif
