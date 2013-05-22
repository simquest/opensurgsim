#ifndef __debug_h__
#define __debug_h__

#include <stdio.h>
#include <iostream>
using namespace std;

#ifdef _DEBUG

  #ifdef __FUNCSIG__
    #define SqAssert(a) { if(!(a))                                                  \
      {                                                                           \
        cerr << "Assert failed in file " << __FILE__ << ", line ";                \
        cerr << __LINE__ << "." << endl;                                          \
        cerr << "Function : " << __FUNCSIG__ << endl;                             \
      }                                                                           \
    }
  #elif defined(__PRETTY_FUNCTION__)
    #define SqAssert(a) { if(!(a))                                                  \
      {                                                                           \
        cerr << "Assert failed in file " << __FILE__ << ", line ";                \
        cerr << __LINE__ << "." << endl;                                          \
        cerr << "Function : " << __PRETTY_FUNCTION__ << endl;                     \
      }                                                                           \
    }
  #else
    #define SqAssert(a) { if(!(a))                                                  \
      {                                                                           \
        cerr << "Assert failed in file " << __FILE__ << ", line ";                \
        cerr << __LINE__ << "." << endl;                                          \
      }                                                                           \
    }
  #endif
  #define assert_forced(a) SqAssert(a)
  #ifndef CONSTELLATION_DEPENDENCIES
    //#define assert_printf(a,...) { assert(a) if(!(a)) fprintf(stderr,__VA_ARGS__); }
    #define assert_printf(a,...)
    //#define assert_forced_printf(a,...) { assert_forced(a) if(!(a)) fprintf(stderr,__VA_ARGS__); }
    #define assert_forced_printf(a,...)
  #endif // CONSTELLATION_DEPENDENCIES

#else

  #define SqAssert(a)
  #ifdef __FUNCSIG__
    #define assert_forced(a) { if(!(a))                                           \
      {                                                                           \
        cerr << "Assert failed in file " << __FILE__ << ", line ";                \
        cerr << __LINE__ << "." << endl;                                          \
        cerr << "Function : " << __FUNCSIG__ << endl;                             \
      }                                                                           \
    }
  #elif defined(__PRETTY_FUNCTION__)
    #define assert_forced(a) { if(!(a))                                           \
      {                                                                           \
        cerr << "Assert failed in file " << __FILE__ << ", line ";                \
        cerr << __LINE__ << "." << endl;                                          \
        cerr << "Function : " << __PRETTY_FUNCTION__ << endl;                     \
      }                                                                           \
    }
  #else
    #define assert_forced(a) { if(!(a))                                           \
      {                                                                           \
        cerr << "Assert failed in file " << __FILE__ << ", line ";                \
        cerr << __LINE__ << "." << endl;                                          \
      }                                                                           \
    }
  #endif
  #ifndef CONSTELLATION_DEPENDENCIES
    //#define assert_printf(a,string,...)
    #define assert_printf
    //#define assert_forced_printf(a,string,...) { assert_forced(a) if(!(a)) fprintf(stderr,string,__VA_ARGS__); }
    #define assert_forced_printf
  #endif // CONSTELLATION_DEPENDENCIES

#endif

#endif
