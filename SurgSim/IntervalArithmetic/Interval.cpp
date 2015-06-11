#include <IntervalArithmetic/IntervalArithmetic.h>

// Explicit instanciation of the method in double and float
// So the the CommonLib contains the actual code
// Otherwise, no-one is generating the code => link fail to find the symbol !

// = ( + )
template void IntervalArithmetic_add(const Interval<double> &a,const Interval<double> &b,Interval<double> &res);
// += ( + )
template void IntervalArithmetic_addadd(const Interval<double> &a,const Interval<double> &b,Interval<double> &res);
// = ( - )
template void IntervalArithmetic_sub(const Interval<double> &a,const Interval<double> &b,Interval<double> &res);
// += ( - )
template void IntervalArithmetic_addsub(const Interval<double> &a,const Interval<double> &b,Interval<double> &res);
// = ( * )
template void IntervalArithmetic_mul(const Interval<double> &a,const Interval<double> &b,Interval<double> &res);
// += ( * )
template void IntervalArithmetic_addmul(const Interval<double> &a,const Interval<double> &b,Interval<double> &res);
// -= ( * )
template void IntervalArithmetic_submul(const Interval<double> &a,const Interval<double> &b,Interval<double> &res);


// = ( + )
template void IntervalArithmetic_add(const Interval<float> &a,const Interval<float> &b,Interval<float> &res);
// += ( + )
template void IntervalArithmetic_addadd(const Interval<float> &a,const Interval<float> &b,Interval<float> &res);
// = ( - )
template void IntervalArithmetic_sub(const Interval<float> &a,const Interval<float> &b,Interval<float> &res);
// += ( - )
template void IntervalArithmetic_addsub(const Interval<float> &a,const Interval<float> &b,Interval<float> &res);
// = ( * )
template void IntervalArithmetic_mul(const Interval<float> &a,const Interval<float> &b,Interval<float> &res);
// += ( * )
template void IntervalArithmetic_addmul(const Interval<float> &a,const Interval<float> &b,Interval<float> &res);
// -= ( * )
template void IntervalArithmetic_submul(const Interval<float> &a,const Interval<float> &b,Interval<float> &res);




template void IntervalArithmetic_add(const Interval_nD<double,3> &a,const Interval_nD<double,3> &b,Interval_nD<double,3> &res);
template void IntervalArithmetic_sub(const Interval_nD<double,3> &a,const Interval_nD<double,3> &b,Interval_nD<double,3> &res);
template void IntervalArithmetic_dotProduct(const Interval_nD<double,3> &a,const Interval_nD<double,3> &b, Interval<double> &res);
template void IntervalArithmetic_crossProduct(const Interval_nD<double,3> &a,const Interval_nD<double,3> &b,Interval_nD<double,3> &res);

template void IntervalArithmetic_add(const Interval_nD<float,3> &a,const Interval_nD<float,3> &b,Interval_nD<float,3> &res);
template void IntervalArithmetic_sub(const Interval_nD<float,3> &a,const Interval_nD<float,3> &b,Interval_nD<float,3> &res);
template void IntervalArithmetic_dotProduct(const Interval_nD<float,3> &a,const Interval_nD<float,3> &b, Interval<float> &res);
template void IntervalArithmetic_crossProduct(const Interval_nD<float,3> &a,const Interval_nD<float,3> &b,Interval_nD<float,3> &res);
