#ifndef SQ_MIN_MAX__H
#define SQ_MIN_MAX__H

#include <SqAssert.h>

namespace SqMath
{
	template <class T>
	inline void MinMax(const T a1, const  T a2, T& min, T& max)
	{
		if (a1 < a2)
		{
			min = a1;
			max = a2;
		}
		else
		{
			min = a2;
			max = a1;
		}
	}

	template <class T>
	inline void MinMax(const T a1, const  T a2, const  T a3, T& minVal, T& maxVal)
	{
		// TO DO: check how the "else"s affect performance? --bert
		T min = a1;
		T max = a1;
		if (a2 < min)       min = a2;
		else if (a2 > max)  max = a2;
		if (a3 < min)       min = a3;
		else if (a3 > max)  max = a3;

		minVal = min;
		maxVal = max;
	}

	template <class T>
	inline void MinMax(const T a1, const T a2, const T a3, const T a4, T& minVal, T& maxVal)
	{
		// TO DO: check how the "else"s affect performance? --bert
		T min = a1;
		T max = a1;
		if (a2 < min)       min = a2;
		else if (a2 > max)  max = a2;
		if (a3 < min)       min = a3;
		else if (a3 > max)  max = a3;
		if (a4 < min)       min = a4;
		else if (a4 > max)  max = a4;

		minVal = min;
		maxVal = max;
	}

	template <class T>
	inline void MinMax(const T a1, const T a2, const T a3, const T a4, const T a5, T& minVal, T& maxVal)
	{
		// TO DO: check how the "else"s affect performance? --bert
		T min = a1;
		T max = a1;
		if (a2 < min)       min = a2;
		else if (a2 > max)  max = a2;
		if (a3 < min)       min = a3;
		else if (a3 > max)  max = a3;
		if (a4 < min)       min = a4;
		else if (a4 > max)  max = a4;
		if (a5 < min)       min = a5;
		else if (a5 > max)  max = a5;

		minVal = min;
		maxVal = max;
	}

	template <class T>
	inline void MinMax(const T* values, int numValues, T& minVal, T& maxVal)
	{
		SQ_ASSERT(numValues > 0, "MinMax was called with <=0 values; the result is indeterminate");
		T min = values[0];
		T max = values[0];
		for (int i = 1;  i < numValues;  ++i)
		{
			// TO DO: check how the "else" affects performance? --bert
			if (values[i] < min) min = values[i];
			else if (values[i] > max) max = values[i];
		}
		minVal = min;
		maxVal = max;
	}
};

#endif // SQ_MIN_MAX__H
