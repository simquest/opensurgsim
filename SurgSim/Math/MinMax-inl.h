// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest Solutions Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SURGSIM_MATH_MINMAX_INL_H
#define SURGSIM_MATH_MINMAX_INL_H

#include "SurgSim/Framework/Assert.h"

namespace SurgSim
{
namespace Math
{

template <class T>
void minMax(const T& a1, const T& a2, T* minVal, T* maxVal)
{
	if (a1 < a2)
	{
		*minVal = a1;
		*maxVal = a2;
	}
	else
	{
		*minVal = a2;
		*maxVal = a1;
	}
}

template <class T>
void minMax(const T& a1, const T& a2, const T& a3, T* minVal, T* maxVal)
{
	T min = a1;
	T max = a1;
	if (a2 < min)
	{
		min = a2;
	}
	else if (a2 > max)
	{
		max = a2;
	}
	if (a3 < min)
	{
		min = a3;
	}
	else if (a3 > max)
	{
		max = a3;
	}

	*minVal = min;
	*maxVal = max;
}

template <class T>
void minMax(const T& a1, const T& a2, const T& a3, const T& a4, T* minVal, T* maxVal)
{
	T min = a1;
	T max = a1;
	if (a2 < min)
	{
		min = a2;
	}
	else if (a2 > max)
	{
		max = a2;
	}
	if (a3 < min)
	{
		min = a3;
	}
	else if (a3 > max)
	{
		max = a3;
	}
	if (a4 < min)
	{
		min = a4;
	}
	else if (a4 > max)
	{
		max = a4;
	}

	*minVal = min;
	*maxVal = max;
}

template <class T>
void minMax(const T& a1, const T& a2, const T& a3, const T& a4, const T& a5, T* minVal, T* maxVal)
{
	T min = a1;
	T max = a1;
	if (a2 < min)
	{
		min = a2;
	}
	else if (a2 > max)
	{
		max = a2;
	}
	if (a3 < min)
	{
		min = a3;
	}
	else if (a3 > max)
	{
		max = a3;
	}
	if (a4 < min)
	{
		min = a4;
	}
	else if (a4 > max)
	{
		max = a4;
	}
	if (a5 < min)
	{
		min = a5;
	}
	else if (a5 > max)
	{
		max = a5;
	}

	*minVal = min;
	*maxVal = max;
}

template <class T>
void minMax(const T* values, int numValues, T* minVal, T* maxVal)
{
	SURGSIM_ASSERT(numValues > 0) << "MinMax was called with <=0 values; the result is indeterminate";
	T min = values[0];
	T max = values[0];
	for (int i = 1; i < numValues; ++i)
	{
		if (values[i] < min)
		{
			min = values[i];
		}
		else if (values[i] > max)
		{
			max = values[i];
		}
	}
	*minVal = min;
	*maxVal = max;
}

};
};

#endif // SURGSIM_MATH_MINMAX_INL_H

