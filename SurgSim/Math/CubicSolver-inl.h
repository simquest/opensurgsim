// This file is a part of the OpenSurgSim project.
// Copyright 2016, SimQuest Solutions Inc.
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

#ifndef SURGSIM_MATH_CUBICSOLVER_INL_H
#define SURGSIM_MATH_CUBICSOLVER_INL_H

#include <boost/math/tools/roots.hpp>
#include <limits>
#include <vector>

#include "SurgSim/Math/IntervalArithmetic.h"
#include "SurgSim/Math/PolynomialRoots.h"

using boost::math::tools::bisect;


namespace SurgSim
{
namespace Math
{

template <class T>
int findRootsInRange01(const Polynomial<T, 3>& p, std::array<T, 3>* roots)
{
	int numberOfRoots = 0;
	boost::math::tools::eps_tolerance<T> tolerance(std::numeric_limits<T>::digits - 3);

	// Is degenerate?
	if (isNearZero(p.getCoefficient(3), std::numeric_limits<T>::epsilon()))
	{
		Polynomial<T, 2> quadratic(p.getCoefficient(0), p.getCoefficient(1), p.getCoefficient(2));
		PolynomialRoots<T, 2> quadraticRoots(quadratic, std::numeric_limits<T>::epsilon());

		for (int i = 0; i < quadraticRoots.getNumRoots(); ++i)
		{
			if (quadraticRoots[i] >= 0.0 && quadraticRoots[i] <= 1.0)
			{
				(*roots)[numberOfRoots++] = quadraticRoots[i];
			}
		}

		return numberOfRoots;
	}

	// General case, let's analyze the derivative...first we calculate the discriminant:
	T discriminant = p.derivative().discriminant();

	if (discriminant < 0.0 || isNearZero(discriminant, std::numeric_limits<T>::epsilon()))
	{
		// If discriminant = 0, P'(-b/3a) = 0, and P'(x) has the same sign anywhere else
		// Therefore P is monotonic (ascending or descending) and has 1 unique root over ]-Inf +Inf[
		//
		// If discriminant < 0, P' is never null and has always the same sign, the sign of P'(0) (i.e. sign(c))
		// Therefore P is monotonic (stricly ascending or descending) and has 1 unique root over ]-Inf +Inf[

		T p0 = p.getCoefficient(0); // p.evaluate(static_cast<T>(0));
		if (isNearZero(p0, std::numeric_limits<T>::epsilon()))
		{
			(*roots)[0] = 0.0;
			return 1;
		}

		T p1 = p.evaluate(static_cast<T>(1));
		if (isNearZero(p1, std::numeric_limits<T>::epsilon()))
		{
			(*roots)[0] = static_cast<T>(1);
			return 1;
		}

		// P0 and P1 cannot be zero at this stage, so they both have a clear sign
		if (p0 * p1 < 0)
		{
			auto bracket = bisect(p, static_cast<T>(0), static_cast<T>(1), tolerance);
			(*roots)[0] = (bracket.first + bracket.second) * 0.5;
			return 1;
		}
	}
	else
	{
		// If discriminant > 0, P' has 2 roots {x0, x1} which define 3 separate intervals to study:
		// ]-Inf x0[, [x0 x1] and ]x1 +Inf[
		T tmp = std::sqrt(discriminant / 4.0);
		T scale = static_cast<T>(1) / (static_cast<T>(3) * p.getCoefficient(3));
		T x0 = (-p.getCoefficient(2) - tmp) * scale;
		T x1 = (-p.getCoefficient(2) + tmp) * scale;
		if (x0 > x1)
		{
			std::swap(x0, x1);
		}
		// Note that x0 < x1 and x0 != x1 as delta != 0

		Interval<T> intervalx0x1(x0, x1);
		Interval<T> interval01(static_cast<T>(0), static_cast<T>(1));

		if (!intervalx0x1.overlapsWith(interval01))
		{
			// If there is no overlap, the interval [0..1] is isolated on one of the monotonic interval ]-Inf x0[
			// ]x1 +Inf[, therefore it contains at most 1 root

			T p0 = p.getCoefficient(0); // p.evaluate(static_cast<T>(0));
			if (isNearZero(p0, std::numeric_limits<T>::epsilon()))
			{
				(*roots)[0] = 0.0;
				return 1;
			}

			T p1 = p.evaluate(static_cast<T>(1));
			if (isNearZero(p1, std::numeric_limits<T>::epsilon()))
			{
				(*roots)[0] = static_cast<T>(1);
				return 1;
			}

			// P0 and P1 cannot be zero at this stage, so they both have a clear sign
			if (p0 * p1 < 0)
			{
				auto bracket = bisect(p, static_cast<T>(0), static_cast<T>(1), tolerance);
				(*roots)[0] = (bracket.first + bracket.second) * 0.5;
				return 1;
			}
		}
		else
		{
			// Build the monotonic intervals partitioning [0..1] to be analyzed one by one
			std::vector<Interval<T>> intervalsPartitioning01;

			T lastValue = static_cast<T>(0);
			if (x0 > static_cast<T>(0))
			{
				intervalsPartitioning01.emplace_back(lastValue, x0);
				lastValue = x0;
			}
			if (x1 < static_cast<T>(1))
			{
				intervalsPartitioning01.emplace_back(lastValue, x1);
				lastValue = x1;
			}
			intervalsPartitioning01.push_back(Interval<T>(lastValue, static_cast<T>(1)));

			for (auto interval : intervalsPartitioning01)
			{
				// On each interval, only 1 root can be found
				T pMin = p.evaluate(interval.getMin());
				if (isNearZero(pMin, std::numeric_limits<T>::epsilon()))
				{
					(*roots)[numberOfRoots++] = interval.getMin();
				}
				else
				{
					T pMax = p.evaluate(interval.getMax());
					if (isNearZero(pMax, std::numeric_limits<T>::epsilon()))
					{
						(*roots)[numberOfRoots++] = interval.getMax();
					}
					else if (pMin * pMax < 0)
					{
						auto bracket = bisect(p, interval.getMin(), interval.getMax(), tolerance);
						(*roots)[numberOfRoots++] = (bracket.first + bracket.second) * 0.5;
					}
				}
			}
		}
	}

	return numberOfRoots;
}

}; // Math
}; // SurgSim

#endif // SURGSIM_MATH_CUBICSOLVER_INL_H
