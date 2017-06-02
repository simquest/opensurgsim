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
	static const boost::math::tools::eps_tolerance<T> tolerance(std::numeric_limits<T>::digits - 3);

	// Is degenerate?
	if (isNearZero(p.getCoefficient(3)))
	{
		Polynomial<T, 2> quadratic(p.getCoefficient(0), p.getCoefficient(1), p.getCoefficient(2));
		PolynomialRoots<T, 2> quadraticRoots(quadratic);

		for (int i = 0; i < quadraticRoots.getNumRoots(); ++i)
		{
			if (quadraticRoots[i] >= 0.0 && quadraticRoots[i] <= 1.0)
			{
				(*roots)[numberOfRoots++] = quadraticRoots[i];
			}
		}

		return numberOfRoots;
	}

	PolynomialRoots<T, 2> stationaryPoints(p.derivative());
	if (stationaryPoints.getNumRoots() < 2 ||
		!Interval<T>(0, 1).overlapsWith(Interval<T>(stationaryPoints[0], stationaryPoints[1])))
	{
		T p0 = p.getCoefficient(0); // p.evaluate(static_cast<T>(0));
		if (isNearZero(p0))
		{
			(*roots)[0] = 0.0;
			return 1;
		}

		T p1 = p.evaluate(static_cast<T>(1));
		if (isNearZero(p1))
		{
			(*roots)[0] = static_cast<T>(1);
			return 1;
		}
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
		// #performance HS-2016-feb-17 Test with boost::static_vector as this gets used by the CCD
		std::vector<Interval<T>> intervals;

		T lastValue = static_cast<T>(0);
		if (stationaryPoints[0] > static_cast<T>(0))
		{
			intervals.emplace_back(lastValue, stationaryPoints[0]);
			lastValue = stationaryPoints[0];
		}
		if (stationaryPoints[1] < static_cast<T>(1))
		{
			intervals.emplace_back(lastValue, stationaryPoints[1]);
			lastValue = stationaryPoints[1];
		}
		intervals.emplace_back(lastValue, static_cast<T>(1));

		for (auto interval : intervals)
		{
			// On each interval, only 1 root can be found
			T pMin = p.evaluate(interval.getMin());
			if (isNearZero(pMin))
			{
				(*roots)[numberOfRoots++] = interval.getMin();
			}
			else
			{
				T pMax = p.evaluate(interval.getMax());
				if (isNearZero(pMax))
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

	return numberOfRoots;
}

}; // Math
}; // SurgSim

#endif // SURGSIM_MATH_CUBICSOLVER_INL_H
