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

#ifndef SURGSIM_FRAMEWORK_CLOCK_H
#define SURGSIM_FRAMEWORK_CLOCK_H

#include <boost/chrono.hpp>

/// \file
/// Place for a simple wrapper around boost


namespace SurgSim
{
namespace Framework
{

/// Wraps around the actual clock we are using.
typedef boost::chrono::system_clock Clock;

template <class Clock, class Duration>
void sleep_until(const boost::chrono::time_point<Clock, Duration>& abs_time)
{
	// Some system dependant threshold, will probably need to tune this.
	boost::chrono::duration<double> threshold(0.002);

	if (abs_time - threshold > Clock::now())
	{
		boost::this_thread::sleep_until(abs_time - threshold);
	}
	while (Clock::now() < abs_time)
	{
		// This has no effect on my system, still get the requested rate,
		// but it still uses 100% of a core.
		boost::this_thread::yield();
	};
}

}; // Framework
}; // SurgSim

#endif
