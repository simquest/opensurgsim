// This file is a part of the OpenSurgSim project.
// Copyright 2013-2017, SimQuest Solutions Inc.
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

#include "Modules/Parallel/TimerGroup.h"
#include <iomanip>

namespace SurgSim
{
namespace Framework
{

TimerGroup::TimerGroup(const std::string& groupName, std::vector<std::string> names) :
	m_groupName(groupName),
	m_names(names),
	m_timers(names.size())
{

}


size_t TimerGroup::getCount() const
{
	return m_timers.size();
}

const std::string& TimerGroup::getName(size_t index) const
{
	return m_names[index];
}

const std::string& TimerGroup::getGroupName() const
{
	return m_groupName;
}

double TimerGroup::getTotalTime() const
{
	double result = 0;
	for (const auto& timer : m_timers)
	{
		result += timer.getAverageFramePeriod();
	}
	return result;
}

void TimerGroup::reset(size_t frameCount)
{
	for (auto& timer : m_timers)
	{
		timer.setMaxNumberOfFrames(frameCount);
		timer.start();
	}
}

}
}

std::ostream& operator<<(std::ostream& os, const SurgSim::Framework::TimerGroup& timers)
{
	auto count = timers.getCount();

	auto totalTime = timers.getTotalTime();
	for (int i = 0; i < count; ++i)
	{
		const double period = timers[i].getAverageFramePeriod();
		os << std::fixed << std::setprecision(0)
		   << timers.getName(i) << " \taverage duration " << 1e6 * period << " us (max "
		   << 1e6 * timers[i].getMaxFramePeriod() << " us), "
		   << 100.0 * period / totalTime << "% of timers.getGroupName. \n";
	}
	return os;
}

