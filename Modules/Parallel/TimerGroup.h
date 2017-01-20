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

#ifndef SURGSIM_FRAMEWORK_TIMERGROUP_H
#define SURGSIM_FRAMEWORK_TIMERGROUP_H

#include "SurgSim/Framework/Timer.h"

#include <string>
#include <iostream>
#include <vector>

namespace SurgSim
{
namespace Framework
{

class TimerGroup
{
public:
	TimerGroup(const std::string& groupName, std::vector<std::string> names);

	size_t getCount() const;

	const std::string& getName(size_t index) const;

	const std::string& getGroupName() const;

	double getTotalTime() const;

	inline SurgSim::Framework::Timer& operator [](size_t index)
	{
		return m_timers[index];
	}

	inline const SurgSim::Framework::Timer& operator[](size_t index) const
	{
		return m_timers[index];
	}

	void reset(size_t frameCount);

	std::string m_groupName;
	std::vector<std::string> m_names;
	std::vector<SurgSim::Framework::Timer> m_timers;
};


}
}


std::ostream& operator<<(std::ostream& os, const SurgSim::Framework::TimerGroup& timers);

#endif

