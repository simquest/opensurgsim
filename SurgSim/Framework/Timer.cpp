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

#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Framework/Timer.h"

namespace SurgSim {
namespace Framework {

Timer::Timer() :
	m_stopped(true), m_numberOfFrames(100), m_clockFails(0)
{
	start();
}

void Timer::start()
{
	m_frameDurations.clear();
	m_clockFails = 0;
	beginFrame();
}

void Timer::beginFrame()
{
	m_lastTime = now();
}

void Timer::endFrame()
{
	TimerTimePoint currentTime = now();
	m_frameDurations.push_back(currentTime - m_lastTime);
	if (m_frameDurations.size() > m_numberOfFrames)
	{
		m_frameDurations.pop_front();
	}
	m_lastTime = currentTime;
}

double Timer::getAverageFramePeriod() const
{
	SURGSIM_ASSERT(m_frameDurations.size() > 0) << "Attempted to access the last frame period for a Timer with no frames.\n";
	TimerDuration cumulativeTime;
	for (auto it = m_frameDurations.begin(); it != m_frameDurations.end(); ++it)
	{
		cumulativeTime += *it;
	}
	return cumulativeTime.count() / m_frameDurations.size();
}

double Timer::getAverageFrameRate() const
{
	return 1.0 / getAverageFramePeriod();
}

double Timer::getLastFramePeriod() const
{
	SURGSIM_ASSERT(m_frameDurations.size() > 0) << "Attempted to access the last frame period for a Timer with no frames.\n";
	return m_frameDurations.back().count();
}

double Timer::getLastFrameRate() const
{
	return 1.0 / getLastFramePeriod();
}

void Timer::setNumberOfFrames(size_t numberOfFrames)
{
	m_numberOfFrames = (numberOfFrames > 0) ? numberOfFrames : 1;
	while (m_frameDurations.size() > m_numberOfFrames)
	{
		m_frameDurations.pop_front();
	}
}

size_t Timer::getCurrentNumberOfFrames() const
{
	return m_frameDurations.size();
}

int Timer::getNumberOfClockFails() const
{
	return m_clockFails;
}

Timer::TimerTimePoint Timer::now()
{
	boost::system::error_code ec;
	TimerTimePoint currentTime = m_clock.now(ec);
	if (ec.value() != 0)
	{
		int failsThisCall = 0;
		while (ec.value() != 0)
		{
			SURGSIM_ASSERT(++failsThisCall < 4) << "A Timer's clock failed four consecutive calls.";
			currentTime = m_clock.now(ec);
		}
		m_clockFails += failsThisCall;
	}
	return currentTime;
}

}; // namespace Framework
}; // namespace SurgSim
