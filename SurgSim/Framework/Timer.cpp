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

#include <numeric>

#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Framework/Timer.h"

namespace SurgSim {
namespace Framework {

Timer::Timer() :
	m_maxNumberOfFrames(100), m_clockFails(0)
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
	if (m_frameDurations.size() > m_maxNumberOfFrames)
	{
		m_frameDurations.pop_front();
	}
}

void Timer::markFrame()
{
	endFrame();
	beginFrame();
}

double Timer::getCumulativeTime() const
{
	TimerDuration cumulativeTime = std::accumulate(std::begin(m_frameDurations), std::end(m_frameDurations),
		TimerDuration());
	return cumulativeTime.count();
}

double Timer::getAverageFramePeriod() const
{
	SURGSIM_ASSERT(m_frameDurations.size() > 0) <<
		"Attempted to access the frames for a Timer with no frames.";
	return getCumulativeTime() / m_frameDurations.size();
}

double Timer::getAverageFrameRate() const
{
	SURGSIM_ASSERT(m_frameDurations.size() > 0) <<
		"Attempted to access the frames for a Timer with no frames.";
	return 1.0 / getAverageFramePeriod();
}

double Timer::getLastFramePeriod() const
{
	SURGSIM_ASSERT(m_frameDurations.size() > 0) <<
		"Attempted to access the last frame period for a Timer with no frames.";
	return m_frameDurations.back().count();
}

double Timer::getLastFrameRate() const
{
	return 1.0 / getLastFramePeriod();
}

void Timer::setMaxNumberOfFrames(size_t maxNumberOfFrames)
{
	m_maxNumberOfFrames = (maxNumberOfFrames > 0) ? maxNumberOfFrames : 1;
	if (m_frameDurations.size() > m_maxNumberOfFrames)
	{
		m_frameDurations.erase(std::begin(m_frameDurations),
							std::begin(m_frameDurations) + m_frameDurations.size() - m_maxNumberOfFrames);
	}
}

size_t Timer::getCurrentNumberOfFrames() const
{
	return m_frameDurations.size();
}

size_t Timer::getNumberOfClockFails() const
{
	return m_clockFails;
}

Timer::TimerTimePoint Timer::now()
{
	boost::system::error_code ec;
	TimerTimePoint currentTime = m_clock.now(ec);
	if (ec.value() != 0)
	{
		++ m_clockFails;
	}
	return currentTime;
}

double Timer::getMaxFramePeriod() const
{
	SURGSIM_ASSERT(m_frameDurations.size() > 0) <<
		"Attempted to access the maximum frame period for a Timer with no frames.";
	return std::max_element(m_frameDurations.cbegin(), m_frameDurations.cend())->count();
}

double Timer::getMinFramePeriod() const
{
	SURGSIM_ASSERT(m_frameDurations.size() > 0) <<
		"Attempted to access the maximum frame period for a Timer with no frames.";
	return std::min_element(m_frameDurations.cbegin(), m_frameDurations.cend())->count();
}

}; // namespace Framework
}; // namespace SurgSim
