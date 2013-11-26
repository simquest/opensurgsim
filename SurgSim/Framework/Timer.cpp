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

SurgSim::Framework::Timer::Timer() :
	m_stopped(true), m_numberOfFrames(100), m_clockFails(0)
{
	start();
}

void SurgSim::Framework::Timer::start()
{
	m_stopped = false;
	m_frames.clear();
	m_clockFails = 0;
	m_lastTime = now();
}

void SurgSim::Framework::Timer::frameStep()
{
	SURGSIM_ASSERT(!m_stopped) << "Tried to step the frame of a Timer that is stopped.";
	TimerTimePoint currentTime = now();
	m_frames.push_back(currentTime - m_lastTime);
	if (m_frames.size() > m_numberOfFrames)
	{
		m_frames.pop_front();
	}
	m_lastTime = currentTime;
}

void SurgSim::Framework::Timer::stop()
{
	m_stopped = true;
}

double SurgSim::Framework::Timer::getAverageFramePeriod() const
{
	SURGSIM_ASSERT(m_frames.size() > 0) << "Attempted to access the last frame period for a Timer with no frames.\n";
	TimerDuration cumulativeTime;
	for (auto it = m_frames.begin(); it != m_frames.end(); ++it)
	{
		cumulativeTime += *it;
	}
	return cumulativeTime.count() / m_frames.size();
}

double SurgSim::Framework::Timer::getAverageFrameRate() const
{
	return 1.0 / getAverageFramePeriod();
}

double SurgSim::Framework::Timer::getLastFramePeriod() const
{
	SURGSIM_ASSERT(m_frames.size() > 0) << "Attempted to access the last frame period for a Timer with no frames.\n";
	return m_frames.back().count();
}

double SurgSim::Framework::Timer::getLastFrameRate() const
{
	return 1.0 / getLastFramePeriod();
}

void SurgSim::Framework::Timer::setNumberOfFrames(size_t numberOfFrames)
{
	m_numberOfFrames = (numberOfFrames > 0) ? numberOfFrames : 1;
	while (m_frames.size() > m_numberOfFrames)
	{
		m_frames.pop_front();
	}
}

size_t SurgSim::Framework::Timer::getCurrentNumberOfFrames() const
{
	return m_frames.size();
}

int SurgSim::Framework::Timer::getNumberOfClockFails() const
{
	return m_clockFails;
}

SurgSim::Framework::Timer::TimerTimePoint SurgSim::Framework::Timer::now()
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
