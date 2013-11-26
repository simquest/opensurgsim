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
	m_stopped(true), m_number_of_frames(100), m_clock_fails(0)
{
	start();
}

void SurgSim::Framework::Timer::start()
{
	m_stopped = false;
	m_frames.clear();
	m_clock_fails = 0;
	m_last_time = now();
}

void SurgSim::Framework::Timer::frameStep()
{
	SURGSIM_ASSERT(!m_stopped) << "Tried to step the frame of a Timer that is stopped.";
	TimerTimePoint current_time = now();
	m_frames.push_back(current_time - m_last_time);
	if (m_frames.size() > m_number_of_frames)
	{
		m_frames.pop_front();
	}
	m_last_time = current_time;
}

void SurgSim::Framework::Timer::stop()
{
	m_stopped = true;
}

double SurgSim::Framework::Timer::getAverageFramePeriod() const
{
	SURGSIM_ASSERT(m_frames.size() > 0) << "Attempted to access the last frame period for a Timer with no frames.\n";
	TimerDuration cumulative_time;
	for (auto it = m_frames.begin(); it != m_frames.end(); ++it)
	{
		cumulative_time += *it;
	}
	return cumulative_time.count() / m_frames.size();
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

void SurgSim::Framework::Timer::setNumberOfFrames(size_t number_of_frames)
{
	m_number_of_frames = (number_of_frames > 0 ? number_of_frames : 1);
	while (m_frames.size() > m_number_of_frames)
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
	return m_clock_fails;
}

SurgSim::Framework::Timer::TimerTimePoint SurgSim::Framework::Timer::now()
{
	boost::system::error_code ec;
	TimerTimePoint current_time = m_clock.now(ec);
	if (ec.value() != 0)
	{
		int fails_this_call = 0;
		while (ec.value() != 0)
		{
			SURGSIM_ASSERT(++fails_this_call < 4) << "A Timer's clock failed four consecutive calls.";
			current_time = m_clock.now(ec);
		}
		m_clock_fails += fails_this_call;
	}
	return current_time;
}
