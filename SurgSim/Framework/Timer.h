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

#ifndef SURGSIM_FRAMEWORK_TIMER_H
#define SURGSIM_FRAMEWORK_TIMER_H

#include <boost/chrono.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <deque>

namespace SurgSim
{
namespace Framework
{

/// Timer class, measures execution times.  Multiple times can be stored as "frames" to provide an average rate or
/// period.
class Timer
{
public:
	/// Instantiate a TimerClock and start a timing run.
	Timer();

	/// Begin a timing run by clearing out any stored frames and beginning a frame.
	void start();

	/// Begin a frame (storing the current time).
	void beginFrame();

	/// End this frame by storing the duration since the current frame was begun.
	/// \note \c endFrame does not start a new frame, call \c beginFrame to do so.
	/// \sa Timer::markFrame
	void endFrame();

	/// End the current frame and begin a new frame.
	void markFrame();

	/// Return the sum of the durations over all the stored frames.
	/// \return Sum of stored frame durations in seconds.
	double getCumulativeTime() const;

	/// Return the average duration across all stored frames.  Asserts if there are no frames.
	/// \return Average period in seconds.
	double getAverageFramePeriod() const;

	/// Return the inverse of the average duration across all stored frames.  Asserts if there are no frames.
	/// \return The average frequency in Hz.
	double getAverageFrameRate() const;

	/// Return the duration of the most-recent frame (time between last \c endFrame and the previous \c start,
	/// \c beginFrame, or \c endFrame ).  Asserts if there are no frames.
	/// \return Most-recent period in seconds.
	double getLastFramePeriod() const;

	/// Return the inverse of the duration of the most-recent frame.  Asserts if there are no frames.
	/// \return Most-recent frequency in Hz.
	double getLastFrameRate() const;

	/// Set the maximum number of frames to store.
	void setMaxNumberOfFrames(size_t numberOfFrames);

	/// \return The maximum number of frames to store.
	size_t getMaxNumberOfFrames();

	/// \return Number of frames currently stored (not the maximum number of frames).
	size_t getCurrentNumberOfFrames() const;

	/// \return Number of times the clock returned an error code since \c start.  If this is non-zero, the frame
	/// durations may be incorrect.
	size_t getNumberOfClockFails() const;

	/// \return The maximum duration across all the stored frames.  Asserts if there are no frames.
	double getMaxFramePeriod() const;

	/// \return The minimum duration across all the stored frames.  Asserts if there are no frames.
	double getMinFramePeriod() const;

private:
	/// The Clock used by the Timer class.
	typedef boost::chrono::steady_clock TimerClock;

	/// Durations used by the Timer class.
	typedef boost::chrono::duration<double> TimerDuration;

	/// Time points used by the Timer class.
	typedef boost::chrono::time_point<TimerClock, TimerDuration> TimerTimePoint;

	/// Get the current time.  Checks for any error code from the clock.
	/// \return Current time.
	TimerTimePoint now();

	/// The clock used to get the time.
	static const TimerClock m_clock;

	/// The time at last \c start, \c beginFrame, or \c markFrame.
	TimerTimePoint m_lastTime;

	/// Maximum number of frames to store.
	size_t m_maxNumberOfFrames;

	/// Durations of the frames, i.e., the "stored frames".
	std::deque<TimerDuration> m_frameDurations;

	/// Mutex to access the data structure m_frameDurations safely
	mutable boost::shared_mutex m_sharedMutex;

	/// Number of clock errors since last \c start.
	size_t m_clockFails;
};

} // Framework
} // SurgSim

#endif
