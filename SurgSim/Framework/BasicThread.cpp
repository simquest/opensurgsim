// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#include "SurgSim/Framework/BasicThread.h"

#include <boost/thread.hpp>
#include <boost/thread/barrier.hpp>
#include <boost/ref.hpp>
#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Framework/Clock.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/Runtime.h"

namespace SurgSim
{
namespace Framework
{

BasicThread::BasicThread(const std::string& name) :
	m_logger(Logger::getLogger(name)),
	m_name(name),
	m_period(1.0 / 30),
	m_isIdle(false),
	m_isInitialized(false),
	m_isRunning(false),
	m_stopExecution(false),
	m_isSynchronous(false)
{
	// The maximum number of frames in the timer is set to 1,000,000
	// + If the timer is reset every second, that is enough frame to measure real rates up to 1MHz
	// + If the timer is reset every minute, that is enough frame to measure real rates up to 16.66KHz
	// + If the timer is reset every hour, that is enough frame to measure real rates up to 277.77Hz
	m_timer.setMaxNumberOfFrames(1000000);
}

#ifdef _MSC_VER
BasicThread::~BasicThread() throw(...) // Visual Studio does not support noexcept. The throw(...) is optional.
#else
BasicThread::~BasicThread() noexcept(false)  /// C++11 introduced noexcept
#endif
{
	// Still need to stop thread to get a clean exit
	if (m_isRunning || m_thisThread.joinable())
	{
		SURGSIM_FAILURE() <<
			"A BasicThread instance destructor was called while the thread was still running or " <<
			"in the process of being stopped, this is currently not supported. If this was intentional " <<
			"call stop() before destruction of the thread. If this is unintentional, make sure to prevent " <<
			"the destructor from being called.";
	}
}

bool BasicThread::isInitialized()
{
	return m_isInitialized;
}

bool BasicThread::isRunning() const
{
	return m_isRunning;
}

bool BasicThread::initialize()
{
	m_isInitialized = doInitialize();
	return m_isInitialized;
}


bool BasicThread::startUp()
{
	return doStartUp();
}

void BasicThread::start(std::shared_ptr<Barrier> startupBarrier, bool isSynchronized)
{
	boost::unique_lock<boost::mutex> lock(m_mutexStartStop);

	m_startupBarrier = startupBarrier;
	m_stopExecution = false;
	m_isRunning = false;
	m_isSynchronous = isSynchronized;

	// Start the thread with a reference to this
	// prevents making a copy
	m_isRunning = true;
	m_thisThread = boost::thread(boost::ref(*this));
}

boost::thread& BasicThread::getThread()
{
	return m_thisThread;
}

void BasicThread::operator()()
{
	bool success = executeInitialization();
	if (! success)
	{
		m_isRunning = false;
		return;
	}


	size_t numUpdates = 0;
	boost::chrono::duration<double> totalFrameTime(0.0);
	boost::chrono::duration<double> sleepTime(0.0);
	boost::chrono::duration<double> totalSleepTime(0.0);
	Clock::time_point start;

	m_timer.start();
	while (m_isRunning && !m_stopExecution)
	{
		start = Clock::now();
		if (! m_isSynchronous)
		{
			if (!m_isIdle)
			{
				m_timer.beginFrame();
				m_isRunning = doUpdate(m_period.count());
				m_timer.endFrame();
			}

			// Check for frameTime being > desired update period report error, adjust ...
			sleepTime = m_period - (Clock::now() - start);
			if (sleepTime.count() > 0.0)
			{
				totalSleepTime += sleepTime;
				SurgSim::Framework::sleep_until(start + m_period);
			}
		}
		else
		{
			// HS-2014-feb-21 This is not thread safe, if setSynchronous(false) is called while the thread is in the
			// _not_ in the wait state, the thread will exit without having issued a wait, this will cause the
			// all the threads that are waiting to indefinitely wait as there is one less thread on the barrier
			// #threadsafety
			bool success = waitForBarrier(true);
			totalSleepTime += Clock::now() - start;

			if (success && !m_isIdle)
			{
				m_timer.beginFrame();
				m_isRunning = doUpdate(m_period.count());
				m_timer.endFrame();
			}
			if (! success || !m_isRunning)
			{
				m_isRunning = false;
				m_isSynchronous = false;
			}
		}
		totalFrameTime += Clock::now() - start;
		numUpdates++;

		if (m_logger->getThreshold() <= SURGSIM_LOG_LEVEL(INFO))
		{
			if (totalFrameTime.count() > 5.0)
			{
				SURGSIM_LOG_INFO(m_logger) << std::setprecision(4)
					<< "Rate: " << numUpdates / totalFrameTime.count() << "Hz / "
					<<  1.0 / m_period.count() << "Hz, "
					<< "Average doUpdate: " << (totalFrameTime.count() - totalSleepTime.count()) / numUpdates << "s, "
					<< "Sleep: " << 100.0 * totalSleepTime.count() / totalFrameTime.count() << "%";
				totalFrameTime = boost::chrono::duration<double>::zero();
				totalSleepTime = boost::chrono::duration<double>::zero();
				numUpdates = 0;
			}
		}
	}

	doBeforeStop();

	m_isRunning = false;
	m_stopExecution = false;
}

void BasicThread::stop()
{
	boost::unique_lock<boost::mutex> lock(m_mutexStartStop);

	m_stopExecution = true;

	if (! m_isSynchronous)
	{
		if (! m_thisThread.joinable())
		{
			SURGSIM_LOG_INFO(m_logger) << "Thread is detached, cannot wait for it to stop.";
		}
		else
		{
			m_thisThread.join();
		}
	}
	else
	{
		SURGSIM_LOG_INFO(m_logger) << "Thread is in synchronouse mode, stop with a barrier->wait(false).";
	}
}

void BasicThread::setIdle(bool isIdle)
{
	m_isIdle = isIdle;
}

bool BasicThread::isIdle()
{
	return m_isIdle;
}

std::string BasicThread::getName() const
{
	return m_name;
}

bool BasicThread::executeInitialization()
{
	bool success = true;

	success = initialize();
	SURGSIM_ASSERT(success) << "Initialization has failed for thread " << getName();
	SURGSIM_LOG_INFO(m_logger) << "Initialization has succeeded for thread";
	// Waits for all the threads to init and then proceeds
	// If one of the other thread asserts and ends this does not matter
	// as the process will be taken down
	success = waitForBarrier(success);

	if (!success)
	{
		return success;
	}

	success = startUp();

	SURGSIM_ASSERT(success) << "Startup has failed for thread " << getName();
	SURGSIM_LOG_INFO(m_logger) << "Startup has succeeded for thread " << getName();

	// Waits for all the threads to startup and then proceeds
	success = waitForBarrier(success);

	return success;
}

bool BasicThread::waitForBarrier(bool success)
{
	if (m_startupBarrier != nullptr)
	{
		success = m_startupBarrier->wait(success);
	}
	return success;
}

bool BasicThread::setSynchronous(bool val)
{
	if (m_startupBarrier != nullptr)
	{
		m_isSynchronous = val;
	}
	return m_isSynchronous;
}

bool BasicThread::isSynchronous()
{
	return m_isSynchronous;
}

double BasicThread::getCpuTime() const
{
	return m_timer.getCumulativeTime();
}

size_t BasicThread::getUpdateCount() const
{
	return m_timer.getCurrentNumberOfFrames();
}

void BasicThread::resetCpuTimeAndUpdateCount()
{
	m_timer.start();
}

bool BasicThread::doUpdate(double dt)
{
	return true;
}

void SurgSim::Framework::BasicThread::doBeforeStop()
{
}

}; // namespace Framework
}; // namespace SurgSim



