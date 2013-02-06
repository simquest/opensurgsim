// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest LLC.
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

#include "BasicThread.h"

#include <boost/thread.hpp>
#include <boost/thread/barrier.hpp>
#include <boost/chrono.hpp>
#include <boost/ref.hpp>
#include <SurgSim/Framework/Assert.h>
#include <SurgSim/Framework/Log.h>

SurgSim::Framework::BasicThread::BasicThread(std::string name) :
	m_isInitialised(false),
	m_isRunning(false),
	m_rate(1.0/30),
	m_stopExecution(false),
	m_name(name)
{

}

SurgSim::Framework::BasicThread::~BasicThread()
{

}

bool SurgSim::Framework::BasicThread::isInitialised()
{
	return m_isInitialised;
}

bool SurgSim::Framework::BasicThread::isRunning() const
{
	return m_isRunning;
}

bool SurgSim::Framework::BasicThread::doInit()
{
	m_isInitialised = init();
	return m_isInitialised;
}


bool SurgSim::Framework::BasicThread::doStartup()
{
	return startup();
}

void SurgSim::Framework::BasicThread::doRun(std::shared_ptr<Barrier> startupBarrier)
{
	m_startupBarrier = startupBarrier;
	// Start the thread with a reference to this
	// prevents making a copy
	m_thisThread = boost::thread(boost::ref(*this));
}

boost::thread& SurgSim::Framework::BasicThread::getThread()
{
	return m_thisThread;
}

void SurgSim::Framework::BasicThread::operator()()
{
	double dt = 0.0;
	bool success = true;
	m_stopExecution = false;

	success = doInit();
	SURGSIM_ASSERT(success) << "Initialisation has failed for thread " << getName();
	SURGSIM_LOG_INFO(Logger::getDefaultLogger()) << "Initialisation has succeeded for thread " << getName();
	// Waits for all the threads to init and then proceeds
	// If one of the other thread asserts and ends this does not matter
	// as the process will be taken down
	if (m_startupBarrier != nullptr)
	{
		success = m_startupBarrier->wait(success);
		if (!success)
		{
			return;
		}
	}

	success = doStartup();

	SURGSIM_ASSERT(success) << "Startup has failed for thread " << getName();
	SURGSIM_LOG_INFO(Logger::getDefaultLogger()) << "Startup has succeeded for thread " << getName();

	// Waits for all the threads to startup and then proceeds
	if (m_startupBarrier != nullptr)
	{
		success = m_startupBarrier->wait(success);
		if (! success)
		{
			return;
		}
	}

	// Wait again, this gives the runtime a chance to add the scene elements
	if (m_startupBarrier != nullptr)
	{
		success = m_startupBarrier->wait(success);
		if (! success)
		{
			return;
		}
	}

	boost::chrono::duration<double> frameTime(0.0);
	boost::chrono::system_clock::time_point start;

	m_isRunning = true;
	while (m_isRunning && ! m_stopExecution)
	{
		// Check for frameTime being > rate report error, adjust ...
		if (m_rate > frameTime)
		{
			boost::this_thread::sleep_for(m_rate-frameTime);
		} 
		start = boost::chrono::system_clock::now();
		m_isRunning = update(m_rate.count());
		frameTime = boost::chrono::system_clock::now() - start;
	}
	m_isRunning = false;
	m_stopExecution = false;
}

void SurgSim::Framework::BasicThread::doStop(bool waitForExit)
{
	m_stopExecution = true;
	if (waitForExit)
	{
		m_thisThread.join();
	}
}

std::string SurgSim::Framework::BasicThread::getName() const
{
	return m_name;
}



