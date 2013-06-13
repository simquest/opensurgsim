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

#include "SurgSim/Framework/BasicThread.h"

#include <boost/thread.hpp>
#include <boost/thread/barrier.hpp>
#include <boost/chrono.hpp>
#include <boost/ref.hpp>
#include <SurgSim/Framework/Assert.h>
#include <SurgSim/Framework/Log.h>
#include <SurgSim/Framework/Runtime.h>

namespace SurgSim
{
namespace Framework
{

BasicThread::BasicThread(const std::string& name) :
	m_name(name),
	m_period(1.0/30),
	m_isInitialized(false),
	m_isRunning(false),
	m_stopExecution(false)
{
}

BasicThread::~BasicThread()
{
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

void BasicThread::start(std::shared_ptr<Barrier> startupBarrier)
{
	m_startupBarrier = startupBarrier;
	// Start the thread with a reference to this
	// prevents making a copy
	m_thisThread = boost::thread(boost::ref(*this));
}

boost::thread& BasicThread::getThread()
{
	return m_thisThread;
}

void BasicThread::operator()()
{

	m_stopExecution = false;
	bool success = executeInitialization();
	if (! success) return;

	boost::chrono::duration<double> frameTime(0.0);
	boost::chrono::steady_clock::time_point start;

	m_isRunning = true;
	while (m_isRunning && ! m_stopExecution)
	{
		// Check for frameTime being > desired update period report error, adjust ...
		if (m_period > frameTime)
		{
			boost::this_thread::sleep_until(boost::chrono::steady_clock::now() + (m_period - frameTime));
		}
		start = boost::chrono::steady_clock::now();
		m_isRunning = doUpdate(m_period.count());
		frameTime = boost::chrono::steady_clock::now() - start;
	}

	if (m_stopExecution)
	{
		doBeforeStop();
	}

	m_isRunning = false;
	m_stopExecution = false;
}

void BasicThread::stop()
{
	m_stopExecution = true;
	if (m_thisThread.joinable())
	{
		m_thisThread.join();
	}
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
	SURGSIM_LOG_INFO(Logger::getDefaultLogger()) << "Initialization has succeeded for thread " << getName();
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
	SURGSIM_LOG_INFO(Logger::getDefaultLogger()) << "Startup has succeeded for thread " << getName();

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

void SurgSim::Framework::BasicThread::doBeforeStop()
{
}

}; // namespace Framework
}; // namespace SurgSim



