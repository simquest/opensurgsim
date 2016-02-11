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

#ifndef SURGSIM_FRAMEWORK_BASICTHREAD_H
#define SURGSIM_FRAMEWORK_BASICTHREAD_H

#include <memory>
#include <string>

#include <boost/thread.hpp>
#include <boost/chrono.hpp>

#include "SurgSim/Framework/Barrier.h"
#include "SurgSim/Framework/Timer.h"

namespace SurgSim
{
namespace Framework
{

class Component;
class Runtime;

/// Basic thread implementation, tries to maintain a constant rate, supplies
/// startup an initialization, can be synchronized with other threads at startup
/// after calling doRun() a thread be be set off and doInit() and doStartup() will
/// be called in succession. If given a startup barrier the sequence will pause at
/// both steps until all other threads are done with these steps.
/// Initialization can be further customized by implementing a executeInitialization() function.
/// When a barrier was used to start up the thread it can also be used to run the thread in a
/// synchronous fashion. Use setIsSynchrous(true) to switch the thread over, after that the thread
/// will wait for the barrier to trigger before it executes another update. When running asynchronously the thread
/// cannot be stopped with the stop() call, a barrier wait with an argument of false has to be used
/// to stop the thread. The thread can be set back to asynchronous execution, one last barrier wait after
/// the switch has to be executed for the thread to come out of the wait.
class BasicThread
{
public:
	explicit BasicThread(const std::string& name = "Unknown Thread");
#ifdef _MSC_VER
	virtual ~BasicThread() throw(...);  // Visual Studio does not support noexcept. The throw(...) is optional.
#else
	virtual ~BasicThread() noexcept(false);  /// C++11 introduced noexcept
#endif

	/// Live cycle functions, public implementation.
	/// All of these have virtual partners as private functions

	/// Start the thread from the outside, this will call the private
	/// run() function that can be overridden for each implementor of this
	/// interface.
	/// \param startupBarrier is a barrier it synchronizes a group of thread that should go through their startup
	/// sequence in step.
	/// \param isSynchronous when true the thread will wait on the barrier after each call to update(dt), this
	/// 					 means that only one step will be performed at a time
	void start(std::shared_ptr<Barrier> startupBarrier = nullptr, bool isSynchronous = false);

	/// Stopping the execution, blocks until the running thread has actually stopped,
	/// \note When the thread is in synchronous mode, it needs to be stopped with a call to
	/// 	  the barrier wait function with an argument of false, of course it can always be stopped
	/// 	  by going back to asynchronous mode and then calling stop
	void stop();

	/// Set/Unset the thread in an idle state (doUpdate() called or not in the update() method)
	/// \param isIdle True to set the thread in an idle state, false otherwise
	void setIdle(bool isIdle);

	/// Query if this thread is in idle state or not
	/// \return	true if the thread is in idle state, false otherwise.
	bool isIdle();

	/// Query if this object is initialized.
	/// \return	true if initialized, false if not.
	bool isInitialized();

	/// Query if this object is running.
	/// \return	true if the threads update() function is being called
	bool isRunning() const;

	/// This is what boost::thread executes on thread creation.
	void operator()();

	/// \return the boost threading object
	boost::thread& getThread();

	/// \return the name of the thread
	std::string getName() const;

	/// Set the update rate of the thread
	/// \param val	rate in hertz (updates per second) of the thread
	void setRate(double val)
	{
		m_period = boost::chrono::duration<double>(1.0 / val);
	}

	/// Sets the thread to synchronized execution in concert with the startup
	/// barrier, the startup barrier has to exist for this call to succeed.
	/// When the thread is set to run synchronized it will only execute one update at a time
	/// and then wait for the startup barrier to wake it up again.
	/// \param	val	if true the thread will need to be controlled via the barrier.
	/// \return the actual value of isSynchronous()
	/// \note HS-2013-nov-01 Currently mostly for use in unit tests and debugging, when multiple thread with differing
	/// 	  rates are being synchronized the call rates will not correspond to the expected rates.
	bool setSynchronous(bool val);

	/// Query if this object is synchronized.
	/// \return	true if synchronized, false if not.
	bool isSynchronous();

	/// \return the cumulated cpu time taken to run all update since last reset or thread creation
	/// \note Only the latest 1,000,000 frames since last reset are cumulated, so if the timer is never reset,
	/// \note the Cpu time will not increase past that limit.
	double getCpuTime() const;

	/// \return the number of updates done since last reset or thread creation
	/// \note The update count since last reset has a limit of 1,000,000.
	size_t getUpdateCount() const;

	/// Reset the cpu time and the update count to 0
	void resetCpuTimeAndUpdateCount();

protected:

	/// Timer to measure the actual time taken to doUpdate
	Timer m_timer;

	/// Trigger the initialization of this object, this will be called before all other threads doStartup()
	/// are called
	/// \return true on success
	bool initialize();

	/// Trigger the startup of this object, this will be called after all other threads doInit() was called
	/// the thread will only enter the run loop triggering upated() if all threads doInit() and doStartup()
	/// returned true
	/// \return true on success
	bool startUp();

	bool waitForBarrier(bool success);

	virtual bool executeInitialization();

	/// Logger for this thread
	std::shared_ptr<SurgSim::Framework::Logger> m_logger;

private:
	std::string m_name;

	boost::thread m_thisThread;
	boost::chrono::duration<double> m_period;
	std::shared_ptr<Barrier> m_startupBarrier;

	// Protects the start and stop functions so on can only execute once the other is done
	boost::mutex m_mutexStartStop;

	bool m_isIdle;
	bool m_isInitialized;
	bool m_isRunning;
	bool m_stopExecution;
	bool m_isSynchronous;

	virtual bool doInitialize() = 0;
	virtual bool doStartUp() = 0;

	/// Implementation of actual work function for this thread, this has a default implementation to handle
	/// destruction better, as it could be called while the thread is under destruction, if left unimplemented
	/// this would trigger a call to a pure virtual function.
	/// \return false when the thread is done, this will stop execution
	virtual bool doUpdate(double dt);

	/// Prepares the thread for its execution to be stopped
	/// \note	Called from this thread before joined
	virtual void doBeforeStop();
};

}; // namespace Framework
}; // namespace SurgSim

#endif // SURGSIM_FRAMEWORK_BASICTHREAD_H
