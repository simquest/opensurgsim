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
#include <boost/atomic.hpp>

#include <SurgSim/Framework/Barrier.h>

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
/// Initialization can be further customized by implementing a executeInitialization() function
class BasicThread
{
public:
	explicit BasicThread(const std::string& name = "Unknown Thread");
#ifdef _MSC_VER
	~BasicThread() throw(...);  // Visual Studio does not support noexcept. The throw(...) is optional.
#else
	~BasicThread() noexcept(false);  /// C++11 introduced noexcept
#endif

	/// Live cycle functions, public implementation.
	/// All of these have virtual partners as private functions

	/// Start the thread from the outside, this will call the private
	/// run() function that can be overridden for each implementor of this
	/// interface
	/// \param startupBarrier is a barrier it synchronizes a group of thread that should go through their startup
	/// sequence in step.
	void start(std::shared_ptr<Barrier> startupBarrier=nullptr);

	/// Stopping the execution, blocks until the running thread has actually stopped
	void stop();

	/// Query if this object is initialized.
	/// \return	true if initialized, false if not.
	bool isInitialized();

	/// Query if this object is running.
	/// \return	true if the threads update() function is being called.
	bool isRunning() const;

	/// This is what boost::thread executes on thread creation.
	void operator()();

	/// \return the boost threading object
	boost::thread& getThread();

	/// \return the name of the thread
	std::string getName() const;

	/// Set the update rate of the thread
	/// \param val	rate in hertz (updates per second) of the thread
	void setRate(double val) {m_period = boost::chrono::duration<double>(1.0/val);}

protected:

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


private:
	std::string m_name;

	boost::thread m_thisThread;
	boost::chrono::duration<double> m_period;
	std::shared_ptr<Barrier> m_startupBarrier;

	bool m_isInitialized;
	bool m_isRunning;
	bool m_stopExecution;

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
