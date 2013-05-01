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
//! TODO HS Add functionality to stop the thread from the outside
class BasicThread
{
public:
	explicit BasicThread(const std::string& name = "Unknown Thread");
	virtual ~BasicThread();

	/// Live cycle functions, public implementation.
	/// All of these have virtual partners as private functions

	/// Start the thread from the outside, this will call the private
	/// run() function that can be overridden for each implementor of this
	/// interface
	/// \param startupBarrier is a barrier it synchronizes a group of thread that should go through their startup
	/// sequence in step.
	void start(std::shared_ptr<Barrier> startupBarrier);

	/// Stopping the execution, blocks until the running thread has actually stopped
	void stop();

	/// Query if this object is initialized.
	/// \return	true if initialized, false if not.
	bool isInitialized();

	/// Query if this object is running.
	/// \return	true if the threads update() function is being called.
	bool isRunning() const;


	/// Handle representations, override for each thread
	/// \param component	The component to be removed.
	/// \return true on success
	virtual bool removeComponent(std::shared_ptr<Component> component) = 0;

	/// Adds a component.
	/// \param component The component to be added.
	/// \return true if it succeeds or the thread is not concerned with the component, false if it fails.
	virtual bool addComponent(std::shared_ptr<Component> component) = 0;

	/// This is what boost::thread executes on thread creation.
	void operator()();

	/// \return the boost threading object
	boost::thread& getThread();

	/// \return the name of the thread
	std::string getName() const;

	/// @{
	/// Runtime accessors
	std::shared_ptr<Runtime> getRuntime() const;
	void setRuntime(std::shared_ptr<Runtime> val);
	/// @}

	/// Set the update rate of the thread
	/// \param val	rate in hertz (updates per second) of the thread
	void setRate(double val) {m_period = boost::chrono::duration<double>(1.0/val);}

protected:

	/// Trigger the initialisation of this object, this will be called before all other threads doStartup()
	/// are called
	/// \return true on success
	bool initialize();

	/// Trigger the startup of this object, this will be called after all other threads doInit() was called
	/// the thread will only enter the run loop triggering upated() if all threads doInit() and doStartup()
	/// returned true
	/// \return true on success
	bool startUp();

private:
	std::string m_name;

	boost::thread m_thisThread;
	boost::chrono::duration<double> m_period;
	std::shared_ptr<Barrier> m_startupBarrier;
	std::weak_ptr<Runtime> m_runtime;

	bool m_isInitialized;
	bool m_isRunning;
	bool m_stopExecution;

	virtual bool doInitialize() = 0;
	virtual bool doStartUp() = 0;

	//! \return false when the thread is done, this will stop execution
	virtual bool doUpdate(double dt) = 0;
};

};
};

#endif
