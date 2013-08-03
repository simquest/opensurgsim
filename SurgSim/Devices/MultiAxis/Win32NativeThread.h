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

#ifndef SURGSIM_DEVICES_MULTIAXIS_WIN32NATIVETHREAD_H
#define SURGSIM_DEVICES_MULTIAXIS_WIN32NATIVETHREAD_H

#include <functional>
#include <memory>
#include <string>

namespace SurgSim
{
namespace Framework
{
class Logger;
}; // namespace Framework

namespace Device
{

/// A simple implementation of a native Win32 thread implementation.
/// Just spins in a loop and does not provide any fancy features like synchronization.
class Win32NativeThread
{
public:
	/// Defines a callback that can be executed repeatedly as the body of a thread.
	typedef std::function<bool()> ThreadTask;

	/// Constructor.
	/// \param logger	The logger to use.
	/// \param name	(Optional) the name for this thread.
	explicit Win32NativeThread(std::shared_ptr<SurgSim::Framework::Logger> logger,
		const std::string& name = "Unknown Thread");

#ifdef _MSC_VER
	~Win32NativeThread() throw(...);  // Visual Studio does not support noexcept. The throw(...) is optional.
#else
	/// Destructor.
	~Win32NativeThread() noexcept(false);  /// C++11 introduced noexcept
#endif

	/// Start the thread from the outside.
	/// The thread will run the task specified by the argument until it returns false.
	/// \param threadTask	The task to execute in the thread until it returns false.
	/// \return	true if it succeeds, false if it fails.
	bool start(const ThreadTask& threadTask);

// 	/// Blocks until the running thread has actually stopped.
// 	bool stop();

	/// Stop the execution by immediately killing the thread.
	/// Note that the state of the resources shared by the thread is completely dependent on the precise code
	/// location the thread was executing when it was killed.  Any resulting race conditions, deadlocks, etc. are
	/// entirely the caller's problem.
	/// \return	true if the thread was killed; false if the thread wasn't running or could not be killed.
	bool kill();

	/// Query if this object is running.
	/// Beware of race conditions, however.
	/// \return	true if running.
	bool isRunning() const;

	/// Gets the name of this thread.
	/// \return	The name of the thread.
	std::string getName() const;

	/// Internal state data.
	struct State;

private:
	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	Win32NativeThread(const Win32NativeThread& other) /*= delete*/;
	Win32NativeThread& operator=(const Win32NativeThread& other) /*= delete*/;

	std::unique_ptr<State> m_state;
};

}; // namespace Device
}; // namespace SurgSim

#endif  // SURGSIM_DEVICES_MULTIAXIS_WIN32NATIVETHREAD_H
