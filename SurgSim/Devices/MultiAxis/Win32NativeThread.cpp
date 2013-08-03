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

#include "SurgSim/Devices/MultiAxis/Win32NativeThread.h"

#undef  _WIN32_WINNT
#define _WIN32_WINNT 0x0501   // request Windows XP-compatible SDK APIs
#undef  WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN   // do not automatically include WinSock 1 and some other header files
#include <windows.h>

#include <functional>
#include <memory>
#include <string>

#include "SurgSim/Devices/MultiAxis/GetSystemError.h"
#include <SurgSim/Framework/Log.h>
#include <SurgSim/Framework/Assert.h>


namespace SurgSim
{
namespace Device
{

using SurgSim::Device::Internal::getSystemErrorCode;
using SurgSim::Device::Internal::getSystemErrorText;


struct Win32NativeThread::State
{
public:
	explicit State(std::shared_ptr<SurgSim::Framework::Logger>&& logger_, const std::string& name_) :
		logger(logger_),
		name(name_),
		threadHandle(INVALID_HANDLE_VALUE)
	{
	}

	std::shared_ptr<SurgSim::Framework::Logger> logger;
	std::string name;
	HANDLE threadHandle;
	ThreadTask threadTask;
};


Win32NativeThread::Win32NativeThread(std::shared_ptr<SurgSim::Framework::Logger> logger, const std::string& name) :
	m_state(new Win32NativeThread::State(std::move(logger), name))
{
}

#ifdef _MSC_VER
Win32NativeThread::~Win32NativeThread() throw(...)
#else
Win32NativeThread::~Win32NativeThread() noexcept(false)
#endif
{
	if (! kill())
	{
		if (isRunning())
		{
			// Emergency escape hatch: if the thread is somehow still running, we *LEAK* the state buffer to avoid
			// having the thread access bad memory.
			State* willBeLeaked = m_state.release();
			SURGSIM_LOG_INFO(willBeLeaked->logger) <<
				"Win32NativeThread: Error tearing down thread, will intentionally leak state buffer " << willBeLeaked;
		}
	}
}


static DWORD WINAPI beginThread(LPVOID parameter)
{
	Win32NativeThread::State* state = static_cast<Win32NativeThread::State*>(parameter);

	bool repeat = true;
	while (repeat)
	{
		repeat = state->threadTask();
	}

	return 0;
}

bool Win32NativeThread::start(const ThreadTask& threadTask)
{
	SURGSIM_ASSERT(! isRunning());

	m_state->threadTask = threadTask;

	HANDLE thread = CreateThread(
		NULL,               // default security attributes
		0,                  // use default stack size
		&beginThread,       // thread function
		m_state.get(),      // argument to thread function
		0,                  // use default creation flags
		NULL);              // don't return the thread ID

	if ((! thread) || (thread == INVALID_HANDLE_VALUE))
	{
		DWORD error = GetLastError();
		SURGSIM_LOG_CRITICAL(m_state->logger) << "Win32NativeThread: Thread creation failed with error " << error <<
			", " << getSystemErrorText(error);
		return false;
	}

	m_state->threadHandle = thread;
	return true;
}

bool Win32NativeThread::kill()
{
	bool wasKilled = false;
	if (isRunning())
	{
		HANDLE thread = m_state->threadHandle;
		if (thread != INVALID_HANDLE_VALUE)
		{
			BOOL status = TerminateThread(thread, 4321);
			if (status == FALSE)
			{
				DWORD error = GetLastError();
				if (isRunning())
				{
					SURGSIM_LOG_WARNING(m_state->logger) << "Win32NativeThread: TerminateThread() failed with error " <<
						error << ", " << getSystemErrorText(error);
				}
				else
				{
					SURGSIM_LOG_INFO(m_state->logger) << "Win32NativeThread: thread is dead, but" <<
						" TerminateThread() failed with error " << error << ", " << getSystemErrorText(error);
				}
			}
			else
			{
				wasKilled = true;
				m_state->threadHandle = INVALID_HANDLE_VALUE;
				CloseHandle(thread);  // ignore return status
			}
		}
	}
	return wasKilled;
}

bool Win32NativeThread::isRunning() const
{
	bool threadIsRunning = false;
	HANDLE thread = m_state->threadHandle;
	if (thread != INVALID_HANDLE_VALUE)
	{
		DWORD exitCode;
		BOOL status = GetExitCodeThread(thread, &exitCode);
		if (status == FALSE)
		{
			DWORD error = GetLastError();
			SURGSIM_LOG_WARNING(m_state->logger) << "Win32NativeThread: GetExitCodeThread() failed with error " <<
				error << ", " << getSystemErrorText(error);
			// What exit code do we return in this case???
			threadIsRunning = true;
		}
		else if (exitCode == STILL_ACTIVE)
		{
			threadIsRunning = true;
		}
		else
		{
			threadIsRunning = false;
			m_state->threadHandle = INVALID_HANDLE_VALUE;
			CloseHandle(thread);  // ignore return status
		}
	}
	return threadIsRunning;
}

std::string Win32NativeThread::getName() const
{
	return m_state->name;
}

}; // namespace Device
}; // namespace SurgSim
