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

#include "SurgSim/Devices/Phantom/PhantomManager.h"

#include <vector>
#include <memory>

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include <SurgSim/Framework/Assert.h>
#include <SurgSim/Devices/Phantom/PhantomDevice.h>

#include <HD/hd.h>

namespace SurgSim
{
namespace Device
{


struct PhantomManager::State
{
	/// Initialize the state.
	State() : callbackHandle(0), haveCallback(false)
	{
	}

	/// The haptic loop callback handle.
	HDSchedulerHandle callbackHandle;

	/// True if the callback has been created (and not destroyed).
	bool haveCallback;

	/// The list of active devices.
	std::vector<std::shared_ptr<PhantomDevice>> activeDevices;

	/// The mutex that protects the list of active devices.
	boost::mutex activeDeviceMutex;
};


PhantomManager::PhantomManager(std::shared_ptr<SurgSim::Framework::Logger> logger) :
	m_logger(logger), m_state(new State)
{
	if (m_logger == nullptr)
	{
		m_logger = SurgSim::Framework::Logger::createConsoleLogger("Phantom device");
	}

	{
		// Drain the HDAPI error stack
		HDErrorInfo error = hdGetError();
		while (error.errorCode != HD_SUCCESS)
		{
			error = hdGetError();
		}
	}
}



static HDCallbackCode HDCALLBACK runHapticLoopCallback(void* data)
{
	PhantomManager* manager = static_cast<PhantomManager*>(data);
	if (! manager->runHapticFrame())
	{
	}

	// Should return HD_CALLBACK_CONTINUE to wait for the next frame, or HD_CALLBACK_DONE to terminate the calls.
	return HD_CALLBACK_CONTINUE;
}


PhantomManager::~PhantomManager()
{
	if (m_state->haveCallback)
	{
		destroyHapticLoop();
	}
	boost::lock_guard<boost::mutex> lock(m_state->activeDeviceMutex);
	for (auto it = m_state->activeDevices.begin();  it != m_state->activeDevices.end();  ++it)
	{
		(*it)->finalize();
	}
}


std::shared_ptr<PhantomDevice> PhantomManager::createDevice(const std::string& uniqueName,
                                                            const std::string& initializationName)
{
	std::shared_ptr<PhantomDevice> device(new PhantomDevice(*this, uniqueName, initializationName));
	if (! device->initialize())
	{
		device.reset();  // clear the pointer
		return device;
	}

	{
		boost::lock_guard<boost::mutex> lock(m_state->activeDeviceMutex);
		if (m_state->activeDevices.size() == 0)
		{
			// If this is the first device, create the haptic loop as well.
			// The haptic loop should be created AFTER initializing the device, or OpenHaptics will complain.
			createHapticLoop();
		}

		m_state->activeDevices.push_back(device);  // Can't std::move it because we are returning the value too
	}

	return std::move(device);
}


bool PhantomManager::releaseDevice(std::shared_ptr<PhantomDevice> device)
{
	bool found = false;
	bool haveOtherDevices = false;
	{
		boost::lock_guard<boost::mutex> lock(m_state->activeDeviceMutex);
		for (auto it = m_state->activeDevices.begin();  it != m_state->activeDevices.end();  ++it)
		{
			if (*it == device)
			{
				found = true;
				m_state->activeDevices.erase(it);
				break;
			}
		}
		haveOtherDevices = (m_state->activeDevices.size() > 0);
	}

	if (found)
	{
		// If you attempt to destroy the device while the haptic callback is active, you see lots of nasty errors
		// under OpenHaptics 3.0.  The solution seems to be to disable the haptic callback when destroying the device.
		destroyHapticLoop();

		device->finalize();

		if (haveOtherDevices)
		{
			// If there are other devices left, we need to recreate the haptic callback.
			// If there aren't, we don't need the callback... and moreover, trying to create it will fail.
			createHapticLoop();
		}
	}
	return found;
}

bool PhantomManager::runHapticFrame()
{
	boost::lock_guard<boost::mutex> lock(m_state->activeDeviceMutex);

	for (auto it = m_state->activeDevices.begin();  it != m_state->activeDevices.end();  ++it)
	{
		(*it)->pullOutput();
	}
	for (auto it = m_state->activeDevices.begin();  it != m_state->activeDevices.end();  ++it)
	{
		// TODO(bert): do something with return value?
		(*it)->update();
	}
	for (auto it = m_state->activeDevices.cbegin();  it != m_state->activeDevices.cend();  ++it)
	{
		(*it)->pushInput();
	}

	return true;
}

bool PhantomManager::createHapticLoop()
{
	SURGSIM_ASSERT(! m_state->haveCallback);

	hdStartScheduler();
	if (checkForFatalError("Couldn't start the scheduler"))
	{
		return false;
	}

	m_state->callbackHandle = hdScheduleAsynchronous(runHapticLoopCallback, this, HD_DEFAULT_SCHEDULER_PRIORITY);
	if (checkForFatalError("Couldn't run haptic callback"))
	{
		hdStopScheduler();
		checkForFatalError("Couldn't stop the scheduler");
		return false;
	}

	m_state->haveCallback = true;
	return true;
}

bool PhantomManager::destroyHapticLoop()
{
	SURGSIM_ASSERT(m_state->haveCallback);

	bool sawError = false;
	checkForFatalError("Error prior to stopping haptic callback");  // NOT considered an error for return code!
	hdUnschedule(m_state->callbackHandle);
	sawError = checkForFatalError("Couldn't stop haptic callback") || sawError;
	hdStopScheduler();
	sawError = checkForFatalError("Couldn't stop the scheduler") || sawError;

	m_state->haveCallback = false;
	return !sawError;
}

bool PhantomManager::checkForFatalError(const char* message)
{
	return PhantomDevice::checkForFatalError(m_logger, "PhantomManager: ", message);
}


};  // namespace Device
};  // namespace SurgSim
