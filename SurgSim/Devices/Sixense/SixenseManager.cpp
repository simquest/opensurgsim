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

#include "SurgSim/Devices/Sixense/SixenseManager.h"

#include <vector>
#include <memory>

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include "SurgSim/Devices/Sixense/SixenseDevice.h"
#include "SurgSim/Devices/Sixense/SixenseThread.h"
#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Framework/Log.h"

#include <sixense.h>

namespace SurgSim
{
namespace Device
{


struct SixenseManager::State
{
	/// Initialize the state.
	State() : isApiInitialized(false)
	{
	}

	/// True if the API has been initialized (and not finalized).
	bool isApiInitialized;

	/// Processing thread.
	std::unique_ptr<SixenseThread> thread;

	/// The list of known devices.
	std::vector<SixenseDevice*> activeDevices;

	/// The mutex that protects the list of known devices.
	boost::mutex mutex;
};


SixenseManager::SixenseManager(std::shared_ptr<SurgSim::Framework::Logger> logger) :
	m_logger(logger), m_state(new State)
{
	if (! m_logger)
	{
		m_logger = SurgSim::Framework::Logger::createConsoleLogger("Sixense/Hydra device");
		m_logger->setThreshold(m_defaultLogLevel);
	}
	SURGSIM_LOG_DEBUG(m_logger) << "SixenseManager: Created.";
}


SixenseManager::~SixenseManager()
{
	if (m_state->thread)
	{
		destroyThread();
	}

	{
		boost::lock_guard<boost::mutex> lock(m_state->mutex);

		for (auto it = m_state->activeDevices.begin();  it != m_state->activeDevices.end();  ++it)
		{
			(*it)->finalize();
			*it = nullptr;
		}

		if (m_state->isApiInitialized)
		{
			finalizeSdk();
		}
	}
	SURGSIM_LOG_DEBUG(m_logger) << "SixenseManager: Destroyed.";
}


std::shared_ptr<SixenseDevice> SixenseManager::createDevice(const std::string& uniqueName)
{
	{
		boost::lock_guard<boost::mutex> lock(m_state->mutex);

		if (! m_state->isApiInitialized)
		{
			if (! initializeSdk())
			{
				// Return an empty shared_ptr.
				return std::shared_ptr<SixenseDevice>();
			}
		}
	}

	std::shared_ptr<SixenseDevice> device;
	int numUsedDevicesSeen = 0;
	boost::chrono::steady_clock::time_point tick = boost::chrono::steady_clock::now();

	bool deviceFound = scanForUnusedDevice(uniqueName, &device, &numUsedDevicesSeen);
	if (! deviceFound && (numUsedDevicesSeen == 0))
	{
		// Unfortunately, right after sixenseInit() the library has not yet completed its device discovery!
		// That means that calls to sixenseIsBaseConnected(), sixenseIsControllerEnabled(), etc. will return
		// *false* even if the base/controller is actually present.
		//
		// One way to deal with that would be to dynamically handle any (or no) connected devices, the way the
		// example code does.  But we'd like to report missing devices to the calling code as soon as possible.
		//
		// Another is to simply sleep and retry a few times here.  Ugh.
		// --advornik 2012-08-03
		for (int i = 0;  i < 20;  ++i)
		{
			tick += boost::chrono::milliseconds(100);
			boost::this_thread::sleep_until(tick);

			deviceFound = scanForUnusedDevice(uniqueName, &device, &numUsedDevicesSeen);
			if (deviceFound || (numUsedDevicesSeen > 0))
			{
				break;
			}
		}
	}

	if (! deviceFound)
	{
		if (numUsedDevicesSeen > 0)
		{
			SURGSIM_LOG_SEVERE(m_logger) << "SixenseManager: Failed to find any unused controllers!";
		}
		else
		{
			SURGSIM_LOG_SEVERE(m_logger) << "SixenseManager: Failed to find any devices." <<
				"  Are the base and controllers plugged in?";
		}
		device.reset();
	}

	return device;
}


bool SixenseManager::releaseDevice(const SixenseDevice* device)
{
	bool found = false;
	bool haveOtherDevices = false;
	{
		boost::lock_guard<boost::mutex> lock(m_state->mutex);
		for (auto it = m_state->activeDevices.begin();  it != m_state->activeDevices.end();  ++it)
		{
			if (*it == device)
			{
				m_state->activeDevices.erase(it);
				// the iterator is now invalid but that's OK
				break;
			}
		}
		haveOtherDevices = (m_state->activeDevices.size() > 0);
	}

	if (found)
	{
		// the device is already finalized!
		if (! haveOtherDevices)
		{
			destroyThread();
		}
	}
	return found;
}

bool SixenseManager::runInputFrame()
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);

	for (auto it = m_state->activeDevices.begin();  it != m_state->activeDevices.end();  ++it)
	{
		if (*it)
		{
			// We don't call (*it)->pullOutput() because we don't use any output at all.
			(*it)->update();
			(*it)->pushInput();
		}
	}

	return true;
}

bool SixenseManager::initializeSdk()
{
	SURGSIM_ASSERT(! m_state->isApiInitialized);

	int status = sixenseInit();
	if (status != SIXENSE_SUCCESS)
	{
		SURGSIM_LOG_CRITICAL(m_logger) << "SixenseManager: Could not initialize the Sixense library (status = " <<
			status << ")";
		return false;
	}

	m_state->isApiInitialized = true;
	return true;
}

bool SixenseManager::finalizeSdk()
{
	SURGSIM_ASSERT(m_state->isApiInitialized);

	int status = sixenseExit();
	if (status != SIXENSE_SUCCESS)
	{
		SURGSIM_LOG_CRITICAL(m_logger) << "SixenseManager: Could not shut down the Sixense library (status = " <<
			status << ")";
		return false;
	}

	m_state->isApiInitialized = false;
	return true;
}

bool SixenseManager::scanForUnusedDevice(const std::string& uniqueName, std::shared_ptr<SixenseDevice>* device,
                                         int* numUsedDevicesSeen)
{
	(*device).reset();
	*numUsedDevicesSeen = 0;

	const int maxNumBases = sixenseGetMaxBases();

	for (int b = 0;  b < maxNumBases;  ++b)
	{
		SURGSIM_LOG_DEBUG(m_logger) << "SixenseManager: scanning base #" << b << " (of total " << maxNumBases << ")";
		if (! sixenseIsBaseConnected(b))
		{
			continue;
		}

		int status = sixenseSetActiveBase(b);
		if (status != SIXENSE_SUCCESS)
		{
			SURGSIM_LOG_SEVERE(m_logger) << "SixenseManager: Could not activate connected base #" << b <<
				" (status = " << status << ")";
			continue;
		}

		const int maxNumControllers = sixenseGetMaxControllers();
		for (int c = 0;  c < maxNumControllers;  ++c)
		{
			if (! sixenseIsControllerEnabled(c))
			{
				continue;
			}

			sixenseControllerData data;
			status = sixenseGetNewestData(c, &data);
			if (status != SIXENSE_SUCCESS)
			{
				SURGSIM_LOG_SEVERE(m_logger) << "SixenseManager: Could not get data from enabled controller #" <<
					b << "," << c << " (status = " << status << ")";
				continue;
			}

			SURGSIM_LOG_DEBUG(m_logger) << "SixenseManager: found controller #" << b << "," << c <<
				" (of total " << maxNumControllers << ")";


			if (createDeviceIfUnused(b, c, uniqueName, device, numUsedDevicesSeen))
			{
				return true;
			}
		}
	}

	return false;
}

bool SixenseManager::createDeviceIfUnused(int baseIndex, int controllerIndex, const std::string& uniqueName,
                                          std::shared_ptr<SixenseDevice>* newDevice, int* numUsedDevicesSeen)
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);

	// Check existing devices.
	for (auto it = m_state->activeDevices.begin();  it != m_state->activeDevices.end();  ++it)
	{
		if ((baseIndex == (*it)->getBaseIndex()) && (controllerIndex == (*it)->getControllerIndex()))
		{
			// We found an existing device for this controller.
			++(*numUsedDevicesSeen);
			return false;
		}
	}

	// The controller is not yet in use.

	if (! m_state->thread)
	{
		createThread();
	}

	std::shared_ptr<SixenseDevice> device =
	    std::shared_ptr<SixenseDevice>(new SixenseDevice(uniqueName, baseIndex, controllerIndex, getLogger()));
	// We initialize the device now, because if initialization fails, we don't want to add it to the active list.
	if (! device->initialize())
	{
		(*newDevice).reset();  // clear the pointer
		return false;
	}

	m_state->activeDevices.push_back(device.get());
	*newDevice = std::move(device);
	return true;
}

bool SixenseManager::createThread()
{
	SURGSIM_ASSERT(! m_state->thread);

	std::unique_ptr<SixenseThread> thread(new SixenseThread(this));
	thread->start();
	m_state->thread = std::move(thread);

	return true;
}

bool SixenseManager::destroyThread()
{
	SURGSIM_ASSERT(m_state->thread);

	std::unique_ptr<SixenseThread> thread = std::move(m_state->thread);
	thread->stop();
	thread.release();

	return true;
}

void SixenseManager::setDefaultLogLevel(SurgSim::Framework::LogLevel logLevel)
{
	m_defaultLogLevel = logLevel;
}

SurgSim::Framework::LogLevel SixenseManager::m_defaultLogLevel = SurgSim::Framework::LOG_LEVEL_INFO;


};  // namespace Device
};  // namespace SurgSim
