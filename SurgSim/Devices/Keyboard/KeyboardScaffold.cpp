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

#include "SurgSim/Devices/Keyboard/KeyboardScaffold.h"

#include <SurgSim/Devices/Keyboard/KeyboardDevice.h>
#include <SurgSim/Devices/Keyboard/KeyboardThread.h>

#include <vector>
#include <memory>
#include <utility>

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include <osgGA/GUIEventHandler>

#include <SurgSim/Framework/Assert.h>
#include <SurgSim/Framework/Log.h>
#include <SurgSim/Framework/SharedInstance.h>
#include <SurgSim/DataStructures/DataGroup.h>
#include <SurgSim/DataStructures/DataGroupBuilder.h>


using SurgSim::DataStructures::DataGroup;
using SurgSim::DataStructures::DataGroupBuilder;

namespace SurgSim
{
namespace Device
{

struct KeyboardScaffold::DeviceData
{
	/// Initialize the state.
	//DeviceData(KeyboardDevice* device,
	//	std::unique_ptr<KeyboardHandle>&& handle) :
	//	deviceObject(device),
	//	deviceHandle(std::move(handle))
	//{
	//}
	explicit DeviceData(KeyboardDevice* device):
		deviceObject(device)
	{

	}

	~DeviceData()
	{
	}

	KeyboardDevice* const deviceObject;
	//std::unique_ptr<KeyboardHandle> deviceHandle;

	/// Processing thread.
	std::unique_ptr<KeyboardThread> thread;

private:
	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	DeviceData(const DeviceData&) /*= delete*/;
	DeviceData& operator=(const DeviceData&) /*= delete*/;
};


struct KeyboardScaffold::StateData
{
	/// Initialize the state.
	StateData() : isApiInitialized(false)
	{
	}

	/// True if the API has been initialized (and not finalized).
	bool isApiInitialized;

	/// The list of known devices.
	std::list<std::unique_ptr<KeyboardScaffold::DeviceData>> activeDeviceList;

	/// The mutex that protects the list of known devices.
	boost::mutex mutex;

private:
	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	StateData(const StateData&) /*= delete*/;
	StateData& operator=(const StateData&) /*= delete*/;
};

KeyboardScaffold::KeyboardScaffold(std::shared_ptr<SurgSim::Framework::Logger> logger) :
	m_logger(logger), m_state(new StateData)
{
	if (! m_logger)
	{
		m_logger = SurgSim::Framework::Logger::getLogger("Keyboard device");
		m_logger->setThreshold(m_defaultLogLevel);
	}
	SURGSIM_LOG_DEBUG(m_logger) << "Keyboard: Shared scaffold created.";
}

KeyboardScaffold::~KeyboardScaffold()
{
	{
		boost::lock_guard<boost::mutex> lock(m_state->mutex);

		if (! m_state->activeDeviceList.empty())
		{
			SURGSIM_LOG_SEVERE(m_logger) << "Keyboard: Destroying scaffold while devices are active!?!";
			// do anything special with each device?
			for (auto it = m_state->activeDeviceList.begin();  it != m_state->activeDeviceList.end();  ++it)
			{
				if ((*it)->thread)
				{
					destroyPerDeviceThread(it->get());
				}
			}
			m_state->activeDeviceList.clear();
		}

		if (m_state->isApiInitialized)
		{
			finalizeSdk();
		}
	}
	SURGSIM_LOG_DEBUG(m_logger) << "Keyboard: Shared scaffold destroyed.";
}

std::shared_ptr<SurgSim::Framework::Logger> KeyboardScaffold::getLogger() const
{
	return m_logger;
}

bool KeyboardScaffold::registerDevice(KeyboardDevice* device)
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);

	if (! m_state->isApiInitialized)
	{
		if (! initializeSdk())
		{
			return false;
		}
	}

	// Make sure the object is unique.
	auto sameObject = std::find_if(m_state->activeDeviceList.cbegin(), m_state->activeDeviceList.cend(),
		[device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });
	SURGSIM_ASSERT(sameObject == m_state->activeDeviceList.end()) << "Keyboard: Tried to register a device" <<
		" which is already present!";

	// Make sure the name is unique.
	const std::string deviceName = device->getName();
	auto sameName = std::find_if(m_state->activeDeviceList.cbegin(), m_state->activeDeviceList.cend(),
		[&deviceName](const std::unique_ptr<DeviceData>& info) { return info->deviceObject->getName() == deviceName; });
	if (sameName != m_state->activeDeviceList.end())
	{
		SURGSIM_LOG_CRITICAL(m_logger) << "Keyboard: Tried to register a device when the same name is" <<
			" already present!";
		return false;
	}

	// Construct the object, start its thread, then move it to the list.
	// Note that since Visual Studio 2010 doesn't support multi-argument emplace_back() for STL containers, storing a
	// list of unique_ptr results in nicer code than storing a list of DeviceData values directly.
	std::unique_ptr<DeviceData> info(new DeviceData(device));
	//if (! initializeDeviceState(info.get()))
	//{
	//	return false;   // message already printed
	//}

	createPerDeviceThread(info.get());
	m_state->activeDeviceList.emplace_back(std::move(info));

	return true;
}


bool KeyboardScaffold::unregisterDevice(const KeyboardDevice* const device)
{
	//std::unique_ptr<DeviceData> savedInfo;
	//bool haveOtherDevices = false;
	//{
	//	boost::lock_guard<boost::mutex> lock(m_state->mutex);
	//	auto matching = std::find_if(m_state->activeDeviceList.begin(), m_state->activeDeviceList.end(),
	//		[device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });
	//	if (matching != m_state->activeDeviceList.end())
	//	{
	//		savedInfo = std::move(*matching);
	//		m_state->activeDeviceList.erase(matching);
	//		// the iterator is now invalid but that's OK
	//	}
	//	haveOtherDevices = (m_state->activeDeviceList.size() > 0);
	//}

	//bool status = true;
	//if (! savedInfo)
	//{
	//	SURGSIM_LOG_WARNING(m_logger) << "Keyboard: Attempted to release a non-registered device.";
	//	status = false;
	//}
	//else
	//{
	//	finalizeDeviceState(savedInfo.get());
	//	savedInfo.reset(nullptr);

	//	if (haveOtherDevices)
	//	{
	//		//// If there are other devices left, we need to recreate the haptic callback.
	//		//// If there aren't, we don't need the callback... and moreover, trying to create it will fail.
	//		//createHapticLoop();
	//	}
	//}
	//return status;
	bool found = false;
	{
		boost::lock_guard<boost::mutex> lock(m_state->mutex);
		auto matching = std::find_if(m_state->activeDeviceList.begin(), m_state->activeDeviceList.end(),
			[device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });
		if (matching != m_state->activeDeviceList.end())
		{
			if ((*matching)->thread)
			{
				destroyPerDeviceThread(matching->get());
			}
			m_state->activeDeviceList.erase(matching);
			// the iterator is now invalid but that's OK
			found = true;
		}
	}

	if (! found)
	{
		SURGSIM_LOG_WARNING(m_logger) << "RawMultiAxis: Attempted to release a non-registered device.";
	}
	return found;
}

//bool KeyboardScaffold::initializeDeviceState(DeviceData* info)
//{
//	//SURGSIM_ASSERT(! info->deviceHandle.isValid());
//
//	//if (! info->deviceHandle.create(info->deviceObject->getName(), info->deviceObject->getInitializationName()))
//	//{
//	//	return false;  // message was already printed
//	//}
//	return true;
//}
//
//
//bool KeyboardScaffold::finalizeDeviceState(DeviceData* info)
//{
//	/*bool status = false;
//	if (info->deviceHandle.isValid())
//	{
//	status = info->deviceHandle.destroy();
//	}*/
//	return true;
//}

bool KeyboardScaffold::updateDevice(DeviceData* info)
{
	/*SurgSim::DataStructures::DataGroup& inputData = info->deviceObject->getInputData();

	info->deviceHandle.getKeyStatus(info->keyStatus);

	inputData.integers().set("KEY", info->keyStatus.first);
	inputData.integers().set("KEY_STATUS", info->keyStatus.second);*/

	return true;
}

bool KeyboardScaffold::createPerDeviceThread(DeviceData* data)
{
	SURGSIM_ASSERT(! data->thread);

	std::unique_ptr<KeyboardThread> thread(new KeyboardThread(this, data));
	thread->start();
	data->thread = std::move(thread);

	return true;
}

bool KeyboardScaffold::destroyPerDeviceThread(DeviceData* data)
{
	SURGSIM_ASSERT(data->thread);

	std::unique_ptr<KeyboardThread> thread = std::move(data->thread);
	thread->stop();
	thread.reset();

	return true;
}

bool KeyboardScaffold::runInputFrame(KeyboardScaffold::DeviceData* info)
{
	info->deviceObject->pullOutput();
	if (! updateDevice(info))
	{
		return false;
	}
	info->deviceObject->pushInput();
	return true;
}

/// Builds the data layout for the application input (i.e. device output).
SurgSim::DataStructures::DataGroup KeyboardScaffold::buildDeviceInputData()
{
	DataGroupBuilder builder;
	// KEY_DOWN = 0, KEY_UP = 1
	builder.addInteger("key");
	builder.addInteger("key_modifier");
	return builder.createData();
}

std::shared_ptr<KeyboardScaffold> KeyboardScaffold::getOrCreateSharedInstance()
{
	static SurgSim::Framework::SharedInstance<KeyboardScaffold> sharedInstance;
	return sharedInstance.get();
}

bool KeyboardScaffold::initializeSdk()
{
	SURGSIM_ASSERT(! m_state->isApiInitialized);
	// nothing to do!
	m_state->isApiInitialized = true;
	return true;
}

bool KeyboardScaffold::finalizeSdk()
{
	SURGSIM_ASSERT(m_state->isApiInitialized);
	// nothing to do!
	m_state->isApiInitialized = false;
	return true;
}

void KeyboardScaffold::setDefaultLogLevel(SurgSim::Framework::LogLevel logLevel)
{
	m_defaultLogLevel = logLevel;
}

SurgSim::Framework::LogLevel KeyboardScaffold::m_defaultLogLevel = SurgSim::Framework::LOG_LEVEL_INFO;

};  // namespace Device
};  // namespace SurgSim