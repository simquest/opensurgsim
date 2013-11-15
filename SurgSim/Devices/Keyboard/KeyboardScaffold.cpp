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

#include <SurgSim/Devices/Keyboard/KeyboardDevice.h>
#include <SurgSim/Devices/Keyboard/KeyboardScaffold.h>

#include <memory>
#include <utility>

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include <SurgSim/DataStructures/DataGroup.h>
#include <SurgSim/DataStructures/DataGroupBuilder.h>
#include <SurgSim/Framework/Log.h>
#include <SurgSim/Framework/SharedInstance.h>

using SurgSim::DataStructures::DataGroup;
using SurgSim::DataStructures::DataGroupBuilder;

namespace SurgSim
{
namespace Device
{

struct KeyboardScaffold::DeviceData
{
	/// Constructor
	/// \param device
	explicit DeviceData(KeyboardDevice* device) : deviceObject(device)
	{
	}

	/// Destructor
	~DeviceData()
	{
	}

	KeyboardDevice* const deviceObject;

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
			m_state->activeDeviceList.clear();
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

	std::unique_ptr<DeviceData> info(new DeviceData(device));
	m_state->activeDeviceList.emplace_back(std::move(info));

	return true;
}

bool KeyboardScaffold::unregisterDevice(const KeyboardDevice* const device)
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);

	auto matching = std::find_if(m_state->activeDeviceList.begin(), m_state->activeDeviceList.end(),
		[device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });

	if (matching != m_state->activeDeviceList.end())
	{
		m_state->activeDeviceList.erase(matching);
		return true;
	}
	else
	{
		SURGSIM_LOG_WARNING(m_logger) << "Keyboard: Attempted to release a non-registered device.";
		return false;
	}
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
	SurgSim::DataStructures::DataGroup& inputData = info->deviceObject->getInputData();
	/* osgGA communication
	info->updateKeyStatus(info->keyStatus, info->keyModifier, info->updated);

	inputData.integers().set("key", info->keyStatus[0]);
	inputData.integers().set("key_modifier", info->keyModifier);
	*/
	return true;
}

/// Builds the data layout for the application input (i.e. device output).
SurgSim::DataStructures::DataGroup KeyboardScaffold::buildDeviceInputData()
{
	DataGroupBuilder builder;
	builder.addInteger("key");
	builder.addInteger("key_modifier");
	return builder.createData();
}

std::shared_ptr<KeyboardScaffold> KeyboardScaffold::getOrCreateSharedInstance()
{
	static SurgSim::Framework::SharedInstance<KeyboardScaffold> sharedInstance;
	return sharedInstance.get();
}

void KeyboardScaffold::setDefaultLogLevel(SurgSim::Framework::LogLevel logLevel)
{
	m_defaultLogLevel = logLevel;
}

SurgSim::Framework::LogLevel KeyboardScaffold::m_defaultLogLevel = SurgSim::Framework::LOG_LEVEL_INFO;

};  // namespace Device
};  // namespace SurgSim