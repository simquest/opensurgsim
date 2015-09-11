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

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Devices/Keyboard/KeyboardDevice.h"
#include "SurgSim/Devices/Keyboard/OsgKeyboardHandler.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/SharedInstance.h"

namespace SurgSim
{
namespace Devices
{

using SurgSim::DataStructures::DataGroup;
using SurgSim::DataStructures::DataGroupBuilder;

/// Struct to hold a KeyboardDevice object, a KeyboardHandler object, and a mutex for data passing.
struct KeyboardScaffold::DeviceData
{
	/// Constructor
	/// \param device Device to be managed by this scaffold
	explicit DeviceData(KeyboardDevice* device) : deviceObject(device)
	{
		keyboardHandler = new OsgKeyboardHandler();
	}

	/// Device object managed by this scaffold.
	KeyboardDevice* const deviceObject;
	/// Keyboard Handler to communicate with underneath API.
	osg::ref_ptr<OsgKeyboardHandler> keyboardHandler;
	/// The mutex that protects the externally modifiable parameters.
	boost::mutex mutex;

private:
	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	DeviceData(const DeviceData&) /*= delete*/;
	DeviceData& operator=(const DeviceData&) /*= delete*/;
};

KeyboardScaffold::KeyboardScaffold(std::shared_ptr<SurgSim::Framework::Logger> logger) : m_logger(logger)
{
	if (!m_logger)
	{
		m_logger = SurgSim::Framework::Logger::getLogger("Keyboard device");
		m_logger->setThreshold(m_defaultLogLevel);
	}
	SURGSIM_LOG_DEBUG(m_logger) << "Keyboard: Shared scaffold created.";
}

KeyboardScaffold::~KeyboardScaffold()
{
	unregisterDevice();
}

bool KeyboardScaffold::registerDevice(KeyboardDevice* device)
{
	m_device.reset(new DeviceData(device));
	if (m_device == nullptr)
	{
		SURGSIM_LOG_CRITICAL(m_logger) << "KeyboardScaffold::registerDevice(): failed to create a DeviceData";
		return false;
	}
	return true;
}

bool KeyboardScaffold::unregisterDevice()
{
	m_device.reset();
	if (m_device == nullptr)
	{
		SURGSIM_LOG_DEBUG(m_logger) << "Keyboard: Shared scaffold unregistered.";
		return true;
	}
	return false;
}

bool KeyboardScaffold::updateDevice(int key, int modifierMask)
{
	boost::lock_guard<boost::mutex> lock(m_device->mutex);
	SurgSim::DataStructures::DataGroup& inputData = m_device->deviceObject->getInputData();
	inputData.integers().set("key", key);
	inputData.integers().set("modifierMask", modifierMask);

	m_device->deviceObject->pushInput();
	return true;
}

OsgKeyboardHandler* KeyboardScaffold::getKeyboardHandler() const
{
	return m_device->keyboardHandler.get();
}


/// Builds the data layout for the application input (i.e. device output).
SurgSim::DataStructures::DataGroup KeyboardScaffold::buildDeviceInputData()
{
	DataGroupBuilder builder;
	builder.addInteger("key");
	builder.addInteger("modifierMask");
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

std::shared_ptr<SurgSim::Framework::Logger> KeyboardScaffold::getLogger() const
{
	return m_logger;
}

SurgSim::Framework::LogLevel KeyboardScaffold::m_defaultLogLevel = SurgSim::Framework::LOG_LEVEL_INFO;

};  // namespace Devices
};  // namespace SurgSim