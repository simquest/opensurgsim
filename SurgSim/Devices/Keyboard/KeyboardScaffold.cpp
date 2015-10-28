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

KeyboardScaffold::KeyboardScaffold() : m_logger(Framework::Logger::getLogger("Devices/Keyboard"))
{
	SURGSIM_LOG_DEBUG(m_logger) << "Shared scaffold created.";
}

KeyboardScaffold::~KeyboardScaffold()
{
	if (m_device != nullptr)
	{
		unregisterDevice();
	}
	SURGSIM_LOG_DEBUG(m_logger) << "Shared scaffold destroyed.";
}

bool KeyboardScaffold::registerDevice(KeyboardDevice* device)
{
	SURGSIM_ASSERT(m_device == nullptr) << "Can't register two Keyboard devices.";

	m_device.reset(new DeviceData(device));
	if (m_device == nullptr)
	{
		SURGSIM_LOG_CRITICAL(m_logger) << "Failed to create a DeviceData";
		return false;
	}
	SURGSIM_LOG_DEBUG(m_logger) << "Registered device " << device->getName();
	return true;
}

bool KeyboardScaffold::unregisterDevice()
{
	m_device.reset();
	if (m_device == nullptr)
	{
		SURGSIM_LOG_DEBUG(m_logger) << "Unregistered device";
		return true;
	}
	SURGSIM_LOG_DEBUG(m_logger) << "There is no device to unregister.";
	return false;
}

bool KeyboardScaffold::updateDevice(int key, int modifierMask)
{
	boost::lock_guard<boost::mutex> lock(m_device->mutex);
	DataStructures::DataGroup& inputData = m_device->deviceObject->getInputData();
	inputData.integers().set("key", key);
	inputData.integers().set("modifierMask", modifierMask);

	m_device->deviceObject->pushInput();
	return true;
}

OsgKeyboardHandler* KeyboardScaffold::getKeyboardHandler() const
{
	return m_device->keyboardHandler.get();
}

DataStructures::DataGroup KeyboardScaffold::buildDeviceInputData()
{
	DataStructures::DataGroupBuilder builder;
	builder.addInteger("key");
	builder.addInteger("modifierMask");
	return builder.createData();
}

std::shared_ptr<KeyboardScaffold> KeyboardScaffold::getOrCreateSharedInstance()
{
	static Framework::SharedInstance<KeyboardScaffold> sharedInstance;
	return sharedInstance.get();
}

};  // namespace Devices
};  // namespace SurgSim