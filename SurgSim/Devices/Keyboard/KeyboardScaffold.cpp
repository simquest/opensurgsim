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

#include <SurgSim/DataStructures/DataGroup.h>
#include <SurgSim/DataStructures/DataGroupBuilder.h>
#include <SurgSim/Framework/Log.h>
#include <SurgSim/Framework/SharedInstance.h>

#include <osgGA/GUIEventHandler>

namespace SurgSim
{
namespace Device
{
using SurgSim::DataStructures::DataGroup;
using SurgSim::DataStructures::DataGroupBuilder;


class KeyboardHandler : public osgGA::GUIEventHandler
{
public:

	virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&)
	{
		switch(ea.getEventType())
		{
		case(osgGA::GUIEventAdapter::KEYDOWN):
		{
			key = ea.getUnmodifiedKey();
			key_modifier = 0;  //This is not correct right now
			return true;
		}
		default:
			return false;
		}
	}

	int key;
	int key_modifier;
};



struct KeyboardScaffold::DeviceData
{
	/// Constructor
	/// \param device
	explicit DeviceData(KeyboardDevice* device) : deviceObject(device)
	{
		keyboardEventHandler = std::make_shared<KeyboardHandler>();
	}

	/// Destructor
	~DeviceData()
	{
	}

	KeyboardDevice* const deviceObject;
	std::shared_ptr<KeyboardHandler> keyboardEventHandler;

private:
	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	DeviceData(const DeviceData&) /*= delete*/;
	DeviceData& operator=(const DeviceData&) /*= delete*/;
};

KeyboardScaffold::KeyboardScaffold(std::shared_ptr<SurgSim::Framework::Logger> logger) :
	m_logger(logger)
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
	unregisterDevice();
}

bool KeyboardScaffold::registerDevice(KeyboardDevice* device)
{
	m_device = std::make_shared<DeviceData>(device);
	if (m_device == nullptr)
	{
		SURGSIM_LOG_CRITICAL(m_logger) << "KeyboardScaffold::registerDevice(): failed to create a DeviceData";
		return false;
	}
	else
	{
		return true;
	}
}

bool KeyboardScaffold::unregisterDevice()
{
	m_device.reset();
	if (m_device == nullptr)
	{
		SURGSIM_LOG_DEBUG(m_logger) << "Keyboard: Shared scaffold unregistered.";
		return true;
	}
	else
	{
		return false;
	}
}

bool KeyboardScaffold::updateDevice(DeviceData* info)
{
	SurgSim::DataStructures::DataGroup& inputData = info->deviceObject->getInputData();
	
	inputData.integers().set("key", info->keyboardEventHandler->key);
	inputData.integers().set("key_modifier", info->keyboardEventHandler->key_modifier);

	return true;
}

std::shared_ptr<KeyboardHandler> KeyboardScaffold::getKeyboardHandler() const
{
	return m_device->keyboardEventHandler;
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

std::shared_ptr<SurgSim::Framework::Logger> KeyboardScaffold::getLogger() const
{
	return m_logger;
}


SurgSim::Framework::LogLevel KeyboardScaffold::m_defaultLogLevel = SurgSim::Framework::LOG_LEVEL_INFO;

};  // namespace Device
};  // namespace SurgSim