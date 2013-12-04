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

#include "SurgSim/Devices/Mouse/OsgMouseScaffold.h"

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Devices/Mouse/MouseDevice.h"
#include "SurgSim/Devices/Mouse/MouseHandler.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/SharedInstance.h"

namespace SurgSim
{
namespace Device
{

using SurgSim::DataStructures::DataGroup;
using SurgSim::DataStructures::DataGroupBuilder;

/// Struct to hold a MouseDevice object, a MouseHandler, and a mutex for data passing.
struct OsgMouseScaffold::DeviceData
{
	/// Constructor
	/// \param device Device to be managed by this scaffold
	explicit DeviceData(MouseDevice* device) : deviceObject(device)
	{
		mouseHandler = new MouseHandler();
	}

	/// Device object managed by this scaffold.
	MouseDevice* const deviceObject;
	/// Mouse Handler to communicate with underneath API.
	osg::ref_ptr<MouseHandler> mouseHandler;
	/// The mutex that protects the externally modifiable parameters.
	boost::mutex mutex;

private:
	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	DeviceData(const DeviceData&) /*= delete*/;
	DeviceData& operator=(const DeviceData&) /*= delete*/;
};

OsgMouseScaffold::OsgMouseScaffold(std::shared_ptr<SurgSim::Framework::Logger> logger) : m_logger(logger)
{
	if (nullptr == m_logger)
	{
		m_logger = SurgSim::Framework::Logger::getLogger("Mouse device");
		m_logger->setThreshold(m_defaultLogLevel);
	}
	SURGSIM_LOG_DEBUG(m_logger) << "Mouse: Shared scaffold created.";
}

OsgMouseScaffold::~OsgMouseScaffold()
{
	unregisterDevice();
}

bool OsgMouseScaffold::registerDevice(MouseDevice* device)
{
	m_device.reset(new DeviceData(device));
	if (nullptr == m_device)
	{
		SURGSIM_LOG_CRITICAL(m_logger) << "OsgMouseScaffold::registerDevice(): failed to create a DeviceData";
		return false;
	}
	return true;
}

bool OsgMouseScaffold::unregisterDevice()
{
	m_device.reset();
	if (nullptr == m_device)
	{
		SURGSIM_LOG_DEBUG(m_logger) << "Mouse: Shared scaffold unregistered.";
		return true;
	}
	return false;
}

bool OsgMouseScaffold::updateDevice(int buttonMask, float x, float y, int scrollDeltaX, int scrollDeltaY)
{
	boost::lock_guard<boost::mutex> lock(m_device->mutex);
	SurgSim::DataStructures::DataGroup& inputData = m_device->deviceObject->getInputData();

	inputData.booleans().set("button1", (buttonMask & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) != 0);
	inputData.booleans().set("button2", (buttonMask & osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON) != 0);
	inputData.booleans().set("button3", (buttonMask & osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON) != 0);
	inputData.scalars().set("x", static_cast<double>(x));
	inputData.scalars().set("y", static_cast<double>(y));
	inputData.integers().set("scrollDeltaX", scrollDeltaX);
	inputData.integers().set("scrollDeltaY", scrollDeltaY);

	m_device->deviceObject->pushInput();
	return true;
}

MouseHandler* OsgMouseScaffold::getMouseHandler() const
{
	return m_device->mouseHandler.get();
}


/// Builds the data layout for the application input (i.e. device output).
SurgSim::DataStructures::DataGroup OsgMouseScaffold::buildDeviceInputData()
{
	DataGroupBuilder builder;
	builder.addBoolean("button1");		// Indicates mouse left button
	builder.addBoolean("button2"); 		// Indicates mouse middle button (i.e. wheel)
	builder.addBoolean("button3"); 		// Indicates mouse right button
	builder.addScalar("x");				// Indicates mouse's X-coordinate in the current window, left bottom = (0, 0)
	builder.addScalar("y");				// Indicates mouse's Y-coordinate in the current window, left bottom = (0, 0)
	builder.addInteger("scrollDeltaX"); // Indicates mouse wheel vertical movement
	builder.addInteger("scrollDeltaY"); // Indicates mouse wheel horizontal movement

	return builder.createData();
}

std::shared_ptr<OsgMouseScaffold> OsgMouseScaffold::getOrCreateSharedInstance()
{
	static SurgSim::Framework::SharedInstance<OsgMouseScaffold> sharedInstance;
	return sharedInstance.get();
}

void OsgMouseScaffold::setDefaultLogLevel(SurgSim::Framework::LogLevel logLevel)
{
	m_defaultLogLevel = logLevel;
}

std::shared_ptr<SurgSim::Framework::Logger> OsgMouseScaffold::getLogger() const
{
	return m_logger;
}


SurgSim::Framework::LogLevel OsgMouseScaffold::m_defaultLogLevel = SurgSim::Framework::LOG_LEVEL_INFO;

};  // namespace Device
};  // namespace SurgSim