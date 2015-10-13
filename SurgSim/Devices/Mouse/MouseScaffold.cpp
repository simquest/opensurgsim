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

#include "SurgSim/Devices/Mouse/MouseScaffold.h"

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Devices/Mouse/MouseDevice.h"
#include "SurgSim/Devices/Mouse/OsgMouseHandler.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/SharedInstance.h"

namespace SurgSim
{
namespace Devices
{

using SurgSim::DataStructures::DataGroup;
using SurgSim::DataStructures::DataGroupBuilder;

/// Struct to hold a MouseDevice object, a OsgMouseHandler, and a mutex for data passing.
struct MouseScaffold::DeviceData
{
	/// Constructor
	/// \param device Device to be managed by this scaffold
	explicit DeviceData(MouseDevice* device) : deviceObject(device)
	{
		mouseHandler = new OsgMouseHandler();
	}

	/// Device object managed by this scaffold.
	MouseDevice* const deviceObject;
	/// Mouse Handler to communicate with underneath API.
	osg::ref_ptr<OsgMouseHandler> mouseHandler;
	/// The mutex that protects the externally modifiable parameters.
	boost::mutex mutex;

private:
	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	DeviceData(const DeviceData&) /*= delete*/;
	DeviceData& operator=(const DeviceData&) /*= delete*/;
};

MouseScaffold::MouseScaffold() : m_logger(Framework::Logger::getLogger("Devices/Mouse"))
{
	SURGSIM_LOG_DEBUG(m_logger) << "Shared scaffold created.";
}

MouseScaffold::~MouseScaffold()
{
	if (m_device != nullptr)
	{
		unregisterDevice();
	}
	SURGSIM_LOG_DEBUG(m_logger) << "Shared scaffold destroyed.";
}

bool MouseScaffold::registerDevice(MouseDevice* device)
{
	m_device.reset(new DeviceData(device));
	if (nullptr == m_device)
	{
		SURGSIM_LOG_CRITICAL(m_logger) << "Failed to create a DeviceData for " << device->getName();
		return false;
	}
	SURGSIM_LOG_INFO(m_logger) << "Device " << device->getName() << " registered.";
	return true;
}

bool MouseScaffold::unregisterDevice()
{
	m_device.reset();
	SURGSIM_LOG_DEBUG(m_logger) << "Unregistered device.";
	return true;
}

bool MouseScaffold::updateDevice(int buttonMask, float x, float y, int scrollDeltaX, int scrollDeltaY)
{
	boost::lock_guard<boost::mutex> lock(m_device->mutex);
	SurgSim::DataStructures::DataGroup& inputData = m_device->deviceObject->getInputData();
	inputData.booleans().set(SurgSim::DataStructures::Names::BUTTON_1,
		(buttonMask & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) != 0);
	inputData.booleans().set(SurgSim::DataStructures::Names::BUTTON_2,
		(buttonMask & osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON) != 0);
	inputData.booleans().set(SurgSim::DataStructures::Names::BUTTON_3,
		(buttonMask & osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON) != 0);
	inputData.scalars().set("x", static_cast<double>(x));
	inputData.scalars().set("y", static_cast<double>(y));
	inputData.integers().set("scrollDeltaX", scrollDeltaX);
	inputData.integers().set("scrollDeltaY", scrollDeltaY);

	m_device->deviceObject->pushInput();
	return true;
}

OsgMouseHandler* MouseScaffold::getMouseHandler() const
{
	return m_device->mouseHandler.get();
}

/// Builds the data layout for the application input (i.e. device output).
SurgSim::DataStructures::DataGroup MouseScaffold::buildDeviceInputData()
{
	DataGroupBuilder builder;
	builder.addBoolean(SurgSim::DataStructures::Names::BUTTON_1);	// Indicates mouse left button
	builder.addBoolean(SurgSim::DataStructures::Names::BUTTON_2); 	// Indicates mouse middle button (i.e. wheel)
	builder.addBoolean(SurgSim::DataStructures::Names::BUTTON_3);	// Indicates mouse right button
	builder.addScalar("x");				// Indicates mouse's X-coordinate in the current window, left bottom = (0, 0)
	builder.addScalar("y");				// Indicates mouse's Y-coordinate in the current window, left bottom = (0, 0)
	builder.addInteger("scrollDeltaX"); // Indicates mouse wheel vertical movement
	builder.addInteger("scrollDeltaY"); // Indicates mouse wheel horizontal movement

	return builder.createData();
}

std::shared_ptr<MouseScaffold> MouseScaffold::getOrCreateSharedInstance()
{
	static SurgSim::Framework::SharedInstance<MouseScaffold> sharedInstance;
	return sharedInstance.get();
}

};  // namespace Devices
};  // namespace SurgSim