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

#ifndef SURGSIM_DEVICES_MOUSE_MOUSESCAFFOLD_H
#define SURGSIM_DEVICES_MOUSE_MOUSESCAFFOLD_H

#include <memory>

#include "SurgSim/Framework/Logger.h"

namespace SurgSim
{

namespace DataStructures
{
class DataGroup;
}

namespace Device
{
class MouseDevice;
class OsgMouseHandler;

/// A class that implements the behavior of MouseDevice objects.
/// \sa SurgSim::Device::MouseDevice
class MouseScaffold
{
	friend class MouseDevice;
	friend class MouseDeviceTest;
	friend class OsgMouseHandler;

public:
	/// Constructor.
	/// \param logger (optional) The logger to be used by the scaffold object and the devices it manages.
	/// If unspecified or empty, a console logger will be created and used.
	explicit MouseScaffold(std::shared_ptr<SurgSim::Framework::Logger> logger = nullptr);
	/// Destructor
	~MouseScaffold();

	/// Gets the logger used by this object and the devices it manages.
	/// \return The logger.
	std::shared_ptr<SurgSim::Framework::Logger> getLogger() const;

	/// Gets or creates the scaffold shared by all MouseDevice instances.
	/// The scaffold is managed using a SharedInstance object, so it will be destroyed when all devices are released.
	/// \return the scaffold object.
	static std::shared_ptr<MouseScaffold> getOrCreateSharedInstance();

	/// Sets the default log level.
	/// Has no effect unless called before a scaffold is created (i.e. before the first device).
	/// \param logLevel The log level.
	static void setDefaultLogLevel(SurgSim::Framework::LogLevel logLevel);

private:
	/// Internal per-device information.
	struct DeviceData;

	/// Registers the specified device object.
	/// If successful, the device object will become connected to an hardware device.
	/// \param device The device object to be used, which should have a unique name.
	/// \return True if the initialization succeeds, false if it fails.
	bool registerDevice(MouseDevice* device);
	/// Unregisters the specified device object.
	/// The corresponding controller will become unused, and can be re-registered later.
	/// \return True on success, false on failure.
	bool unregisterDevice();

	/// Updates the device information for a single device.
	/// \param buttons Buttons being pressed.
	/// \param x X-Coordinate for the device.
	/// \param y Y-Coordinate for the device.
	/// \param scrollDeltaX Horizontal indicator for mouse wheel.
	/// \param scrollDeltaY Vertical indicator for mouse wheel.
	/// \return	True on success.
	bool updateDevice(int buttons, float x, float y, int scrollDeltaX, int scrollDeltaY);

	/// Get mouse handler
	/// \return The mouse handler associated with this device
	OsgMouseHandler* getMouseHandler() const;

	/// Builds the data layout for the application input (i.e. device output).
	static SurgSim::DataStructures::DataGroup buildDeviceInputData();


	/// Logger used by the scaffold and all devices.
	std::shared_ptr<SurgSim::Framework::Logger> m_logger;
	/// The default logging level.
	static SurgSim::Framework::LogLevel m_defaultLogLevel;
	/// The mouse device managed by this scaffold
	std::unique_ptr<DeviceData> m_device;
};

};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_MOUSE_MOUSESCAFFOLD_H
