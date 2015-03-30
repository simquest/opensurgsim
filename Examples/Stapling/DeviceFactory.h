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

#ifndef EXAMPLES_STAPLING_DEVICEFACTORY_H
#define EXAMPLES_STAPLING_DEVICEFACTORY_H

#include <memory>
#include <string>

#include "SurgSim/Input/DeviceInterface.h"

/// A class that creates an instance of a suitable subclass of DeviceInterface.
///
/// The device created depends on the compiler flags (from CMake) and success/failure of device initialization attempts.
class DeviceFactory
{
public:
	/// Constructor.
	DeviceFactory();

	/// Destructor.
	~DeviceFactory();

	/// Creates a device.
	/// \param name The name passed to the device.
	/// \return A shared pointer to an instance of a subclass of DeviceInterface, or nullptr on failure.
	std::shared_ptr<SurgSim::Input::DeviceInterface> getDevice(const std::string& name);
};


#endif  // EXAMPLES_STAPLING_DEVICEFACTORY_H
