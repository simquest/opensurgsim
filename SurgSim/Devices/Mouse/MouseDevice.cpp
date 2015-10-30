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

#include "SurgSim/Devices/Mouse/MouseDevice.h"
#include "SurgSim/Devices/Mouse/MouseScaffold.h"
#include "SurgSim/Devices/Mouse/OsgMouseHandler.h"
#include "SurgSim/Framework/Log.h"

namespace SurgSim
{
namespace Devices
{

SURGSIM_REGISTER(SurgSim::Input::DeviceInterface, SurgSim::Devices::MouseDevice, MouseDevice);

MouseDevice::MouseDevice(const std::string& deviceName) :
	SurgSim::Input::CommonDevice(deviceName, MouseScaffold::buildDeviceInputData())
{
}

MouseDevice::~MouseDevice()
{
	if (isInitialized())
	{
		finalize();
	}
}

bool MouseDevice::initialize()
{
	SURGSIM_ASSERT(!isInitialized()) << getName() << " already initialized.";
	auto scaffold = MouseScaffold::getOrCreateSharedInstance();
	SURGSIM_ASSERT(scaffold != nullptr);
	bool registered = false;
	if (scaffold->registerDevice(this))
	{
		m_scaffold = std::move(scaffold);
		registered = true;
	}
	return registered;
}

bool MouseDevice::finalize()
{
	SURGSIM_ASSERT(isInitialized()) << getName() << " is not initialized, cannot finalize.";
	bool unregistered = m_scaffold->unregisterDevice();
	m_scaffold.reset();
	return unregistered;
}

bool MouseDevice::isInitialized() const
{
	return (m_scaffold != nullptr);
}

OsgMouseHandler* MouseDevice::getMouseHandler() const
{
	return m_scaffold->getMouseHandler();
}


};  // namespace Devices
};  // namespace SurgSim
