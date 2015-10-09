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
	SURGSIM_ASSERT(!isInitialized());

	m_scaffold = MouseScaffold::getOrCreateSharedInstance();
	SURGSIM_ASSERT(m_scaffold);

	m_scaffold->registerDevice(this);
	SURGSIM_LOG_INFO(m_scaffold->getLogger()) << "Device " << getName() << ": " << "Initialized.";

	return true;
}

bool MouseDevice::finalize()
{
	SURGSIM_ASSERT(isInitialized());
	SURGSIM_LOG_INFO(m_scaffold->getLogger()) << "Device " << getName() << ": " << "Finalizing.";
	m_scaffold.reset();
	return true;
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
