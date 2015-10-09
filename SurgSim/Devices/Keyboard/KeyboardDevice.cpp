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

#include "SurgSim/Devices/Keyboard/KeyboardDevice.h"

#include "SurgSim/Devices/Keyboard/KeyboardScaffold.h"
#include "SurgSim/Devices/Keyboard/OsgKeyboardHandler.h"
#include "SurgSim/Framework/Log.h"

namespace SurgSim
{
namespace Device
{

SURGSIM_REGISTER(SurgSim::Input::DeviceInterface, SurgSim::Device::KeyboardDevice, KeyboardDevice);

KeyboardDevice::KeyboardDevice(const std::string& deviceName) :
	SurgSim::Input::CommonDevice(deviceName, KeyboardScaffold::buildDeviceInputData())
{
}

KeyboardDevice::~KeyboardDevice()
{
	if (isInitialized())
	{
		finalize();
	}
}

bool KeyboardDevice::initialize()
{
	SURGSIM_ASSERT(!isInitialized());

	m_scaffold = KeyboardScaffold::getOrCreateSharedInstance();
	SURGSIM_ASSERT(m_scaffold);

	m_scaffold->registerDevice(this);
	SURGSIM_LOG_INFO(m_scaffold->getLogger()) << "Device " << getName() << ": " << "Initialized.";

	return true;
}

bool KeyboardDevice::finalize()
{
	SURGSIM_ASSERT(isInitialized());
	SURGSIM_LOG_INFO(m_scaffold->getLogger()) << "Device " << getName() << ": " << "Finalizing.";
	m_scaffold.reset();
	return true;
}

bool KeyboardDevice::isInitialized() const
{
	return (m_scaffold != nullptr);
}

OsgKeyboardHandler* KeyboardDevice::getKeyboardHandler() const
{
	return m_scaffold->getKeyboardHandler();
}


};  // namespace Device
};  // namespace SurgSim
