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

#include "SurgSim/Devices/Nimble/NimbleDevice.h"

#include "SurgSim/Devices/Nimble/NimbleScaffold.h"
#include "SurgSim/Framework/Log.h"


namespace SurgSim
{
namespace Device
{

SURGSIM_REGISTER(SurgSim::Input::DeviceInterface, SurgSim::Device::NimbleDevice, NimbleDevice);

NimbleDevice::NimbleDevice(const std::string& uniqueName) :
	SurgSim::Input::CommonDevice(uniqueName, NimbleScaffold::buildDeviceInputData()), m_trackedHandDataIndex(0)
{
}

NimbleDevice::~NimbleDevice()
{
	if (isInitialized())
	{
		finalize();
	}
}

void NimbleDevice::setupToTrackLeftHand()
{
	m_trackedHandDataIndex = 0;
}

void NimbleDevice::setupToTrackRightHand()
{
	m_trackedHandDataIndex = 1;
}

bool NimbleDevice::initialize()
{
	SURGSIM_ASSERT(!isInitialized()) << "Nimble: Attempt to initialize already initialized device.";
	m_scaffold = NimbleScaffold::getOrCreateSharedInstance();
	SURGSIM_ASSERT(m_scaffold != nullptr) << "Unable to acquire a NimbleScaffold instance";

	if (!m_scaffold->registerDevice(this))
	{
		return false;
	}

	SURGSIM_LOG_INFO(m_scaffold->getLogger()) << "Device " << getName() << ": " << "Initialized.";
	return true;
}

bool NimbleDevice::finalize()
{
	SURGSIM_ASSERT(isInitialized()) << "Nimble: Attempt to finalize an uninitialized device.";
	SURGSIM_LOG_INFO(m_scaffold->getLogger()) << "Device " << getName() << ": " << "Finalizing.";

	bool ok = m_scaffold->unregisterDevice(this);
	SURGSIM_LOG_INFO(m_scaffold->getLogger()) << "Device " << getName() << ": " << "Finalized.";
	m_scaffold.reset();
	return ok;
}

bool NimbleDevice::isInitialized() const
{
	return (m_scaffold != nullptr);
}

};  // namespace Device
};  // namespace SurgSim
