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

#include "SurgSim/Devices/Sixense/SixenseDevice.h"

#include "SurgSim/Devices/Sixense/SixenseScaffold.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/Assert.h"


namespace SurgSim
{
namespace Device
{

SURGSIM_REGISTER(SurgSim::Input::DeviceInterface, SurgSim::Device::SixenseDevice, SixenseDevice);

SixenseDevice::SixenseDevice(const std::string& uniqueName) :
	SurgSim::Input::CommonDevice(uniqueName, SixenseScaffold::buildDeviceInputData())
{
}


SixenseDevice::~SixenseDevice()
{
	if (isInitialized())
	{
		finalize();
	}
}


bool SixenseDevice::initialize()
{
	SURGSIM_ASSERT(! isInitialized());
	std::shared_ptr<SixenseScaffold> scaffold = SixenseScaffold::getOrCreateSharedInstance();
	SURGSIM_ASSERT(scaffold);

	if (! scaffold->registerDevice(this))
	{
		return false;
	}

	m_scaffold = std::move(scaffold);
	SURGSIM_LOG_INFO(m_scaffold->getLogger()) << "Device " << getName() << ": " << "Initialized.";
	return true;
}

bool SixenseDevice::finalize()
{
	SURGSIM_ASSERT(isInitialized());
	SURGSIM_LOG_INFO(m_scaffold->getLogger()) << "Device " << getName() << ": " << "Finalizing.";
	bool ok = m_scaffold->unregisterDevice(this);
	m_scaffold.reset();
	return ok;
}


bool SixenseDevice::isInitialized() const
{
	return (m_scaffold != nullptr);
}


};  // namespace Device
};  // namespace SurgSim
