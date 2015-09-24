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


namespace SurgSim
{
namespace Devices
{

SURGSIM_REGISTER(SurgSim::Input::DeviceInterface, SurgSim::Devices::SixenseDevice, SixenseDevice);

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
	SURGSIM_ASSERT(!isInitialized());
	auto scaffold = SixenseScaffold::getOrCreateSharedInstance();
	SURGSIM_ASSERT(scaffold != nullptr);

	bool initialize = false;
	if (scaffold->registerDevice(this))
	{
		m_scaffold = std::move(scaffold);
		initialize = true;
	}
	return initialize;
}

bool SixenseDevice::finalize()
{
	SURGSIM_ASSERT(isInitialized());
	bool ok = m_scaffold->unregisterDevice(this);
	m_scaffold.reset();
	return ok;
}

bool SixenseDevice::isInitialized() const
{
	return (m_scaffold != nullptr);
}


};  // namespace Devices
};  // namespace SurgSim
