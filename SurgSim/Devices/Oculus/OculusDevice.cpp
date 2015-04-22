// This file is a part of the OpenSurgSim project.
// Copyright 2015, SimQuest Solutions Inc.
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

#include "SurgSim/Devices/Oculus/OculusDevice.h"

#include "SurgSim/Devices/Oculus/OculusScaffold.h"

namespace SurgSim 
{
namespace Device
{

OculusDevice::OculusDevice(const std::string& name) :
	SurgSim::Input::CommonDevice(name, OculusScaffold::buildDeviceInputData())
{
}

OculusDevice::~OculusDevice()
{
	if (isInitialized())
	{
		finalize();
	}
}


bool OculusDevice::initialize()
{
	SURGSIM_ASSERT(!isInitialized()) << getName() << " is already initialized, cannot initialize again.";

	m_scaffold = OculusScaffold::getOrCreateSharedInstance();
	SURGSIM_ASSERT(m_scaffold != nullptr) << "OculusDevice::initialize(): Failed to obtain an Oculus scaffold.";

	return m_scaffold->registerDevice(this);
}

bool OculusDevice::finalize()
{
	SURGSIM_ASSERT(isInitialized()) << getName() << " is not initialized, cannot finalized.";

	bool result = m_scaffold->unregisterDevice(this);
	m_scaffold.reset();

	return result;
}

bool OculusDevice::isInitialized() const
{
	return (m_scaffold != nullptr);
}

};  // namespace Device
};  // namespace SurgSim
