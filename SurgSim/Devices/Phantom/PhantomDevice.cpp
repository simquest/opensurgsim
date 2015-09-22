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

#include "SurgSim/Devices/Phantom/PhantomDevice.h"

#include "SurgSim/Devices/Phantom/PhantomScaffold.h"
#include "SurgSim/Framework/Log.h"

namespace SurgSim
{
namespace Devices
{

SURGSIM_REGISTER(SurgSim::Input::DeviceInterface, SurgSim::Devices::PhantomDevice, PhantomDevice);

PhantomDevice::PhantomDevice(const std::string& uniqueName) :
	SurgSim::Input::CommonDevice(uniqueName, PhantomScaffold::buildDeviceInputData())
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(PhantomDevice, std::string, InitializationName,
		getInitializationName, setInitializationName);
}


PhantomDevice::~PhantomDevice()
{
	if (isInitialized())
	{
		finalize();
	}
}

void PhantomDevice::setInitializationName(const std::string& initializationName)
{
	m_initializationName = initializationName;
}

std::string PhantomDevice::getInitializationName() const
{
	return m_initializationName;
}


bool PhantomDevice::initialize()
{
	SURGSIM_ASSERT(! isInitialized());
	std::shared_ptr<PhantomScaffold> scaffold = PhantomScaffold::getOrCreateSharedInstance();
	SURGSIM_ASSERT(scaffold);

	if (! scaffold->registerDevice(this))
	{
		return false;
	}

	m_scaffold = std::move(scaffold);
	SURGSIM_LOG_INFO(m_scaffold->getLogger()) << "Device " << getName() << ": " << "Initialized.";
	return true;
}


bool PhantomDevice::finalize()
{
	SURGSIM_ASSERT(isInitialized());
	SURGSIM_LOG_INFO(m_scaffold->getLogger()) << "Device " << getName() << ": " << "Finalizing.";
	bool ok = m_scaffold->unregisterDevice(this);
	m_scaffold.reset();
	return ok;
}


bool PhantomDevice::isInitialized() const
{
	return (m_scaffold != nullptr);
}


};  // namespace Devices
};  // namespace SurgSim
