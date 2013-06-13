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

#include <iostream>
#include <iomanip>

#include <sixense.h>

#include "SurgSim/Devices/Sixense/SixenseManager.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/Assert.h"


namespace SurgSim
{
namespace Device
{


SixenseDevice::SixenseDevice(const std::string& uniqueName) :
	SurgSim::Input::CommonDevice(uniqueName, SixenseManager::buildDeviceInputData())
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
	std::shared_ptr<SixenseManager> manager = SixenseManager::getOrCreateSharedInstance();
	SURGSIM_ASSERT(manager);

	if (! manager->registerDevice(this))
	{
		return false;
	}

	m_manager = std::move(manager);
	SURGSIM_LOG_INFO(m_manager->getLogger()) << "Device " << getName() << ": " << "Initialized.";
	return true;
}

bool SixenseDevice::finalize()
{
	SURGSIM_ASSERT(isInitialized());
	SURGSIM_LOG_DEBUG(m_manager->getLogger()) << "Device " << getName() << ": " << "Finalizing.";
	bool ok = m_manager->unregisterDevice(this);
	m_manager.reset();
	return ok;
}


bool SixenseDevice::isInitialized() const
{
	return (m_manager != nullptr);
}


};  // namespace Device
};  // namespace SurgSim
