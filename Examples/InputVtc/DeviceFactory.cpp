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

#include "Examples/InputVtc/DeviceFactory.h"

#include "SurgSim/Framework/Log.h"

#ifdef MULTIAXISDEVICE_LIBRARY_AVAILABLE
#include "SurgSim/Devices/MultiAxis/MultiAxisDevice.h"
#endif // MULTIAXISDEVICE_LIBRARY_AVAILABLE

#ifdef NOVINT_LIBRARY_AVAILABLE
#include "SurgSim/Devices/Novint/NovintDevice.h"
#endif // NOVINT_LIBRARY_AVAILABLE

using SurgSim::Framework::Logger;


DeviceFactory::DeviceFactory()
{
}
DeviceFactory::~DeviceFactory()
{
}

std::shared_ptr<SurgSim::Input::DeviceInterface> DeviceFactory::getDevice(const std::string& name)
{
	std::shared_ptr<Logger> logger = Logger::getDefaultLogger();

	// First check for a Falcon.  Did we build NovintDevice, and will the device initialize?
#ifdef NOVINT_LIBRARY_AVAILABLE
	SURGSIM_LOG_INFO(logger) << "DeviceFactory is going to try using a NovintDevice, the first available Falcon.";
	std::shared_ptr<SurgSim::Device::NovintDevice> novintDevice = std::make_shared<SurgSim::Device::NovintDevice>(name);
	novintDevice->setPositionScale(novintDevice->getPositionScale() * 10.0);

	if (novintDevice->initialize())
	{
		return novintDevice;
	}
	SURGSIM_LOG_WARNING(logger) << "Could not initialize the NovintDevice.";
#endif // NOVINT_LIBRARY_AVAILABLE

	// Then try MultiAxisDevice.
#ifdef MULTIAXISDEVICE_LIBRARY_AVAILABLE
	SURGSIM_LOG_INFO(logger) << "DeviceFactory is going to try using a MultiAxisDevice.";
	std::shared_ptr<SurgSim::Device::MultiAxisDevice> multiAxisDevice =
		std::make_shared<SurgSim::Device::MultiAxisDevice>(name);
	multiAxisDevice->setPositionScale(multiAxisDevice->getPositionScale() * 10.0);
	multiAxisDevice->setOrientationScale(multiAxisDevice->getOrientationScale() * 3.0);

	if (multiAxisDevice->initialize())
	{
		return multiAxisDevice;
	}
	SURGSIM_LOG_WARNING(logger) << "Could not initialize the MultiAxisDevice.";
#endif // MULTIAXISDEVICE_LIBRARY_AVAILABLE

	// failed to instantiate a device
	return nullptr;
}
