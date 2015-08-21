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

#include "SurgSim/Input/DeviceInterface.h"

#include "SurgSim/Framework/Log.h"

namespace SurgSim
{
namespace Input
{

DeviceInterface::~DeviceInterface()
{
}

std::shared_ptr<DeviceInterface> DeviceInterface::createDevice(const std::string& deviceName,
	std::vector<std::string> types)
{
	std::shared_ptr<DeviceInterface> device;
	auto& factory = getFactory();
	for (const auto& type : types)
	{
		std::string qualifiedType = "SurgSim::Device::" + type;
		if (factory.isRegistered(qualifiedType))
		{
			SURGSIM_LOG_INFO(Framework::Logger::getLogger("Devices")) << "Trying to create a " << type;
			device = factory.create(qualifiedType, deviceName);
			if (device->initialize())
			{
				break;
			}
			else
			{
				device = nullptr;
			}
		}
		else
		{
			SURGSIM_LOG_INFO(Framework::Logger::getLogger("Devices")) << "Cannot create a " << type <<
				" because the executable was built without support for that device.  To use such a device, enable " <<
				"the BUILD_DEVICE_* setting in cmake.";
		}
	}
	return device;
}

}; // namespace Input
}; // namespace SurgSim

