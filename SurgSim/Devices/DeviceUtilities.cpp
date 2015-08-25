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

#include "SurgSim/Devices/DeviceUtilities.h"

#include "SurgSim/Input/DeviceInterface.h"
#include "SurgSim/Framework/Log.h"


namespace SurgSim
{
namespace Device 
{

std::shared_ptr<Input::DeviceInterface> createDevice(const std::vector<std::string>& classNames,
		const std::string& name)
{
	std::shared_ptr<Input::DeviceInterface> device;
	auto& factory = Input::DeviceInterface::getFactory();
	for (const auto& className : classNames)
	{
		std::string qualifiedType = "SurgSim::Device::" + className;
		if (factory.isRegistered(qualifiedType))
		{
			SURGSIM_LOG_INFO(Framework::Logger::getLogger("Devices")) << "Trying to create a " << className;
			device = factory.create(qualifiedType, name);
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
			SURGSIM_LOG_INFO(Framework::Logger::getLogger("Devices")) << "Cannot create a " << className <<
				" because the executable was built without support for that device.  To use such a device, enable " <<
				"the BUILD_DEVICE_* setting in cmake.";
		}
	}
	return device;
}

}; // namespace Device
}; // namespace SurgSim

