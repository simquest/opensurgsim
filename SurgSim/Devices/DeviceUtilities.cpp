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

#include <yaml-cpp/yaml.h>

#include "SurgSim/Input/DeviceInterface.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/Runtime.h"

using SurgSim::Input::DeviceInterface;

namespace
{
const std::string NamePropertyName = "Name";

/// Convert node to device
/// \param fileName the original filename for error reporting
/// \param node the node to be converted
/// \return the device if it was created and initialized, otherwise nullptr
std::shared_ptr<DeviceInterface> tryConvertDevice(const YAML::Node& possibleDevice, const std::string& fileName)
{
	auto logger = SurgSim::Framework::Logger::getLogger("Devices");
	std::shared_ptr<DeviceInterface> device;
	if (possibleDevice.IsMap())
	{
		std::string className = possibleDevice.begin()->first.as<std::string>();
		if (DeviceInterface::getFactory().isRegistered(className))
		{
			const YAML::Node& data = possibleDevice.begin()->second;
			if (data.IsMap() && data[NamePropertyName].IsDefined())
			{
				SURGSIM_LOG_DEBUG(logger) << "Loading: " << std::endl << possibleDevice;
				std::string name = data[NamePropertyName].as<std::string>();
				device = DeviceInterface::getFactory().create(className, name);
				std::vector<std::string> ignoredProperties;
				ignoredProperties.push_back(NamePropertyName);
				device->decode(data, ignoredProperties);
				if (device->initialize())
				{
					SURGSIM_LOG_INFO(logger) << "Successfully loaded " << className << " named " << name;
				}
				else
				{
					device = nullptr;
					SURGSIM_LOG_INFO(logger) << "Failed to initialize " << className << " named " << name;
				}
			}
			else
			{
				SURGSIM_LOG_WARNING(logger) << "File " << fileName << " contains an entry for class " <<
					className << " that is not a map or does not have a Name property:" << std::endl << possibleDevice;
			}
		}
		else
		{
			SURGSIM_LOG_INFO(logger) << "Class " << className << " is not registered in the factory.";
		}
	}
	else
	{
		SURGSIM_LOG_WARNING(logger) <<
			"File " << fileName << " contains a non-map where a map is expected:" << std::endl << possibleDevice;
	}
	return device;
}
}

namespace SurgSim
{
namespace Devices
{
	
std::shared_ptr<DeviceInterface> createDevice(const std::vector<std::string>& classNames,
		const std::string& name)
{
	std::shared_ptr<DeviceInterface> device;
	auto& factory = DeviceInterface::getFactory();
	for (const auto& className : classNames)
	{
		if (factory.isRegistered(className))
		{
			SURGSIM_LOG_INFO(Framework::Logger::getLogger("Devices")) << "Trying to create a " << className;
			device = factory.create(className, name);
			if (device->initialize())
			{
				SURGSIM_LOG_INFO(Framework::Logger::getLogger("Devices")) << "Successfully created a " << className;
				break;
			}
			else
			{
				device = nullptr;
				SURGSIM_LOG_INFO(Framework::Logger::getLogger("Devices")) << "Failed to initialize a " << className;
			}
		}
		else
		{
			SURGSIM_LOG_INFO(Framework::Logger::getLogger("Devices")) << className <<
					" is not registered in the Devices factory.";
		}
	}
	return device;
}

std::shared_ptr<Input::DeviceInterface> createDevice(const std::string& className, const std::string& name)
{
	std::vector<std::string> names;
	names.push_back(className);
	return createDevice(names, name);
}

std::shared_ptr<DeviceInterface> loadDevice(const std::string& fileName)
{
	SURGSIM_LOG_INFO(Framework::Logger::getLogger("Devices")) << "Adding a device from " << fileName;
	YAML::Node node;
	SURGSIM_ASSERT(Framework::tryLoadNode(fileName, &node)) <<
		"Could not load a device from the YAML file: " << fileName;

	std::shared_ptr<DeviceInterface> device;
	if (node.IsSequence())
	{
		for (const auto& possibleDevice : node)
		{
			device = tryConvertDevice(possibleDevice, fileName);
			if (device != nullptr)
			{
				break;
			}
		}
	}
	else
	{
		SURGSIM_LOG_SEVERE(Framework::Logger::getLogger("Devices")) <<
			"File " << fileName << " not a YAML sequence; cannot load device:" << std::endl << node;
	}
	return device;
}

}; // namespace Devices
}; // namespace SurgSim

