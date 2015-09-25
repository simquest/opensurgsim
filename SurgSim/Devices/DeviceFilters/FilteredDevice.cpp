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

#include "SurgSim/Devices/DeviceFilters/FilteredDevice.h"

#include "SurgSim/Devices/DeviceFilters/DeviceFilter.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Input/InputConsumerInterface.h"
#include "SurgSim/Input/OutputProducerInterface.h"

using SurgSim::Input::InputConsumerInterface;
using SurgSim::Input::OutputProducerInterface;

namespace SurgSim
{
namespace Devices
{

SURGSIM_REGISTER(SurgSim::Input::DeviceInterface, SurgSim::Devices::FilteredDevice, FilteredDevice);

FilteredDevice::FilteredDevice(const std::string& name) : m_name(name)
{
}

FilteredDevice::~FilteredDevice()
{
	finalize();
}

bool FilteredDevice::finalize()
{
	for (auto& device : m_devices)
	{
		device->clearInputConsumers();
		device->clearOutputProducer();
	}
	return true;
}

std::string FilteredDevice::getName() const
{
	return m_name;
}

bool FilteredDevice::initialize()
{
	boost::unique_lock<boost::shared_mutex>(m_deviceMutex);
	bool result = true;
	auto logger = Framework::Logger::getLogger("Devices/FilteredDevice");

	if (m_devices.size() == 0)
	{
		SURGSIM_LOG_WARNING(logger) << "At least one device is required.";
		result = false;
	}

	if (result)
	{
		for (size_t i = 0; i < m_devices.size() - 1; ++i)
		{
			auto deviceFilter = std::dynamic_pointer_cast<DeviceFilter>(m_devices[i + 1]);
			if (deviceFilter == nullptr)
			{
				result = false;
				SURGSIM_LOG_SEVERE(logger) << getName() <<
					" contains a device that is not a DeviceFilter and was not the first device added.";
			}
			else
			{
				result &= m_devices[i]->addInputConsumer(deviceFilter) &&
					m_devices[i]->setOutputProducer(deviceFilter);
			}
		}

		for (auto& device : m_devices)
		{
			result &= device->initialize();
		}
	}

	SURGSIM_LOG_IF(!result, logger, WARNING) << "Failed to initialize.";
	return result;
}

bool FilteredDevice::addInputConsumer(std::shared_ptr<InputConsumerInterface> inputConsumer)
{
	boost::shared_lock<boost::shared_mutex>(m_deviceMutex);
	return m_devices.back()->addInputConsumer(inputConsumer);
}

bool FilteredDevice::removeInputConsumer(std::shared_ptr<InputConsumerInterface> inputConsumer)
{
	boost::shared_lock<boost::shared_mutex>(m_deviceMutex);
	return m_devices.back()->removeInputConsumer(inputConsumer);
}

void FilteredDevice::clearInputConsumers()
{
	boost::shared_lock<boost::shared_mutex>(m_deviceMutex);
	m_devices.back()->clearInputConsumers();
}

bool FilteredDevice::setOutputProducer(std::shared_ptr<OutputProducerInterface> outputProducer)
{
	boost::shared_lock<boost::shared_mutex>(m_deviceMutex);
	return m_devices.back()->setOutputProducer(outputProducer);
}

bool FilteredDevice::removeOutputProducer(std::shared_ptr<OutputProducerInterface> outputProducer)
{
	boost::shared_lock<boost::shared_mutex>(m_deviceMutex);
	return m_devices.back()->removeOutputProducer(outputProducer);
}

bool FilteredDevice::hasOutputProducer()
{
	boost::shared_lock<boost::shared_mutex>(m_deviceMutex);
	return m_devices.back()->hasOutputProducer();
}

void FilteredDevice::clearOutputProducer()
{
	boost::shared_lock<boost::shared_mutex>(m_deviceMutex);
	m_devices.back()->clearOutputProducer();
}

bool FilteredDevice::addDevice(std::shared_ptr<Input::DeviceInterface> device)
{
	SURGSIM_ASSERT(device != nullptr) << "device cannot be nullptr";
	bool result = true;
	if ((m_devices.size() > 0) && (std::dynamic_pointer_cast<DeviceFilter>(device) == nullptr))
	{
		result = false;
		SURGSIM_LOG_SEVERE(Framework::Logger::getLogger("Devices")) <<
			"Any device added after the first must be a DeviceFilter.";
	}
	if (result)
	{
		boost::unique_lock<boost::shared_mutex>(m_deviceMutex);
		m_devices.push_back(device);
	}
	return result;
}

};  // namespace Devices
};  // namespace SurgSim
