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

FilteredDevice::FilteredDevice(const std::string& name) :
	m_name(name),
	m_initialized(false),
	m_logger(Framework::Logger::getLogger("Devices/FilteredDevice"))
{
	m_devices.push_back(nullptr);
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
	SURGSIM_ASSERT(!m_initialized) << "Already initialized.";
	boost::unique_lock<boost::shared_mutex>(m_deviceMutex);
	SURGSIM_ASSERT(m_devices.size() > 0) << "There must be at least one device.";

	bool result = true;
	if (m_devices[0] == nullptr)
	{
		SURGSIM_LOG_WARNING(m_logger) << "A raw/base device is required for " << getName();
		result = false;
	}

	if (result)
	{
		for (size_t i = 0; i < m_devices.size() - 1; ++i)
		{
			auto deviceFilter = std::dynamic_pointer_cast<DeviceFilter>(m_devices[i + 1]);
			SURGSIM_ASSERT(deviceFilter != nullptr) << "Expected a device filter.";
			result = result && m_devices[i]->addInputConsumer(deviceFilter) &&
				m_devices[i]->setOutputProducer(deviceFilter);
		}

		for (auto& device : m_devices)
		{
			result = result && device->initialize();
		}
	}

	m_initialized = result;
	SURGSIM_LOG_IF(!result, m_logger, WARNING) << "Failed to initialize " << getName();
	return result;
}

bool FilteredDevice::addInputConsumer(std::shared_ptr<InputConsumerInterface> inputConsumer)
{
	boost::shared_lock<boost::shared_mutex>(m_deviceMutex);
	SURGSIM_ASSERT(m_devices.size() > 0) << "There must be at least one device.";
	bool result = false;
	if (m_devices.back() == nullptr)
	{
		SURGSIM_LOG_WARNING(m_logger) <<
			"Cannot addInputConsumer on " << getName() << " before setting the device and/or adding a filter.";
	}
	else
	{
		result = m_devices.back()->addInputConsumer(inputConsumer);
	}
	return result;
}

bool FilteredDevice::removeInputConsumer(std::shared_ptr<InputConsumerInterface> inputConsumer)
{
	boost::shared_lock<boost::shared_mutex>(m_deviceMutex);
	SURGSIM_ASSERT(m_devices.size() > 0) << "There must be at least one device.";
	bool result = false;
	if (m_devices.back() == nullptr)
	{
		SURGSIM_LOG_WARNING(m_logger) <<
			getName() << " does not have a device or filter from which to removeInputConsumer.";
	}
	else
	{
		result = m_devices.back()->removeInputConsumer(inputConsumer);
	}
	return result;
}

void FilteredDevice::clearInputConsumers()
{
	boost::shared_lock<boost::shared_mutex>(m_deviceMutex);
	SURGSIM_ASSERT(m_devices.size() > 0) << "There must be at least one device.";
	if (m_devices.back() != nullptr)
	{
		m_devices.back()->clearInputConsumers();
	}
}

bool FilteredDevice::setOutputProducer(std::shared_ptr<OutputProducerInterface> outputProducer)
{
	boost::shared_lock<boost::shared_mutex>(m_deviceMutex);
	SURGSIM_ASSERT(m_devices.size() > 0) << "There must be at least one device.";
	bool result = false;
	if (m_devices.back() == nullptr)
	{
		SURGSIM_LOG_WARNING(m_logger) <<
			"Cannot setOutputProducer on " << getName() << " before setting the device and/or adding a filter.";
	}
	else
	{
		result = m_devices.back()->setOutputProducer(outputProducer);
	}
	return result;
}

bool FilteredDevice::removeOutputProducer(std::shared_ptr<OutputProducerInterface> outputProducer)
{
	boost::shared_lock<boost::shared_mutex>(m_deviceMutex);
	SURGSIM_ASSERT(m_devices.size() > 0) << "There must be at least one device.";
	bool result = false;
	if (m_devices.back() == nullptr)
	{
		SURGSIM_LOG_WARNING(m_logger) <<
			getName() << " does not have a device or filter from which to removeOutputProducer.";
	}
	else
	{
		result = m_devices.back()->removeOutputProducer(outputProducer);
	}
	return result;
}

bool FilteredDevice::hasOutputProducer()
{
	boost::shared_lock<boost::shared_mutex>(m_deviceMutex);
	SURGSIM_ASSERT(m_devices.size() > 0) << "There must be at least one device.";
	return ((m_devices.back() != nullptr) && m_devices.back()->hasOutputProducer());
}

void FilteredDevice::clearOutputProducer()
{
	boost::shared_lock<boost::shared_mutex>(m_deviceMutex);
	SURGSIM_ASSERT(m_devices.size() > 0) << "There must be at least one device.";
	if (m_devices.back() != nullptr)
	{
		m_devices.back()->clearOutputProducer();
	}
}

void FilteredDevice::setDevice(std::shared_ptr<Input::DeviceInterface> device)
{
	SURGSIM_ASSERT(!m_initialized) << "Cannot set device after initialization";
	SURGSIM_ASSERT(device != nullptr) << "Cannot set a nullptr device.";
	boost::unique_lock<boost::shared_mutex>(m_deviceMutex);
	m_devices[0] = device;
}

void FilteredDevice::addFilter(std::shared_ptr<DeviceFilter> filter)
{
	SURGSIM_ASSERT(!m_initialized) << "Cannot add filter after initialization";
	SURGSIM_ASSERT(filter != nullptr) << "Cannot add a nullptr filter.";
	boost::unique_lock<boost::shared_mutex>(m_deviceMutex);
	m_devices.push_back(filter);
}

const std::vector<std::shared_ptr<Input::DeviceInterface>>& FilteredDevice::getDevices() const
{
	boost::shared_lock<boost::shared_mutex>(m_deviceMutex);
	return m_devices;
}

bool FilteredDevice::setDevices(const std::vector<std::shared_ptr<Input::DeviceInterface>>& devices)
{
	SURGSIM_ASSERT(!m_initialized) << "Cannot set devices after initialization";
	bool result = true;
	boost::unique_lock<boost::shared_mutex>(m_deviceMutex);
	if (devices.size() > 0)
	{
		for (const auto& device : devices)
		{
			if (device == nullptr)
			{
				SURGSIM_LOG_WARNING(m_logger) << "Cannot set a nullptr device on " << getName();
				result = false;
			}
			else
			{
				if ((device != devices[0]) && (std::dynamic_pointer_cast<DeviceFilter>(device) == nullptr))
				{
					SURGSIM_LOG_WARNING(m_logger) << getName() << " setDevices got a " << device->getClassName() <<
						" named " << device->getName() << " where a DeviceFilter is required.";
					result = false;
				}
			}
		}
	}
	else
	{
		SURGSIM_LOG_WARNING(m_logger) << "Cannot set 0 devices on " << getName();
		result = false;
	}

	if (result)
	{
		m_devices = devices;
	}
	return result;
}

};  // namespace Devices
};  // namespace SurgSim
