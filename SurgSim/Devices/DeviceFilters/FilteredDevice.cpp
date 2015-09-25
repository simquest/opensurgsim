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

FilteredDevice::FilteredDevice(const std::string& name) : m_name(name), m_initialized(false)
{
}

FilteredDevice::~FilteredDevice()
{
	finalize();
}

bool FilteredDevice::finalize()
{
	m_device->clearInputConsumers();
	m_device->clearOutputProducer();
	for (auto& device : m_filters)
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
	SURGSIM_ASSERT(!m_initialized) << "Cannot initialize more than once.";
	bool result = true;
	auto logger = Framework::Logger::getLogger("Devices/FilteredDevice");

	if (m_device == nullptr)
	{
		SURGSIM_LOG_WARNING(logger) << "A raw/base device is required.";
		result = false;
	}
	if (m_filters.size() == 0)
	{
		SURGSIM_LOG_WARNING(logger) << "At least one filter is required.";
		result = false;
	}

	if (result)
	{
		result = m_device->addInputConsumer(m_filters.front()) && m_device->setOutputProducer(m_filters.front());
		for (size_t i = 0; i < m_filters.size() - 1; ++i)
		{
			result &= m_filters[i]->addInputConsumer(m_filters[i + 1]) &&
				m_filters[i]->setOutputProducer(m_filters[i + 1]);
		}

		for (auto& filter : m_filters)
		{
			result &= filter->initialize();
		}
		result &= m_device->initialize();
	}

	m_initialized = result;
	SURGSIM_LOG_IF(!result, logger, WARNING) << "Failed to initialize.";
	return result;
}

bool FilteredDevice::addInputConsumer(std::shared_ptr<InputConsumerInterface> inputConsumer)
{
	return m_filters.back()->addInputConsumer(inputConsumer);
}

bool FilteredDevice::removeInputConsumer(std::shared_ptr<InputConsumerInterface> inputConsumer)
{
	return m_filters.back()->removeInputConsumer(inputConsumer);
}

void FilteredDevice::clearInputConsumers()
{
	m_filters.back()->clearInputConsumers();
}

bool FilteredDevice::setOutputProducer(std::shared_ptr<OutputProducerInterface> outputProducer)
{
	return m_filters.back()->setOutputProducer(outputProducer);
}

bool FilteredDevice::removeOutputProducer(std::shared_ptr<OutputProducerInterface> outputProducer)
{
	return m_filters.back()->removeOutputProducer(outputProducer);
}

bool FilteredDevice::hasOutputProducer()
{
	return m_filters.back()->hasOutputProducer();
}

void FilteredDevice::clearOutputProducer()
{
	m_filters.back()->clearOutputProducer();
}

void FilteredDevice::setDevice(std::shared_ptr<Input::DeviceInterface> device)
{
	SURGSIM_ASSERT(!m_initialized) << "Cannot set device after initialization";
	SURGSIM_ASSERT(device != nullptr) << "Cannot set a nullptr device.";
	m_device = device;
}

void FilteredDevice::addFilter(std::shared_ptr<DeviceFilter> device)
{
	SURGSIM_ASSERT(!m_initialized) << "Cannot add filter after initialization";
	m_filters.push_back(device);
}

};  // namespace Devices
};  // namespace SurgSim
