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

#ifndef SURGSIM_DEVICES_DEVICEFILTERS_FILTEREDDEVICE_H
#define SURGSIM_DEVICES_DEVICEFILTERS_FILTEREDDEVICE_H

#include <boost/thread/shared_mutex.hpp>
#include <memory>
#include <string>
#include <vector>

#include "SurgSim/Input/DeviceInterface.h"

namespace SurgSim
{
namespace Input
{
class InputConsumerInterface;
class OutputProducerInterface;
}

namespace Devices
{
class DeviceFilter;

/// A DeviceInterface connected in series with one or more DeviceFilters.  Useful for serialization.
class FilteredDevice : public Input::DeviceInterface
{
public:
	/// Constructor.
	/// \param name	Name of this device.
	explicit FilteredDevice(const std::string& name);

	SURGSIM_CLASSNAME(SurgSim::Devices::FilteredDevice);

	/// Destructor.
	virtual ~FilteredDevice();

	std::string getName() const override;
	bool initialize() override;
	bool addInputConsumer(std::shared_ptr<Input::InputConsumerInterface> inputConsumer) override;
	bool removeInputConsumer(std::shared_ptr<Input::InputConsumerInterface> inputConsumer) override;
	void clearInputConsumers() override;
	bool setOutputProducer(std::shared_ptr<Input::OutputProducerInterface> outputProducer) override;
	bool removeOutputProducer(std::shared_ptr<Input::OutputProducerInterface> outputProducer) override;
	bool hasOutputProducer() override;
	void clearOutputProducer() override;
	bool finalize() override;

	/// Adds a device.  The first device added should be the raw/base device. The next device added will be connected
	/// to the raw/base device.  The last device added will interface with InputComponents and/or an OutputComponent.
	/// Any filters added in-between will be connected in order.
	/// \param device A device or device filter.
	/// \return true if successful.
	bool addDevice(std::shared_ptr<Input::DeviceInterface> device);

private:
	/// The name of this device.
	std::string m_name;

	/// The devices.  m_devices.back() is the DeviceFilter (if there are any) to be connected to any
	/// InputComponent or OutputComponent.  m_devices.front() is the raw/base device to be filtered.
	std::vector<std::shared_ptr<Input::DeviceInterface>> m_devices;

	/// The mutex to protect access to the devices.
	boost::shared_mutex m_deviceMutex;
};

};  // namespace Devices
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_DEVICEFILTERS_FILTEREDDEVICE_H
