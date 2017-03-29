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

#ifndef SURGSIM_DEVICES_DEVICEFILTERS_DEVICEFILTER_H
#define SURGSIM_DEVICES_DEVICEFILTERS_DEVICEFILTER_H

#include <string>

#include "SurgSim/Input/CommonDevice.h"
#include "SurgSim/Input/InputConsumerInterface.h"
#include "SurgSim/Input/OutputProducerInterface.h"

namespace SurgSim
{
namespace DataStructures
{
class DataGroup;
}

namespace Devices
{

/// A device filter can be connected between a device and the InputConsumerInterface (e.g., InputComponent) and/or
/// the OutputProducerInterface (e.g., OutputComponent), and can alter the data being passed from/to the device.
class DeviceFilter :
	public Input::CommonDevice, public Input::InputConsumerInterface, public Input::OutputProducerInterface
{
public:
	/// Constructor.
	/// \param name	Name of this device filter.
	explicit DeviceFilter(const std::string& name);

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	SURGSIM_CLASSNAME(SurgSim::Devices::DeviceFilter);

	bool initialize() override;

	bool isInitialized() const override;

	void initializeInput(const std::string& device, const DataStructures::DataGroup& inputData) override;

	void handleInput(const std::string& device, const DataStructures::DataGroup& inputData) override;

	bool requestOutput(const std::string& device, DataStructures::DataGroup* outputData) override;

protected:
	/// Filter the input data.
	/// \param device The name of the device pushing the input data.
	/// \param dataToFilter The data that will be filtered.
	/// \param [in,out] result A pointer to a DataGroup object that must be assignable to by the dataToFilter object.
	///		Will contain the filtered data.
	virtual void filterInput(const std::string& device, const DataStructures::DataGroup& dataToFilter,
		DataStructures::DataGroup* result);

	/// Filter the output data.
	/// \param device The name of the device pulling the output data.
	/// \param dataToFilter The data that will be filtered.
	/// \param [in,out] result A pointer to a DataGroup object that must be assignable to by the dataToFilter object.
	///		Will contain the filtered data.
	virtual void filterOutput(const std::string& device, const DataStructures::DataGroup& dataToFilter,
		DataStructures::DataGroup* result);

protected:
	bool finalize() override;

	/// true if initialized and not finalized.
	bool m_initialized;
};

};  // namespace Devices
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_DEVICEFILTERS_DEVICEFILTER_H
