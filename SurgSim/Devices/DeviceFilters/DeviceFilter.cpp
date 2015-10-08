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

#include "SurgSim/Devices/DeviceFilters/DeviceFilter.h"

#include "SurgSim/DataStructures/DataGroup.h"

using SurgSim::DataStructures::DataGroup;

namespace SurgSim
{
namespace Devices
{

DeviceFilter::DeviceFilter(const std::string& name) : CommonDevice(name), m_initialized(false)
{
}

bool DeviceFilter::initialize()
{
	SURGSIM_ASSERT(!isInitialized());
	m_initialized = true;
	return true;
}

bool DeviceFilter::finalize()
{
	SURGSIM_ASSERT(isInitialized());
	m_initialized = false;
	return true;
}

bool DeviceFilter::isInitialized() const
{
	return m_initialized;
}

void DeviceFilter::initializeInput(const std::string& device, const DataGroup& inputData)
{
	filterInput(device, inputData, &getInputData());
}

void DeviceFilter::handleInput(const std::string& device, const DataGroup& inputData)
{
	filterInput(device, inputData, &getInputData());
	pushInput();
}

bool DeviceFilter::requestOutput(const std::string& device, DataGroup* outputData)
{
	bool state = pullOutput();
	if (state)
	{
		filterOutput(device, getOutputData(), outputData);
	}
	return state;
}

void DeviceFilter::filterInput(const std::string& device, const DataGroup& dataToFilter, DataGroup* result)
{
	*result = dataToFilter;
}

void DeviceFilter::filterOutput(const std::string& device, const DataGroup& dataToFilter, DataGroup* result)
{
	*result = dataToFilter;
}

};  // namespace Devices
};  // namespace SurgSim
