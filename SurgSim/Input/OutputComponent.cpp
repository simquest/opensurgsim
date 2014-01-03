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

#include "SurgSim/Input/OutputComponent.h"

namespace SurgSim
{
namespace Input
{

OutputComponent::OutputComponent(const std::string& name) :
	Component(name)
{
}

OutputComponent::~OutputComponent()
{
}

void OutputComponent::setDeviceName(const std::string& deviceName)
{
	m_deviceName = deviceName;
}

void OutputComponent::setOutputData(const SurgSim::DataStructures::DataGroup& outputData)
{
	m_outputData = outputData;
}

bool OutputComponent::requestOutput(const std::string& device, SurgSim::DataStructures::DataGroup* outputData)
{
	bool result = false;
	if (outputData != nullptr)
	{
		*outputData = m_outputData;
		result = true;
	}
	return result;
}

SurgSim::DataStructures::DataGroup& OutputComponent::getOutputData()
{
	return m_outputData;
}

bool OutputComponent::doInitialize()
{
	return true;
}

bool OutputComponent::doWakeUp()
{
	return true;
}

std::string OutputComponent::getDeviceName() const
{
	return m_deviceName;
}

}; // namespace Input
}; // namespace SurgSim
