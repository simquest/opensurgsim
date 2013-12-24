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

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Input/DeviceInterface.h"
#include "SurgSim/Input/OutputProducerInterface.h"
#include "SurgSim/Framework/LockedContainer.h"

namespace SurgSim
{
namespace Input
{
/// An input consumer monitors device and signal state update
class OutputProducer: public OutputProducerInterface
{
public:
	/// Constructor
	OutputProducer()
	{
	}
	/// Destructor
	virtual ~OutputProducer()
	{
	}

	/// Send the output to the device.
	/// \param device The name of the device to receive the output.
	/// \param outputData The output data going to the device.
	virtual bool requestOutput(const std::string& device, SurgSim::DataStructures::DataGroup* outputData) override
	{
		bool result = false;
		if (outputData != nullptr)
		{
			m_lastOutput.get(outputData);
			SURGSIM_ASSERT((*outputData).isValid()) << "Attempting to output invalid data to device (" << device << ")";
			result = true;
		}
		return result;
	}

	/// Set the output data information stored in this output producer
	/// \param [out] dataGroup Used to accept the retrieved input data information
	void setData(const SurgSim::DataStructures::DataGroup& dataGroup)
	{
		m_lastOutput.set(dataGroup);
	}

private:
	/// Used to store output data information to be passed out to device
	SurgSim::Framework::LockedContainer<SurgSim::DataStructures::DataGroup> m_lastOutput;
};

OutputComponent::OutputComponent(const std::string& name, const std::string& deviceName) :
	Component(name),
	m_deviceName(deviceName),
	m_deviceConnected(false),
	m_output(std::make_shared<OutputProducer>())
{
}

OutputComponent::~OutputComponent()
{
}

bool OutputComponent::isDeviceConnected()
{
	return m_deviceConnected;
}

void OutputComponent::setData(const SurgSim::DataStructures::DataGroup& dataGroup)
{
	SURGSIM_ASSERT(m_deviceConnected) << "No device connected to " << getName() << ". Unable to setData.";
	m_output->setData(dataGroup);
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

void OutputComponent::connectDevice(std::shared_ptr<SurgSim::Input::DeviceInterface> device)
{
	device->setOutputProducer(m_output);
	m_deviceConnected = true;
}

void OutputComponent::disconnectDevice(std::shared_ptr<SurgSim::Input::DeviceInterface> device)
{
	device->removeOutputProducer(m_output);
	m_deviceConnected = false;
}

}; // namespace Input
}; // namespace SurgSim
