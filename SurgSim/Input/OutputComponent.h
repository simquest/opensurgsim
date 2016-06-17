// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#ifndef SURGSIM_INPUT_OUTPUTCOMPONENT_H
#define SURGSIM_INPUT_OUTPUTCOMPONENT_H

#include <string>
#include <memory>

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Framework/LockedContainer.h"
#include "SurgSim/Framework/Representation.h"
#include "SurgSim/Input/OutputProducerInterface.h"


namespace SurgSim
{

namespace Input
{
class DeviceInterface;

SURGSIM_STATIC_REGISTRATION(OutputComponent);

/// OutputComponents connect SceneElements to devices, facilitating data
/// transfer from a SceneElement to a device.
class OutputComponent : public SurgSim::Framework::Representation, public OutputProducerInterface
{
public:
	/// Constructor
	/// \param name Name of this output component
	explicit OutputComponent(const std::string& name);
	/// Destructor
	virtual ~OutputComponent();

	SURGSIM_CLASSNAME(SurgSim::Input::OutputComponent);

	/// Set name of the device of output component.
	/// param	deviceName	The name of the device that will receive the output data.
	void setDeviceName(const std::string& deviceName);

	/// Gets device name.
	/// \return	The device name.
	std::string getDeviceName() const;

	/// Sets the output data.
	/// \param dataGroup The data to output.
	virtual void setData(const SurgSim::DataStructures::DataGroup& dataGroup);

	/// \return The data which may be empty.
	DataStructures::DataGroup getData();

	/// Overridden from Component, do nothing
	virtual bool doInitialize();

	/// Overridden from Component, do nothing
	virtual bool doWakeUp();

	bool requestOutput(const std::string& device, SurgSim::DataStructures::DataGroup* outputData) override;

private:
	/// Name of the device to which this output component connects
	std::string m_deviceName;

	/// Thread safe container of most recent output data
	SurgSim::Framework::LockedContainer<SurgSim::DataStructures::DataGroup> m_lastOutput;

	/// True if there is data available
	bool m_haveData;
};

}; // namespace Input
}; // namespace SurgSim


#endif
