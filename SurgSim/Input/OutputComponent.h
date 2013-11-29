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

#ifndef SURGSIM_INPUT_OUTPUTCOMPONENT_H
#define SURGSIM_INPUT_OUTPUTCOMPONENT_H

#include <SurgSim/Framework/Component.h>
#include <SurgSim/Input/OutputProducerInterface.h>
#include <SurgSim/DataStructures/DataGroup.h>

namespace SurgSim
{
namespace Input
{

class OutputComponent : public SurgSim::Framework::Component, public SurgSim::Input::OutputProducerInterface
{
public:
	OutputComponent(const std::string& name, const std::string& deviceName,
					const SurgSim::DataStructures::DataGroup& outputData);
	virtual ~OutputComponent();

	/// Overridden callback from OutputProducerInterface, the device will call this to fetch the
	/// data
	/// \param	device			  	The name of the device to which we want to write the data.
	/// \param [in,out]	outputData	If non-null, information pointer to the devices output data.
	/// \return	true if it succeeds, false if it fails.
	virtual bool requestOutput(const std::string& device, SurgSim::DataStructures::DataGroup* outputData);

	/// Reference to this components output data
	/// \return	The output data.
	SurgSim::DataStructures::DataGroup& getOutputData();

	/// Do Nothing, overridden from Component
	virtual bool doInitialize();

	/// Do Nothing, overridden from Component
	virtual bool doWakeUp();

	/// Gets device name.
	/// \return	The device name.
	std::string getDeviceName() const;

private:
	std::string m_deviceName;
	SurgSim::DataStructures::DataGroup m_outputData;
};

}; // namespace Input
}; // namespace SurgSim


#endif
