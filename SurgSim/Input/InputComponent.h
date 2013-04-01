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

#ifndef SURGSIM_INPUT_INPUTCOMPONENT_H
#define SURGSIM_INPUT_INPUTCOMPONENT_H

#include <SurgSim/Framework/Component.h>
#include <SurgSim/Input/InputConsumerInterface.h>
#include <SurgSim/DataStructures/DataGroup.h>

namespace SurgSim
{
namespace Input
{

/// InputComponent combines the Component interface and the InputConsumerInterface so that input devices can
/// provide input through the normal component interface. Multiple InputComponents can be added to
/// the same device.
class InputComponent : public SurgSim::Framework::Component, public SurgSim::Input::InputConsumerInterface
{
public:
	InputComponent(std::string name, std::string deviceName);;
	virtual ~InputComponent();;

	/// Overridden from InputComsumerInterface, callback from the device to set the input data in here.
	/// \param	device   	The name of the device from which we want to pull input.
	/// \param	inputData	The actual input data.
	virtual void handleInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData);

	/// Fetches a reference to the input data.
	/// \return	The input data.
	const SurgSim::DataStructures::DataGroup& getInputData();

	/// Overriden from Component, do nothing
	virtual bool doInitialize();

	/// Overriden from Component, do nothing
	virtual bool doWakeUp();

	/// Gets device name.
	/// \return	The device name.
	std::string getDeviceName() const;

private:
	std::string m_deviceName;
	SurgSim::DataStructures::DataGroup m_lastInput;
};

}; // namespace Input
}; // namespace SurgSim


#endif
