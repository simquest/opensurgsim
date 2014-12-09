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

#include <string>
#include <memory>

#include "SurgSim/Framework/Representation.h"

namespace SurgSim
{

namespace DataStructures
{
class DataGroup;
}

namespace Input
{
class DeviceInterface;
class InputConsumer;

SURGSIM_STATIC_REGISTRATION(InputComponent);

/// InputComponents connect devices to SceneElements, facilitating data transfer
/// from a device to SceneElements and other Components.
class InputComponent : public SurgSim::Framework::Representation
{
public:
	/// Constructor
	/// \param name Name of this input component
	explicit InputComponent(const std::string& name);

	/// Destructor
	virtual ~InputComponent();

	SURGSIM_CLASSNAME(SurgSim::Input::InputComponent);

	/// Set name of the device this input component connects to.
	/// \param deviceName Name of the device this input component connects
	void setDeviceName(const std::string& deviceName);

	/// Is a device connected
	/// \return true if a device has been connected.
	bool isDeviceConnected();

	/// Connect to a device
	/// This call will be made by the InputManager, and should generally not be called directly.
	/// \param device The device to connect to.
	void connectDevice(std::shared_ptr<SurgSim::Input::DeviceInterface> device);

	/// Disconnect from a device
	/// This call will be made by the InputManager, and should generally not be called directly.
	/// \param device The device to disconnect from.
	void disconnectDevice(std::shared_ptr<SurgSim::Input::DeviceInterface> device);

	/// Gets the input data.
	/// \param [out] dataGroup The location to write the data.  The pointer must be non-null.
	/// \exception Asserts if the InputComponent is not connected to a device.
	void getData(SurgSim::DataStructures::DataGroup* dataGroup);

	/// Overridden from Component, do nothing
	virtual bool doInitialize() override;

	/// Overridden from Component, do nothing
	virtual bool doWakeUp() override;

	/// Gets device name.
	/// \return	The device name.
	std::string getDeviceName() const;

private:
	/// Name of the device to which this input component connects
	std::string m_deviceName;
	/// Indicates if this input component is connected to a device
	bool m_deviceConnected;
	/// Input consumer which brings in information from hardware device
	std::shared_ptr<InputConsumer> m_input;
};

}; // namespace Input
}; // namespace SurgSim


#endif
