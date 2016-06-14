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
class OutputProducerInterface;

SURGSIM_STATIC_REGISTRATION(OutputComponent);

/// OutputComponents connect SceneElements to devices, facilitating data
/// transfer from a SceneElement to a device.
class OutputComponent : public SurgSim::Framework::Representation
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

	/// Sets the output data.
	/// \param dataGroup The data to output.
	void setData(const SurgSim::DataStructures::DataGroup& dataGroup);

	/// Get the data from the OutputProducer.
	/// \param data [out] The data, unchanged if the return value is false.
	/// \return true if data was provided.
	bool getData(DataStructures::DataGroup* data) const;

	/// Overridden from Component, do nothing
	virtual bool doInitialize();

	/// Overridden from Component, do nothing
	virtual bool doWakeUp();

	/// Gets device name.
	/// \return	The device name.
	std::string getDeviceName() const;

protected:
	/// Name of the device to which this output component connects
	std::string m_deviceName;
	/// Indicates if this output component is connected to a device
	bool m_deviceConnected;
	/// Output producer which sends data to hardware device
	std::shared_ptr<OutputProducerInterface> m_output;
};

}; // namespace Input
}; // namespace SurgSim


#endif
