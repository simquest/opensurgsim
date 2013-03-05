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

#ifndef SURGSIM_INPUT_COMMON_DEVICE_H
#define SURGSIM_INPUT_COMMON_DEVICE_H

#include <memory>
#include <string>

#include <SurgSim/Input/DeviceInterface.h>
#include <SurgSim/Input/DeviceListenerInterface.h>
#include <SurgSim/DataStructures/DataGroup.h>

namespace SurgSim
{
namespace Input
{


/// A class that implements some common management code on top of the DeviceInterface.
///
/// Note that despite the class name, many devices that inherit from it will not be input-only.  Haptic devices
/// are the most obvious example, but many other devices can, for example, display visual indicators such as
/// LEDs, etc.
///
/// Derived classes will likely want to hide their constructor and only allow creation through a manager object
/// for that type of device.
class CommonDevice : public DeviceInterface
{
public:
	/// Constructor.
	///
	/// \param name The name associated with the input device.
	/// \param inputData An initial value for the application's input from the device (i.e. the device's output).
	/// 	The concrete device implementation should pass in a DataGroup whose contents has been set up, e.g. by
	/// 	using a DataGroupBuilder, to that device's supported values.
	CommonDevice(const std::string& name, const SurgSim::DataStructures::DataGroup& inputData);

	/// Constructor.
	///
	/// \param name The name associated with the input device.
	/// \param inputData An initial value for the application's input from the device (i.e. the device's output).
	/// 	The concrete device implementation should pass in a DataGroup whose contents has been set up, e.g. by
	/// 	using a DataGroupBuilder, to that device's supported values.
	CommonDevice(const std::string& name, SurgSim::DataStructures::DataGroup&& inputData);

	/// Return a (hopefully unique) device name.
	virtual std::string getName() const;

	virtual bool addListener(std::shared_ptr<DeviceListenerInterface> listener);

	virtual bool addInputListener(std::shared_ptr<DeviceListenerInterface> listener);

	virtual bool removeListener(std::shared_ptr<DeviceListenerInterface> listener);

protected:

	/// Push application input (i.e. device output) to listeners.
	virtual void pushInput();

	/// Pull application output (i.e. device input) from a listener.
	virtual bool pullOutput();

	/// Provides access to the input data \ref DataGroup.
	/// \return A const reference to the input data.
	const SurgSim::DataStructures::DataGroup& getInputData() const
	{
		return m_inputData;
	}
	/// Provides access to the input data \ref DataGroup.
	/// \return A writable reference to the input data.
	SurgSim::DataStructures::DataGroup& getInputData()
	{
		return m_inputData;
	}

	/// Provides access to the output data \ref DataGroup.
	/// \return A const reference to the output data.
	const SurgSim::DataStructures::DataGroup& getOutputData() const
	{
		return m_outputData;
	}

private:
	struct State;

	std::string m_name;
	SurgSim::DataStructures::DataGroup m_inputData;
	SurgSim::DataStructures::DataGroup m_outputData;
	State* m_state;
};

};  // namespace Input
};  // namespace SurgSim

#endif // SURGSIM_INPUT_COMMON_DEVICE_H
