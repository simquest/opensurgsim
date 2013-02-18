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

#ifndef SURGSIM_INPUT_COMMON_INPUT_DEVICE_H
#define SURGSIM_INPUT_COMMON_INPUT_DEVICE_H

#include <memory>
#include <string>

#include <SurgSim/Input/InputDeviceInterface.h>
#include <SurgSim/Input/InputDeviceListenerInterface.h>

namespace SurgSim
{
namespace Input
{

class DataGroup;

/// A class that implements some common management code on top of the InputDeviceInterface.
///
/// Note that despite the class name, many devices that inherit from it will not be input-only.  Haptic devices
/// are the most obvious example, but many other devices can, for example, display visual indicators such as
/// LEDs, etc.
///
/// Derived classes will likely want to hide their constructor and only allow creation through a manager object
/// for that type of device.
class CommonInputDevice : public InputDeviceInterface
{
public:
	/// Constructor.
	///
	/// \param name The name associated with the input device.
	/// \param inputData An initial value for the application's input from the device (i.e. the device's output).
	/// 	It should be initialized with data layout that will be used for the input coming from this device; the actual values in the .
	CommonInputDevice(const std::string& name, const DataGroup& inputData);

	/// Constructor.
	///
	/// \param name The name associated with the input device.
	/// \param inputData An initial value for the application's input from the device (i.e. the device's output).
	/// 	It should be initialized with data layout that will be used for the input coming from this device; the actual values in the .
	CommonInputDevice(const std::string& name, DataGroup&& inputData);

	/// Return a (hopefully unique) device name.
	virtual std::string getName() const;

	virtual bool addListener(std::shared_ptr<InputDeviceListenerInterface> listener);

	virtual bool addInputListener(std::shared_ptr<InputDeviceListenerInterface> listener);

	virtual bool removeListener(std::shared_ptr<InputDeviceListenerInterface> listener);

protected:

	/// Push application input (i.e. device output) to listeners.
	virtual void pushInput();

	/// Pull application output (i.e. device input) from a listener.
	virtual bool pullOutput();

	/// Provides access to the input data \ref DataGroup.
	/// \return A const reference to the input data.
	const SurgSim::Input::DataGroup& getInputData() const
	{
		return m_inputData;
	}
	/// Provides access to the input data \ref DataGroup.
	/// \return A writable reference to the input data.
	SurgSim::Input::DataGroup& getInputData()
	{
		return m_inputData;
	}

	/// Provides access to the output data \ref DataGroup.
	/// \return A const reference to the output data.
	const SurgSim::Input::DataGroup& getOutputData() const
	{
		return m_outputData;
	}
// 	/// Provides access to the output data \ref DataGroup.
// 	/// \return A writable reference to the output data.
// 	SurgSim::Input::DataGroup& getOutputData() { return m_outputData; }

private:
	struct State;

	std::string m_name;
	DataGroup m_inputData;
	DataGroup m_outputData;
	State* m_state;
};

};  // namespace Input
};  // namespace SurgSim

#endif // SURGSIM_INPUT_COMMON_INPUT_DEVICE_H
