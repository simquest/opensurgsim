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
#include <SurgSim/Input/InputConsumerInterface.h>
#include <SurgSim/DataStructures/DataGroup.h>

namespace SurgSim
{
namespace Input
{


/// A class that implements some common management code on top of the DeviceInterface.
/// Practically every class that implements DeviceInterface will likely want to inherit from CommonDevice.
class CommonDevice : public DeviceInterface
{
public:
	/// Constructor.
	///
	/// \param name The name associated with the input device.
	/// \param inputData An initial value for the application's input from the device (e.g. pose etc).
	/// 	The concrete device implementation should pass in a DataGroup whose contents has been set up, e.g. by
	/// 	using a DataGroupBuilder, to that device's supported values that it will push to the application.
	CommonDevice(const std::string& name, const SurgSim::DataStructures::DataGroup& inputData);

	/// Constructor.
	///
	/// \param name The name associated with the input device.
	/// \param inputData An initial value for the application's input from the device (e.g. pose etc).
	/// 	The concrete device implementation should pass in a DataGroup whose contents has been set up, e.g. by
	/// 	using a DataGroupBuilder, to that device's supported values that it will push to the application.
	CommonDevice(const std::string& name, SurgSim::DataStructures::DataGroup&& inputData);

	/// Destructor.
	virtual ~CommonDevice();

	/// Return a (hopefully unique) device name.
	virtual std::string getName() const;

	virtual bool addInputConsumer(std::shared_ptr<InputConsumerInterface> inputConsumer);

	virtual bool removeInputConsumer(std::shared_ptr<InputConsumerInterface> inputConsumer);

	virtual bool setOutputProducer(std::shared_ptr<OutputProducerInterface> outputProducer);

	virtual bool removeOutputProducer(std::shared_ptr<OutputProducerInterface> outputProducer);

	virtual bool hasOutputProducer();

protected:

	/// Push application input to consumers.
	virtual void pushInput();

	/// Pull application output from a producer.
	virtual bool pullOutput();

	/// Provides access to the input data \ref SurgSim::DataStructures::DataGroup "DataGroup" for derived classes.
	/// \return A const reference to the input data.
	const SurgSim::DataStructures::DataGroup& getInputData() const
	{
		return m_inputData;
	}
	/// Provides access to the input data \ref SurgSim::DataStructures::DataGroup "DataGroup" for derived classes.
	/// \return A writable reference to the input data.
	SurgSim::DataStructures::DataGroup& getInputData()
	{
		return m_inputData;
	}

	/// Provides access to the output data \ref SurgSim::DataStructures::DataGroup "DataGroup" for derived classes.
	/// Note that a writable variant is not provided, since derived classes will not need to write to the application
	/// output data (an output producer registered via \ref setOutputProducer will be called to do that).
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
	std::unique_ptr<State> m_state;
};


};  // namespace Input
};  // namespace SurgSim

#endif // SURGSIM_INPUT_COMMON_DEVICE_H
