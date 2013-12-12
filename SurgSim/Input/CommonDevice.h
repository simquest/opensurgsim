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

#ifndef SURGSIM_INPUT_COMMONDEVICE_H
#define SURGSIM_INPUT_COMMONDEVICE_H

#include <memory>
#include <string>

#include "SurgSim/Input/DeviceInterface.h"
#include "SurgSim/Input/InputConsumerInterface.h"
#include "SurgSim/DataStructures/DataGroup.h"

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

	/// Constructor.
	///
	/// Note that this form of the constructor does NOT initialize the initial input data from the device.
	/// Therefore, the derived class MUST initialize it itself before any input consumers are added!
	///
	/// If you can use one of the other constructors, you should, but this one can be used when solving some
	/// tricky situations with device filters.
	///
	/// \param name	The name associated with the input device.
	explicit CommonDevice(const std::string& name);

	/// Destructor.
	virtual ~CommonDevice();

	/// Return a (hopefully unique) device name.
	virtual std::string getName() const override;

	/// Set the name used for calling the input consumers and output producer.
	/// By default, this will be the same as the name of the device that was passed to the constructor.
	/// \param name	The name to be used.
	void setNameForCallback(const std::string& name);

	/// Get the name used for calling the input consumers and output producer.
	/// By default, this will be the same as the name of the device that was passed to the constructor.
	/// \return	The name being used.
	std::string getNameForCallback() const;

	virtual bool addInputConsumer(std::shared_ptr<InputConsumerInterface> inputConsumer) override;
	virtual bool removeInputConsumer(std::shared_ptr<InputConsumerInterface> inputConsumer) override;

	virtual bool setOutputProducer(std::shared_ptr<OutputProducerInterface> outputProducer) override;
	virtual bool removeOutputProducer(std::shared_ptr<OutputProducerInterface> outputProducer) override;

	virtual bool hasOutputProducer() override;

protected:

	/// Push application input to consumers.
	virtual void pushInput();

	/// Pull application output from a producer.
	virtual bool pullOutput();

	/// Provides access to the initial input data \ref SurgSim::DataStructures::DataGroup "DataGroup".
	/// \return A const reference to the initial input data.
	const SurgSim::DataStructures::DataGroup& getInitialInputData() const
	{
		return m_initialInputData;
	}

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

	/// Set the entire input data \ref SurgSim::DataStructures::DataGroup "DataGroup" from derived classes.
	/// Also sets the initial input data \ref SurgSim::DataStructures::DataGroup "DataGroup" if it has not been set.
	/// Otherwise equivalent to <code>getInputData() = data;</code> but has more readable syntax.
	/// \param data	The input data to be set.
	void setInputData(const SurgSim::DataStructures::DataGroup& data)
	{
		if (! m_initialInputData.isValid())
		{
			m_initialInputData = data;
			m_initialInputData.resetAll();
		}
		m_inputData = data;
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

	const std::string m_name;
	std::string m_nameForCallback;
	SurgSim::DataStructures::DataGroup m_initialInputData;
	SurgSim::DataStructures::DataGroup m_inputData;
	SurgSim::DataStructures::DataGroup m_outputData;
	std::unique_ptr<State> m_state;
};


};  // namespace Input
};  // namespace SurgSim

#endif // SURGSIM_INPUT_COMMONDEVICE_H
