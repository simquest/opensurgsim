// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
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
#include "SurgSim/DataStructures/DataGroup.h"

namespace SurgSim
{
namespace Input
{

class InputConsumerInterface;
class OutputProducerInterface;

/// A class that implements some common management code on top of the DeviceInterface.
/// Practically every class that implements DeviceInterface will likely want to inherit from CommonDevice.
class CommonDevice : public DeviceInterface
{
public:
	/// Constructor.  Sets the input data to an empty DataGroup.
	/// \param name The name associated with the input device.
	explicit CommonDevice(const std::string& name);

	/// Constructor.
	///
	/// \param name The name associated with the input device.
	/// \param inputData An initial value for the application's input from the device (e.g. pose etc).
	/// 	The concrete device implementation should pass in a DataGroup whose contents has been set up, e.g. by
	/// 	using a DataGroupBuilder, to that device's supported values that it will push to the application.
	CommonDevice(const std::string& name, const DataStructures::DataGroup& inputData);

	/// Constructor.
	///
	/// \param name The name associated with the input device.
	/// \param inputData An initial value for the application's input from the device (e.g. pose etc).
	/// 	The concrete device implementation should pass in a DataGroup whose contents has been set up, e.g. by
	/// 	using a DataGroupBuilder, to that device's supported values that it will push to the application.
	CommonDevice(const std::string& name, DataStructures::DataGroup&& inputData);

	/// Destructor.
	virtual ~CommonDevice();

	std::string getName() const override;

	std::string getClassName() const override;

	/// Set the name used for calling the input consumers and output producer.
	/// By default, this will be the same as the name of the device that was passed to the constructor.
	/// \param name	The name to be used.
	void setNameForCallback(const std::string& name);

	/// Get the name used for calling the input consumers and output producer.
	/// By default, this will be the same as the name of the device that was passed to the constructor.
	/// \return	The name being used.
	std::string getNameForCallback() const;

	bool addInputConsumer(std::shared_ptr<InputConsumerInterface> inputConsumer) override;

	bool removeInputConsumer(std::shared_ptr<InputConsumerInterface> inputConsumer) override;

	void clearInputConsumers() override;

	bool setOutputProducer(std::shared_ptr<OutputProducerInterface> outputProducer) override;

	bool removeOutputProducer(std::shared_ptr<OutputProducerInterface> outputProducer) override;

	bool hasOutputProducer() override;

	void clearOutputProducer() override;

protected:

	/// Push application input to consumers.
	virtual void pushInput();

	/// Pull application output from a producer.
	virtual bool pullOutput();

	/// Getter for the input data \ref SurgSim::DataStructures::DataGroup "DataGroup".  This function is typically
	/// called by friend scaffolds, to get a DataGroup they can modify then set back to the device to send to the
	/// device's input consumers.
	/// \return A reference to the input data.
	DataStructures::DataGroup& getInputData();

	/// Getter for the output data \ref SurgSim::DataStructures::DataGroup "DataGroup".  This function is typically
	/// called by friend scaffolds, to get the data that the output producer wants to send to the device (and then send
	/// that data through the device's SDK). Note that a writable variant is not provided, an output producer registered
	/// via \ref setOutputProducer will set the output data.
	/// \return A reference to the output data.
	const DataStructures::DataGroup& getOutputData() const;

private:
	struct State;

	const std::string m_name;

	/// The name used for the callbacks, defaults to the device name.
	std::string m_nameForCallback;

	/// The data the device is providing to its input consumers.
	DataStructures::DataGroup m_inputData;

	/// The data the output producer (if any) is providing to the device.
	DataStructures::DataGroup m_outputData;

	/// The list of input consumers.
	std::vector<std::weak_ptr<InputConsumerInterface>> m_inputConsumerList;

	/// The output producer, if any.
	std::weak_ptr<OutputProducerInterface> m_outputProducer;

	/// The mutex that protects the consumers and the producer.
	boost::mutex m_consumerProducerMutex;

};


};  // namespace Input
};  // namespace SurgSim

#endif // SURGSIM_INPUT_COMMONDEVICE_H
