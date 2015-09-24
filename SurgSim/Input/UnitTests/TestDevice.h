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

#ifndef SURGSIM_INPUT_UNITTESTS_TESTDEVICE_H
#define SURGSIM_INPUT_UNITTESTS_TESTDEVICE_H

#include "SurgSim/Input/CommonDevice.h"
#include "SurgSim/Input/InputConsumerInterface.h"
#include "SurgSim/Input/OutputProducerInterface.h"
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"

using SurgSim::Input::CommonDevice;
using SurgSim::DataStructures::DataGroup;
using SurgSim::DataStructures::DataGroupBuilder;
using SurgSim::Input::InputConsumerInterface;
using SurgSim::Input::OutputProducerInterface;


class TestDevice : public CommonDevice
{
public:
	explicit TestDevice(const std::string& uniqueName);

	bool initialize() override;

	bool finalize() override;

	bool isInitialized() const override;

	void pushInput() override;

	// Send some data down the stream
	void pushInput(const std::string& data);

	bool pullOutput() override;

	const DataGroup& getOutputData() const;

	/// Builds the data layout for the application input (i.e. device output).
	static DataGroup buildInputData();
	DataGroup buildOutputData();

	std::string lastPulledData;
};


struct TestInputConsumer : public InputConsumerInterface
{
public:
	TestInputConsumer() :
	  m_numTimesReceivedInput(0)
	  {
	  }

	  virtual void initializeInput(const std::string& device, const DataGroup& initialInput)
	  {
	  }
	  virtual void handleInput(const std::string& device, const DataGroup& inputData);

	  int m_numTimesReceivedInput;
	  DataGroup m_lastReceivedInput;
};

struct TestOutputProducer : public OutputProducerInterface
{
public:
	TestOutputProducer() :
	  m_numTimesRequestedOutput(0),
		  m_refuseToProduce(false)
	  {
		  DataGroupBuilder builder;
		  builder.addInteger("value");
		  m_nextSentOutput = builder.createData();
		  m_nextSentOutput.integers().set("value", 123);
	  }

	  virtual bool requestOutput(const std::string& device, DataGroup* outputData);

	  int m_numTimesRequestedOutput;
	  bool m_refuseToProduce;
	  DataGroup m_nextSentOutput;
};

#endif  // SURGSIM_INPUT_UNITTESTS_TESTDEVICE_H
