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

#include "SurgSim/Input/UnitTests/TestDevice.h"

TestDevice::TestDevice(const std::string& uniqueName) :
	CommonDevice(uniqueName, buildInputData())
{

}

// required by the DeviceInterface API
bool TestDevice::initialize()
{
	return true;
}

// required by the DeviceInterface API
bool TestDevice::finalize()
{
	return true;
}

bool TestDevice::isInitialized() const
{
	return true;
}

// expose the pushInput method to the world
void TestDevice::pushInput()
{
	CommonDevice::pushInput();
}

void TestDevice::pushInput(const std::string& data)
{
	getInputData().strings().set("helloWorld", data);
	pushInput();
}

// expose the pullOutput method to the world
bool TestDevice::pullOutput()
{
	bool result = CommonDevice::pullOutput();
	getOutputData().strings().get("data", &lastPulledData);
	return result;
}

// expose the getOutputData method to the world
const DataGroup& TestDevice::getOutputData() const
{
	return CommonDevice::getOutputData();
}

DataGroup TestDevice::buildInputData()
{
	DataGroupBuilder builder;
	builder.addString("helloWorld");
	DataGroup data = builder.createData();
	data.strings().set("helloWorld", "data");
	return data;
}

// Consumer Class Callback Function
void TestInputConsumer::handleInput(const std::string& device, const DataGroup& inputData)
{
	++m_numTimesReceivedInput;
	m_lastReceivedInput = inputData;
}

// Producer class Hook
bool TestOutputProducer::requestOutput(const std::string& device, DataGroup* outputData)
{
	++m_numTimesRequestedOutput;

	if (m_refuseToProduce)
	{
		return false;
	}
	else
	{
		*outputData = m_nextSentOutput;
		return true;
	}
}

