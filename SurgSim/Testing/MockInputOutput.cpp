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

#include "SurgSim/Testing/MockInputOutput.h"

using SurgSim::DataStructures::DataGroup;

namespace SurgSim
{
namespace Testing
{

MockInputOutput::MockInputOutput() :
	m_numTimesInitializedInput(0),
	m_numTimesReceivedInput(0),
	m_numTimesRequestedOutput(0)
{
}

bool MockInputOutput::requestOutput(const std::string& device, DataGroup* outputData)
{
	++m_numTimesRequestedOutput;
	bool result = false;
	if (m_output.hasValue())
	{
		*outputData = m_output.getValue();
		result = true;
	}
	return result;
}

void MockInputOutput::handleInput(const std::string& device, const DataGroup& inputData)
{
	++m_numTimesReceivedInput;
	m_lastReceivedInput = inputData;
}

void MockInputOutput::initializeInput(const std::string& device, const DataGroup& inputData)
{
	++m_numTimesInitializedInput;
	m_lastReceivedInput = inputData;
}

}; // Testing
}; // SurgSim
