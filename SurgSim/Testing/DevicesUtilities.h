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

#ifndef SURGSIM_TESTING_DEVICESUTILITIES_H
#define SURGSIM_TESTING_DEVICESUTILITIES_H

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Input/InputConsumerInterface.h"
#include "SurgSim/Input/OutputProducerInterface.h"

namespace SurgSim
{
namespace Testing
{

struct MockInputOutput : public SurgSim::Input::InputConsumerInterface, public SurgSim::Input::OutputProducerInterface
{
public:
	MockInputOutput();

	virtual void initializeInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData);
	virtual void handleInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData);
	virtual bool requestOutput(const std::string& device, SurgSim::DataStructures::DataGroup* outputData);

	int m_numTimesInitializedInput;
	int m_numTimesReceivedInput;
	int m_numTimesRequestedOutput;
	SurgSim::DataStructures::DataGroup m_lastReceivedInput;
};

};
};
#endif // SURGSIM_TESTING_DEVICESUTILITIES_H
