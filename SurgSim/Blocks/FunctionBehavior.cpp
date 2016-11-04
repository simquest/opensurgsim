// This file is a part of the OpenSurgSim project.
// Copyright 2016, SimQuest Solutions Inc.
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

#include "SurgSim/Blocks/FunctionBehavior.h"

#include "SurgSim/Framework/Log.h"


namespace SurgSim
{
namespace Blocks
{

SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Blocks::FunctionBehavior, FunctionBehavior);

FunctionBehavior::FunctionBehavior(const std::string& name) :
	Framework::Behavior(name),
	m_targetManager(Framework::MANAGER_TYPE_BEHAVIOR),
	m_function(nullptr)
{
}

void FunctionBehavior::update(double dt)
{
	m_function(dt);
}

void FunctionBehavior::setTargetManagerType(int type)
{
	SURGSIM_ASSERT(type >= 0 && type < Framework::MANAGER_TYPE_COUNT) << "Invalid manager type.";
	SURGSIM_ASSERT(!isInitialized()) << "Cannot change the manager type after initialization.";
	m_targetManager = type;
}

int FunctionBehavior::getTargetManagerType() const
{
	return m_targetManager;
}

void FunctionBehavior::setFunction(std::function<void(double)> function)
{
	SURGSIM_ASSERT(!isInitialized()) << "Cannot change the function after initialization.";
	m_function = function;
}

bool FunctionBehavior::doInitialize()
{
	if (m_function == nullptr)
	{
		SURGSIM_LOG_SEVERE(Framework::Logger::getDefaultLogger())
			<< "FunctionBehavior named " << getFullName() << " missing behavior function.";
		return false;
	}

	return true;
}

bool FunctionBehavior::doWakeUp()
{
	return true;
}

};
};
