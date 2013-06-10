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

#include <SurgSim/Framework/ComponentManager.h>
#include <SurgSim/Framework/Log.h>
#include <SurgSim/Framework/Runtime.h>

namespace SurgSim
{
namespace Framework
{

ComponentManager::ComponentManager(const std::string& name /*= "Unknown Component Manager"*/) :
	m_logger (Logger::createConsoleLogger(name)), BasicThread(name)
{
}

ComponentManager::~ComponentManager()
{

}

void ComponentManager::setRuntime(std::shared_ptr<Runtime> val)
{
	m_runtime = val;

	// Now pull the shared logger out of the runtime
	m_logger = val->getLogger(getName());
}

}; // Framework
}; // SurgSim
