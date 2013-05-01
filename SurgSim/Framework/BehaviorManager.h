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

#ifndef SURGSIM_FRAMEWORK_BEHAVIORMANAGER_H
#define SURGSIM_FRAMEWORK_BEHAVIORMANAGER_H

#include <memory>
#include <vector>

#include "BasicThread.h"

namespace SurgSim 
{
namespace Framework
{

class Logger;
class Behavior;

/// Manager to handle Behaviors. The manager will collect all the behaviors
/// in the scene through addComponent (and removeComponent) calls. All the
/// behaviors will be update once per period (default 30Hz) once the 
/// BehaviorManager is started.
/// NOTE: Currently, BehaviorManager is not thread safe. Behaviors should only
///       be added before runtime starts.
/// TODO: Thread safety. Add ability to add/remove components during runtime.
class BehaviorManager : public BasicThread
{
public:
	BehaviorManager();
	~BehaviorManager();
	virtual bool addComponent(std::shared_ptr<Component> component);
	virtual bool removeComponent(std::shared_ptr<Component> component);

private:
	virtual bool doUpdate(double dt);
	virtual bool doInitialize();
	virtual bool doStartUp();

	std::vector<std::shared_ptr<Behavior>> m_behaviors;

	std::shared_ptr<SurgSim::Framework::Logger> m_logger;
};


}; // namespace Framework
}; // namespace SurgSim

#endif
