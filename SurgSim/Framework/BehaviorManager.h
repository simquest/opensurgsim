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

class Behavior;

class BehaviorManager : public BasicThread
{
public:
	BehaviorManager();
	~BehaviorManager();
	virtual bool addComponent(std::shared_ptr<Component> component);
	virtual bool removeComponent(std::shared_ptr<Component> component);

private:
	//! \return false when the thread is done
	virtual bool doUpdate(double dt);

	virtual bool doInitialize() {return true;};
	virtual bool doStartUp() {return true;};

	std::vector<std::shared_ptr<Behavior>> m_behaviors;
};


}; // namespace Framework
}; // namespace SurgSim

#endif