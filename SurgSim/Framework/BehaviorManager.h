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

#include "SurgSim/Framework/ComponentManager.h"

namespace SurgSim
{
namespace Framework
{

/// Manager to handle Behaviors. The manager will collect all the behaviors
/// in the scene through addComponent/removeComponent calls. All the
/// behaviors will be update once per period (default 30Hz) once the
/// BehaviorManager is started.
class BehaviorManager : public ComponentManager
{
public:
	BehaviorManager();
	~BehaviorManager();

	int getType() const override;

protected:
	bool executeAdditions(const std::shared_ptr<Component>& component) override;
	bool executeRemovals(const std::shared_ptr<Component>& component) override;

private:
	bool doInitialize() override;
	bool doStartUp() override;
	bool doUpdate(double dt) override;
};


}; // namespace Framework
}; // namespace SurgSim

#endif // SURGSIM_FRAMEWORK_BEHAVIORMANAGER_H
