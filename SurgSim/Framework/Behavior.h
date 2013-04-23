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

#ifndef SURGSIM_FRAMEWORK_BEHAVIOR_H
#define SURGSIM_FRAMEWORK_BEHAVIOR_H

#include "Component.h"

namespace SurgSim
{
namespace Framework
{

/// Behaviors perform actions. They can update components, facilitate
/// communication between components, and create new components. They are
/// updated periodicly by the BehaviorManager through update() call.
class Behavior: public Component
{
public:
	Behavior(std::string name) : Component(name) {};
	virtual ~Behavior() {};

	/// Update the behavior
	/// \param dt	The length of time (seconds) between update calls.
	virtual void update(double dt) = 0;
};

}; //namespace Framework
}; //namespace SurgSim

#endif
