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

#include "SurgSim/Framework/Component.h"

namespace SurgSim
{
namespace Framework
{

/// Fixed List of enums for the available behavior types, do not explicitly assign values,
/// BEHAVIOR_TYPE_COUNT is used to determine the number of actual behavior types
enum {
	BEHAVIOR_TYPE_NONE = -1,
	BEHAVIOR_TYPE_REPRESENTATIONPOSE,
	BEHAVIOR_TYPE_INPUTVTC,
	BEHAVIOR_TYPE_COUNT
};

/// Behaviors perform actions. They can update components, facilitate
/// communication between components, and create new components. They are
/// updated periodicly by the BehaviorManager through update() call.
class Behavior: public Component
{
public:
	explicit Behavior(const std::string& name) : Component(name)
	{
	}
	virtual ~Behavior()
	{
	}

	/// Update the behavior
	/// \param dt	The length of time (seconds) between update calls.
	virtual void update(double dt) = 0;
	virtual int getBehaviorType() const { return BEHAVIOR_TYPE_NONE; }
};

}; //namespace Framework
}; //namespace SurgSim

#endif // SURGSIM_FRAMEWORK_BEHAVIOR_H
