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

#include <SurgSim/Physics/ContactConstraintGeneration.h>

namespace SurgSim
{
namespace Physics
{

ContactConstraintGeneration::ContactConstraintGeneration()
{

}

ContactConstraintGeneration::~ContactConstraintGeneration()
{

}

std::shared_ptr<PhysicsManagerState> ContactConstraintGeneration::doUpdate(double dt, std::shared_ptr<PhysicsManagerState> state)
{
	return state;
}

}; // Physics
}; // SurgSim
