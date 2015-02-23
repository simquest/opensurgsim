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


#include <memory>
#include <vector>

#include "SurgSim/Physics/FreeMotion.h"
#include "SurgSim/Physics/Representation.h"
#include "SurgSim/Physics/PhysicsManagerState.h"

namespace SurgSim
{
namespace Physics
{


FreeMotion::FreeMotion(bool doCopyState) : Computation(doCopyState)
{

}

FreeMotion::~FreeMotion()
{

}

std::shared_ptr<PhysicsManagerState> FreeMotion::doUpdate(
	const double& dt,
	const std::shared_ptr<PhysicsManagerState>& state)
{
	// Copy state to new state
	std::shared_ptr<PhysicsManagerState> result = state;
	auto& representations = result->getActiveRepresentations();

	for (auto& representation : representations)
	{
		representation->update(dt);
	}

	return result;
}


}; // Physics
}; // SurgSim
