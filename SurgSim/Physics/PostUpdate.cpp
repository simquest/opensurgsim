// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#include "SurgSim/Physics/PostUpdate.h"
#include "SurgSim/Physics/Representation.h"
#include "SurgSim/Physics/PhysicsManagerState.h"

namespace SurgSim
{
namespace Physics
{

PostUpdate::PostUpdate(bool doCopyState) : Computation(doCopyState)
{

}

PostUpdate::~PostUpdate()
{

}

std::shared_ptr<PhysicsManagerState> PostUpdate::doUpdate(
	const double& dt,
	const std::shared_ptr<PhysicsManagerState>& state)
{
	std::shared_ptr<PhysicsManagerState> result = state;
	{
		const auto& representations = result->getActiveRepresentations();
		for (auto& representation : representations)
		{
			representation->afterUpdate(dt);
		}
	}

	{
		// Clear the collisions
		auto const& representations = result->getCollisionRepresentations();

		for (auto& representation : representations)
		{
			representation->getCollisions().unsafeGet().clear();
		}

		// Update the representations with the contact data.
		auto& pairs = result->getCollisionPairs();
		for (auto& pair : pairs)
		{
			pair->updateRepresentations();
		}

		// Publish Results
		for (auto& representation : representations)
		{
			representation->getCollisions().publish();
		}
	}

	return result;
}


}; // Physics
}; // SurgSim
