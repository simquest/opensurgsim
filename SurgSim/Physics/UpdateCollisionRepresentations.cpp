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

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/ThreadPool.h"
#include "SurgSim/Physics/UpdateCollisionRepresentations.h"
#include "SurgSim/Physics/PhysicsManagerState.h"
#include "SurgSim/Collision/Representation.h"

namespace SurgSim
{
namespace Physics
{
UpdateCollisionRepresentations::UpdateCollisionRepresentations(bool doCopyState) :
	Computation(doCopyState)
{

}

UpdateCollisionRepresentations::~UpdateCollisionRepresentations()
{

}

std::shared_ptr<PhysicsManagerState> SurgSim::Physics::UpdateCollisionRepresentations::doUpdate(const double& dt,
		const std::shared_ptr<PhysicsManagerState>& state)
{
	std::shared_ptr<PhysicsManagerState> result = state;

	auto& collisionPairs = state->getCollisionPairs();
	std::unordered_set<Collision::Representation*> colliding;
	for (auto& pair : collisionPairs)
	{
		colliding.emplace(pair->getFirst().get());
		colliding.emplace(pair->getSecond().get());
	}

	auto threadPool = Framework::Runtime::getThreadPool();
	std::vector<std::future<void>> tasks;
	auto& representations = result->getActiveCollisionRepresentations();
	for (auto& representation : representations)
	{
		if (colliding.count(representation.get()) > 0)
		{
			tasks.push_back(threadPool->enqueue<void>([dt, &representation]() { representation->update(dt); }));
		}
	}
	for (auto& task : tasks)
	{
		task.get();
	}

	return result;
}

}
}

