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

#include "SurgSim/Physics/UpdateCcdData.h"

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/ThreadPool.h"
#include "SurgSim/Physics/PhysicsManagerState.h"
#include "SurgSim/Collision/Representation.h"

#include <unordered_set>

namespace SurgSim
{

namespace Physics
{

UpdateCcdData::UpdateCcdData(bool copyState):
	Computation(copyState)
{

}

UpdateCcdData::~UpdateCcdData()
{

}


std::shared_ptr<PhysicsManagerState> UpdateCcdData::doUpdate(
	const double& interval,
	const std::shared_ptr<PhysicsManagerState>& state)
{
	std::shared_ptr<PhysicsManagerState> result = state;

	auto threadPool = Framework::Runtime::getThreadPool();
	std::vector<std::future<void>> tasks;

	const auto& pairs = result->getCollisionPairs();
	std::unordered_set<Collision::Representation*> representations;

	for (const auto& pair : pairs)
	{
		if (pair->getType() == SurgSim::Collision::COLLISION_DETECTION_TYPE_CONTINUOUS)
		{
			representations.insert(pair->getFirst().get());
			representations.insert(pair->getSecond().get());
		}
	}

	for (auto& representation : representations)
	{
		tasks.push_back(threadPool->enqueue<void>([interval, &representation]()
		{
			representation->updateCcdData(interval);
		}));
	}

	for (auto& task : tasks)
	{
		task.get();
	}

	return result;
}

}
}