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


FreeMotion::FreeMotion(bool doCopyState, bool parallelExecution) :
	Computation(doCopyState),
	m_parallelExecution(parallelExecution)
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

	// Vector keeping track of all the tasks running in parallel
	std::vector<std::future<void>> tasks;

	auto& representations = result->getActiveRepresentations();
	for (auto& representation : representations)
	{
		if (m_parallelExecution)
		{
			tasks.push_back(m_threadPool.enqueue<void>([&]() { representation->update(dt); }));
		}
		else
		{
			representation->update(dt);
		}
	}

	auto& particleRepresentations = result->getActiveParticleRepresentations();
	for (auto& representation : particleRepresentations)
	{
		if (m_parallelExecution)
		{
			tasks.push_back(m_threadPool.enqueue<void>([&](){ representation->update(dt); }));
		}
		else
		{
			representation->update(dt);
		}
	}

	// Making sure this method returns when all calculation are done by waiting for all thread if (multi-threaded).
	if (m_parallelExecution)
	{
		for (auto& task : tasks)
		{
			task.wait();
		}
	}

	return result;
}


}; // Physics
}; // SurgSim
