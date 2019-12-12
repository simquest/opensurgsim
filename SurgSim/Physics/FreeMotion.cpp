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

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/ThreadPool.h"
#include "SurgSim/Physics/FreeMotion.h"
#include "SurgSim/Physics/PhysicsManagerState.h"
#include "SurgSim/Physics/Representation.h"

namespace SurgSim
{
namespace Physics
{


FreeMotion::FreeMotion(bool doCopyState) :
	Computation(doCopyState)
{

}

FreeMotion::~FreeMotion()
{

}

std::shared_ptr<PhysicsManagerState> FreeMotion::doUpdate(const double& dt,
		const std::shared_ptr<PhysicsManagerState>& state)
{
	// Copy state to new state
	std::shared_ptr<PhysicsManagerState> result = state;

	auto threadPool = Framework::Runtime::getThreadPool();
	std::vector<std::future<std::pair<std::string, size_t>>> tasks;

	auto& representations = result->getActiveRepresentations();
	for (auto& representation : representations)
	{
		tasks.push_back(threadPool->enqueue<std::pair<std::string, size_t>>([dt, &representation]()
		{
			auto start = std::chrono::high_resolution_clock::now();
			representation->update(dt);
			auto end = std::chrono::high_resolution_clock::now();
			return std::make_pair(representation->getFullName(),
								  std::chrono::duration_cast<std::chrono::microseconds>(end - start).count());
		}));
	}

	auto& particleRepresentations = result->getActiveParticleRepresentations();
	for (auto& representation : particleRepresentations)
	{
		tasks.push_back(threadPool->enqueue<std::pair<std::string, size_t>>([dt, &representation]()
		{
			auto start = std::chrono::high_resolution_clock::now();
			representation->update(dt);
			auto end = std::chrono::high_resolution_clock::now();
			return std::make_pair(representation->getFullName(),
								  std::chrono::duration_cast<std::chrono::microseconds>(end - start).count());
		}));
	}

	for (auto& task : tasks)
	{
		auto result = task.get();
	}

	/*
	static size_t i = 1000;
	--i;

	for (auto& task : tasks)
	{
		auto result = task.get();
		if (i == 0)
		{
			std::cout << result.first << ": " << result.second << "\n";
		}
	}

	if (i < 1)
	{
		i = 1000;
	}
	*/
	return result;
}


}; // Physics
}; // SurgSim
