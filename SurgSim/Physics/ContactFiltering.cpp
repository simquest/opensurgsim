// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
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

#include <vector>

#include "SurgSim/Collision/ContactFilter.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/ThreadPool.h"
#include "SurgSim/Physics/ContactFiltering.h"
#include "SurgSim/Physics/PhysicsManagerState.h"

namespace SurgSim
{
namespace Physics
{

ContactFiltering::ContactFiltering(bool doCopyState) :
	Computation(doCopyState)
{
}

ContactFiltering::~ContactFiltering()
{
}

std::shared_ptr<PhysicsManagerState> ContactFiltering::doUpdate(
	const double& dt,
	const std::shared_ptr<PhysicsManagerState>& state)
{
	static auto isActive = [](std::shared_ptr<Collision::ContactFilter> f)
	{
		return f->isActive();
	};

	static auto hasContacts = [](std::shared_ptr<Collision::CollisionPair> p)
	{
		return p->hasContacts();
	};

	std::shared_ptr<PhysicsManagerState> result = state;
	auto threadPool = Framework::Runtime::getThreadPool();
	std::vector<std::future<void>> tasks;

	const auto& stateFilters = state->getContactFilters();
	std::vector<std::shared_ptr<Collision::ContactFilter>> filters;
	filters.reserve(stateFilters.size());
	std::copy_if(stateFilters.begin(), stateFilters.end(), std::back_inserter(filters), isActive);

	if (filters.size() == 0)
	{
		return result;
	}

	const auto& statePairs = state->getCollisionPairs();
	std::vector <std::shared_ptr<Collision::CollisionPair>> pairs;
	pairs.reserve(statePairs.size());
	std::copy_if(statePairs.begin(), statePairs.end(), std::back_inserter(pairs), hasContacts);

	for (auto& pair : pairs)
	{
		tasks.push_back(threadPool->enqueue<void>([&state, &filters, &pair]()
		{
			for (const auto& filter : filters)
			{
				filter->filterContacts(state, pair);
			}
		}));
	}


	std::for_each(tasks.begin(), tasks.end(), [](std::future<void>& p)
	{
		p.get();
	});

	return result;
}

}; // Physics
}; // SurgSim

