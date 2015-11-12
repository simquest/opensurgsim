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

#include <future>
#include <vector>

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/ContactCalculation.h"
#include "SurgSim/Collision/DcdCollision.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Physics/DcdCollision.h"
#include "SurgSim/Physics/PhysicsManagerState.h"

using SurgSim::Collision::CollisionPair;
using SurgSim::Collision::ContactCalculation;

namespace SurgSim
{
namespace Physics
{

DcdCollision::DcdCollision(bool doCopyState) :
	Computation(doCopyState)
{
}

DcdCollision::~DcdCollision()
{
}

std::shared_ptr<PhysicsManagerState> DcdCollision::doUpdate(
	const double& dt,
	const std::shared_ptr<PhysicsManagerState>& state)
{
	std::shared_ptr<PhysicsManagerState> result = state;
	std::vector<std::future<void>> tasks;

	const auto& calculations = ContactCalculation::getContactTable();

	updatePairs(result);

	for (auto& pair : result->getCollisionPairs())
	{
		if (pair->getType() == Collision::COLLISION_DETECTION_TYPE_DISCRETE)
		{
			tasks.push_back(std::async(std::launch::async, [&]()
			{
				calculations[pair->getFirst()->getShapeType()]
				[pair->getSecond()->getShapeType()]->calculateContact(pair);
			}));
		}
	}

	std::for_each(tasks.begin(), tasks.end(), [](std::future<void>& p){p.get();});

	return result;
}

void DcdCollision::updatePairs(std::shared_ptr<PhysicsManagerState> state)
{
	auto& representations = state->getActiveCollisionRepresentations();

	if (representations.size() > 1)
	{
		std::vector<std::shared_ptr<CollisionPair>> pairs;
		auto firstEnd = std::end(representations);
		for (auto first = std::begin(representations); first != firstEnd; ++first)
		{
			auto second = first;
			for (; second != std::end(representations); ++second)
			{
				if (!(*first)->isIgnoring(*second) && !(*second)->isIgnoring(*first))
				{
					auto pair = std::make_shared<CollisionPair>(*first, *second);
					if (pair->getType() != Collision::COLLISION_DETECTION_TYPE_NONE)
					{
						pairs.push_back(std::move(pair));
					}
				}
			}
		}

		state->setCollisionPairs(pairs);
	}
}

}; // Physics
}; // SurgSim

