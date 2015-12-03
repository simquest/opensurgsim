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

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Physics/PhysicsManagerState.h"
#include "SurgSim/Physics/PrepareCollisionPairs.h"

namespace SurgSim
{
namespace Physics
{

PrepareCollisionPairs::PrepareCollisionPairs(bool doCopyState) :
	Computation(doCopyState)
{
}

PrepareCollisionPairs::~PrepareCollisionPairs()
{
}

std::shared_ptr<PhysicsManagerState> PrepareCollisionPairs::doUpdate(
	const double& dt,
	const std::shared_ptr<PhysicsManagerState>& state)
{
	std::shared_ptr<PhysicsManagerState> result = state;
	auto& representations = result->getActiveCollisionRepresentations();

	if (representations.size() > 1)
	{
		std::vector<std::shared_ptr<Collision::CollisionPair>> pairs;
		auto firstEnd = std::end(representations);
		for (auto first = std::begin(representations); first != firstEnd; ++first)
		{
			for (auto second = first; second != std::end(representations); ++second)
			{
				if (!(*first)->isIgnoring(*second) && !(*second)->isIgnoring(*first))
				{
					auto pair = std::make_shared<Collision::CollisionPair>(*first, *second);

					if (pair->getType() != Collision::COLLISION_DETECTION_TYPE_NONE)
					{
						pairs.emplace_back(pair);
					}
				}
			}
		}

		result->setCollisionPairs(pairs);
	}

	return result;
}

}; // Physics
}; // SurgSim
