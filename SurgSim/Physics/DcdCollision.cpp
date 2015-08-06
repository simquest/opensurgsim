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

#include <vector>

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/ContactCalculation.h"
#include "SurgSim/Collision/DcdCollision.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/ThreadPool.h"
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
	populateCalculationTable();
}

DcdCollision::~DcdCollision()
{
}

std::shared_ptr<PhysicsManagerState> DcdCollision::doUpdate(
	const double& dt,
	const std::shared_ptr<PhysicsManagerState>& state)
{
	std::shared_ptr<PhysicsManagerState> result = state;
	auto threadPool = Framework::Runtime::getThreadPool();
	auto const& representations = state->getActiveCollisionRepresentations();
	std::vector<std::future<void>> tasks;

	for (auto& representation : representations)
	{
		representation->getCollisions().unsafeGet().clear();
	}

	updatePairs(result);

	for (auto& pair : result->getCollisionPairs())
	{
		tasks.push_back(threadPool->enqueue<void>([&] ()
		{
			m_contactCalculations[pair->getFirst()->getShapeType()]
								 [pair->getSecond()->getShapeType()]->calculateContact(pair);
		}));
	}

	std::for_each(tasks.begin(), tasks.end(), [](std::future<void>& p){p.wait();});

	for (auto& representation : representations)
	{
		representation->getCollisions().publish();
	}

	return result;
}

void DcdCollision::populateCalculationTable()
{
	for (int i = 0; i < SurgSim::Math::SHAPE_TYPE_COUNT; ++i)
	{
		for (int j = 0; j < SurgSim::Math::SHAPE_TYPE_COUNT; ++j)
		{
			m_contactCalculations[i][j].reset(new Collision::DefaultContactCalculation(false));
		}
	}
	setDcdContactInTable(std::make_shared<Collision::BoxCapsuleDcdContact>());
	setDcdContactInTable(std::make_shared<Collision::BoxDoubleSidedPlaneDcdContact>());
	setDcdContactInTable(std::make_shared<Collision::BoxPlaneDcdContact>());
	setDcdContactInTable(std::make_shared<Collision::BoxSphereDcdContact>());
	setDcdContactInTable(std::make_shared<Collision::CapsuleSphereDcdContact>());
	setDcdContactInTable(std::make_shared<Collision::OctreeCapsuleDcdContact>());
	setDcdContactInTable(std::make_shared<Collision::OctreeDoubleSidedPlaneDcdContact>());
	setDcdContactInTable(std::make_shared<Collision::OctreePlaneDcdContact>());
	setDcdContactInTable(std::make_shared<Collision::OctreeSphereDcdContact>());
	setDcdContactInTable(std::make_shared<Collision::SphereSphereDcdContact>());
	setDcdContactInTable(std::make_shared<Collision::SphereDoubleSidedPlaneDcdContact>());
	setDcdContactInTable(std::make_shared<Collision::SpherePlaneDcdContact>());
	setDcdContactInTable(std::make_shared<Collision::TriangleMeshParticlesDcdContact>());
	setDcdContactInTable(std::make_shared<Collision::TriangleMeshPlaneDcdContact>());
	//setDcdContactInTable(std::make_shared<Collision::TriangleMeshTriangleMeshDcdContact>());
}

void DcdCollision::updatePairs(std::shared_ptr<PhysicsManagerState> state)
{
	auto& representations = state->getActiveCollisionRepresentations();

	if (representations.size() > 1)
	{
		std::vector<std::shared_ptr<CollisionPair>> pairs;
		auto firstEnd = std::end(representations);
		--firstEnd;
		for (auto first = std::begin(representations); first != firstEnd; ++first)
		{
			auto second = first;
			++second;
			for (; second != std::end(representations); ++second)
			{
				std::shared_ptr<CollisionPair> pair = std::make_shared<CollisionPair>();
				pair->setRepresentations(*first,*second);
				pairs.push_back(pair);
			}
		}

		auto& excludedPairs = state->getExcludedCollisionPairs();
		for (auto it = excludedPairs.cbegin(); it != excludedPairs.cend(); ++it)
		{
			auto candidate = std::find_if(pairs.begin(), pairs.end(), [&it](const std::shared_ptr<CollisionPair> &pair)
			{
				return (pair->getFirst() == (*it)->getFirst() && pair->getSecond() == (*it)->getSecond())
					|| (pair->getFirst() == (*it)->getSecond() && pair->getSecond() == (*it)->getFirst());
			});

			if (candidate != pairs.end())
			{
				pairs.erase(candidate);
			}
		}

		state->setCollisionPairs(pairs);
	}
}

void DcdCollision::setDcdContactInTable(std::shared_ptr<ContactCalculation> dcdContact)
{
	std::pair<int,int> shapeTypes = dcdContact->getShapeTypes();
	m_contactCalculations[shapeTypes.first][shapeTypes.second] = dcdContact;
	if (shapeTypes.first != shapeTypes.second)
	{
		m_contactCalculations[shapeTypes.second][shapeTypes.first] = dcdContact;
	}
}

}; // Physics
}; // SurgSim

