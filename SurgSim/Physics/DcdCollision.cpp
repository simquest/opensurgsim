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

#include <SurgSim/Physics/DcdCollision.h>
#include <SurgSim/Physics/CollisionRepresentation.h>
#include <SurgSim/Physics/RigidCollisionRepresentation.h>
#include <SurgSim/Physics/RigidRepresentation.h>
#include <SurgSim/Physics/CollisionPair.h>
#include <SurgSim/Physics/ContactCalculation.h>
#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Math/Vector.h>

namespace SurgSim
{
namespace Physics
{

DcdCollision::DcdCollision(bool doCopyState) : Computation(doCopyState)
{
	populateCalculationTable();
}

DcdCollision::~DcdCollision()
{
	m_pairs.clear();
}

std::shared_ptr<PhysicsManagerState> DcdCollision::doUpdate(
	const double& dt, 
	const std::shared_ptr<PhysicsManagerState>& state)
{
	std::shared_ptr<PhysicsManagerState> result = state;
	updatePairs(result);

	std::vector<std::shared_ptr<CollisionPair>> pairs = result->getCollisionPairs();
	auto it = m_pairs.cbegin();
	auto itEnd = m_pairs.cend();
	while (it != itEnd)
	{
		int i = (*it)->getFirst()->getShapeType();
		int j = (*it)->getSecond()->getShapeType();
		if (m_contactCalculations[i][j]->needsSwap())
		{
			(*it)->swapRepresentations();
		}
		m_contactCalculations[i][j]->calculateContact(*it);
		++it;
	}
	return result;
}

void DcdCollision::populateCalculationTable()
{
	for (int i = 0; i < RIGID_SHAPE_TYPE_COUNT; ++i)
	{
		for (int j = 0; j < RIGID_SHAPE_TYPE_COUNT; ++j)
		{
			m_contactCalculations[i][j].reset(new DefaultContactCalculation(false));
		}
	}
	m_contactCalculations[RIGID_SHAPE_TYPE_SPHERE][RIGID_SHAPE_TYPE_SPHERE].reset(new SphereSphereDcdContact());
	m_contactCalculations[RIGID_SHAPE_TYPE_SPHERE][RIGID_SHAPE_TYPE_PLANE].reset(new SpherePlaneDcdContact(false));
	m_contactCalculations[RIGID_SHAPE_TYPE_PLANE][RIGID_SHAPE_TYPE_SPHERE].reset(new SpherePlaneDcdContact(true));
}

void DcdCollision::updatePairs(std::shared_ptr<PhysicsManagerState> state)
{
	std::vector<std::shared_ptr<Representation>> representations = state->getRepresentations();
	std::list<std::shared_ptr<RigidRepresentation>> rigidRepresentations;

	if (representations.size() > 1)
	{
		for (auto it = representations.cbegin(); it != representations.cend(); ++it)
		{
			std::shared_ptr<RigidRepresentation> rigid = std::dynamic_pointer_cast<RigidRepresentation>(*it);
			if (rigid != nullptr && rigid->isActive())
			{
				rigidRepresentations.push_back(rigid);
			}
		}
	}

	if (rigidRepresentations.size() > 1)
	{
		std::vector<std::shared_ptr<CollisionPair>> pairs;
		auto firstEnd = rigidRepresentations.end();
		--firstEnd;
		for (auto first = rigidRepresentations.begin(); first != firstEnd; ++first)
		{
			std::list<std::shared_ptr<RigidRepresentation>>::iterator second = first;
			++second;
			for (; second != rigidRepresentations.end(); ++second)
			{
				std::shared_ptr<CollisionPair> pair = std::make_shared<CollisionPair>();
				pair->setRepresentations(std::make_shared<RigidCollisionRepresentation>(*first),
					std::make_shared<RigidCollisionRepresentation>(*second));
				pairs.push_back(pair);
			}
		}
		state->setCollisionPairs(pairs);
	}
}

}; // Physics
}; // SurgSim

