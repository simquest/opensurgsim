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
#include <SurgSim/Physics/PhysicsManagerState.h>

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
}

std::shared_ptr<PhysicsManagerState> DcdCollision::doUpdate(
	const double& dt,
	const std::shared_ptr<PhysicsManagerState>& state)
{
	std::shared_ptr<PhysicsManagerState> result = state;
	updatePairs(result);

	std::vector<std::shared_ptr<CollisionPair>> pairs = result->getCollisionPairs();
	auto it = pairs.cbegin();
	auto itEnd = pairs.cend();
	while (it != itEnd)
	{
		m_contactCalculations[(*it)->getFirst()->getShapeType()][(*it)->getSecond()->getShapeType()]->
			calculateContact(*it);
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
	setDcdContactInTable(std::make_shared<SphereSphereDcdContact>());
	setDcdContactInTable(std::make_shared<SphereDoubleSidedPlaneDcdContact>());
}

void DcdCollision::updatePairs(std::shared_ptr<PhysicsManagerState> state)
{
	std::vector<std::shared_ptr<CollisionRepresentation>> representations = state->getCollisionRepresentations();

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
		state->setCollisionPairs(pairs);
	}
}

void DcdCollision::setDcdContactInTable(std::shared_ptr<ContactCalculation> dcdContact)
{
	std::pair<int,int> shapeTypes = dcdContact->getShapeTypes();
	m_contactCalculations[shapeTypes.first][shapeTypes.second] = dcdContact;
	if(shapeTypes.first != shapeTypes.second)
	{
		m_contactCalculations[shapeTypes.second][shapeTypes.first] = dcdContact;
	}
}

}; // Physics
}; // SurgSim

