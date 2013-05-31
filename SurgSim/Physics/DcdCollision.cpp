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
#include <SurgSim/Physics/RigidActorCollisionRepresentation.h>
#include <SurgSim/Physics/RigidActor.h>
#include <SurgSim/Physics/CollisionPair.h>
#include <SurgSim/Physics/ContactCalculation.h>
#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Math/Vector.h>

namespace SurgSim
{
namespace Physics
{

DcdCollision::DcdCollision(std::shared_ptr< std::vector<std::shared_ptr<Actor>>> actors) :
	m_actors(actors), m_pairCount(0)
{
	populateCollisionTable();
	if (m_actors->size()*m_actors->size() != m_pairCount)
	{
		updatePairs();
	}
}

DcdCollision::~DcdCollision()
{
	m_pairs.clear();
}

void DcdCollision::doUpdate(double dt)
{
	updatePairs();
	auto it = m_pairs.cbegin();
	auto itEnd = m_pairs.cend();
	while (it != itEnd)
	{
		int i = (*it)->getFirst()->getShapeType();
		int j = (*it)->getSecond()->getShapeType();
		m_contactCalculations[i][j]->calculateContact(*it);
		++it;
	}
}

void DcdCollision::populateCollisionTable()
{
	m_contactFactory = std::make_shared<ContactFactory>();
	for (int i = 0; i < RIGID_SHAPE_TYPE_COUNT; ++i)
	{
		for (int j = 0; j < RIGID_SHAPE_TYPE_COUNT; ++j)
		{
			m_contactCalculations[i][j].reset(new DefaultContactCalculation(false));
		}
	}
	m_contactCalculations[RIGID_SHAPE_TYPE_SPHERE][RIGID_SHAPE_TYPE_SPHERE].reset(new SphereSphereDcdContact(m_contactFactory));
	m_contactCalculations[RIGID_SHAPE_TYPE_SPHERE][RIGID_SHAPE_TYPE_PLANE].reset(new SpherePlaneDcdContact(m_contactFactory));
}

void DcdCollision::updatePairs()
{
	m_pairs.clear();

	std::list<std::shared_ptr<RigidActor>> rigidActors;
	for (auto it = m_actors->cbegin(); it != m_actors->cend(); ++it)
	{
		std::shared_ptr<RigidActor> rigid = std::dynamic_pointer_cast<RigidActor>(*it);
		if (rigid != nullptr)
		{
			rigidActors.push_back(rigid);
		}
	}
	
	auto rigidEnd = rigidActors.cend();
	for (auto first = rigidActors.cbegin(); first != rigidEnd; ++first)
	{
		for (auto second = rigidActors.cbegin(); second != rigidEnd; ++second)
		{
			std::shared_ptr<CollisionPair> pair = m_pairFactory.getInstance();		
			pair->setRepresentations(std::make_shared<RigidActorCollisionRepresentation>(*first),
									 std::make_shared<RigidActorCollisionRepresentation>(*second));
			m_pairs.push_back(pair);
		}
	}
}

}; // Physics
}; // SurgSim

