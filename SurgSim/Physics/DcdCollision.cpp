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
#include <SurgSim/Physics/CollisionPair.h>
#include <SurgSim/Physics/ContactCalculation.h>
#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Math/Vector.h>

namespace SurgSim
{
namespace Physics
{

DcdCollision::DcdCollision(std::shared_ptr<std::vector<std::shared_ptr<CollisionPair>>> pairs) :
	m_pairs(pairs)
{
	populateCollisionTable();
}

DcdCollision::~DcdCollision()
{

}

void DcdCollision::doUpdate(double dt)
{
	auto it = m_pairs->cbegin();
	auto itEnd = m_pairs->cend();
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
	std::shared_ptr<ContactFactory> contactFactory = std::make_shared<ContactFactory>();
	for (int i = 0; i < RIGID_SHAPE_TYPE_COUNT; ++i)
	{
		for (int j = 0; j < RIGID_SHAPE_TYPE_COUNT; ++j)
		{
			m_contactCalculations[i][j].reset(new DefaultContactCalculation(false));
		}
	}
	m_contactCalculations[RIGID_SHAPE_TYPE_SPHERE][RIGID_SHAPE_TYPE_SPHERE].reset(new SphereSphereDcdContact(contactFactory));
}

}; // Physics
}; // SurgSim

