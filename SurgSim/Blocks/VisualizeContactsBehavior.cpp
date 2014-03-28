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

#include <algorithm>

#include "SurgSim/Blocks/VisualizeContactsBehavior.h"
#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/DataStructures/Vertex.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Graphics/OsgVectorFieldRepresentation.h"
#include "SurgSim/Graphics/VectorField.h"

using SurgSim::DataStructures::Vertex;
using SurgSim::Collision::Contact;
using SurgSim::Collision::Representation;
using SurgSim::Graphics::OsgVectorFieldRepresentation;
using SurgSim::Graphics::VectorField;
using SurgSim::Graphics::VectorFieldData;

namespace SurgSim
{

namespace Blocks
{

VisualizeContactsBehavior::VisualizeContactsBehavior(const std::string& name):
	SurgSim::Framework::Behavior(name),
	m_vectorField(std::make_shared<OsgVectorFieldRepresentation>("VisualizeContacts"))
{
}

void VisualizeContactsBehavior::setCollisionRepresentation(
	std::shared_ptr<Representation> collisionRepresentation)
{
	m_collisionRepresentation = collisionRepresentation;
}

void VisualizeContactsBehavior::update(double dt)
{
	if (m_collisionRepresentation->hasCollision())
	{
		std::unordered_map<std::shared_ptr<Representation>,
						   std::list<std::shared_ptr<Contact>>> collisions = m_collisionRepresentation->getCollisions();

		size_t totalContacts = 0;
		for (auto collision = collisions.cbegin(); collision != collisions.cend(); ++collision)
		{
			totalContacts += collision->second.size();
		}

		std::shared_ptr<VectorField> vectorField = m_vectorField->getVectorField();
		vectorField->getVertices().reserve(2 * totalContacts);
		vectorField->clear();

		for (auto it = std::begin(collisions); it != std::end(collisions); ++it)
		{
			for (auto iter = std::begin((*it).second); iter != std::end((*it).second); ++iter)
			{
				VectorFieldData vectorData1;
				VectorFieldData vectorData2;
				vectorData1.direction = -(*iter)->normal * (*iter)->depth;
				vectorData2.direction =  (*iter)->normal * (*iter)->depth;

				Vertex<VectorFieldData> vertex1 =
					Vertex<VectorFieldData>((*iter)->penetrationPoints.first.globalPosition.getValue(), vectorData1);
				Vertex<VectorFieldData> vertex2 =
					Vertex<VectorFieldData>((*iter)->penetrationPoints.second.globalPosition.getValue(), vectorData2);

				vectorField->addVertex(vertex1);
				vectorField->addVertex(vertex2);
			}
		}
		m_vectorField->setVisible(true);
	}
	else
	{
		m_vectorField->setVisible(false);
	}
}

int VisualizeContactsBehavior::getTargetManagerType() const
{
	return SurgSim::Framework::MANAGER_TYPE_GRAPHICS;
}

bool VisualizeContactsBehavior::doInitialize()
{
	SURGSIM_ASSERT(m_collisionRepresentation) << "VisualizeContactsBehavior: no collision representation held.";
	return true;
}

bool VisualizeContactsBehavior::doWakeUp()
{
	getSceneElement()->addComponent(m_vectorField);
	return true;
}

void VisualizeContactsBehavior::setVectorFieldScale(double scale)
{
	SURGSIM_ASSERT(scale > 0.0) << "Scale of vector field must be positive.";
	m_vectorField->setScale(scale);
}

} // namespace Blocks
} // namespace SurgSim