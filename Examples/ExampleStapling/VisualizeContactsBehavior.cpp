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

#include "Examples/ExampleStapling/VisualizeContactsBehavior.h"

#include <algorithm>

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/DataStructures/Vertex.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Graphics/OsgVectorFieldRepresentation.h"
#include "SurgSim/Graphics/VectorField.h"
#include "SurgSim/Graphics/VectorFieldRepresentation.h"

using SurgSim::DataStructures::Vertex;
using SurgSim::Collision::Contact;
using SurgSim::Collision::Representation;
using SurgSim::Graphics::OsgVectorFieldRepresentation;
using SurgSim::Graphics::VectorField;
using SurgSim::Graphics::VectorFieldData;
using SurgSim::Math::Vector3d;

VisualizeContactsBehavior::VisualizeContactsBehavior(const std::string& name):
	SurgSim::Framework::Behavior(name),
	m_vectorFiled(std::make_shared<OsgVectorFieldRepresentation>("VisualizeContacts"))
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

		unsigned int totalContacts = 0;
		std::for_each(std::begin(collisions), std::end(collisions),
			[&totalContacts](const std::pair<std::shared_ptr<Representation>,
											 std::list<std::shared_ptr<Contact>>>& collision)
			{
				totalContacts += static_cast<int>(collision.second.size());
			});

		std::shared_ptr<VectorField> vectorField = m_vectorFiled->getVectorField();
		int itemsToPop = static_cast<int>(vectorField->getNumVertices()) - static_cast<int>(2 * totalContacts);
		if (itemsToPop < 0)
		{
			vectorField->getVertices().reserve(2 * totalContacts);
		}
		m_vectorFiled->getVectorField()->clear();

		unsigned int index = 0;
		for (auto it = std::begin(collisions); it != std::end(collisions); ++it)
		{
			for (auto iter = std::begin((*it).second); iter != std::end((*it).second); ++iter)
			{
				VectorFieldData vectorData1;
				VectorFieldData vectorData2;
				// Note: Since usually the 'depth' of penetration is so small (at the magnitude of mm),
				// the length of vector will be too small to be seen on the screen.
				// We multiply the length of vector by 200 to make it 'visible'.
				// The number '200' serves as a magic number here.
				vectorData1.direction = -(*iter)->normal * (*iter)->depth * 200;
				vectorData2.direction =  (*iter)->normal * (*iter)->depth * 200;

				Vertex<VectorFieldData> vertex1 =
					Vertex<VectorFieldData>((*iter)->penetrationPoints.first.globalPosition.getValue(), vectorData1);
				Vertex<VectorFieldData> vertex2 =
					Vertex<VectorFieldData>((*iter)->penetrationPoints.second.globalPosition.getValue(), vectorData2);

				vectorField->addVertex(vertex1);
				vectorField->addVertex(vertex2);
			}
		}
		m_vectorFiled->setVisible(true);
	}
	else
	{
		m_vectorFiled->setVisible(false);
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
	getSceneElement()->addComponent(m_vectorFiled);
	return true;
}
