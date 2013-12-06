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

#ifndef SURGSIM_COLLISION_OCTREEDCDCONTACT_INL_H
#define SURGSIM_COLLISION_OCTREEDCDCONTACT_INL_H

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Collision/ShapeCollisionRepresentation.h"
#include "SurgSim/Math/BoxShape.h"
#include "SurgSim/Math/OctreeShape.h"
#include "SurgSim/Math/Shape.h"
#include "SurgSim/Math/Vector.h"


namespace SurgSim
{
namespace Collision
{

template <class T, class Data>
OctreeDcdContact<T, Data>::OctreeDcdContact()
{
	SURGSIM_ASSERT(m_contactCalculator.getShapeTypes().first == SurgSim::Math::SHAPE_TYPE_BOX) <<
		"OctreeDcdContact needs a contact calculator that works with Boxes";
	m_shapeTypes = m_contactCalculator.getShapeTypes();
	m_shapeTypes.first = SurgSim::Math::SHAPE_TYPE_OCTREE;
}

template <class T, class Data>
std::pair<int, int> OctreeDcdContact<T, Data>::getShapeTypes()
{
	return m_shapeTypes;
}

template <class T, class Data>
void OctreeDcdContact<T, Data>::doCalculateContact(std::shared_ptr<CollisionPair> pair)
{
	typedef SurgSim::Math::OctreeShape<Data> OctreeShapeType;
	std::shared_ptr<OctreeShapeType> octree = std::static_pointer_cast<OctreeShapeType>(pair->getFirst()->getShape());
	calculateContactWithNode(octree->getRootNode(), pair);
}

template <class T, class Data>
void OctreeDcdContact<T, Data>::calculateContactWithNode(
	std::shared_ptr<SurgSim::DataStructures::OctreeNode<Data>> node, std::shared_ptr<CollisionPair> pair,
	std::shared_ptr<SurgSim::Math::OctreePath> nodePath)
{
	if (! node->isActive())
		return;

	SurgSim::Math::Vector3d boxSize = node->getBoundingBox().sizes();
	std::shared_ptr<SurgSim::Math::Shape> boxShape =
		std::make_shared<SurgSim::Math::BoxShape>(boxSize.x(), boxSize.y(), boxSize.z());
	SurgSim::Math::RigidTransform3d boxPose = pair->getFirst()->getPose();
	boxPose.translation() += boxPose.linear() * node->getBoundingBox().center();

	std::shared_ptr<Representation> box =
		std::make_shared<ShapeCollisionRepresentation>("Octree Node", boxShape, boxPose);

	std::shared_ptr<CollisionPair> localPair = std::make_shared<CollisionPair>(box, pair->getSecond());
	m_contactCalculator.calculateContact(localPair);
	if (localPair->hasContacts())
	{
		if (node->hasChildren())
		{
			for (size_t i = 0; i < node->getChildren().size(); i++)
			{
				nodePath->push_back(i);
				calculateContactWithNode(node->getChild(i), pair, nodePath);
				nodePath->pop_back();
			}
		}
		else
		{
			const std::list<std::shared_ptr<Contact>>& newContacts = localPair->getContacts();
			for(auto contact=newContacts.cbegin(); contact!=newContacts.cend(); ++contact)
			{
				(*contact)->penetrationPoints.first.octreeNodePath.setValue(*nodePath);
				pair->addContact(*contact);
			}
		}
	}
}

};
};

#endif
