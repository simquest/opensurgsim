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

#include "SurgSim/Collision/OctreeDcdContact.h"

#include <boost/functional/hash.hpp>

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Collision/ShapeCollisionRepresentation.h"
#include "SurgSim/Math/BoxShape.h"
#include "SurgSim/Math/Shape.h"
#include "SurgSim/Math/Vector.h"


namespace SurgSim
{
namespace Collision
{

size_t OctreeDcdContact::Vector3dHash::operator()(const SurgSim::Math::Vector3d& id) const
{
	return boost::hash_range(id.data(), id.data() + 3);
}

OctreeDcdContact::OctreeDcdContact(std::shared_ptr<ContactCalculation> calculator) :
	m_calculator(calculator)
{
	SURGSIM_ASSERT(m_calculator->getShapeTypes().first == SurgSim::Math::SHAPE_TYPE_BOX) <<
			"OctreeDcdContact needs a contact calculator that works with Boxes";
	m_shapeTypes = m_calculator->getShapeTypes();
	m_shapeTypes.first = SurgSim::Math::SHAPE_TYPE_OCTREE;

	m_nodeCollisionRepresentation = std::make_shared<ShapeCollisionRepresentation>("Octree Node");
}

std::pair<int, int> OctreeDcdContact::getShapeTypes()
{
	return m_shapeTypes;
}

void OctreeDcdContact::doCalculateContact(std::shared_ptr<CollisionPair> pair)
{
	typedef SurgSim::Math::OctreeShape OctreeShapeType;
	std::shared_ptr<OctreeShapeType> shape = std::static_pointer_cast<OctreeShapeType>(pair->getFirst()->getShape());
	calculateContactWithNode(shape->getOctree(), pair, std::make_shared<SurgSim::DataStructures::OctreePath>());
	pair->getSecond()->getCollisions().unsafeGet().erase(m_nodeCollisionRepresentation);
}

void OctreeDcdContact::calculateContactWithNode(
	std::shared_ptr<const SurgSim::Math::OctreeShape::NodeType> node,
	std::shared_ptr<CollisionPair> pair,
	std::shared_ptr<SurgSim::DataStructures::OctreePath> nodePath)
{
	if (! node->isActive())
	{
		return;
	}

	SurgSim::Math::Vector3d nodeSize = node->getBoundingBox().sizes();
	std::shared_ptr<SurgSim::Math::Shape> nodeShape;
	if (m_shapes.count(nodeSize) > 0)
	{
		nodeShape = m_shapes[nodeSize];
	}
	else
	{
		nodeShape = std::make_shared<SurgSim::Math::BoxShape>(nodeSize.x(), nodeSize.y(), nodeSize.z());
		m_shapes[nodeSize] = nodeShape;
	}
	SurgSim::Math::Vector3d nodeCenter = node->getBoundingBox().center();
	SurgSim::Math::RigidTransform3d nodePose = pair->getFirst()->getPose();
	nodePose.translation() += nodePose.linear() * nodeCenter;

	m_nodeCollisionRepresentation->setShape(nodeShape);
	m_nodeCollisionRepresentation->setLocalPose(nodePose);

	std::shared_ptr<CollisionPair> localPair = std::make_shared<CollisionPair>(m_nodeCollisionRepresentation,
			pair->getSecond());
	m_calculator->calculateContact(localPair);

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
			SurgSim::Math::Vector3d contactPosition;
			for (auto contact = newContacts.cbegin(); contact != newContacts.cend(); ++contact)
			{
				(*contact)->penetrationPoints.first.octreeNodePath.setValue(*nodePath);

				contactPosition = (*contact)->penetrationPoints.first.rigidLocalPosition.getValue();
				contactPosition += nodeCenter;
				(*contact)->penetrationPoints.first.rigidLocalPosition.setValue(contactPosition);

				pair->addContact(*contact);
			}
		}
	}
}

};
};

