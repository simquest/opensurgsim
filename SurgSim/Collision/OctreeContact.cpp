// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#include "SurgSim/Collision/OctreeContact.h"

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

size_t OctreeContact::Vector3dHash::operator()(const SurgSim::Math::Vector3d& id) const
{
	return boost::hash_range(id.data(), id.data() + 3);
}


std::list<std::shared_ptr<Contact>> OctreeContact::doCalculateDcdContact(
	const Math::PosedShape<std::shared_ptr<Math::Shape>>& posedShape1,
	const Math::PosedShape<std::shared_ptr<Math::Shape>>& posedShape2)
{
	SURGSIM_ASSERT(posedShape1.getShape()->getType() == Math::SHAPE_TYPE_OCTREE) <<
		"Octree Contact needs an OctreeShape.";

	SurgSim::DataStructures::OctreePath nodePath;
	std::list<std::shared_ptr<Contact>> result;
	std::shared_ptr<Math::OctreeShape> shape = std::static_pointer_cast<Math::OctreeShape>(posedShape1.getShape());

	calculateContactWithNode(shape->getOctree(), posedShape1.getPose(),
		posedShape2.getShape(), posedShape2.getPose(), &nodePath, &result);

	return result;
}

void OctreeContact::calculateContactWithNode(
	std::shared_ptr<const Math::OctreeShape::NodeType> node,
	Math::RigidTransform3d octreePose,
	const std::shared_ptr<Math::Shape>& shape,
	const Math::RigidTransform3d& shapePose,
	SurgSim::DataStructures::OctreePath* nodePath,
	std::list<std::shared_ptr<Contact>>* result)
{
	if (! node->isActive())
	{
		return;
	}

	auto nodeBoundingBox = Math::transformAabb(octreePose, node->getBoundingBox());
	auto shapeBoundingBox = shape->getBoundingBox();
	if (!shape->isTransformable())
	{
		shapeBoundingBox = Math::transformAabb(shapePose, shapeBoundingBox);
	}
	if (!shapeBoundingBox.isEmpty() && !Math::doAabbIntersect(nodeBoundingBox, shapeBoundingBox))
	{
		return;
	}

	Math::Vector3d nodeSize = node->getBoundingBox().sizes();
	std::shared_ptr<Math::BoxShape> nodeShape;
	if (m_shapes.count(nodeSize) > 0)
	{
		nodeShape = m_shapes[nodeSize];
	}
	else
	{
		nodeShape = std::make_shared<SurgSim::Math::BoxShape>(nodeSize.x(), nodeSize.y(), nodeSize.z());
		m_shapes[nodeSize] = nodeShape;
	}

	Math::Vector3d nodeCenter = node->getBoundingBox().center();
	Math::RigidTransform3d nodePose = octreePose;
	nodePose.translation() += nodePose.linear() * nodeCenter;

	if (node->hasChildren())
	{
		if ((!shapeBoundingBox.isEmpty() && nodeBoundingBox.contains(shapeBoundingBox)) ||
			 !boxContactCalculation(*nodeShape, nodePose, *shape, shapePose).empty())
		{
			for (size_t i = 0; i < node->getChildren().size(); i++)
			{
				nodePath->push_back(i);
				calculateContactWithNode(node->getChild(i), octreePose, shape, shapePose, nodePath, result);
				nodePath->pop_back();
			}
		}
	}
	else
	{
		auto contacts = boxContactCalculation(*nodeShape, nodePose, *shape, shapePose);
		for (auto& contact : contacts)
		{
			contact->penetrationPoints.first.octreeNodePath.setValue(*nodePath);

			Math::Vector3d contactPosition = contact->penetrationPoints.first.rigidLocalPosition.getValue();
			contactPosition += nodeCenter;
			contact->penetrationPoints.first.rigidLocalPosition.setValue(contactPosition);
		}
		result->splice(result->end(), contacts);
	}
}

};
};

