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


std::list<std::shared_ptr<Contact>> OctreeDcdContact::doCalculateContact(
									 const std::shared_ptr<Math::Shape>& shape1,
									 const Math::RigidTransform3d& pose1,
									 const std::shared_ptr<Math::Shape>& shape2,
									 const Math::RigidTransform3d& pose2)
{
	SurgSim::DataStructures::OctreePath nodePath;
	std::list<std::shared_ptr<Contact>> result;
	std::shared_ptr<Math::OctreeShape> shape = std::static_pointer_cast<Math::OctreeShape>(shape1);
	calculateContactWithNode(shape->getOctree(), pose1, shape2, pose2, &nodePath, &result);
	return result;
}

void OctreeDcdContact::calculateContactWithNode(
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

	auto contacts = boxContactCalculation(*nodeShape, nodePose, *shape, shapePose);

	if (!contacts.empty())
	{
		if (node->hasChildren())
		{
			for (size_t i = 0; i < node->getChildren().size(); i++)
			{
				nodePath->push_back(i);
				calculateContactWithNode(node->getChild(i), octreePose, shape, shapePose, nodePath, result);
				nodePath->pop_back();
			}
		}
		else
		{
			Math::Vector3d contactPosition;
			for (auto& contact : contacts)
			{
				contact->penetrationPoints.first.octreeNodePath.setValue(*nodePath);

				contactPosition = contact->penetrationPoints.first.rigidLocalPosition.getValue();
				contactPosition += nodeCenter;
				contact->penetrationPoints.first.rigidLocalPosition.setValue(contactPosition);
			}
			result->splice(result->end(), contacts);
		}
	}
}

};
};

