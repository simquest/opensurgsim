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

#include "SurgSim/Collision/ContactCalculation.h"

#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Math/Shape.h"

namespace SurgSim
{
namespace Collision
{

ContactCalculation::ContactCalculation()
{
}

ContactCalculation::~ContactCalculation()
{
}

void ContactCalculation::calculateContact(std::shared_ptr<CollisionPair> pair)
{
	doCalculateContact(pair);
}

std::list<std::shared_ptr<Contact>> ContactCalculation::calculateContact(
									 const std::shared_ptr<Math::Shape>& shape1,
									 const Math::RigidTransform3d& pose1,
									 const std::shared_ptr<Math::Shape>& shape2,
									 const Math::RigidTransform3d& pose2)
{
	auto types = getShapeTypes();
	auto incoming = std::make_pair(shape1->getType(), shape2->getType());
	if (incoming == types)
	{
		return doCalculateContact(shape1, pose1, shape2, pose2);
	}
	if (incoming.first == types.second && incoming.second == types.first)
	{
		SURGSIM_FAILURE() << "The shapes need to be passed in the correct order";
		/// HS-14-oct-2015 This section is currently unused as the collision pair interface will take care of the
		/// correct order, this will be correctly implemented and tested for the compound collision object
		return doCalculateContact(shape2, pose2, shape1, pose1);
	}

	SURGSIM_FAILURE() << "Incorrect shape type for this calculation expected "
					  << types.first << ", " << types.second
					  << " received " << incoming.first << ", " << incoming.second << ".";
	return std::list<std::shared_ptr<Contact>>();
}

void ContactCalculation::doCalculateContact(std::shared_ptr<CollisionPair> pair)
{
	std::pair<int, int> shapeTypes = getShapeTypes();
	int firstShapeType = pair->getFirst()->getShapeType();
	int secondShapeType = pair->getSecond()->getShapeType();

	if (firstShapeType != secondShapeType && firstShapeType == shapeTypes.second &&
		secondShapeType == shapeTypes.first)
	{
		pair->swapRepresentations();
		std::swap(firstShapeType, secondShapeType);
	}

	if (shapeTypes.first != SurgSim::Math::SHAPE_TYPE_NONE)
	{
		SURGSIM_ASSERT(firstShapeType == shapeTypes.first) <<
				"First Object, wrong type of object" << firstShapeType;
	}

	if (shapeTypes.second != SurgSim::Math::SHAPE_TYPE_NONE)
	{
		SURGSIM_ASSERT(secondShapeType == shapeTypes.second) <<
				"Second Object, wrong type of object" << secondShapeType;
	}

	std::shared_ptr<Math::Shape> shape1 = pair->getFirst()->getShape();
	if (shape1->isTransformable())
	{
		shape1 = pair->getFirst()->getPosedShape();
	}

	std::shared_ptr<Math::Shape> shape2 = pair->getSecond()->getShape();
	if (shape2->isTransformable())
	{
		shape2 = pair->getSecond()->getPosedShape();
	}

	auto contacts = doCalculateContact(shape1, pair->getFirst()->getPose(),
									   shape2, pair->getSecond()->getPose());
	for (auto& contact : contacts)
	{
		pair->addContact(contact);
	}
}

}; // namespace Collision
}; // namespace SurgSim
