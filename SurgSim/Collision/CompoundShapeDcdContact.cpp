// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
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

#include <memory>

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/CompoundShapeDcdContact.h"
#include "SurgSim/Math/CompoundShape.h"

namespace SurgSim
{

namespace Collision
{

CompoundShapeDcdContact::CompoundShapeDcdContact(const std::pair<int, int>& types) : m_types(types)
{

}

std::pair<int, int> CompoundShapeDcdContact::getShapeTypes()
{
	return m_types;
}

std::list<std::shared_ptr<Contact>> CompoundShapeDcdContact::doCalculateDcdContact(
	const std::shared_ptr<Math::Shape>& shape1, const Math::RigidTransform3d& pose1,
	const std::shared_ptr<Math::Shape>& shape2, const Math::RigidTransform3d& pose2)
{
	std::list<std::shared_ptr<Contact>> contacts;

	const auto& calculations = ContactCalculation::getContactTable();

	// Shape1 is compound shape
	const auto& compoundShape = std::static_pointer_cast<Math::CompoundShape>(shape1);

	SURGSIM_ASSERT(compoundShape->getType() == shape1->getType()) << "Invalid static cast to compound shape";

	for (const auto& subShape : compoundShape->getShapes())
	{
		const auto& calculation = calculations[subShape.first->getType()][shape2->getType()];

		std::list<std::shared_ptr<Contact>> localContacts;

		if (subShape.first->isTransformable())
		{
			auto pose = pose1 * subShape.second;
			localContacts = calculation->calculateDcdContact(subShape.first->getTransformed(pose), pose, shape2, pose2);
		}
		else
		{
			localContacts = calculation->calculateDcdContact(subShape.first, pose1 * subShape.second, shape2, pose2);
		}

		for (auto& contact : localContacts)
		{
			auto& locations = contact->penetrationPoints.first;
			if (locations.rigidLocalPosition.hasValue())
			{
				locations.rigidLocalPosition.setValue(subShape.second * locations.rigidLocalPosition.getValue());
			}
		}

		contacts.splice(contacts.end(), localContacts);
	}
	return contacts;

}

}
}