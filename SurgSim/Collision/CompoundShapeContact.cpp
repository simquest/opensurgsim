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
#include "SurgSim/Collision/CompoundShapeContact.h"
#include "SurgSim/Math/CompoundShape.h"

namespace SurgSim
{

namespace Collision
{

CompoundShapeContact::CompoundShapeContact(const std::pair<int, int>& types) : m_types(types)
{

}

std::pair<int, int> CompoundShapeContact::getShapeTypes()
{
	return m_types;
}

std::list<std::shared_ptr<Contact>> CompoundShapeContact::doCalculateDcdContact(
									 const Math::PosedShape<std::shared_ptr<Math::Shape>>& posedShape1,
									 const Math::PosedShape<std::shared_ptr<Math::Shape>>& posedShape2)
{
	typedef Math::PosedShape<std::shared_ptr<Math::Shape>> PosedShape;

	const auto& calculations = ContactCalculation::getDcdContactTable();

	// Shape1 is compound shape
	const auto& compoundShape = std::static_pointer_cast<Math::CompoundShape>(posedShape1.getShape());
	SURGSIM_ASSERT(compoundShape->getType() == posedShape1.getShape()->getType()) <<
			"Invalid static cast to compound shape";

	size_t index = 0;
	std::list<std::shared_ptr<Contact>> contacts;
	for (const Math::CompoundShape::SubShape& subShape : compoundShape->getShapes())
	{
		const auto& calculation = calculations[subShape.first->getType()][posedShape2.getShape()->getType()];
		std::list<std::shared_ptr<Contact>> localContacts = calculation->calculateDcdContact(
			PosedShape(subShape.first, posedShape1.getPose() * subShape.second), posedShape2);

		for (auto& contact : localContacts)
		{
			auto& locations = contact->penetrationPoints.first;
			locations.index = index;
			if (locations.rigidLocalPosition.hasValue())
			{
				locations.rigidLocalPosition.setValue(subShape.second * locations.rigidLocalPosition.getValue());
			}
		}

		contacts.splice(contacts.end(), localContacts);
		index++;
	}
	return contacts;
}

}
}