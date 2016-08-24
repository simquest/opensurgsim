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

	std::list<std::shared_ptr<Contact>> contacts;

	const auto& calculations = ContactCalculation::getDcdContactTable();

	// Shape1 is compound shape
	const auto& compoundShape = std::static_pointer_cast<Math::CompoundShape>(posedShape1.getShape());

	SURGSIM_ASSERT(compoundShape->getType() == posedShape1.getShape()->getType()) <<
			"Invalid static cast to compound shape";

	Math::RigidTransform3d inversePose = posedShape1.getPose().inverse();
	size_t index = 0;
	for (const Math::CompoundShape::SubShape& subShape : compoundShape->getShapes())
	{
		const auto& calculation = calculations[subShape.first->getType()][posedShape2.getShape()->getType()];
		auto relativePose = inversePose * subShape.second;
		std::list<std::shared_ptr<Contact>> localContacts = calculation->calculateDcdContact(
											 PosedShape(subShape.first, subShape.second), posedShape2);

		for (auto& contact : localContacts)
		{
			auto& locations = contact->penetrationPoints.first;
			locations.index = index;
			if (locations.rigidLocalPosition.hasValue())
			{
				locations.rigidLocalPosition.setValue(relativePose * locations.rigidLocalPosition.getValue());
			}
		}

		contacts.splice(contacts.end(), localContacts);
		index++;
	}
	return contacts;
}

std::list<std::shared_ptr<SurgSim::Collision::Contact>> CompoundShapeContact::doCalculateCcdContact(
			const Math::PosedShapeMotion<std::shared_ptr<Math::Shape>>& shape1,
			const Math::PosedShapeMotion<std::shared_ptr<Math::Shape>>& shape2)
{
	typedef Math::PosedShape<std::shared_ptr<Math::Shape>> PosedShape;
	typedef Math::PosedShapeMotion<std::shared_ptr<Math::Shape>> Motionshape;

	std::list<std::shared_ptr<Contact>> contacts;

	const auto& calculations = ContactCalculation::getCcdContactTable();

	// Shape1 is compound shape
	const auto& startShape = std::static_pointer_cast<Math::CompoundShape>(shape1.first.getShape());
	const auto& endShape = std::static_pointer_cast<Math::CompoundShape>(shape1.second.getShape());


	// Get Motion Shapes from left hand side
	// For each Motion Shape run that shape and the opposing side through the CCD contact calc
	SURGSIM_ASSERT(startShape->getType() == shape1.first.getShape()->getType()) <<
			"Invalid static cast to compound shape";
	SURGSIM_ASSERT(endShape->getType() == shape1.second.getShape()->getType()) <<
			"Invalid static cast to compound shape";


	size_t numShapes = startShape->getNumShapes();
	for (size_t index = 0; index < numShapes; ++index)
	{
		auto posedSubshape1 = PosedShape(startShape->getShape(index), startShape->getPose(index));
		auto type = posedSubshape1.getShape()->getType();
		auto posedSubshape2 = PosedShape(endShape->getShape(index), endShape->getPose(index));

		auto motionSubShape = Motionshape(posedSubshape1, posedSubshape2);

		const auto& calculation = calculations[type][shape2.second.getShape()->getType()];

		std::list<std::shared_ptr<Contact>> localContacts = calculation->calculateCcdContact(
											 motionSubShape, shape2);
		for (auto& contact : localContacts)
		{
			auto t = contact->time;
			Math::RigidTransform3d inverseGlobal =
				Math::interpolate(shape1.first.getPose(), shape1.second.getPose(), t).inverse();

			Math::RigidTransform3d relativePose =
				inverseGlobal * Math::interpolate(motionSubShape.first.getPose(), motionSubShape.second.getPose(), t);

			auto& locations = contact->penetrationPoints.first;
			locations.index = index;
			if (locations.rigidLocalPosition.hasValue())
			{
				locations.rigidLocalPosition.setValue(relativePose * locations.rigidLocalPosition.getValue());
			}
		}

		contacts.splice(contacts.end(), localContacts);
	}

	return contacts;
}

}
}