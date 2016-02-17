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

#ifndef SURGSIM_COLLISION_SHAPESHAPECONTACTCALCULATION_H
#define SURGSIM_COLLISION_SHAPESHAPECONTACTCALCULATION_H

#include "SurgSim/Collision/ContactCalculation.h"
#include "SurgSim/Math/Shape.h"

namespace SurgSim
{

namespace Collision
{

/// Class that can automate the type conversion and provides a consistent interface to the typed call
/// Takes the shapes to convert as template parameters
template <class Shape1, class Shape2>
class ShapeShapeContactCalculation : public ContactCalculation
{
	/// Virtual function to be overridden, this provides the typed contact calculation between two shapes
	/// it takes two shapes and their respective poses, and will return the contacts between those two shapes
	virtual std::list<std::shared_ptr<Contact>> calculateDcdContact(
		const Shape1& shape1, const Math::RigidTransform3d& pose1,
		const Shape2& shape2, const Math::RigidTransform3d& pose2) const
	{
		SURGSIM_FAILURE() << "Not implemented";
		return std::list<std::shared_ptr<Contact>>();
	}

	virtual std::list<std::shared_ptr<Contact>> calculateCcdContact(
		const Shape1& shape1AtTime0, const Math::RigidTransform3d& pose1AtTime0,
		const Shape1& shape1AtTime1, const Math::RigidTransform3d& pose1AtTime1,
		const Shape2& shape2AtTime0, const Math::RigidTransform3d& pose2AtTime0,
		const Shape2& shape2AtTime1, const Math::RigidTransform3d& pose2AtTime1) const
	{
		SURGSIM_FAILURE() << "Not implemented";
		return std::list<std::shared_ptr<Contact>>();
	}

	/// Overrides the dcd contact calculation to go from untyped shapes to the typed shapes
	std::list<std::shared_ptr<Contact>> doCalculateDcdContact(const std::shared_ptr<Math::Shape>& shape1,
		const std::shared_ptr<Math::Shape>& shape2) override
	{
		auto one = std::static_pointer_cast<Shape1>(shape1);
		auto two = std::static_pointer_cast<Shape2>(shape2);

		SURGSIM_ASSERT(one->getType() == shape1->getType()) << "Invalid Shape 1";
		SURGSIM_ASSERT(two->getType() == shape2->getType()) << "Invalid Shape 2";

		return calculateDcdContact(*one, shape1->getPose(), *two, shape2->getPose());
	}

	/// Overrides the ccd contact calculation to go from untyped shapes to the typed shapes
	std::list<std::shared_ptr<Contact>> doCalculateCcdContact(const Math::ShapeMotion& shapeMotion1,
		const Math::ShapeMotion& shapeMotion2) override
	{
		auto oneAtTime0 = std::static_pointer_cast<Shape1>(shapeMotion1.first);
		auto oneAtTime1 = std::static_pointer_cast<Shape1>(shapeMotion1.second);
		auto twoAtTime0 = std::static_pointer_cast<Shape2>(shapeMotion2.first);
		auto twoAtTime1 = std::static_pointer_cast<Shape2>(shapeMotion2.second);

		SURGSIM_ASSERT(oneAtTime0->getType() == shapeMotion1.first->getType()) << "Invalid Shape 1 detected at time 0";
		SURGSIM_ASSERT(oneAtTime1->getType() == shapeMotion1.second->getType()) << "Invalid Shape 1 detected at time 1";
		SURGSIM_ASSERT(twoAtTime0->getType() == shapeMotion2.first->getType()) << "Invalid Shape 2 detected at time 0";
		SURGSIM_ASSERT(twoAtTime1->getType() == shapeMotion2.second->getType()) << "Invalid Shape 2 detected at time 1";

		return calculateCcdContact(
			*oneAtTime0, oneAtTime0->getPose(), *oneAtTime1, oneAtTime1->getPose(),
			*twoAtTime0, twoAtTime0->getPose(), *twoAtTime1, twoAtTime1->getPose());
	}
};

}
}

#endif
