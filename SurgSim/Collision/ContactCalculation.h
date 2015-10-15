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

#ifndef SURGSIM_COLLISION_CONTACTCALCULATION_H
#define SURGSIM_COLLISION_CONTACTCALCULATION_H

#include <memory>

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Math/RigidTransform.h"



namespace SurgSim
{

namespace Math
{
class Shape;
}

namespace Collision
{

/// Base class responsible for calculating contact data between two objects.
/// It is used for determining whether two objects intersect. If there is
/// contact, new Contacts are calculated.
/// \sa Contact
class ContactCalculation
{
public:

	/// Constructor
	ContactCalculation();

	/// Destructor
	virtual ~ContactCalculation();

	/// Calculate the contacts
	/// \param	pair	A CollisionPair that is under consideration, new contacts will be added to this pair
	void calculateContact(std::shared_ptr<CollisionPair> pair);

	/// Calculate the contacts between two shapes
	/// \param shape1, shape2 The shapes for which to calculate the contacts
	/// \param pose1, pose2 The respective poses for the shapes
	/// \return a list of contacts between the two given shapes
	std::list<std::shared_ptr<Contact>> calculateContact(
										 const std::shared_ptr<Math::Shape>& shape1,
										 const Math::RigidTransform3d& pose1,
										 const std::shared_ptr<Math::Shape>& shape2,
										 const Math::RigidTransform3d& pose2);

	/// Virtual function that returns the shapes that this ContactCalculation class handles.
	/// \return Return the shape types this class handles.
	virtual std::pair<int, int> getShapeTypes() = 0;

private:

	/// Calculate the actual contact between two shapes of the given CollisionPair.
	/// \param	pair	The symmetric pair that is under consideration.
	virtual void doCalculateContact(std::shared_ptr<CollisionPair> pair);

	/// Virtual function receives the call from the public interface, usually will type the
	/// shapes statically to their known types and then execute a specific contact calculation
	/// between the two shapes
	/// \param shape1, shape2 The shapes for which to calculate the contacts
	/// \param pose1, pose2 The respective poses for the shapes
	/// \return a list of contacts between the two given shapes
	virtual std::list<std::shared_ptr<Contact>> doCalculateContact(
				const std::shared_ptr<Math::Shape>& shape1, const Math::RigidTransform3d& pose1,
				const std::shared_ptr<Math::Shape>& shape2, const Math::RigidTransform3d& pose2) = 0;

};

}; // namespace Collision
}; // namespace SurgSim

#endif
