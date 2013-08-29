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

#include <SurgSim/Framework/ReuseFactory.h>
#include <SurgSim/Collision/CollisionPair.h>

using SurgSim::Physics::RigidShapeType;

namespace SurgSim
{
namespace Collision
{

/// Base class responsible for calculating contact data between two given shapes, calculateContact needs to
/// determine whether the two shapes intersect, and if yes calculate the correct data for this contact, which
/// consists of, the normal to displace the first shape so that the two shapes just barely touch. And the
/// penetration point (the point that is furthest inside the other object) for each shape.
/// This base class also handles the swapping of the shapes if the pair is asymmetric. The sub classes
/// assume that the pair is always in correct order.
class ContactCalculation
{
public:

	/// Constructor
	explicit ContactCalculation()
	{
	}


	/// Destructor
	virtual ~ContactCalculation()
	{
	}

	/// Function that handles asymmetric pair and calls the actual contact calculation routine of the sub class.
	/// \param	pair	The pair that is under consideration.
	void calculateContact(std::shared_ptr<CollisionPair> pair)
	{
		if (needsSwap(pair->getFirst()->getShapeType(), pair->getSecond()->getShapeType()))
		{
			pair->swapRepresentations();
		}

		if(getShapeTypes().first != RigidShapeType::RIGID_SHAPE_TYPE_NONE)
		{
			SURGSIM_ASSERT(pair->getFirst()->getShapeType() == getShapeTypes().first) <<
				"First Object, wrong type of object" << pair->getFirst()->getShapeType();
		}

		if(getShapeTypes().second != RigidShapeType::RIGID_SHAPE_TYPE_NONE)
		{
		SURGSIM_ASSERT(pair->getSecond()->getShapeType() == getShapeTypes().second) <<
			"Second Object, wrong type of object" << pair->getSecond()->getShapeType();
		}

		doCalculateContact(pair);
	}

	virtual std::pair<int,int> getShapeTypes() = 0;

private:

	/// Calculate the actual contact between two shapes of the given CollisionPair.
	/// \param	pair	The symmetric pair that is under consideration.
	virtual void doCalculateContact(std::shared_ptr<CollisionPair> pair) = 0;

	bool needsSwap(int firstShapeType, int secondShapeType)
	{
		std::pair<int,int> shapeTypes = getShapeTypes();
		return firstShapeType != secondShapeType && firstShapeType == shapeTypes.second &&
			secondShapeType == shapeTypes.first;
	}

};

}; // namespace Collision
}; // namespace SurgSim


#include <SurgSim/Collision/DefaultContactCalculation.h>
#include <SurgSim/Collision/SphereSphereDcdContact.h>
#include <SurgSim/Collision/SphereDoubleSidedPlaneDcdContact.h>
#include <SurgSim/Collision/SpherePlaneDcdContact.h>
#include <SurgSim/Collision/BoxDoubleSidedPlaneDcdContact.h>
#include <SurgSim/Collision/BoxPlaneDcdContact.h>
#include <SurgSim/Collision/BoxSphereDcdContact.h>
#include <SurgSim/Collision/CapsuleSphereDcdContact.h>

#endif
