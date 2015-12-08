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

#ifndef SURGSIM_PHYSICS_CCDCOLLISION_H
#define SURGSIM_PHYSICS_CCDCOLLISION_H

#include <memory>

#include "SurgSim/Math/Shape.h"
#include "SurgSim/Physics/Computation.h"

namespace SurgSim
{

namespace Collision
{
class ContactCalculation;
}

namespace Physics
{
class PhysicsManagerState;

/// Computation to determine the contacts between a list of CollisionPairs.
/// This Computation class takes a list of representations, it will generate a list of collision pairs
/// from this list on every frame, for each CollisionPair, it uses a two dimensional table of
/// function objects (ContactCalculation) to determine how to calculate a contact between the two
/// members of each pair, if no specific function exists a default function will be used.
/// will update the collision pairs accordingly.
/// \note When a new ContactCalculation type gets implemented, the type needs to be registered with the table
/// inside of ContactCalculation
class CcdCollision : public Computation
{
public:

	/// Constructor
	/// \param doCopyState Specify if the output state in Computation::Update() is a copy or not of the input state
	explicit CcdCollision(bool doCopyState = false);

	SURGSIM_CLASSNAME(SurgSim::Physics::CcdCollision);

	/// Destructor
	virtual ~CcdCollision();

protected:

	/// Executes the update operation, overridden from Computation.
	/// \param dt	The time passed.
	/// \param state The PhysicsManagerState from previous computation.
	std::shared_ptr<PhysicsManagerState> doUpdate(const double& dt, const std::shared_ptr<PhysicsManagerState>& state)
	override;
};

}; // Physics
}; // SurgSim

#endif // SURGSIM_PHYSICS_CCDCOLLISION_H
