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

#ifndef SURGSIM_PHYSICS_UPDATECOLLISIONS_H
#define SURGSIM_PHYSICS_UPDATECOLLISIONS_H

#include "SurgSim/Physics/Computation.h"

namespace SurgSim
{
namespace Physics
{

/// Computation that calls the CollisionRepresentations update() function
class UpdateCollisions : public Computation
{
public:
	/// Constructor
	/// \param doCopyState whether to copy the PhysicsManagerState on update
	explicit UpdateCollisions(bool doCopyState);

	/// Destructor
	virtual ~UpdateCollisions();

	std::shared_ptr<PhysicsManagerState> doUpdate(const double& dt, const std::shared_ptr<PhysicsManagerState>& state)
		override;

};

}
}

#endif
