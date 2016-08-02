// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#ifndef SURGSIM_PHYSICS_CONTACTFILTERING_H
#define SURGSIM_PHYSICS_CONTACTFILTERING_H

#include <memory>

#include "SurgSim/Framework/Macros.h"
#include "SurgSim/Physics/Computation.h"

namespace SurgSim
{


namespace Physics
{
class PhysicsManagerState;

/// Computation to determine filter the contacts on collisions pairs.
/// This Computation class takes the list of collision pairs and the list of contact filters and will apply all
/// the filters to the collision pairs' contacts.
class ContactFiltering : public Computation
{
public:

	/// Constructor
	/// \param doCopyState Specify if the output state in Computation::Update() is a copy or not of the input state
	explicit ContactFiltering(bool doCopyState = false);

	SURGSIM_CLASSNAME(SurgSim::Physics::ContactFiltering);

	/// Destructor
	virtual ~ContactFiltering();

protected:

	std::shared_ptr<PhysicsManagerState> doUpdate(const double& dt, const std::shared_ptr<PhysicsManagerState>& state)
	override;
};

}; // Physics
}; // SurgSim

#endif
