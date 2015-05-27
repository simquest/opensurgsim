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


#include <memory>
#include <vector>

#include "SurgSim/Particles/Representation.h"
#include "SurgSim/Physics/ParticleCollisionResponse.h"
#include "SurgSim/Physics/PhysicsManagerState.h"


namespace SurgSim
{
namespace Physics
{

ParticleCollisionResponse::ParticleCollisionResponse(bool doCopyState) :
	Computation(doCopyState)
{
}

std::shared_ptr<PhysicsManagerState> ParticleCollisionResponse::doUpdate(const double& dt,
		const std::shared_ptr<PhysicsManagerState>& state)
{
	std::shared_ptr<PhysicsManagerState> result = state;

	auto& particleRepresentations = result->getActiveParticleRepresentations();
	for (auto& representation : particleRepresentations)
	{
		representation->handleCollisions(dt);
	}

	return result;
}


}; // Physics
}; // SurgSim
