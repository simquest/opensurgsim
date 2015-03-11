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

#include "StaplingPhysicsManager.h"

#include "SurgSim/Physics/PreUpdate.h"
#include "SurgSim/Physics/FreeMotion.h"
#include "SurgSim/Physics/UpdateCollisionRepresentations.h"
#include "SurgSim/Physics/DcdCollision.h"
#include "SurgSim/Physics/ContactConstraintGeneration.h"
#include "SurgSim/Physics/BuildMlcp.h"
#include "SurgSim/Physics/SolveMlcp.h"
#include "SurgSim/Physics/PostUpdate.h"
#include "SurgSim/Physics/PushResults.h"

bool StaplingPhysicsManager::doInitialize()
{
	bool copyState = false;
	addComputation(std::make_shared<SurgSim::Physics::PreUpdate>(copyState));
	addComputation(std::make_shared<SurgSim::Physics::FreeMotion>(copyState));
	addComputation(std::make_shared<SurgSim::Physics::UpdateCollisionRepresentations>(copyState));
	addComputation(std::make_shared<SurgSim::Physics::DcdCollision>(copyState));
	addComputation(std::make_shared<SurgSim::Physics::ContactConstraintGeneration>(copyState));
	addComputation(std::make_shared<SurgSim::Physics::BuildMlcp>(copyState));
	auto solveMlcp = std::make_shared<SurgSim::Physics::SolveMlcp>(copyState);
	solveMlcp->setMaxIterations(150);
	addComputation(solveMlcp);
	addComputation(std::make_shared<SurgSim::Physics::PushResults>(copyState));
	addComputation(std::make_shared<SurgSim::Physics::PostUpdate>(copyState));
	return true;
}
