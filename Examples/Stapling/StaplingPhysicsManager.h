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

#ifndef EXAMPLES_STAPLING_STAPLINGPHYSICSMANAGER_H 
#define EXAMPLES_STAPLING_STAPLINGPHYSICSMANAGER_H

#include "SurgSim/Physics/PhysicsManager.h"

/// StaplingPhyicsManager can use different Computations and/or different settings for those Computations to
/// improve performance.
class StaplingPhysicsManager : public SurgSim::Physics::PhysicsManager
{
protected:
	bool doInitialize() override;
};
#endif // EXAMPLES_STAPLING_STAPLINGPHYSICSMANAGER_H
