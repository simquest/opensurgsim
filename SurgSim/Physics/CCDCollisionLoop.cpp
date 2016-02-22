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

#include "SurgSim/Physics/CcdCollisionLoop.h"
#include "SurgSim/Physics/CcdCollision.h"

namespace SurgSim
{

namespace Physics
{

CcdCollisionLoop::CcdCollisionLoop(bool copyState) :
	ComputationGroup(copyState)
{
	addComputation(std::make_shared<CcdCollision>(copyState));
}

CcdCollisionLoop::~CcdCollisionLoop()
{

}


bool CcdCollisionLoop::endIteration()
{
	return true;
}

}
}