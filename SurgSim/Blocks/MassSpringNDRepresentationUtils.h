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

#ifndef SURGSIM_BLOCKS_MASSSPRINGNDREPRESENTATIONUTILS_H
#define SURGSIM_BLOCKS_MASSSPRINGNDREPRESENTATIONUTILS_H

#include <memory>

#include <SurgSim/Physics/LinearSpring.h>
#include <SurgSim/Physics/DeformableRepresentationState.h>

using SurgSim::Physics::LinearSpring;
using SurgSim::Physics::DeformableRepresentationState;

namespace SurgSim
{

namespace Blocks
{

/// Helper method to create a LinearSpring
/// \param state The state to initialize the spring with (rest length calculation)
/// \param nodeId0, nodeId1 Node ids of the 2 connected masses
/// \param stiffness, damping The spring parameters
/// \return The newly create spring
std::shared_ptr<LinearSpring> createLinearSpring(const std::shared_ptr<DeformableRepresentationState> state,
	unsigned int nodeId0, unsigned int nodeId1,
	double stiffness, double damping);

};  // namespace Blocks

};  // namespace SurgSim

#endif  // SURGSIM_BLOCKS_MASSSPRINGNDREPRESENTATIONUTILS_H
