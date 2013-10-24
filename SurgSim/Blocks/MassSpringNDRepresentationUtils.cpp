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

#include <SurgSim/Blocks/MassSpringNDRepresentationUtils.h>

namespace SurgSim
{

namespace Blocks
{

std::shared_ptr<LinearSpring> createLinearSpring(const std::shared_ptr<DeformableRepresentationState> state,
	unsigned int nodeId0, unsigned int nodeId1,
	double stiffness, double damping)
{
	using SurgSim::Math::Vector3d;

	std::shared_ptr<LinearSpring> spring = std::make_shared<LinearSpring>(nodeId0, nodeId1);

	const Vector3d& A = SurgSim::Math::getSubVector(state->getPositions(), nodeId0, 3);
	const Vector3d& B = SurgSim::Math::getSubVector(state->getPositions(), nodeId1, 3);
	spring->setStiffness(stiffness);
	spring->setDamping(damping);
	spring->setRestLength((B-A).norm());

	return spring;
}

}; // namespace Blocks

}; // namespace SurgSim
